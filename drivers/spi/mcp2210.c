/*
 *  MCP2210 USB to SPI bridge driver
 *
 *  Copyright (c) 2018 Stefan Schmidt <stefan@datenfreihafen.org>
 *
 *  Based on other drivers with following copyright:
 *  Copyright (c) 2013 Mathew King <mking@trilithic.com> for Trilithic, Inc
 *  Copyright (c) 2013-2017 Daniel Santos <daniel.santos@pobox.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/usb.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/types.h>
#include <linux/device.h>

#define USB_VENDOR_ID_MICROCHIP		0x04d8
#define USB_DEVICE_ID_MCP2210		0x00de

/* Product page: http://www.microchip.com/wwwproducts/en/MCP2210
 * Datasheet 2011-12-14: http://ww1.microchip.com/downloads/en/DeviceDoc/22288A.pdf
 */
#define MCP2210_BUFFER_SIZE	64
#define MCP2210_MAX_SPEED	(12 * 1000 * 1000)
#define MCP2210_MIN_SPEED	1500
#define MCP2210_EEPROM_SIZE	256
#define MCP2210_NUM_PINS	9

/* Command codes */
#define MCP2210_CMD_GET_STATUS		0x10 /* Section 3.6.1 */
#define MCP2210_CMD_SPI_CANCEL		0x11 /* Section 3.5.2 */
#define MCP2210_CMD_GET_INTERRUPTS	0x12 /* Section 3.4.1 */
#define MCP2210_CMD_GET_GPIO_CONFIG	0x20 /* Section 3.2.3 */
#define MCP2210_CMD_SET_GPIO_CONFIG	0x21 /* Section 3.2.4 */
#define MCP2210_CMD_SET_PIN_VALUE	0x30 /* Section 3.2.8 */
#define MCP2210_CMD_GET_PIN_VALUE	0x31 /* Section 3.2.7 */
#define MCP2210_CMD_SET_PIN_DIR		0x32 /* Section 3.2.6 */
#define MCP2210_CMD_GET_PIN_DIR		0x33 /* Section 3.2.5 */
#define MCP2210_CMD_SET_SPI_CONFIG	0x40 /* Section 3.2.2 */
#define MCP2210_CMD_GET_SPI_CONFIG	0x41 /* Section 3.2.1 */
#define MCP2210_CMD_SPI_TRANSFER	0x42 /* Section 3.5.1 */
#define MCP2210_CMD_READ_EEPROM		0x50 /* Section 3.3.1 */
#define MCP2210_CMD_WRITE_EEPROM	0x51 /* Section 3.3.2 */
#define MCP2210_CMD_SET_NVRAM		0x60 /* Section 3.1.1 - 3.1.5 */
#define MCP2210_CMD_GET_NVRAM		0x61 /* Section 3.1.6 - 3.1.10 */
#define MCP2210_CMD_SEND_PASSWORD	0x70 /* Section 3.1.11 */
#define MCP2210_CMD_SPI_RELEASE		0x80 /* Section 3.5.3 */

/* Subcommand codes */
#define MCP2210_NVRAM_SPI		0x10
#define MCP2210_NVRAM_GPIO		0x20
#define MCP2210_NVRAM_USB		0x30
#define MCP2210_NVRAM_PROD_NAME		0x40
#define MCP2210_NVRAM_MF_NAME		0x50

/* Status codes */
#define MCP2210_STATUS_SUCCESS		0x00
#define MCP2210_STATUS_SPI_NOT_OWNED	0xF7
#define MCP2210_STATUS_BUSY		0xF8
#define MCP2210_STATUS_UNKNOWN_CMD	0xF9
#define MCP2210_STATUS_WRITE_FAIL	0xFA
#define MCP2210_STATUS_BLOCKED_ACCESS	0xFB
#define MCP2210_STATUS_PERM_LOCKED	0xFC
#define MCP2210_STATUS_BAD_PASSWORD	0xFD

/* Multi function pin modes */
#define	MCP2210_PIN_GPIO	0
#define MCP2210_PIN_CS		1
#define MCP2210_PIN_DEDICATED	2

/* GPIO direction */
#define MCP2210_GPIO_NO_CHANGE -1
#define MCP2210_GPIO_OUTPUT	0
#define MCP2210_GPIO_INPUT	1

/**
 * enum mcp2210_other_settings
 *
 * Represents byte 17 of chip settings message (section 3.1.1, 3.1.2, etc)
 *
 * These values are generally ORed together except that only one
 * MCP2210_INTERRUPT_* value may be chosen (they do not OR together). To disable
 * any of these options, exclude them (zero is "disabled" for all options).
 * See table 3-1 in the datasheet for more information.
 */
#define MCP2210_SPI_BUS_RELEASE_DISABLED	0x01
#define MCP2210_INTERRUPT_HIGH_PULSE		0x08
#define MCP2210_INTERRUPT_LOW_PULSE		0x06
#define MCP2210_INTERRUPT_RISING_EDGE		0x04
#define MCP2210_INTERRUPT_FALLING_EDGE		0x02
#define MCP2210_REMOTE_WAKEUP_ENABLED		0x80

#define MCP2210_EEPROM_UNREAD		0
#define MCP2210_EEPROM_READ_PENDING	1
#define MCP2210_EEPROM_READ		2
#define MCP2210_EEPROM_DIRTY		3

#define EP_OUT	0
#define EP_IN	1

enum mcp2210_urb_cmd_state {
	MCP2210_STATE_NEW,
	MCP2210_STATE_SUBMITTED,
	MCP2210_STATE_COMPLETE,
	MCP2210_STATE_DEAD
};

struct mcp2210_device {
	struct device *dev;
	struct spi_master *master;
	struct usb_device *usbdev;
	u8 requeust_buffer[MCP2210_BUFFER_SIZE];
	void *spi_data;
};

struct mcp2210_spi {
	struct mcp2210_device *dev;
};

struct mcp2210_spi_message {
	struct spi_device *spi;
	struct spi_message *msg;
	struct spi_transfer *current_transfer;
	unsigned int tx_pos;
	unsigned int rx_pos;
	unsigned int settings_set;
	unsigned int tx_in_process;
	unsigned int tx_bytes_in_process;
	unsigned int kill;
	struct list_head *next;
};

/***** USB handling *****/
#define ATUSB_REQ_FROM_DEV      (USB_TYPE_VENDOR | USB_DIR_IN)
#define ATUSB_REQ_TO_DEV        (USB_TYPE_VENDOR | USB_DIR_OUT)

static int atusb_control_msg(struct mcp2210_device *atusb, unsigned int pipe,
				__u8 request, __u8 requesttype,
				__u16 value, __u16 index,
				void *data, __u16 size, int timeout)
{
	struct usb_device *usbdev = atusb->usbdev;
	int ret;

	ret = usb_control_msg(usbdev, pipe, request, requesttype,
				value, index, data, size, timeout);
	if (ret < 0) {
		dev_err(&usbdev->dev,
		"%s: req 0x%02x val 0x%x idx 0x%x, error %d\n",
		__func__, request, value, index, ret);
	}
	return ret;
}

static int atusb_command(struct mcp2210_device *atusb, u8 cmd, u8 arg)
{
	struct usb_device *usbdev = atusb->usbdev;

	dev_dbg(&usbdev->dev, "%s: cmd = 0x%x\n", __func__, cmd);
	return atusb_control_msg(atusb, usb_sndctrlpipe(usbdev, 0),
				cmd, ATUSB_REQ_TO_DEV, arg, 0, NULL, 0, 1000);
}

static int atusb_write_reg(struct mcp2210_device *atusb, u8 reg, u8 value)
{
	struct usb_device *usbdev = atusb->usbdev;

	dev_dbg(&usbdev->dev, "%s: 0x%02x <- 0x%02x\n", __func__, reg, value);
	return atusb_control_msg(atusb, usb_sndctrlpipe(usbdev, 0),
				0x20, ATUSB_REQ_TO_DEV,
				value, reg, NULL, 0, 1000);
}

static int atusb_read_reg(struct mcp2210_device *atusb, u8 reg)
{
	struct usb_device *usbdev = atusb->usbdev;
	int ret;
	u8 *buffer;
	u8 value;

	buffer = kmalloc(1, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	dev_dbg(&usbdev->dev, "%s: reg = 0x%x\n", __func__, reg);
	ret = atusb_control_msg(atusb, usb_rcvctrlpipe(usbdev, 0),
				0x21, ATUSB_REQ_FROM_DEV,
				0, reg, buffer, 1, 1000);

	if (ret >= 0) {
		value = buffer[0];
		kfree(buffer);
		return value;
	} else {
		kfree(buffer);
		return ret;
	}
}

/***** SPI master handling *****/

static int mcp2210_spi_setup(struct spi_device *spi)
{
	struct mcp2210_spi *ms;
	ms = spi_master_get_devdata(spi->master);
	
	return 0;
}

static int mcp2210_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct mcp2210_spi *ms;
	struct mcp2210_spi_message *mcp_msg;
	ms = spi_master_get_devdata(spi->master);
	
	mcp_msg = kzalloc(sizeof(struct mcp2210_spi_message), GFP_ATOMIC);
	if(!mcp_msg)
		return -ENOMEM;
	
	mcp_msg->spi = spi;
	mcp_msg->msg = msg;
	mcp_msg->next = msg->transfers.next;
	msg->status = 0;
	
	return 0;
}

static void mcp2210_spi_cleanup(struct spi_device *spi)
{
	struct mcp2210_spi *ms;
	ms = spi_master_get_devdata(spi->master);
}


/***** Module setup, probe and disconnect *****/

static int mcp2210_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct mcp2210_device *dev;
	struct mcp2210_spi *ms;
	struct spi_master *master;
	int ret;
	static struct spi_board_info board_info = {
		.modalias = "mrf24j40",
		.bus_num = 0,
		.chip_select = 1,
		.max_speed_hz =  8000000, /* 8MHz */
		.mode = SPI_MODE_3,
	};
	
	printk("%s\n", __func__);				
	
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &intf->dev;
	usb_set_intfdata(intf, dev);

	master = spi_alloc_master(dev->dev, 0);	
	if (!master)
		goto err_spi;


	/* TODO
	 * 1) Setup GP0 or GP1 as chip select pins
	 * 2) Setup GP6 as external interupt pin, if needed
	 * 3) How to configure power up settings (PID/VID, string descriptors,
	 *           chip settings and SPI transfer paramters-> NVRAM storage)
	 *    -> Userspace libusb based config utility
	 * 4) Setup GP2 for USB suspend and resume
	 * 5) Go with USB bus powered only mode for now
	 */


	/* Default SPI config after reset
	 * 1 Mbit
	 * 4 bytes transfer per SPI transaction
	 * GP1 as chip select line
	 */

	dev->master = master;	
	master->bus_num = -1;
	master->num_chipselect = 4;
	master->setup = mcp2210_spi_setup;
	master->transfer = mcp2210_spi_transfer;
	master->cleanup = mcp2210_spi_cleanup;
	master->mode_bits = SPI_CPOL | SPI_CPHA;

	spi_master_set_devdata(master, dev);
	//ms = spi_master_get_devdata(master);
	//ms->dev = dev;	
	//ms->master = master;
	//dev->spi_data = ms;
	
	ret = spi_register_master(master);
	if (ret)
		goto err_power;
	
	board_info.bus_num = master->bus_num;
	printk("mcp2210 spi master registered bus number %d\n", board_info.bus_num);
	
	spi_new_device(master, &board_info);
	
	return 0;

err_power:
	spi_master_put(master);
err_spi:
	kfree(dev);
	return -ENOMEM;
}

static void mcp2210_disconnect(struct usb_interface *intf)
{
	struct mcp2210_device *dev = usb_get_intfdata(intf);
	
	printk("%s\n", __func__);
	
	spi_unregister_master(dev->master);
	spi_master_put(dev->master);

	usb_set_intfdata(intf, NULL);
	kfree(dev);
}

static const struct usb_device_id mcp2210_devices[] = {
	{USB_DEVICE(USB_VENDOR_ID_MICROCHIP, USB_DEVICE_ID_MCP2210)},
	{}
};

MODULE_DEVICE_TABLE(usb, mcp2210_devices);

static struct usb_driver mcp2210_driver = {
	.name = "mcp2210",
	.probe = mcp2210_probe,
	.disconnect = mcp2210_disconnect,
	.id_table = mcp2210_devices,
};

module_usb_driver(mcp2210_driver);
MODULE_AUTHOR("Stefan Schmidt <stefan@datenfreihafen.org>");
MODULE_DESCRIPTION("Microchip MCP2210 USB-to-SPI bridge");
MODULE_LICENSE("GPLv2");

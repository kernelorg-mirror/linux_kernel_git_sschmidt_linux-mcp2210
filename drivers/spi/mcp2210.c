/*
 *  MCP 2210 driver for linux
 *
 *  Copyright (c) 2013 Mathew King <mking@trilithic.com> for Trilithic, Inc
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

#define MCP2210_BUFFER_SIZE		64
#define MCP2210_MAX_SPEED	(12 * 1000 * 1000)

struct mcp2210_device {
	struct device *dev;
	struct spi_master *master;
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

static int mcp2210_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
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
MODULE_LICENSE("GPL");

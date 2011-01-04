/*
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * da9052-spi.c: SPI SSC (Synchronous Serial Communication) driver for DA9052
 */

#include <linux/device.h>
#include <linux/mfd/core.h>
#include <linux/spi/spi.h>
#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>


struct da9052 *da9052_spi;

#define SPI_CONNECTED 0

static int da9052_spi_is_connected(void)
{

        struct da9052_ssc_msg msg;
       
	//printk("Entered da9052_spi_is_connected.............\n");

        msg.addr = DA9052_INTERFACE_REG;

        /* Test spi connectivity by performing read of the GPIO_0-1 register and then verify the read value*/
       if ( 0 != da9052_spi_read(da9052_spi, &msg)) {
                printk("da9052_spi_is_connected - spi read failed.............\n");
                return -1;
        }
        else if( 0x88 != msg.data ){
                printk("da9052_spi_is_connected - spi read failed. Msg data =%x ..............\n",msg.data);
                        return -1;
        }

	return 0;

}

static int da9052_spi_probe(struct spi_device *spi)
{
	//printk("\n\tEntered da9052_spi_probe.....\n");
	
	da9052_spi = kzalloc(sizeof(struct da9052), GFP_KERNEL);
	
	if (!da9052_spi)
		return -ENOMEM;
	
	
	spi->mode = SPI_MODE_0 | SPI_CPOL;
	spi->bits_per_word = 8;
	spi_setup(spi);
	
	da9052_spi->dev = &spi->dev;

	da9052_spi->spi_dev = spi;
	
	/*
	 * Allocate memory for RX/TX bufferes used in single register read/write
	 */
	da9052_spi->spi_rx_buf = kmalloc(2, GFP_KERNEL | GFP_DMA);
	if (!da9052_spi->spi_rx_buf)
		return -ENOMEM;
	
	da9052_spi->spi_tx_buf = kmalloc(2, GFP_KERNEL | GFP_DMA);
	if (!da9052_spi->spi_tx_buf)
		return -ENOMEM;

	da9052_spi->spi_active_page  = PAGECON_0;
	da9052_spi->rw_pol = 1;


	dev_set_drvdata(&spi->dev, da9052_spi);


	 /* Validate SPI connectivity */
        if ( SPI_CONNECTED  == da9052_spi_is_connected()) {
		/* SPI is connected */
		da9052_spi->connecting_device = SPI;
                if( 0 != da9052_ssc_init(da9052_spi) )
			return -ENODEV;
        }
        else {
                return -ENODEV;
        }

	//printk("Exiting da9052_spi_probe.....\n");
	
	return 0;
}

static int da9052_spi_remove(struct spi_device *spi)
{
	struct da9052 *da9052 = dev_get_drvdata(&spi->dev);

	printk("Entered da9052_spi_remove()\n");
	if(SPI == da9052->connecting_device ) {
		da9052_ssc_exit(da9052);
	}
	mfd_remove_devices(&spi->dev);
	kfree(da9052->spi_rx_buf);
	kfree(da9052->spi_tx_buf);
	kfree(da9052);
	return 0;
}

static struct spi_driver da9052_spi_driver = {
	.driver = {
		.name	 = DA9052_SSC_SPI_DEVICE_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= da9052_spi_probe,
	.remove		= __devexit_p(da9052_spi_remove),
};


static int da9052_spi_set_page(struct da9052 *da9052, unsigned char page)
{

	struct da9052_ssc_msg sscmsg;
	struct spi_message message;
	struct spi_transfer xfer;
	int ret = 0;

	printk("Entered da9052_spi_set_page.....\n");
	if ((page != PAGECON_0) && ((page != PAGECON_128)))
		return INVALID_PAGE;

	/* Current configuration is PAGE-0 and write request for PAGE-1 */
	/* set register address */
	sscmsg.addr = DA9052_PAGECON0_REG;
	/* set value */
	sscmsg.data = page;

	/* Check value of R/W_POL bit of INTERFACE register */
	if (!da9052->rw_pol) {
		/* We need to set 0th bit for write operation */
		sscmsg.addr = ((sscmsg.addr << 1) | RW_POL);
	} else {
		/* We need to reset 0th bit for write operation */
		sscmsg.addr = (sscmsg.addr << 1);
	}

	/* SMDK-6410 host SPI driver specific stuff */

	/* Build our spi message */
	printk("da9052_spi_set_page - Calling spi_message_init.....\n");
	spi_message_init(&message);
	memset(&xfer, 0, sizeof(xfer));

	xfer.len = 2;
	xfer.tx_buf = da9052->spi_tx_buf;
	xfer.rx_buf = da9052->spi_rx_buf;

	da9052->spi_tx_buf[0] = sscmsg.addr;
	da9052->spi_tx_buf[1] = sscmsg.data;

	printk("da9052_spi_set_page - Calling spi_message_add_tail.....\n");
	spi_message_add_tail(&xfer, &message);

	/* Now, do the i/o */
	printk("da9052_spi_set_page - Calling spi_sync.....\n");
	ret = spi_sync(da9052->spi_dev, &message);

	if (ret == 0) {
		/* Active Page set successfully */
		da9052->spi_active_page = page;
		return 0;
	} else {
		/* Error in setting Active Page */
		return ret;
	}

	return 0;
}

int da9052_spi_write(struct da9052 *da9052, struct da9052_ssc_msg *msg)
{

	struct spi_message message;
	struct spi_transfer xfer;
	int ret;

	/*
	 * We need a seperate copy of da9052_ssc_msg so that caller's
	 * copy remains intact
	*/
	struct da9052_ssc_msg sscmsg;

	/* Copy callers data in to our local copy */
	sscmsg.addr = msg->addr;
	sscmsg.data = msg->data;

	if ((sscmsg.addr > PAGE_0_END) &&
		(da9052->spi_active_page == PAGECON_0)) {
		/*
		* Current configuration is PAGE-0 and write request
		* for PAGE-1
		*/
		da9052_spi_set_page(da9052, PAGECON_128);
		/* Set register address accordindly */
		sscmsg.addr = (sscmsg.addr - PAGE_1_START);
	} else if ((sscmsg.addr < PAGE_1_START) &&
		(da9052->spi_active_page == PAGECON_128)) {
		/*
		* Current configuration is PAGE-1 and write request
		* for PAGE-0
		*/
		da9052_spi_set_page(da9052, PAGECON_0);
	} else if (sscmsg.addr > PAGE_0_END) {
		/*
		* Current configuration is PAGE-1 and write request
		* for PAGE-1. Just need to adjust register address
		*/
		sscmsg.addr = (sscmsg.addr - PAGE_1_START);
	}

	/* Check value of R/W_POL bit of INTERFACE register */
	if (!da9052->rw_pol) {
		/* We need to set 0th bit for write operation */
		sscmsg.addr = ((sscmsg.addr << 1) | RW_POL);
	} else {
		/* We need to reset 0th bit for write operation */
		sscmsg.addr = (sscmsg.addr << 1);
	}

	/* SMDK-6410 host SPI driver specific stuff */

	/* Build our spi message */
	spi_message_init(&message);
	memset(&xfer, 0, sizeof(xfer));

	xfer.len = 2;
	xfer.tx_buf = da9052->spi_tx_buf;
	xfer.rx_buf = da9052->spi_rx_buf;

	da9052->spi_tx_buf[0] = sscmsg.addr;
	da9052->spi_tx_buf[1] = sscmsg.data;

	spi_message_add_tail(&xfer, &message);

	/* Now, do the i/o */
	ret = spi_sync(da9052->spi_dev, &message);

	return ret;
}

int da9052_spi_write_many(struct da9052 *da9052, struct da9052_ssc_msg *sscmsg,
                                int msg_no)
{
	int cnt,ret=0;

	for(cnt = 0; cnt < msg_no; cnt++, sscmsg++) {
		ret = da9052_ssc_write(da9052,sscmsg);
		if(ret != 0)
		{
			printk("Error in %s", __FUNCTION__);
			return -EIO;
		}
	}

	return 0;
}

int da9052_spi_read(struct da9052 *da9052, struct da9052_ssc_msg *msg)
{

	struct spi_message message;
	struct spi_transfer xfer;
	int ret;

	/*
	* We need a seperate copy of da9052_ssc_msg so that
	* caller's copy remains intact
	*/
	struct da9052_ssc_msg sscmsg;
	

	/* Copy callers data in to our local copy */
	sscmsg.addr = msg->addr;
	sscmsg.data = msg->data;

	if ((sscmsg.addr > PAGE_0_END) &&
		(da9052->spi_active_page == PAGECON_0)) {
		/*
		* Current configuration is PAGE-0 and
		* read request for PAGE-1
		*/
		printk("da9052_spi_read - if PAGECON_128.....\n");
		da9052_spi_set_page(da9052, PAGECON_128);
		/* Set register address accordindly */
		sscmsg.addr = (sscmsg.addr - PAGE_1_START);
	} else if ((sscmsg.addr < PAGE_1_START) &&
		(da9052->spi_active_page == PAGECON_128)) {
		/*
		* Current configuration is PAGE-1 and
		* write request for PAGE-0
		*/
		printk("da9052_spi_read - if PAGECON_0.....\n");
		da9052_spi_set_page(da9052, PAGECON_0);
	} else if (sscmsg.addr > PAGE_0_END) {
		/*
		* Current configuration is PAGE-1 and write
		* request for PAGE-1
		* Just need to adjust register address
		*/
		sscmsg.addr = (sscmsg.addr - PAGE_1_START);
	}

	/* Check value of R/W_POL bit of INTERFACE register */
	if (da9052->rw_pol) {
		/* We need to set 0th bit for read operation */
		sscmsg.addr = ((sscmsg.addr << 1) | RW_POL);
	} else {
		/* We need to reset 0th bit for write operation */
		sscmsg.addr = (sscmsg.addr << 1);
	}

	/* SMDK-6410 host SPI driver specific stuff */

	/* Build our spi message */
	spi_message_init(&message);
	memset(&xfer, 0, sizeof(xfer));

	xfer.len = 2;
	xfer.tx_buf = da9052->spi_tx_buf;
	xfer.rx_buf = da9052->spi_rx_buf;

	da9052->spi_tx_buf[0] = sscmsg.addr;
	da9052->spi_tx_buf[1] = 0xff;

	da9052->spi_rx_buf[0] = 0;
	da9052->spi_rx_buf[1] = 0;

	spi_message_add_tail(&xfer, &message);

	/* Now, do the i/o */
	ret = spi_sync(da9052->spi_dev, &message);

	if (ret == 0) {
		/* Update read value in callers copy */
		msg->data = da9052->spi_rx_buf[1];
		return 0;
	} else {
		return ret;
	}


	return 0;
}

int da9052_spi_read_many(struct da9052 *da9052, struct da9052_ssc_msg *sscmsg,
                                int msg_no)
{
	int cnt,ret=0;

	for(cnt = 0; cnt < msg_no; cnt++, sscmsg++) {
		ret = da9052_ssc_read(da9052,sscmsg);
		if(ret != 0)
		{
			printk("Error in %s", __FUNCTION__);
			return -EIO;
		}
	}

	return 0;
}

static int __init da9052_spi_init(void)
{
	int ret = 0;
	//printk("Entered da9052_spi_init.....\n");
	ret = spi_register_driver(&da9052_spi_driver);
	if (ret != 0) {
		printk(KERN_ERR "Unable to register %s\n", DA9052_SSC_SPI_DEVICE_NAME);
		return ret;
	}
        return 0;
}
module_init(da9052_spi_init);

static void __exit da9052_spi_exit(void)
{
        spi_unregister_driver(&da9052_spi_driver);
}

module_exit(da9052_spi_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <dchen@diasemi.com>");
MODULE_DESCRIPTION("SPI driver for Dialog DA9052 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9052_SSC_SPI_DEVICE_NAME);

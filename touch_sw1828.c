/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * touch_sw1828.c - SiW touch driver glue for SW1828
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/firmware.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"

//#define CHIP_SW1828_SPI

#define CHIP_ID						"1828"
#define CHIP_DEVICE_NAME			"SW1828"
#define CHIP_COMPATIBLE_NAME		"siw,sw1828"
#define CHIP_DEVICE_DESC			"SiW Touch SW1828 Driver"

#define CHIP_TYPE					CHIP_SW1828

#define CHIP_MODE_ALLOWED			(0 |	\
									LCD_MODE_BIT_U3 |	\
									LCD_MODE_BIT_STOP |	\
									0)

#define CHIP_FW_SIZE				(64<<10)

#define CHIP_FLAGS					(0 |	\
									TOUCH_USE_MON_THREAD |	\
									0)

#define CHIP_IRQFLAGS				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT)

#if defined(CHIP_SW1828_SPI)
#define CHIP_INPUT_ID_BUSTYPE		BUS_SPI
#else
#define CHIP_INPUT_ID_BUSTYPE		BUS_I2C
#endif
#define CHIP_INPUT_ID_VENDOR		0xABCD
#define CHIP_INPUT_ID_PRODUCT		0x9876
#define CHIP_INPUT_ID_VERSION		0x1234

#define CHIP_QUIRKS					0

#if defined(CHIP_SW1828_SPI)
#define CHIP_BUS_TYPE				BUS_IF_SPI
#define	CHIP_BUF_SIZE				0
#define CHIP_SPI_MODE				SPI_MODE_0
#define CHIP_BPW					8
#define CHIP_MAX_FREQ				(5 * 1000* 1000)
#define CHIP_TX_HDR_SZ				SPI_BUS_TX_HDR_SZ
#define CHIP_RX_HDR_SZ				SPI_BUS_RX_HDR_SZ
#define CHIP_TX_DUMMY_SZ			SPI_BUS_TX_DUMMY_SZ
#define CHIP_RX_DUMMY_SZ			SPI_BUS_RX_DUMMY_SZ
#else	/* CHIP_SW1828_SPI */
#define CHIP_BUS_TYPE				BUS_IF_I2C
#define	CHIP_BUF_SIZE				0
#define CHIP_SPI_MODE				-1
#define CHIP_BPW					-1
#define CHIP_MAX_FREQ				-1
#define CHIP_TX_HDR_SZ				I2C_BUS_TX_HDR_SZ
#define CHIP_RX_HDR_SZ				I2C_BUS_RX_HDR_SZ
#define CHIP_TX_DUMMY_SZ			I2C_BUS_TX_DUMMY_SZ
#define CHIP_RX_DUMMY_SZ			I2C_BUS_RX_DUMMY_SZ
#endif	/* CHIP_SW1828_SPI */

static const struct siw_hal_reg_quirk chip_reg_quirks[] = {
	{ .old_addr = TC_CONFDN_BASE_ADDR, .new_addr = 0x284 },
	{ .old_addr = ~0, .new_addr = ~0 },		// End signal
};

#if defined(__SIW_CONFIG_OF)
/*
 * of_device_is_compatible(dev->of_node, CHIP_COMPATIBLE_NAME)
 */
static const struct of_device_id chip_match_ids[] = {
	{ .compatible = CHIP_COMPATIBLE_NAME },
	{ },
};
MODULE_DEVICE_TABLE(of, chip_match_ids);
#else
enum CHIP_CAPABILITY {
	CHIP_MAX_X			= 800,
	CHIP_MAX_Y			= 480,
	CHIP_MAX_PRESSURE	= 255,
	CHIP_MAX_WIDTH		= 15,
	CHIP_MAX_ORI		= 1,
	CHIP_MAX_ID			= 10,
	/* */
	CHIP_HW_RST_DELAY	= 210,
	CHIP_SW_RST_DELAY	= 90,
};

#define CHIP_PIN_RESET			0
#define CHIP_PIN_IRQ			0
#define CHIP_PIN_VDD			-1
#define CHIP_PIN_VIO			-1

#if (CHIP_PIN_RESET == 0) || (CHIP_PIN_IRQ == 0)
//	#error Assign external pin & flag first!!!
#endif
#endif	/* __SIW_CONFIG_OF */

#define __SW1828_FLASH_WP
#define __SW1828_SENSE_LEVEL_CONTROL

#if defined(__SW1828_FLASH_WP)
#define SPR_CLK_CTL					(0x003)
#define SPR_SPIFLASH_CTRL			(0x05C)
#define SPR_SPIFLASH_CTRL_CS_START	BIT(0)
#define SPR_SPIFLASH_CTRL_CS_STOP	BIT(1)
#define SPR_SPIFLASH_CTRL_WR		BIT(2)
#define SPR_SPIFLASH_CTRL_RD		BIT(3)
#define SPR_SPIFLASH_WDATA0			(0x05E)
#define SPR_SPIFLASH_RDATA0			(0x062)

static int sw1828_flash_wp(struct device *dev, int wp)
{
	struct __flash_wp_ctrl {
		u32 cmd;
		u32 ctrl;
	} flash_wp_ctrl[3] = {
		{ .cmd = 0x00AB, .ctrl = SPR_SPIFLASH_CTRL_WR },
		{ .cmd = 0x0006, .ctrl = SPR_SPIFLASH_CTRL_WR },
		{ .cmd = (wp) ? 0x0C01 : 0x0001, .ctrl = (1<<4) | SPR_SPIFLASH_CTRL_WR },
	};
	u32 flash_sts_ctrl = 0x0005;
	u32 status = 0;
	int retry_cnt = 5;
	int retry_delay = 10;
	int i;

	siw_hal_write_value(dev, SPR_CLK_CTL, 0x03E7);

	for (i = 0; i < ARRAY_SIZE(flash_wp_ctrl); i++) {
		siw_hal_write_value(dev, SPR_SPIFLASH_CTRL, SPR_SPIFLASH_CTRL_CS_START);
		siw_hal_write_value(dev, SPR_SPIFLASH_WDATA0, flash_wp_ctrl[i].cmd);
		siw_hal_write_value(dev, SPR_SPIFLASH_CTRL, flash_wp_ctrl[i].ctrl);
		siw_hal_write_value(dev, SPR_SPIFLASH_CTRL, SPR_SPIFLASH_CTRL_CS_STOP);
	}

	/* busy check */
	while (retry_cnt--) {
		siw_hal_write_value(dev, SPR_SPIFLASH_CTRL, SPR_SPIFLASH_CTRL_CS_START);
		siw_hal_write_value(dev, SPR_SPIFLASH_WDATA0, flash_sts_ctrl);
		siw_hal_write_value(dev, SPR_SPIFLASH_CTRL, SPR_SPIFLASH_CTRL_WR);

		siw_hal_write_value(dev, SPR_SPIFLASH_CTRL, SPR_SPIFLASH_CTRL_RD);
		siw_hal_read_value(dev, SPR_SPIFLASH_RDATA0, &status);
		siw_hal_write_value(dev, SPR_SPIFLASH_CTRL, SPR_SPIFLASH_CTRL_CS_STOP);

		if (!(status & 0x01)) {
			break;
		}

		/* busy */
		touch_msleep(retry_delay);
	}

	return 0;
}
#define CHIP_FLASH_WP	sw1828_flash_wp
#else	/* __SW1828_FLASH_WP */
#define CHIP_FLASH_WP	NULL
#endif	/* __SW1828_FLASH_WP */

#if defined(__SW1828_SENSE_LEVEL_CONTROL)
/*
 * Sensitivity level control
 *
 * [read]
 * # cat sys_con
 * 1 ~ 3	: Valid level
 * 0		: Not supported
 *
 * [write]
 * # echo 0x1 > sys_con		//set level 1
 * # echo 0x81 > sys_con	//set level 1 then save
 *
 */
#include "siw_touch_hal.h"

#define LEVEL_MIN		1
#define LEVEL_MAX		3
#define LEVEL_ERR		0

#define LEVEL_ADDR		0x270
#define LEVEL_WR_CMD	((0xC657<<16) | (1<<2))
#define LEVEL_WR_STS	0xD8
#define LEVEL_WR_DLY	2
#define LEVEL_WR_CNT	50

static int sw1828_read_level(struct device *dev, int *level)
{
	int __level;
	int ret = 0;

	ret = siw_hal_read_value(dev, LEVEL_ADDR, &__level);
	if (ret < 0) {
		t_dev_err(dev, "failed to get level\n");
		goto out;
	}

	if ((__level < LEVEL_MIN) || (__level > LEVEL_MAX)) {
		t_dev_err(dev,
			"The current FW doesn't support this, %d\n", __level);
		ret = -ESRCH;
		goto out;
	}

	if (level != NULL)
		*level = __level;

out:
	return ret;
}

static int sw1828_show_level(struct device *dev)
{
	int level = 0;
	int ret = 0;

	ret = sw1828_read_level(dev, &level);
	if (ret < 0) {
		level = LEVEL_ERR;
		goto out;
	}

	t_dev_info(dev, "current level is %d\n", level);

out:
	return level;
}

static int sw1828_save_level(struct device *dev, int level)
{
	int save = 0;
	int ret = 0;

	ret = sw1828_read_level(dev, NULL);
	if (ret < 0) {
		return ret;
	}

	save = level & 0x80;

	level &= 0x7F;
	if ((level < LEVEL_MIN) || (level > LEVEL_MAX)) {
		t_dev_err(dev, "invalid level, %d\n", level);
		return ret;
	}

	ret = siw_hal_write_value(dev, LEVEL_ADDR, level);
	if (ret < 0) {
		t_dev_err(dev,
			"failed to set level(%d), %d\n",
			level, ret);
		return ret;
	}
	t_dev_info(dev, "new level is %d\n", level);

	if (save) {
		struct siw_touch_chip *chip = to_touch_chip(dev);
		struct siw_hal_reg *reg = chip->reg;
		u32 wr_cmd, chk_resp = 0;
		int wr_failed = 1;
		int retry = LEVEL_WR_CNT;

		t_dev_info(dev, "saving level(%d) begins\n", level);

		wr_cmd = LEVEL_WR_CMD;
		ret = siw_hal_write_value(dev, reg->tc_flash_dn_ctl, wr_cmd);
		if (ret < 0) {
			t_dev_err(dev, "failed to set wr_cmd(%Xh), %d\n",
				wr_cmd, ret);
			return ret;
		}

		do {
			touch_msleep(LEVEL_WR_DLY);

			ret = siw_hal_read_value(dev, reg->tc_flash_dn_status, &chk_resp);
			if (ret >= 0) {
				if (chk_resp == LEVEL_WR_STS) {
					t_dev_info(dev, "saving level(%d) done\n", level);
					wr_failed = 0;
					break;
				}
			}
		} while (--retry);

		if (wr_failed) {
			t_dev_err(dev,
				"save fail: addr[%04Xh] data[%08Xh], expect[%08Xh]\n",
				reg->tc_flash_dn_status, chk_resp, LEVEL_WR_STS);
		}
	}

	return ret;
}

static ssize_t _sw1828_show_sys_con(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int size = 0;

	mutex_lock(&ts->lock);
	size = siw_snprintf(buf, 0, "%d\n", sw1828_show_level(dev));
	mutex_unlock(&ts->lock);

	return (ssize_t)size;
}

static ssize_t _sw1828_store_sys_con(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	int level;

	if (sscanf(buf, "%X", &level) <= 0) {
		t_dev_err(dev, "Invalid param\n");
		return count;
	}

	mutex_lock(&ts->lock);
	sw1828_save_level(dev, level);
	mutex_unlock(&ts->lock);

	return count;
}

#define SIW_TOUCH_ATTR(_name, _show, _store)	\
		TOUCH_ATTR(_name, _show, _store)

#define _SIW_TOUCH_ATTR_T(_name)	\
		touch_attr_##_name

static SIW_TOUCH_ATTR(sys_con, _sw1828_show_sys_con, _sw1828_store_sys_con);

static struct attribute *sw1828_sysfs_list[] = {
	&_SIW_TOUCH_ATTR_T(sys_con).attr,
	NULL,
};

static const struct attribute_group sw1828_sysfs_group = {
	.attrs = sw1828_sysfs_list,
};

#define	CHIP_SYSFS_GROUP	(void *)&sw1828_sysfs_group
#else	/* __SW1828_SENSE_LEVEL_CONTROL */
#define CHIP_SYSFS_GROUP	NULL
#endif	/* __SW1828_SENSE_LEVEL_CONTROL */

/* use eg. copmname=arc1 to change name */
static char compatible_name[32] = CHIP_COMPATIBLE_NAME;
module_param_string(compname, compatible_name, sizeof(compatible_name), S_IRUGO);

/* use eg. cname=arc1 to change name */
static char chip_name[32] = CHIP_DEVICE_NAME;
module_param_string(cname, chip_name, sizeof(chip_name), S_IRUGO);

/* use eg. dname=arc1 to change name */
static char chip_drv_name[32] = SIW_TOUCH_NAME;
module_param_string(dname, chip_drv_name, sizeof(chip_drv_name), S_IRUGO);

/* use eg. iname=arc1 to change input name */
static char chip_idrv_name[32] = SIW_TOUCH_INPUT;
module_param_string(iname, chip_idrv_name, sizeof(chip_idrv_name), S_IRUGO);

static const struct siw_touch_pdata chip_pdata = {
	/* Configuration */
	.compatible			= compatible_name,
	.chip_id			= CHIP_ID,
	.chip_name			= chip_name,
	.drv_name			= chip_drv_name,
	.idrv_name			= chip_idrv_name,
	.owner				= THIS_MODULE,
	.chip_type			= CHIP_TYPE,
	.mode_allowed		= CHIP_MODE_ALLOWED,
	.fw_size			= CHIP_FW_SIZE,
	.flags				= CHIP_FLAGS,	/* Caution : MSB(bit31) unavailable */
	.irqflags			= CHIP_IRQFLAGS,
	.quirks				= CHIP_QUIRKS,
	/* */
	.bus_info			= {
		.bus_type			= CHIP_BUS_TYPE,
		.buf_size			= CHIP_BUF_SIZE,
		.spi_mode			= CHIP_SPI_MODE,
		.bits_per_word		= CHIP_BPW,
		.max_freq			= CHIP_MAX_FREQ,
		.bus_tx_hdr_size	= CHIP_TX_HDR_SZ,
		.bus_rx_hdr_size	= CHIP_RX_HDR_SZ,
		.bus_tx_dummy_size	= CHIP_TX_DUMMY_SZ,
		.bus_rx_dummy_size	= CHIP_RX_DUMMY_SZ,
	},
#if defined(__SIW_CONFIG_OF)
	.of_match_table 	= of_match_ptr(chip_match_ids),
#else
	.pins				= {
		.reset_pin		= CHIP_PIN_RESET,
		.reset_pin_pol	= OF_GPIO_ACTIVE_LOW,
		.irq_pin		= CHIP_PIN_IRQ,
		.vdd_pin		= CHIP_PIN_VDD,
		.vio_pin		= CHIP_PIN_VIO,
	},
	.caps				= {
		.max_x			= CHIP_MAX_X,
		.max_y			= CHIP_MAX_Y,
		.max_pressure	= CHIP_MAX_PRESSURE,
		.max_width		= CHIP_MAX_WIDTH,
		.max_orientation = CHIP_MAX_ORI,
		.max_id			= CHIP_MAX_ID,
		.hw_reset_delay	= CHIP_HW_RST_DELAY,
		.sw_reset_delay	= CHIP_SW_RST_DELAY,
	},
#endif
	/* Input Device ID */
	.i_id				= {
		.bustype		= CHIP_INPUT_ID_BUSTYPE,
		.vendor 		= CHIP_INPUT_ID_VENDOR,
		.product 		= CHIP_INPUT_ID_PRODUCT,
		.version 		= CHIP_INPUT_ID_VERSION,
	},
	/* */
	//See 'siw_hal_get_default_ops' [siw_touch_hal.c]
	.ops				= NULL,
	/* */
	//See 'siw_setup_operations' [siw_touch.c]
	.reg_quirks			= (void *)chip_reg_quirks,
	//function quirks
	.fquirks = {
		.flash_wp		= CHIP_FLASH_WP,
		.sysfs_group	= CHIP_SYSFS_GROUP,
	},
};

static struct siw_touch_chip_data chip_data = {
	.pdata = &chip_pdata,
	.bus_drv = NULL,
};

siw_chip_module_init(CHIP_DEVICE_NAME,
				chip_data,
				CHIP_DEVICE_DESC,
				"kimhh@siliconworks.co.kr");


__siw_setup_str("siw_chip_name=", siw_setup_chip_name, chip_name);
__siw_setup_str("siw_drv_name=", siw_setup_drv_name, chip_drv_name);
__siw_setup_str("siw_idrv_name=", siw_setup_idrv_name, chip_idrv_name);




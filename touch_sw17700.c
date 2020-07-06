/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * touch_sw17700.c - SiW touch driver glue for SW17700
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

//#define CHIP_SW17700_SPI

#define CHIP_ID						"1770"
#define CHIP_DEVICE_NAME			"SW17700"
#define CHIP_COMPATIBLE_NAME		"siw,sw17700"
#define CHIP_DEVICE_DESC			"SiW Touch SW17700 Driver"

#define CHIP_TYPE					CHIP_SW17700

#define CHIP_MODE_ALLOWED			(0 |	\
									LCD_MODE_BIT_U3 |	\
									LCD_MODE_BIT_STOP |	\
									0)

#define CHIP_FW_SIZE				(64<<10)

#define CHIP_FLAGS					(0 |	\
									0)

#define CHIP_IRQFLAGS				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT)

#if defined(CHIP_SW17700_SPI)
#define CHIP_INPUT_ID_BUSTYPE		BUS_SPI
#else
#define CHIP_INPUT_ID_BUSTYPE		BUS_I2C
#endif
#define CHIP_INPUT_ID_VENDOR		0xABCD
#define CHIP_INPUT_ID_PRODUCT		0x9876
#define CHIP_INPUT_ID_VERSION		0x1234

#define CHIP_QUIRKS					0

#if defined(CHIP_SW17700_SPI)
#define CHIP_BUS_TYPE				BUS_IF_SPI
#define	CHIP_BUF_SIZE				0
#define CHIP_SPI_MODE				SPI_MODE_0
#define CHIP_BPW					8
#define CHIP_MAX_FREQ				(1 * 1000* 1000)
#define CHIP_TX_HDR_SZ				SPI_BUS_TX_HDR_SZ
#define CHIP_TX_DUMMY_SZ			SPI_BUS_TX_DUMMY_SZ

#if (CHIP_MAX_FREQ >= (30 * 1000* 1000))
#define CHIP_RX_DUMMY_128BIT
#endif

#if defined(CHIP_RX_DUMMY_128BIT)
#define CHIP_RX_HDR_SZ				SPI_BUS_RX_HDR_SZ_128BIT
#define CHIP_RX_DUMMY_SZ			SPI_BUS_RX_DUMMY_SZ_128BIT
#else
#define CHIP_RX_HDR_SZ				SPI_BUS_RX_HDR_SZ_32BIT
#define CHIP_RX_DUMMY_SZ			SPI_BUS_RX_DUMMY_SZ_32BIT
#endif
#else	/* CHIP_SW42103_SPI */
#define CHIP_BUS_TYPE				BUS_IF_I2C
#define	CHIP_BUF_SIZE				0
#define CHIP_SPI_MODE				-1
#define CHIP_BPW					-1
#define CHIP_MAX_FREQ				-1
#define CHIP_TX_HDR_SZ				I2C_BUS_TX_HDR_SZ
#define CHIP_RX_HDR_SZ				I2C_BUS_RX_HDR_SZ
#define CHIP_TX_DUMMY_SZ			I2C_BUS_TX_DUMMY_SZ
#define CHIP_RX_DUMMY_SZ			I2C_BUS_RX_DUMMY_SZ
#endif	/* CHIP_SW42103_SPI */

static const struct siw_hal_reg_quirk chip_reg_quirks[] = {
	{ .old_addr = SPR_CHIP_TEST, .new_addr = 0x0BE, },
	{ .old_addr = SPR_CHIP_ID, .new_addr = 0x100, },
	{ .old_addr = SPR_RST_CTL, .new_addr = 0x082, },
	{ .old_addr = SPR_BOOT_CTL, .new_addr = 0x002, },
	{ .old_addr = SPR_SRAM_CTL, .new_addr = 0x003, },
	{ .old_addr = SPR_BOOT_STS, .new_addr = 0x107, },
	{ .old_addr = SPR_SUBDISP_STS, .new_addr = 0x10C, },
	{ .old_addr = INFO_CHIP_VERSION, .new_addr = 0x101, },
	{ .old_addr = TC_CONFDN_BASE_ADDR, .new_addr = 0x24C },
	/* */
	{ .old_addr = CODE_ACCESS_ADDR, .new_addr = 0xFD0, },
	{ .old_addr = DATA_I2CBASE_ADDR, .new_addr = 0xFD1, },
	{ .old_addr = PRD_TCM_BASE_ADDR, .new_addr = 0xFD3, },
	/* */
	{ .old_addr = (1<<31), .new_addr = 0, },	/* switch : don't show log */
	/* */
#if defined(CHIP_SW17700_SPI)
	{ .old_addr = SPR_CODE_OFFSET, .new_addr = 0x024, },
	{ .old_addr = SERIAL_DATA_OFFSET, .new_addr = 0x028, },
#else	/* CHIP_SW17700_SPI */
	{ .old_addr = SPR_CODE_OFFSET, .new_addr = 0x019, },
	{ .old_addr = SERIAL_DATA_OFFSET, .new_addr = 0x01D, },
#endif	/* CHIP_SW17700_SPI */
	{ .old_addr = TC_TSP_TEST_STS, .new_addr = 0x2A0, },
	{ .old_addr = TC_TSP_TEST_PF_RESULT, .new_addr = 0x2A1, },
	{ .old_addr = TC_FLASH_DN_STS, .new_addr = 0x266, },
	/* */
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
/* Resolution
 *
 * LA123WF7 : 1920 x 720
 * LA103WF5 : 1920 x 720
 * LA102WH3 : 1280 x 720
 * LA080WH1 : 1280 x 720
 *
 *     [10.25] [12.3] [10.2] [8.0]
 * X :	1920    1920   1280  1280
 * Y :	 720     720    720   720
 */
enum CHIP_CAPABILITY {
	CHIP_MAX_X			= 1920,
	CHIP_MAX_Y			= 720,
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

#if defined(__SIW_SUPPORT_ALIVE_DETECTION)
static int __sw17700_alive_allowed(struct device *dev)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_hal_fw_info *fw = &chip->fw;

	return 1;
}

static int sw17700_alive_setup(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int alive_reg = 0;
	int alive_level_min = 0;
	int alive_level_def = 0;
	int alive_level_max = 0;
	u32 level = 0;
	int ret = 0;

	if (__sw17700_alive_allowed(dev)) {
		alive_reg = 0x274;
		alive_level_min = 1;
		alive_level_def = 7;
		alive_level_max = 10;
	}

	if (!alive_reg)
		return 0;

	ret = siw_hal_read_value(dev, alive_reg, &level);
	if (ret < 0) {
		return ret;
	}

	if ((level < alive_level_min) || (level > alive_level_max)) {
		return EPERM;
	}

	/* Enable alive check framework */
	chip->alive_reg = alive_reg;
	chip->alive_level_min = alive_level_min;
	chip->alive_level_def = alive_level_def;
	chip->alive_level_max = alive_level_max;

	/* Enable self alive monitoring */
	chip->alive_mon = 1;

	return 0;
}

static int sw17700_alive_pause_set(struct device *dev, int level)
{
	return 0;
}

static int sw17700_alive_check(struct device *dev, int irq_type)
{
	return 0;
}

#define CHIP_ALIVE_SETUP		sw17700_alive_setup
#define CHIP_ALIVE_PAUSE_SET	sw17700_alive_pause_set
#define CHIP_ALIVE_CHECK		sw17700_alive_check
#else	/* !__SIW_SUPPORT_ALIVE_DETECTION */
#define CHIP_ALIVE_SETUP		NULL
#define CHIP_ALIVE_PAUSE_SET	NULL
#define CHIP_ALIVE_CHECK		NULL
#endif	/* __SIW_SUPPORT_ALIVE_DETECTION */

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
		.alive_setup		= CHIP_ALIVE_SETUP,
		.alive_pause_set	= CHIP_ALIVE_PAUSE_SET,
		.alive_check		= CHIP_ALIVE_CHECK,
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




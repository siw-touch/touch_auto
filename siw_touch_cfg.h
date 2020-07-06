/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * SiW touch core driver
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#include <linux/version.h>

#ifndef __SIW_TOUCH_CFG_H
#define __SIW_TOUCH_CFG_H

#if defined(CONFIG_PINCTRL)
#define __SIW_SUPPORT_PINCTRL
#endif

#if defined(CONFIG_REGULATOR)
#define __SIW_SUPPORT_PWRCTRL
#endif

//#define __SIW_SUPPORT_MISC

//#define __SIW_SUPPORT_PRD

#define __SIW_SUPPORT_PM_WAKEUP

#if defined(CONFIG_OF)
#define __SIW_CONFIG_OF
#endif

/*****************************************************************************
 * Large
 *****************************************************************************/
#if defined(CONFIG_TOUCHSCREEN_SIW_SW42101)
#define __SIW_PANEL_CLASS_LARGE

#define __SIW_CONFIG_SYSTEM_PM
/*****************************************************************************
 * Automotive
 *****************************************************************************/
#else
#define __SIW_PANEL_CLASS_AUTO

#if defined(CONFIG_TOUCHSCREEN_SIW_SW42103) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW17700)
#define __SIW_FW_TYPE_1
#endif

#define __SIW_SUPPORT_PROBE_POST_RETRY
#define __SIW_SUPPORT_INIT_RETRY

#define __SIW_SUPPORT_MON_THREAD

#define __SIW_SUPPORT_ALIVE_DETECTION

#ifdef __SIW_SUPPORT_PM_WAKEUP
#undef __SIW_SUPPORT_PM_WAKEUP
#endif

#define __SIW_CONFIG_SYSTEM_PM
#endif
/*****************************************************************************/

#if defined(__SIW_CONFIG_SYSTEM_PM)
#if defined(CONFIG_HIBERNATE_CALLBACKS)
//#define __SIW_CONFIG_FASTBOOT		//Custom option for special scenario
#endif	/* CONFIG_HIBERNATE_CALLBACKS */
#endif


#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 15, 0))
#define __SIW_ATTR_PERMISSION_ALL
#endif

#define __SIW_ATTR_RST_BY_READ

#define SIW_TOUCH_NAME				"siw_touch"
#define SIW_TOUCH_CORE				"siw_touch_core"
#define SIW_TOUCH_INPUT				"siw_touch_input"

#define MAX_FINGER					10

enum _SIW_CHIP_TYPE {
	CHIP_NONE		= 0x0000,
	//
	CHIP_SW1828		= 0x0080,
	//
	CHIP_SW42101	= 0x0211,
	CHIP_SW42103	= 0x0213,
	//
	CHIP_SW17700	= 0x7700,
};

//#define __SIW_TEST_IRQ_OFF

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0))
#define mod_delayed_work	queue_delayed_work
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
#define subsys_system_register(_subsys, _group)	bus_register(_subsys)
#endif

#endif	/* __SIW_TOUCH_CFG_H */


/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_sys.c - SiW touch system interface
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#if defined(CONFIG_PLAT_SAMSUNG)
#include <plat/cpu.h>
#include <plat/gpio-core.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-cfg-helpers.h>
#include <plat/pm.h>
#endif

#include "siw_touch.h"
#include "siw_touch_gpio.h"
#include "siw_touch_sys.h"

int siw_touch_get_boot_mode(void)
{
	return 0;
}

int siw_touch_sys_gpio_set_pull(int pin, int value)
{
	int ret = 0;

#if defined(CONFIG_PLAT_SAMSUNG)
	{
		int pull_val;

		switch (value) {
		case GPIO_PULL_UP:
			pull_val = S3C_GPIO_PULL_UP;
			break;
		case GPIO_PULL_DOWN:
			pull_val = S3C_GPIO_PULL_DOWN;
			break;
		default:
			pull_val = S3C_GPIO_PULL_NONE;
			break;
		}
		ret = s3c_gpio_setpull(pin, pull_val);
	}
#endif

	return ret;
}

int siw_touch_sys_power_state(struct device *dev)
{
	/*
	if (invalid_power_state) {
		t_dev_warn(dev, "power status not invalid\n");
		return -EPERM;
	}
	*/

	return 0;
}

int siw_touch_sys_power_lock(struct device *dev, int set)
{
	if (set) {
		/* keep touch power on */
	} else {
		/* allow touch power off */
	}

	return 0;
}


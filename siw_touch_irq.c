/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_irq.c - SiW touch interrupt driver
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
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_irq.h"


void siw_touch_enable_irq(struct device *dev, unsigned int irq)
{
	enable_irq(irq);
	t_dev_info(dev, "irq(%d) enabled\n", irq);
}

void siw_touch_disable_irq(struct device *dev, unsigned int irq)
{
	disable_irq_nosync(irq);	/* No waiting */
	t_dev_info(dev, "irq(%d) disabled\n", irq);
}

void siw_touch_irq_control(struct device *dev, int on_off)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (!ts->irqflags_curr) {
		t_dev_warn(dev, "irq not initialized\n");
		return;
	}

	t_dev_dbg_irq(dev, "touch_irq_control(%d)\n", on_off);

	if (on_off == INTERRUPT_ENABLE) {
		if (atomic_cmpxchg(&ts->state.irq_enable, 0, 1)) {
			t_dev_warn(dev, "(warn) already irq enabled\n");
			return;
		}

		siw_touch_enable_irq(dev, ts->irq);

		return;
	}

	if (!atomic_cmpxchg(&ts->state.irq_enable, 1, 0)) {
		t_dev_warn(dev, "(warn) already irq disabled\n");
		return;
	}

	siw_touch_disable_irq(dev, ts->irq);
}

#define __SIW_SUPPORT_IRQ_INDEX_CHECK

/*
 * Verify irq handler index(ts->irq) using gpio_to_irq
 */
#if defined(__SIW_SUPPORT_IRQ_INDEX_CHECK)
static int siw_touch_irq_index_check(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int irq_pin = touch_irq_pin(ts);
	int irq = 0;

	if (!gpio_is_valid(irq_pin)) {
		return 0;
	}

	irq = gpio_to_irq(irq_pin);
	if (irq <= 0) {
		t_dev_warn(dev, "check irq pin: gpio_to_irq(%d) = %d\n",
			irq_pin, irq);
		goto out;
	}

	if (ts->irq) {
		if (ts->irq != irq) {
			t_dev_warn(dev,
				"check irq index: ts->irq = %d vs. gpio_to_irq(%d) = %d\n",
				ts->irq, irq_pin, irq);
		}
		goto out;
	}

	t_dev_info(dev,
		"irq index(%d) is obtained via gpio_to_irq(%d)\n",
		irq, irq_pin);

	ts->irq = irq;

out:
	return irq;
}
#else	/* __SIW_SUPPORT_IRQ_INDEX_CHECK */
#define siw_touch_irq_index_check(_ts)	do {	} while (0)
#endif	/* __SIW_SUPPORT_IRQ_INDEX_CHECK */

int siw_touch_request_irq(struct siw_ts *ts,
								irq_handler_t handler,
							    irq_handler_t thread_fn,
							    unsigned long flags,
							    const char *name)
{
	struct device *dev = ts->dev;
	int ret = 0;

	siw_touch_irq_index_check(ts);

	if (!ts->irq) {
		t_dev_err(dev, "failed to request irq : zero irq\n");
		ret = -EFAULT;
		goto out;
	}

	ret = request_threaded_irq(ts->irq, handler, thread_fn, flags, name, (void *)ts);
	if (ret) {
		t_dev_err(dev, "failed to request irq(%d, %s, 0x%X), %d\n",
				ts->irq, name, (u32)flags, ret);
		goto out;
	}

	ts->irqflags_curr = flags;

	t_dev_info(dev, "threaded irq request done(%d, %s, 0x%X)\n",
			ts->irq, name, (u32)flags);

out:
	return ret;
}

void siw_touch_free_irq(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	if (ts->irqflags_curr) {
		free_irq(ts->irq, (void *)ts);
		t_dev_info(dev, "irq(%d) release done\n", ts->irq);
		ts->irqflags_curr = 0;
		return;
	}

//	t_dev_warn(dev, "(warn) already free_irq done\n");
}



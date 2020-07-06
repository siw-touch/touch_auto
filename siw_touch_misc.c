/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_misc.c - SiW touch misc driver
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#include "siw_touch_cfg.h"

#if defined(__SIW_SUPPORT_MISC)	//See siw_touch_cfg.h

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
#include <linux/miscdevice.h>

#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"
#include "siw_touch_event.h"
#include "siw_touch_gpio.h"
#include "siw_touch_irq.h"
#include "siw_touch_sys.h"

/*
 * Helper layer to support direct access via device node
 * This misc layer makes a dedicated device node(/dev/{misc name}) for touch device
 * and user app can access the bus regardless the interface type.
 *
 */

#define SIW_MISC_NAME	"siw_touch_misc"

enum {
	SIW_MISC_BUF_SZ = PAGE_SIZE,
	/* */
	SIW_MISC_NAME_SZ = 32,
};

struct siw_misc_data {
	struct miscdevice misc;
	struct device *dev;
	/* */
	char name[SIW_MISC_NAME_SZ];
	struct mutex lock;
//	spinlock_t bus_lock;
	int users;
	u8 *buf;
};

static ssize_t siw_misc_read(struct file *filp,
					char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct miscdevice *misc = filp->private_data;
	struct siw_misc_data *misc_data = \
		container_of(misc, struct siw_misc_data, misc);
	struct device *dev = NULL;
	u32 addr;
	ssize_t result = 0;
	ssize_t ret = -EFAULT;

	if (count > SIW_MISC_BUF_SZ) {
		return -EMSGSIZE;
	}

	mutex_lock(&misc_data->lock);

	dev = misc_data->dev;

	/*
	 * buf[0 ~ 3] : addr
	 */
	ret = copy_from_user(&addr, buf, 4);
	if (ret) {
		t_dev_err(dev, "rd: can't get addr, %d\n", (int)ret);
		goto out;
	}

	ret = siw_hal_reg_read(dev, addr, misc_data->buf, count);
	if (ret < 0) {
		goto out;
	}

	result = copy_to_user((void __user *)buf, misc_data->buf, count);
	if (result) {
		t_dev_err(dev, "rd: can't copy buf(%d), %d\n",
			(int)count, (int)result);
		ret = result;
		goto out;
	}

	ret = count;

out:
	mutex_unlock(&misc_data->lock);

	return ret;
}

enum {
	MISC_WR_DATA_OFFSET	= 4,
};

static ssize_t siw_misc_write(struct file *filp,
					const char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct miscdevice *misc = filp->private_data;
	struct siw_misc_data *misc_data = \
		container_of(misc, struct siw_misc_data, misc);
	struct device *dev = NULL;
	u32 addr;
	ssize_t ret = -EFAULT;

	if (count > SIW_MISC_BUF_SZ) {
		return -EMSGSIZE;
	}

	if (count <= MISC_WR_DATA_OFFSET) {
		return -EINVAL;
	}

	mutex_lock(&misc_data->lock);

	dev = misc_data->dev;

	/*
	 * buf[0 ~ 3] : addr
	 * buf[3 ~ (count - 1)] : data
	 */
	ret = copy_from_user(misc_data->buf, buf, count);
	if (ret) {
		t_dev_err(dev, "wr: can't copy buf(%d), %d\n",
			(int)count, (int)ret);
		goto out;
	}
	memcpy(&addr, misc_data->buf, MISC_WR_DATA_OFFSET);

	ret = siw_hal_reg_write(dev, addr,
				&misc_data->buf[MISC_WR_DATA_OFFSET],
				count - MISC_WR_DATA_OFFSET);
	if (ret < 0) {
		goto out;
	}

	ret = count;

out:
	mutex_unlock(&misc_data->lock);

	return ret;
}

static int siw_misc_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct siw_misc_data *misc_data = \
		container_of(misc, struct siw_misc_data, misc);
	struct device *dev = NULL;
	int ret = 0;

	mutex_lock(&misc_data->lock);

	dev = misc_data->dev;

	if (!misc_data->users) {
		siw_touch_irq_control(dev, INTERRUPT_DISABLE);

		siw_touch_mon_pause(dev);
	}

	misc_data->users++;

	t_dev_info(dev, "siw_misc: opened(%d)\n", misc_data->users);

	mutex_unlock(&misc_data->lock);
	return ret;
}

static int siw_misc_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct siw_misc_data *misc_data = \
		container_of(misc, struct siw_misc_data, misc);
	struct device *dev;
	int ret = 0;

	mutex_lock(&misc_data->lock);

	dev = misc_data->dev;

	if (!misc_data->users) {
		goto out;
	}

	misc_data->users--;
	if (!misc_data->users) {
		siw_touch_mon_resume(dev);

		siw_touch_irq_control(dev, INTERRUPT_ENABLE);
	}

	t_dev_info(dev, "siw_misc: released(%d)\n", misc_data->users);

out:
	mutex_unlock(&misc_data->lock);

	return ret;
}

static int siw_misc_do_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	/* test log */
	t_dev_info(dev, "siw_misc: ioctl: D %Xh, T %Xh, N %Xh, S %Xh (%08Xh)\n",
		_IOC_DIR(cmd), _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_SIZE(cmd), cmd);

	return 0;
}

static long __used siw_misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *misc = filp->private_data;
	struct siw_misc_data *misc_data = \
		container_of(misc, struct siw_misc_data, misc);
	struct device *dev = NULL;
	struct siw_ts *ts = NULL;
	int ret = 0;

	mutex_lock(&misc_data->lock);

	dev = misc_data->dev;

	ts = to_touch_core(dev);

	if (touch_get_dev_data(ts) == NULL) {
		t_dev_info(dev, "siw_misc_ioctl: touch not connected, skipped\n");
		goto out;
	}

	ret = siw_misc_do_ioctl(dev, cmd, arg);

out:
	mutex_unlock(&misc_data->lock);

	return (long)ret;
}

static const struct file_operations siw_misc_fops = {
	.owner			= THIS_MODULE,
	.read			= siw_misc_read,
	.write			= siw_misc_write,
	.unlocked_ioctl	= siw_misc_ioctl,
	.open 			= siw_misc_open,
	.release		= siw_misc_release,
	.llseek			= no_llseek,
};

int siw_touch_misc_init(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_misc_data *misc_data = NULL;
	struct miscdevice *misc = NULL;
	char *name;
	int str_sz = sizeof(struct siw_misc_data);
	int ret = 0;

	if (ts->misc != NULL) {
		t_dev_warn(dev, "siw misc already alloacted\n");
		goto out;
	}

	misc_data = kzalloc(str_sz + SIW_MISC_BUF_SZ, GFP_KERNEL);
	if (misc_data == NULL) {
		t_dev_err(dev, "can't alloacte misc data\n");
		ret = -ENOMEM;
		goto out;
	}

	misc_data->buf = ((u8 *)misc_data) + str_sz;
	misc_data->dev = dev;
//	spin_lock_init(&misc_data->bus_lock);

	misc = &misc_data->misc;

	name = touch_drv_name(ts);
	if (!name) {
		name = SIW_MISC_NAME;
	}
	snprintf(misc_data->name, SIW_MISC_NAME_SZ,
		"%s", name);

	misc->minor	= MISC_DYNAMIC_MINOR;
	misc->name = misc_data->name;
	misc->fops = &siw_misc_fops;

	mutex_init(&misc_data->lock);

	/* register misc device */
	ret = misc_register(misc);
	if (ret < 0) {
		t_dev_err(dev, "siw misc_register failed\n");
		goto out_register;
	}

	ts->misc = misc_data;

	t_dev_info(dev, "siw misc register done (%d)\n", misc->minor);

	return 0;

out_register:
	kfree(misc_data);

out:

	return ret;
}

void siw_touch_misc_free(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_misc_data *misc_data = ts->misc;

	if (misc_data == NULL) {
		return;
	}

	mutex_destroy(&misc_data->lock);

	misc_deregister(&misc_data->misc);

	kfree(misc_data);

	ts->misc = NULL;
}

#endif	/* __SIW_SUPPORT_MISC */


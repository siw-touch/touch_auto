/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_sysfs.c - SiW touch sysfs driver
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

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_gpio.h"
#include "siw_touch_irq.h"
#include "siw_touch_sys.h"


#define siw_sysfs_err_invalid_param(_dev)	\
		t_dev_err(_dev, "Invalid param\n");

#define _plat_data_snprintf(_buf, _size, args...)	\
		siw_snprintf(_buf, _size, " %-25s = %d\n", ##args)

static void siw_touch_sysfs_gen_symlink(struct siw_ts *ts, char *name)
{
	struct device *dev = ts->dev;
	struct kobject *kobj = &ts->kobj;
	int ret = 0;

	if (!name || !strlen(name)) {
		return;
	}

	if (strcmp(kobj->name, name) != 0) {
		ret = sysfs_create_link(kobj->parent, kobj, name);
		if (ret >= 0) {
			t_dev_info(dev, "symlink generated, %s\n", name);
		}
	}
}

static void siw_touch_sysfs_del_symlink(struct siw_ts *ts, char *name)
{
	struct kobject *kobj = &ts->kobj;

	if (!name || !strlen(name)) {
		return;
	}

	if (strcmp(kobj->name, name) != 0) {
		sysfs_remove_link(kobj->parent, name);
	}
}


static ssize_t _show_do_plat_data(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct touch_device_caps *caps = &ts->caps;
	struct touch_operation_role *role = &ts->role;
	int i;
	int size = 0;

	size += siw_snprintf(buf, size, "=== Platform Data ===\n");
	size += _plat_data_snprintf(buf, size,
					"reset_pin", touch_reset_pin(ts));
	size += _plat_data_snprintf(buf, size,
					"reset_pin_pol", touch_reset_pin_pol(ts));
	size += _plat_data_snprintf(buf, size,
					"irq_pin", touch_irq_pin(ts));

	size += siw_snprintf(buf, size, "power:\n");
	size += _plat_data_snprintf(buf, size,
					"vdd-gpio", touch_vdd_pin(ts));
	size += _plat_data_snprintf(buf, size,
					"vio-gpio", touch_vio_pin(ts));

	size += siw_snprintf(buf, size, "caps:\n");
	size += _plat_data_snprintf(buf, size,
					"max_x", caps->max_x);
	size += _plat_data_snprintf(buf, size,
					"max_y", caps->max_y);
	size += _plat_data_snprintf(buf, size,
					"max_pressure", caps->max_pressure);
	size += _plat_data_snprintf(buf, size,
					"max_width", caps->max_width);
	size += _plat_data_snprintf(buf, size,
					"max_orientation", caps->max_orientation);
	size += _plat_data_snprintf(buf, size,
					"max_id", caps->max_id);
	size += _plat_data_snprintf(buf, size,
					"hw_reset_delay", caps->hw_reset_delay);
	size += _plat_data_snprintf(buf, size,
					"sw_reset_delay", caps->sw_reset_delay);

	size += siw_snprintf(buf, size, "role:\n");
	if (role->use_cancel_opt) {
		size += _plat_data_snprintf(buf, size,
					"use_cancel_opt", role->use_cancel_opt);
	}
	if (role->use_palm_opt) {
		size += _plat_data_snprintf(buf, size,
					"use_palm_opt", role->use_palm_opt);
	}
	size += _plat_data_snprintf(buf, size,
					"use_firmware", role->use_firmware);
	size += _plat_data_snprintf(buf, size,
					"use_fw_upgrade", role->use_fw_upgrade);
	if (role->use_fw_pwr_rst) {
		size += _plat_data_snprintf(buf, size,
					"use_fw_pwr_rst", role->use_fw_pwr_rst);
	}
	if (role->use_fw_ver_diff) {
		size += _plat_data_snprintf(buf, size,
					"use_fw_ver_diff", role->use_fw_ver_diff);
	}
	if (role->use_fw_skip_pid) {
		size += _plat_data_snprintf(buf, size,
					" use_fw_skip_pid", role->use_fw_skip_pid);
	}

	size += siw_snprintf(buf, size, "firmware:\n");
	size += _plat_data_snprintf(buf, size,
					"def_fwcnt", ts->def_fwcnt);
	for (i = 0; i < ts->def_fwcnt; i++)
		size += siw_snprintf(buf, size, " %-25s : [%d] %s\n",
						"def_fwpath", i, ts->def_fwpath[i]);

	return size;
}

static ssize_t _show_plat_data(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int size = 0;

	mutex_lock(&ts->lock);
	size = _show_do_plat_data(dev, buf);
	mutex_unlock(&ts->lock);

	return size;
}

static ssize_t _show_do_driver_data(struct device *dev, char *buf)
{
//	struct siw_ts *ts = to_touch_core(dev);
	int size = 0;

	size += siw_snprintf(buf, size, "=== Driver Data ===\n");

	size += siw_snprintf(buf, size,
						"Version : %s\n", SIW_DRV_VERSION);

	return size;
}

static ssize_t _show_driver_data(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int size = 0;

	mutex_lock(&ts->lock);
	size = _show_do_driver_data(dev, buf);
	mutex_unlock(&ts->lock);

	return size;
}

static ssize_t _store_upgrade(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	int data = 0;

	if (sscanf(buf, "%255s %X", ts->test_fwpath, &data) <= 0) {
		siw_sysfs_err_invalid_param(dev);
		return count;
	}

	t_dev_info(dev, "Manual F/W upgrade with %s\n", ts->test_fwpath);

	ts->force_fwup |= FORCE_FWUP_SYS_STORE;
	if (data == 0x5A5A) {
		ts->force_fwup |= FORCE_FWUP_SKIP_PID;
	}

	siw_touch_qd_upgrade_work_now(ts);

	return count;
}

static ssize_t _show_upgrade(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);

	ts->force_fwup |= FORCE_FWUP_SYS_SHOW;
	ts->test_fwpath[0] = '\0';

	siw_touch_qd_upgrade_work_now(ts);

	return 0;
}

static ssize_t _show_version_info(struct device *dev, char *buf)
{
	return siw_touch_get(dev, CMD_VERSION, buf);
}

static ssize_t _show_atcmd_version_info(struct device *dev, char *buf)
{
	return siw_touch_get(dev, CMD_ATCMD_VERSION, buf);
}

static ssize_t _show_irq_state(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int irq_status = atomic_read(&ts->state.irq_enable);
	int size = 0;

	size = siw_snprintf(buf, size,
				"Irq State : %s\n",
				(irq_status) ? "Enabled" : "Disabled");

	return size;
}

static ssize_t _store_irq_state(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		siw_sysfs_err_invalid_param(dev);
		return count;
	}

	mutex_lock(&ts->lock);

	siw_touch_irq_control(ts->dev,
			(value) ? INTERRUPT_ENABLE : INTERRUPT_DISABLE);

	mutex_unlock(&ts->lock);

	return (ssize_t)count;
}

static ssize_t _show_irq_level(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int irq_level = siw_touch_gpio_get_value(dev, touch_irq_pin(ts));
	int size = 0;

//	t_dev_dbg_base(dev, "irq_level : %d\n", irq_level);
	size = siw_snprintf(buf, size,
				"%d\n",
				irq_level);

	return size;
}

static ssize_t _show_module_info(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int size = 0;

	size += siw_snprintf(buf, size, "%s/%s/%s, %s\n",
			dev_name(dev->parent->parent),
			dev_name(dev->parent),
			dev_name(dev),
			dev_name(&ts->input->dev));

	return (ssize_t)size;
}

u32 __weak t_mon_dbg_mask;	//instead of using extern
u32 __weak t_bus_dbg_mask;	//instead of using extern

static ssize_t _show_dbg_mask(struct device *dev, char *buf)
{
//	struct siw_ts *ts = to_touch_core(dev);
	int size = 0;

	size += siw_snprintf(buf, size,
				"t_dev_dbg_mask %08Xh\n", t_dev_dbg_mask);
	size += siw_snprintf(buf, size,
				"t_pr_dbg_mask  %08Xh\n", t_pr_dbg_mask);
	size += siw_snprintf(buf, size,
				"t_mon_dbg_mask %08Xh\n", t_mon_dbg_mask);
	size += siw_snprintf(buf, size,
				"t_bus_dbg_mask %08Xh\n", t_bus_dbg_mask);

	size += siw_snprintf(buf, size,
				"Usage:\n");
	size += siw_snprintf(buf, size,
				" t_dev_dbg_mask : echo 0 {mask_value} > dbg_mask\n");
	size += siw_snprintf(buf, size,
				" t_pr_dbg_mask  : echo 1 {mask_value} > dbg_mask\n");
	size += siw_snprintf(buf, size,
				" t_mon_dbg_mask : echo 2 {mask_value} > dbg_mask\n");
	size += siw_snprintf(buf, size,
				" t_bus_dbg_mask : echo 3 {mask_value} > dbg_mask\n");

	return (ssize_t)size;
}

static void _store_dbg_mask_usage(struct device *dev)
{
	t_dev_info(dev, "Usage:\n");
	t_dev_info(dev, " t_dev_dbg_mask : echo 0 {mask_value(hex)} > dbg_mask\n");
	t_dev_info(dev, " t_pr_dbg_mask  : echo 1 {mask_value(hex)} > dbg_mask\n");
	t_dev_info(dev, " t_mon_dbg_mask : echo 2 {mask_value(hex)} > dbg_mask\n");
	t_dev_info(dev, " t_bus_dbg_mask : echo 3 {mask_value(hex)} > dbg_mask\n");
}

static ssize_t _store_dbg_mask(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	int type = 0;
	u32 old_value, new_value = 0;

	if (sscanf(buf, "%d %X", &type, &new_value) <= 0) {
		siw_sysfs_err_invalid_param(dev);
		_store_dbg_mask_usage(dev);
		return count;
	}

	switch (type) {
	case 0 :
		old_value = t_dev_dbg_mask;
		t_dev_dbg_mask = new_value;
		t_dev_info(dev, "t_dev_dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	case 1 :
		old_value = t_pr_dbg_mask;
		t_pr_dbg_mask = new_value;
		t_dev_info(dev, "t_pr_dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	case 2 :
		if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
			t_dev_info(dev, "mon thread not activated\n");
			break;
		}

		old_value = t_mon_dbg_mask;
		t_mon_dbg_mask = new_value;
		t_dev_info(dev, "t_mon_dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	case 3 :
		old_value = t_bus_dbg_mask;
		t_bus_dbg_mask = new_value;
		t_dev_info(dev, "t_bus_dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	default :
		_store_dbg_mask_usage(dev);
		break;
	}

	return count;
}

static ssize_t _show_dbg_flag(struct device *dev, char *buf)
{
//	struct siw_ts *ts = to_touch_core(dev);
	int size = 0;

	size += siw_snprintf(buf, size,
				"t_dbg_flag %08Xh\n\n",
				t_dbg_flag);

	return (ssize_t)size;
}

static ssize_t _store_dbg_flag(struct device *dev,
				const char *buf, size_t count)
{
//	struct siw_ts *ts = to_touch_core(dev);
	u32 old_value, new_value = 0;

	if (sscanf(buf, "%X", &new_value) <= 0) {
		siw_sysfs_err_invalid_param(dev);
		return count;
	}

	old_value = t_dbg_flag;
	t_dbg_flag = new_value;
	t_dev_info(dev, "t_dbg_flag changed : %08Xh -> %08xh\n",
		old_value, new_value);

	return count;
}

static ssize_t _show_irq_flag(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int size = 0;

	size += siw_snprintf(buf, size,
				"irq flag : %08Xh\n\n",
				(u32)ts->irqflags_curr);

	return (ssize_t)size;
}


static ssize_t _store_irq_flag(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	u32 old_value, new_value = 0;

	if (sscanf(buf, "%X", &new_value) <= 0) {
		siw_sysfs_err_invalid_param(dev);
		return count;
	}

	old_value = (u32)ts->irqflags_curr;

	mutex_lock(&ts->lock);

	irq_set_irq_type(ts->irq, new_value);
	ts->irqflags_curr &= ~IRQ_TYPE_SENSE_MASK;
	ts->irqflags_curr |= (new_value & IRQ_TYPE_SENSE_MASK);

	mutex_unlock(&ts->lock);

	t_dev_info(dev, "irq flag changed : %08Xh -> %08xh\n",
		old_value, (int)ts->irqflags_curr);

	return count;
}

#if defined(__SIW_SUPPORT_PROBE_POST_RETRY)
static ssize_t _store_init_late(struct device *dev,
				const char *buf, size_t count)
{
	t_dev_info(dev, "probe_post retry is working\n");

	return count;
}
#else	/* __SIW_SUPPORT_PROBE_POST_RETRY */
static ssize_t _store_init_late(struct device *dev,
				const char *buf, size_t count)
{
//	struct siw_ts *ts = to_touch_core(dev);
	int sig = 0;
	int time = 0;
	int retry = 0;

	if (sscanf(buf, "%X %d %d", &sig, &time, &retry) <= 0) {
		siw_sysfs_err_invalid_param(dev);
		return count;
	}

	if (sig == 0x55AA) {
		siw_touch_init_late_queue(dev, sig, time, retry);
	}

	return count;
}
#endif	/* __SIW_SUPPORT_PROBE_POST_RETRY */

static ssize_t _show_dbg_mon(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_mon_thread *mon_thread = &ts->mon_thread;
	const char *name = NULL;
	int size = 0;

	if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
		size += siw_snprintf(buf, size, "mon thread not enabled\n");
		return (ssize_t)size;
	}

	name = (atomic_read(&mon_thread->state) == TS_THREAD_ON) ? "resume" : "pause";

	size += siw_snprintf(buf, size, "mon thread is %s state\n", name);

	size += siw_snprintf(buf, size, "\nUsage:\n");
	size += siw_snprintf(buf, size, " pause  : echo 1 > dbg_mon\n");
	size += siw_snprintf(buf, size, " resume : echo 0 > dbg_mon\n");

	return (ssize_t)size;
}

static ssize_t _store_dbg_mon(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_chip *chip = NULL;
	int pause = 0;

	if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
		t_dev_info(dev, "mon thread not enabled\n");
		return count;
	}

	if (sscanf(buf, "%d", &pause) <= 0) {
		siw_sysfs_err_invalid_param(dev);
		return count;
	}

	chip = to_touch_chip(dev);

	if (chip->driving_mode != LCD_MODE_U3) {
		t_dev_info(dev, "can be controlled only in U3\n");
		return count;
	}

	if (pause)
		siw_touch_mon_pause(dev);
	else
		siw_touch_mon_resume(dev);

	return count;
}

enum {
	SYSFS_DBG_TEST_NONE = 0,
	SYSFS_DBG_TEST_SYMLINK,
};

static ssize_t _store_dbg_test(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	char name[16+1] = {0, };
	int magic = 0;
	int code = 0;
	int value = 0;

	if (sscanf(buf, "%X %X %X %16s",
			&magic, &code, &value, name) <= 0) {
		siw_sysfs_err_invalid_param(dev);
		return count;
	}

	if (magic != 0x5A) {	//magic code
		goto out;
	}

	switch (code) {
	case SYSFS_DBG_TEST_SYMLINK :
		siw_touch_sysfs_del_symlink(ts, name);
		if (value) {
			siw_touch_sysfs_gen_symlink(ts, name);
		}
		break;
	default :
		break;
	}

out:
	return count;
}


#define SIW_TOUCH_ATTR(_name, _show, _store)	\
		TOUCH_ATTR(_name, _show, _store)

#define _SIW_TOUCH_ATTR_T(_name)	\
		touch_attr_##_name


static SIW_TOUCH_ATTR(platform_data,
						_show_plat_data, NULL);
static SIW_TOUCH_ATTR(driver_data,
						_show_driver_data, NULL);
static SIW_TOUCH_ATTR(fw_upgrade,
						_show_upgrade,
						_store_upgrade);
static SIW_TOUCH_ATTR(firmware,
						_show_version_info, NULL);
static SIW_TOUCH_ATTR(version,
						_show_version_info, NULL);
static SIW_TOUCH_ATTR(testmode_ver,
						_show_atcmd_version_info, NULL);
static SIW_TOUCH_ATTR(irq_state,
						_show_irq_state,
						_store_irq_state);
static SIW_TOUCH_ATTR(irq_level,
						_show_irq_level, NULL);
static SIW_TOUCH_ATTR(module_info,
						_show_module_info, NULL);
static SIW_TOUCH_ATTR(dbg_mask,
						_show_dbg_mask,
						_store_dbg_mask);
static SIW_TOUCH_ATTR(dbg_flag,
						_show_dbg_flag,
						_store_dbg_flag);
static SIW_TOUCH_ATTR(irq_flag,
						_show_irq_flag,
						_store_irq_flag);
static SIW_TOUCH_ATTR(init_late, NULL,
						_store_init_late);
static SIW_TOUCH_ATTR(dbg_mon,
						_show_dbg_mon,
						_store_dbg_mon);
static SIW_TOUCH_ATTR(dbg_test, NULL,
						_store_dbg_test);


static struct attribute *siw_touch_attribute_list[] = {
	&_SIW_TOUCH_ATTR_T(platform_data).attr,
	&_SIW_TOUCH_ATTR_T(driver_data).attr,
	&_SIW_TOUCH_ATTR_T(module_info).attr,
	&_SIW_TOUCH_ATTR_T(dbg_mask).attr,
	&_SIW_TOUCH_ATTR_T(dbg_flag).attr,
	&_SIW_TOUCH_ATTR_T(init_late).attr,
	NULL,
};

static struct attribute *siw_touch_attribute_list_normal[] = {
	&_SIW_TOUCH_ATTR_T(fw_upgrade).attr,
	&_SIW_TOUCH_ATTR_T(firmware).attr,
	&_SIW_TOUCH_ATTR_T(version).attr,
	&_SIW_TOUCH_ATTR_T(testmode_ver).attr,
	&_SIW_TOUCH_ATTR_T(irq_state).attr,
	&_SIW_TOUCH_ATTR_T(irq_level).attr,
	&_SIW_TOUCH_ATTR_T(irq_flag).attr,
	&_SIW_TOUCH_ATTR_T(dbg_mon).attr,
	&_SIW_TOUCH_ATTR_T(dbg_test).attr,
	NULL,
};

static const struct attribute_group siw_touch_attribute_group_normal = {
	.attrs = siw_touch_attribute_list_normal,
};

static ssize_t siw_touch_attr_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct siw_ts *ts =
		container_of(kobj, struct siw_ts, kobj);
	struct siw_touch_attribute *priv =
		container_of(attr, struct siw_touch_attribute, attr);
	ssize_t ret = 0;

	if (priv->show)
		ret = priv->show(ts->dev, buf);

	return ret;
}

static ssize_t siw_touch_attr_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct siw_ts *ts =
		container_of(kobj, struct siw_ts, kobj);
	struct siw_touch_attribute *priv =
		container_of(attr, struct siw_touch_attribute, attr);
	ssize_t ret = count;

	if (priv->store)
		ret = priv->store(ts->dev, buf, count);

	return ret;
}

/*
 * To reegister SiW's unique sysfs functions
 */
static const struct sysfs_ops siw_touch_sysfs_ops = {
	.show	= siw_touch_attr_show,
	.store	= siw_touch_attr_store,
};

#define siw_touch_attr_default	siw_touch_attribute_list

static struct kobj_type siw_touch_kobj_type = {
	.sysfs_ops		= &siw_touch_sysfs_ops,
	.default_attrs	= siw_touch_attr_default,
};


int siw_touch_init_sysfs(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct device *idev = &ts->input->dev;
	struct kobject *kobj = &ts->kobj;
	char *name = NULL;
	int ret = 0;

	name = (touch_flags(ts) & TOUCH_USE_DRV_NAME_SYSFS) ?
			touch_drv_name(ts) : touch_idrv_name(ts);
	if (!name) {
		name = SIW_TOUCH_INPUT;
	}

	ret = kobject_init_and_add(kobj, &siw_touch_kobj_type,
			idev->kobj.parent, "%s", name);
	if (ret < 0) {
		t_dev_err(dev, "failed to create sysfs entry\n");
		goto out;
	}

	return 0;

out:
	return ret;
}

void siw_touch_free_sysfs(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct device *idev = &ts->input->dev;

	if (ts->kobj.parent != idev->kobj.parent) {
		t_dev_warn(dev, "Invalid kobject\n");
		return;
	}

	kobject_del(&ts->kobj);
	kobject_put(&ts->kobj);
}

int __weak siw_touch_misc_init(struct device *dev)
{
	return 0;
}

void __weak siw_touch_misc_free(struct device *dev)
{

}

static int siw_touch_add_sysfs_normal(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct kobject *kobj = &ts->kobj;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	const struct attribute_group *sysfs_grp = fquirks->sysfs_group;
	int ret;

	ret = sysfs_create_group(kobj, &siw_touch_attribute_group_normal);
	if (ret < 0) {
		t_dev_err(dev, "failed to add sysfs(normal)\n");
		goto out;
	}

	if (sysfs_grp != NULL) {
		ret = sysfs_create_group(kobj, sysfs_grp);
		if (ret < 0) {
			t_dev_err(dev, "failed to add fquirks sysfs\n");
			goto out_sysfs_grp;
		}
	}

	ret = siw_ops_sysfs(ts, DRIVER_INIT);
	if (ret < 0) {
		t_dev_err(dev, "failed to add sysfs(ops)\n");
		goto out_ops_sysfs;
	}

	return 0;

out_ops_sysfs:
	if (sysfs_grp != NULL) {
		sysfs_remove_group(kobj, sysfs_grp);
	}

out_sysfs_grp:
	sysfs_remove_group(kobj, &siw_touch_attribute_group_normal);

out:
	return ret;
}

static void siw_touch_del_sysfs_normal(struct siw_ts *ts)
{
//	struct device *dev = ts->dev;
	struct kobject *kobj = &ts->kobj;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	const struct attribute_group *sysfs_grp = fquirks->sysfs_group;

	siw_ops_sysfs(ts, DRIVER_FREE);

	if (sysfs_grp != NULL) {
		sysfs_remove_group(kobj, sysfs_grp);
	}

	sysfs_remove_group(kobj, &siw_touch_attribute_group_normal);
}

static int siw_touch_do_add_sysfs(struct siw_ts *ts)
{
	if (ts->is_charger) {
		return 0;
	}

	return siw_touch_add_sysfs_normal(ts);
}

static void siw_touch_do_del_sysfs(struct siw_ts *ts)
{
	if (ts->is_charger) {
		return;
	}

	siw_touch_del_sysfs_normal(ts);
}

int siw_touch_add_sysfs(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct device *idev = &ts->input->dev;
	struct kobject *kobj = &ts->kobj;
	int ret = 0;

	if (kobj->parent != idev->kobj.parent) {
		t_dev_err(dev, "Invalid kobject\n");
		return -EINVAL;
	}

	ret = siw_touch_do_add_sysfs(ts);
	if (ret < 0) {
		return ret;
	}

	siw_touch_misc_init(dev);

	return 0;
}

void siw_touch_del_sysfs(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct device *idev = &ts->input->dev;
	struct kobject *kobj = &ts->kobj;

	if (kobj->parent != idev->kobj.parent) {
		t_dev_warn(dev, "Invalid kobject\n");
		return;
	}

	siw_touch_misc_free(dev);

	siw_touch_do_del_sysfs(ts);
}



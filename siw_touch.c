/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch.c - SiW touch core driver
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
#include <linux/kthread.h>
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

#if !defined(__SIW_CONFIG_OF)
//#pragma message("[SiW - Warning] No COFIG_OF")
#endif

extern int siw_touch_init_sysfs(struct siw_ts *ts);
extern void siw_touch_free_sysfs(struct siw_ts *ts);

extern int siw_touch_add_sysfs(struct siw_ts *ts);
extern void siw_touch_del_sysfs(struct siw_ts *ts);

extern int siw_touch_parse_data(struct siw_ts *ts);

#if 0
u32 t_pr_dbg_mask = DBG_NONE | DBG_INFO;
u32 t_dev_dbg_mask = DBG_NONE | DBG_BASE | DBG_INFO | DBG_GPIO | DBG_OF;
#else
u32 t_pr_dbg_mask = DBG_NONE;
u32 t_dev_dbg_mask = DBG_NONE | DBG_BUTTON;
/*
 * DBG_NONE | DBG_BASE | DBG_IRQ | DBG_NOTI | DBG_EVENT = 201326721(0xC000081)
 */
#endif

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/pr_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko pr_dbg_mask=<value>
 */
module_param_named(pr_dbg_mask, t_pr_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/dev_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko dev_dbg_mask=<value>
 */
module_param_named(dev_dbg_mask, t_dev_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);


u32 t_dbg_flag = 0;
/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/dbg_flag
 * (2) insmod {Siw Touch Module Name}.ko dbg_flag=<value>
 */
module_param_named(dbg_flag, t_dbg_flag, uint, S_IRUGO|S_IWUSR|S_IWGRP);


static void siw_config_status(struct device *dev)
{
#if defined(__SIW_PANEL_CLASS_LARGE)
	t_dev_info(dev, "cfg status : __SIW_PANEL_CLASS_LARGE\n");
#endif

#if defined(__SIW_PANEL_CLASS_AUTO)
	t_dev_info(dev, "cfg status : __SIW_PANEL_CLASS_AUTO\n");
#endif

#if defined(__SIW_CONFIG_SYSTEM_PM)
	t_dev_info(dev, "cfg status : __SIW_CONFIG_SYSTEM_PM\n");
#endif

#if defined(__SIW_CONFIG_FASTBOOT)
	t_dev_info(dev, "cfg status : __SIW_CONFIG_FASTBOOT\n");
#endif

#if defined(__SIW_SUPPORT_PINCTRL)
	t_dev_info(dev, "cfg status : __SIW_SUPPORT_PINCTRL\n");
#endif

#if defined(__SIW_SUPPORT_PWRCTRL)
	t_dev_info(dev, "cfg status : __SIW_SUPPORT_PWRCTRL\n");
#endif

#if defined(__SIW_SUPPORT_PM_WAKEUP)
	t_dev_info(dev, "cfg status : __SIW_SUPPORT_PM_WAKEUP\n");
#endif

#if defined(__SIW_SUPPORT_PRD)
	t_dev_info(dev, "cfg status : __SIW_SUPPORT_PRD\n");
#endif

#if defined(__SIW_SUPPORT_PROBE_POST_RETRY)
	t_dev_info(dev, "cfg status : __SIW_SUPPORT_PROBE_POST_RETRY\n");
#endif

#if defined(__SIW_SUPPORT_INIT_RETRY)
	t_dev_info(dev, "cfg status : __SIW_SUPPORT_INIT_RETRY\n");
#endif

#if defined(__SIW_SUPPORT_MON_THREAD)
	t_dev_info(dev, "cfg status : __SIW_SUPPORT_MON_THREAD\n");
#endif
}

static int siw_setup_names(struct siw_ts *ts, struct siw_touch_pdata *pdata)
{
	struct device *dev = ts->dev;
	char *name;

	/*
	 * Mandatory
	 */
	name = pdata_chip_id(pdata);
	if (name == NULL) {
		return -EFAULT;
	}
	ts->chip_id = name;
	t_dev_info(dev, "chip id    : %s\n", name);

	name = pdata_chip_name(pdata);
	if (name == NULL) {
		return -EFAULT;
	}
	ts->chip_name = name;
	t_dev_info(dev, "chip name  : %s\n", name);

	/*
	 * Optional
	 */
	name = pdata_drv_name(pdata);
	if (name == NULL) {
		name = SIW_TOUCH_NAME;
	}
	ts->drv_name = name;
	t_dev_info(dev, "drv name   : %s\n", name);

	name = pdata_idrv_name(pdata);
	if (name == NULL) {
		name = SIW_TOUCH_INPUT;
	}
	ts->idrv_name = name;
	t_dev_info(dev, "idrv name  : %s\n", name);

	return 0;
}

int siw_setup_params(struct siw_ts *ts, struct siw_touch_pdata *pdata)
{
	struct device *dev = ts->dev;
	int max_finger = 0;
	int type = 0;
	u32 mode_allowed = 0;
	int ret = 0;

	siw_config_status(dev);

	max_finger = pdata_max_finger(pdata);
	if ((max_finger < 0) || (max_finger > MAX_FINGER)) {
		t_dev_err(dev, "invalid max finger, %d\n", max_finger);
		return -EFAULT;
	}
	ts->max_finger = max_finger;
	t_dev_info(dev, "max finger : %d\n", max_finger);

	type = pdata_chip_type(pdata);
	if (!type) {
		return -EFAULT;
	}
	ts->chip_type = type;
	t_dev_info(dev, "chip type  : 0x%04X\n", type);

	mode_allowed = pdata->mode_allowed;
	if (!mode_allowed) {
		return -EFAULT;
	}
#if defined(__SIW_CONFIG_SYSTEM_PM)
	mode_allowed &= ~(LCD_MODE_BIT_U0|LCD_MODE_BIT_U3_PARTIAL);
	mode_allowed &= ~(LCD_MODE_BIT_U2|LCD_MODE_BIT_U2_UNBLANK);
#endif	/* __SIW_CONFIG_SYSTEM_PM */
	ts->mode_allowed = mode_allowed;
	t_dev_info(dev, "mode bit   : 0x%08X\n", mode_allowed);

	ret = siw_setup_names(ts, pdata);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/*
 * SiW Operations
 */
static void siw_setup_reg_quirks(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct siw_hal_reg_quirk *reg_quirks = ts->pdata->reg_quirks;

	if (reg_quirks) {
	//	struct siw_hal_reg *reg = siw_ops_reg(ts);
		u32 *curr_reg;
		u32 *copy_reg;
		struct siw_hal_reg *reg_org;
		int cnt = sizeof(struct siw_hal_reg)>>2;
		char *name = touch_chip_name(ts);
		u32 new_addr, old_addr;
		int total = 0;
		int missing = 0;
		int show_log = 1;
		int found = 0;
		int i;

		reg_org = ts->ops_ext->reg;

		while (1) {
			old_addr = reg_quirks->old_addr;
			new_addr = reg_quirks->new_addr;

			if (old_addr == (1<<31)) {
				t_dev_info(dev, "%s reg quirks: ...\n",
					name);
				show_log = 0;
				reg_quirks++;
				continue;
			}

			if ((old_addr == ~0) || (new_addr == ~0)) {
				break;
			}

			found = 0;
			copy_reg = (u32 *)reg_org;
			curr_reg = (u32 *)siw_ops_reg(ts);
			for (i = 0; i < cnt; i++) {
				if ((*copy_reg) == old_addr) {
					(*curr_reg) = new_addr;
					found = 1;

					break;
				}
				copy_reg++;
				curr_reg++;
			}
			if (found) {
				if (show_log) {
					t_dev_info(dev, "%s reg quirks: [%d] %04Xh -> %04Xh\n",
						name, total,
						old_addr, new_addr);
				}
			} else {
				t_dev_warn(dev, "%s reg quirks: [%d] %04Xh not found\n",
					name, total,
					old_addr);
				missing++;
			}
			total++;

			reg_quirks++;
		}
		t_dev_info(dev, "%s reg quirks: t %d, m %d\n",
			name, total, missing);
	}
}

void *siw_setup_operations(struct siw_ts *ts, struct siw_touch_operations *ops_ext)
{
	if (!ops_ext)
		return NULL;

	if (ops_ext->reg == NULL) {
		t_dev_warn(ts->dev, "%s reg is NULL : default reg selected\n",
				touch_chip_name(ts));
		ops_ext->reg = siw_hal_get_default_reg(0);
	}

	ts->ops_ext = ops_ext;
	memcpy(&ts->ops_in, ops_ext, sizeof(struct siw_touch_operations));

	ts->ops_in.reg = &ts->__reg;
	memcpy(ts->ops_in.reg, ops_ext->reg, sizeof(struct siw_hal_reg));

	ts->ops = &ts->ops_in;

	siw_setup_reg_quirks(ts);

	return ts->ops;
}

/**
 * siw_touch_set() - set touch data
 * @dev: device to use
 * @cmd: set command
 * @buf: data to store
 *
 * Return:
 * On success, the total number of bytes of data stored to device.
 * Otherwise, it returns zero or minus value
 */
int siw_touch_set(struct device *dev, u32 cmd, void *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (!buf)
		t_dev_err(dev, "NULL buf\n");

	mutex_lock(&ts->lock);
	ret = siw_ops_set(ts, cmd, buf);
	mutex_unlock(&ts->lock);

	return ret;
}

/**
 * siw_touch_get() - get touch data
 * @dev: device to use
 * @cmd: set command
 * @buf: data to store
 *
 * Return:
 * On success, the total number of bytes of data loaded from device.
 * Otherwise, it returns zero or minus value
 */
int siw_touch_get(struct device *dev, u32 cmd, void *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (!buf)
		t_dev_err(dev, "NULL buf\n");

	mutex_lock(&ts->lock);
	ret = siw_ops_get(ts, cmd, buf);
	mutex_unlock(&ts->lock);

	return ret;
}

#if defined(__SIW_SUPPORT_PM_WAKEUP)
static void __used siw_touch_init_wake_lock(struct siw_ts *ts)
{
	device_init_wakeup(ts->dev, true);
}

static void __used siw_touch_free_wake_lock(struct siw_ts *ts)
{
	device_init_wakeup(ts->dev, false);
}

void __used siw_touch_core_wake_lock(struct device *dev, int timeout)
{
	if (!timeout) {
		pm_stay_awake(dev);
		return;
	}

	pm_wakeup_event(dev, timeout);
}

void __used siw_touch_core_wake_unlock(struct device *dev)
{
	pm_relax(dev);
}
#else	/* !__SIW_SUPPORT_PM_WAKEUP */
static inline void __used siw_touch_init_wake_lock(struct siw_ts *ts)
{

}

static inline void __used siw_touch_free_wake_lock(struct siw_ts *ts)
{

}

void siw_touch_core_wake_lock(struct device *dev, int timeout)
{

}

void siw_touch_core_wake_unlock(struct device *dev)
{

}
#endif	/* __SIW_SUPPORT_WAKE_LOCK */

int siw_touch_power_state(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	mutex_lock(&ts->power_lock);
	ret = siw_touch_sys_power_state(dev);
	mutex_unlock(&ts->power_lock);

	return ret;
}

int siw_touch_power_lock(struct device *dev, int set)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	mutex_lock(&ts->power_lock);

	if (set) {
		siw_touch_core_wake_lock(dev, 0);
	} else {
		siw_touch_core_wake_unlock(dev);
	}

	ret = siw_touch_sys_power_lock(dev, set);

	mutex_unlock(&ts->power_lock);

	return ret;
}

/**
 * siw_touch_suspend() - touch suspend
 * @dev: device to use
 *
 */
static void siw_touch_suspend(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	t_dev_info(dev, "touch core pm suspend start\n");

	cancel_delayed_work_sync(&ts->init_work);
	cancel_delayed_work_sync(&ts->upgrade_work);

	mutex_lock(&ts->lock);
	siw_touch_report_all_event(ts);
	atomic_set(&ts->state.fb, FB_SUSPEND);
	/* if need skip, return value is not 0 in pre_suspend */
	ret = siw_ops_suspend(ts);
	mutex_unlock(&ts->lock);

	t_dev_info(dev, "touch core pm suspend end(%d)\n", ret);

	if (ret == 1) {
		mod_delayed_work(ts->wq, &ts->init_work, 0);
	}
}

/**
 * siw_touch_resume() - touch resume
 * @dev: device to use
 *
 */
static void siw_touch_resume(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	t_dev_info(dev, "touch core pm resume start\n");

	mutex_lock(&ts->lock);
	atomic_set(&ts->state.fb, FB_RESUME);
	/* if need skip, return value is not 0 in pre_resume */
	ret = siw_ops_resume(ts);
	mutex_unlock(&ts->lock);

	t_dev_info(dev, "touch core pm resume end(%d)\n", ret);

	if (ret == 0) {
		mod_delayed_work(ts->wq, &ts->init_work, 0);
	}
}

static void siw_touch_pm_ops_setup(struct siw_ts *ts)
{
	struct siw_touch_pm_ops *pm_ops = &ts->pm_ops;

	pm_ops->suspend = siw_touch_suspend;
	pm_ops->resume = siw_touch_resume;

	pm_ops->suspend_bus = siw_touch_suspend;
	pm_ops->resume_bus = siw_touch_resume;
}

static int __used __siw_touch_init_pm_none(struct siw_ts *ts, char *title)
{
	t_dev_info(ts->dev, "%s pm - none\n", title);
	return 0;
}
#define siw_touch_init_pm(_ts)		__siw_touch_init_pm_none(_ts, "init")
#define siw_touch_free_pm(_ts)		__siw_touch_init_pm_none(_ts, "free")

static void __used siw_touch_init_locks(struct siw_ts *ts)
{
	t_dev_dbg_base(ts->dev, "touch init locks\n");

	mutex_init(&ts->lock);
	mutex_init(&ts->reset_lock);
	mutex_init(&ts->probe_lock);
	mutex_init(&ts->power_lock);

	siw_touch_init_wake_lock(ts);
}

static void __used siw_touch_free_locks(struct siw_ts *ts)
{
	t_dev_dbg_base(ts->dev, "free locks\n");

	mutex_destroy(&ts->lock);
	mutex_destroy(&ts->reset_lock);
	mutex_destroy(&ts->probe_lock);
	mutex_destroy(&ts->power_lock);

	siw_touch_free_wake_lock(ts);
}

static void siw_touch_initialize(struct siw_ts *ts)
{
	/* lockscreen */
	siw_touch_report_all_event(ts);
}

static void siw_touch_init_work_func(struct work_struct *work)
{
	struct siw_ts *ts =
			container_of(to_delayed_work(work),
						struct siw_ts, init_work);
	struct device *dev = ts->dev;
	int is_probe = (atomic_read(&ts->state.core) == CORE_PROBE);
	int do_fw_upgrade = 0;
	int ret = 0;

	t_dev_info(dev, "%s init work start(%s)\n",
		touch_chip_name(ts), SIW_DRV_VERSION);

	if (is_probe) {
		do_fw_upgrade |= !!(ts->role.use_fw_upgrade);
	}

	mutex_lock(&ts->lock);
	siw_touch_initialize(ts);
	ret = siw_ops_init(ts);
	if (!ret) {
		siw_touch_irq_control(dev, INTERRUPT_ENABLE);
	}
	mutex_unlock(&ts->lock);
	if (ret == -ETDSENTESD) {
		/* boot fail detected, but skip(postpone) fw_upgrade */
		return;
	} else if (ret == -ETDBOOTFAIL) {
		/* boot fail detected, do fw_upgrade */
		do_fw_upgrade |= 0x2;
	} else if (ret < 0) {
	#if defined(__SIW_SUPPORT_INIT_RETRY)
#define INIT_RETRY_DELAY	500

		t_dev_err(dev, "%s init work failed(%d), retry after %d msec.\n",
				touch_chip_name(ts), ret, INIT_RETRY_DELAY);
		siw_touch_qd_init_work_jiffies(ts, INIT_RETRY_DELAY);
	#else	/* __SIW_SUPPORT_INIT_RETRY */
		if (is_probe) {
			t_dev_err(dev, "%s init work failed(%d), try again\n",
				touch_chip_name(ts), ret);
			atomic_set(&ts->state.core, CORE_NORMAL);
			siw_touch_qd_init_work_now(ts);
			return;
		}

		t_dev_err(dev, "%s init work failed, %d\n",
				touch_chip_name(ts), ret);
	#endif	/* __SIW_SUPPORT_INIT_RETRY */
		return;
	}

	if (do_fw_upgrade) {
		t_dev_info(dev, "Touch F/W upgrade triggered(%Xh)\n", do_fw_upgrade);
		siw_touch_qd_upgrade_work_now(ts);
		return;
	}

	atomic_set(&ts->state.core, CORE_NORMAL);

	t_dev_dbg_base(dev, "init work done\n");
}

static int siw_touch_upgrade_work(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int core_state = atomic_read(&ts->state.core);
	int irq_state = atomic_read(&ts->state.irq_enable);
	int ret = 0;

	t_dev_info(dev, "FW upgrade work func\n");

	atomic_set(&ts->state.core, CORE_UPGRADE);
	ts->role.use_fw_upgrade = 0;

	mutex_lock(&ts->lock);
	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	ret = siw_ops_upgrade(ts);
	mutex_unlock(&ts->lock);

	/* init force_upgrade */
	ts->force_fwup = FORCE_FWUP_CLEAR;
	ts->test_fwpath[0] = '\0';

	/* upgrade not granted */
	if (ret == EACCES) {
		atomic_set(&ts->state.core, core_state);
		if (irq_state) {
			siw_touch_irq_control(dev, INTERRUPT_ENABLE);
		}
		return 0;	/* skip reset */
	}

	if (ret < 0) {
		if (ret == -EPERM) {
			t_dev_err(dev, "FW upgrade skipped\n");
		} else {
			t_dev_err(dev, "FW upgrade halted, %d\n", ret);
		}
	}

#if defined(__SIW_SUPPORT_INIT_RETRY)
	if (core_state == CORE_PROBE) {
		atomic_set(&ts->state.core, core_state);
	}
#endif	/* __SIW_SUPPORT_INIT_RETRY */

	return 1;		/* do reset */
}

static void siw_touch_upgrade_work_func(struct work_struct *work)
{
	struct siw_ts *ts =
			container_of(to_delayed_work(work),
						struct siw_ts, upgrade_work);
	struct device *dev = ts->dev;
	int pwr_con = (ts->role.use_fw_pwr_rst) ? 0x80 : 0;
	int ret = 0;

	siw_touch_mon_pause(dev);

	ret = siw_touch_power_state(dev);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade work canceled by power state\n");
		goto out;
	}

	siw_touch_power_lock(dev, 1);
	ret = siw_touch_upgrade_work(ts);
	siw_touch_power_lock(dev, 0);

	if (ret) {
		siw_ops_reset(ts, pwr_con | HW_RESET_ASYNC);
	}

out:
	siw_touch_mon_resume(dev);
}

static int siw_touch_reset(struct siw_ts *ts)
{
	return siw_ops_reset(ts, HW_RESET_ASYNC);
}

#if defined(__SIW_SUPPORT_MON_THREAD)
static int siw_touch_mon_chk_pause(struct siw_ts *ts)
{
	struct siw_mon_thread *mon_thread = &ts->mon_thread;
	int curr_state;

	mutex_lock(&mon_thread->lock);
	curr_state = (atomic_read(&ts->state.mon_ignore)) ?	\
				TS_THREAD_PAUSE : TS_THREAD_ON;
	atomic_set(&mon_thread->state, curr_state);
	mutex_unlock(&mon_thread->lock);

	return (curr_state == TS_THREAD_PAUSE);
}

static int siw_touch_mon_can_handler(struct siw_ts *ts)
{
	if (touch_get_dev_data(ts) == NULL) {
		return 0;
	}

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		return 0;
	}

	return 1;
}

static void siw_touch_mon_work_run_queue(struct siw_ts *ts, int delay)
{
	struct siw_mon_thread *mon_thread = &ts->mon_thread;
	int interval = mon_thread->interval;
	unsigned long timeout;

#if 0
	if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
		return;
	}
#endif

	if (mon_thread->work == NULL) {
		return;
	}

	timeout = msecs_to_jiffies((delay) ? delay : interval);

	queue_delayed_work(ts->wq, mon_thread->work, timeout);
}

static void siw_touch_mon_work_stop_queue(struct siw_ts *ts)
{
	struct siw_mon_thread *mon_thread = &ts->mon_thread;

#if 0
	if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
		return;
	}
#endif

	if (mon_thread->work == NULL) {
		return;
	}

	cancel_delayed_work_sync(mon_thread->work);
}

static void siw_touch_mon_work_func(struct work_struct *work)
{
	struct siw_mon_thread *mon_thread =
			container_of(to_delayed_work(work),
						struct siw_mon_thread, __work);
	struct siw_ts *ts =
			container_of(mon_thread,
						struct siw_ts, mon_thread);
	struct device *dev = ts->dev;
	siw_mon_handler_t handler;
	int ret = 0;

	if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
		return;
	}

	handler = mon_thread->handler;

	if (atomic_read(&ts->state.mon_ignore)) {
		return;
	}

	if (siw_touch_mon_can_handler(ts)) {
		ret = handler(dev, 0);
	}

	if (atomic_read(&ts->state.mon_ignore)) {
		return;
	}

	siw_touch_mon_work_run_queue(ts, 0);
}

static int siw_touch_mon_hold_set(struct device *dev, int pause)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_mon_thread *mon_thread = &ts->mon_thread;
	char *name = (pause) ? "pause" : "resume";
	int prev_state = (pause) ? TS_THREAD_ON : TS_THREAD_PAUSE;

	if (mon_thread->work == NULL) {
		return -EINVAL;
	}

	if (atomic_read(&mon_thread->state) != prev_state) {
		return -EINVAL;
	}

	t_dev_info(dev, "mon thread %s\n", name);

	atomic_set(&ts->state.mon_ignore, !!pause);

	if (pause) {
		siw_touch_mon_work_stop_queue(ts);
	} else {
		siw_touch_mon_work_run_queue(ts, 100);
	}

	return 0;
}

static void siw_touch_mon_hold_post(struct device *dev, int pause)
{
	struct siw_ts *ts = to_touch_core(dev);

	siw_touch_mon_chk_pause(ts);
}

static void siw_touch_mon_hold(struct device *dev, int pause)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_mon_thread *mon_thread = &ts->mon_thread;
	int ret = 0;

	if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
		return;
	}

	mutex_lock(&mon_thread->lock);
	ret = siw_touch_mon_hold_set(dev, pause);
	mutex_unlock(&mon_thread->lock);
	if (ret < 0) {
		return;
	}

	siw_touch_mon_hold_post(dev, pause);
}

void siw_touch_mon_pause(struct device *dev)
{
	siw_touch_mon_hold(dev, 1);
}

void siw_touch_mon_resume(struct device *dev)
{
	siw_touch_mon_hold(dev, 0);
}

static int siw_touch_mon_chk_thread(struct siw_ts *ts)
{
	return (ts->mon_thread.work != NULL);
}

static int siw_touch_mon_set_thread(struct siw_ts *ts)
{
	struct siw_mon_thread *mon_thread = NULL;

	mon_thread = &ts->mon_thread;

	mon_thread->work = &mon_thread->__work;

	INIT_DELAYED_WORK(mon_thread->work, siw_touch_mon_work_func);

	siw_touch_mon_work_run_queue(ts, 0);

	atomic_set(&mon_thread->state, TS_THREAD_ON);

	return 0;
}

static void siw_touch_mon_clr_thread(struct siw_ts *ts)
{
	struct siw_mon_thread *mon_thread = NULL;
	struct device *dev = ts->dev;

	mon_thread = &ts->mon_thread;

	t_dev_info(dev, "stopping mon thread\n");

	siw_touch_mon_work_stop_queue(ts);

	atomic_set(&mon_thread->state, TS_THREAD_OFF);

	mon_thread->work = NULL;

	mutex_destroy(&mon_thread->lock);
}

#define MON_INTERVAL_MIN	500

static int __used siw_touch_mon_init_thread(struct siw_ts *ts)
{
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	struct device *dev = ts->dev;
	struct siw_mon_thread *mon_thread = NULL;
	siw_mon_handler_t handler = NULL;
	int interval = 0;
	int ret = 0;

	if (t_dbg_flag & DBG_FLAG_SKIP_MON_THREAD) {
		ts->flags &= ~TOUCH_USE_MON_THREAD;
	}
	if (t_dbg_flag & DBG_FLAG_TEST_MON_THREAD) {
		ts->flags |= TOUCH_USE_MON_THREAD;
	}

	if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
		goto out;
	}

	if (siw_touch_mon_chk_thread(ts)) {
		goto out;
	}

	mon_thread = &ts->mon_thread;

	mutex_init(&mon_thread->lock);

	if (fquirks->mon_handler) {
		handler = fquirks->mon_handler;
		interval = fquirks->mon_interval;
	} else {
		handler = ts->ops->mon_handler;
		interval = ts->ops->mon_interval;
	}

	if (!handler) {
		t_dev_warn(dev, "No mon_handler defined!\n");
		goto out;
	}

	if (!interval) {
		t_dev_warn(dev, "mon_interval is zero!\n");
		goto out;
	}

	if (interval < MON_INTERVAL_MIN) {
		t_dev_warn(dev, "mon_interval is too short, %d msec\n", interval);
		goto out;
	}

	atomic_set(&mon_thread->state, TS_THREAD_OFF);
	mon_thread->interval = interval;
	mon_thread->handler = handler;

	ret = siw_touch_mon_set_thread(ts);
	if (ret < 0) {
		goto out;
	}

	t_dev_info(dev, "mon thread[%d msec] begins\n", interval);

out:
	return ret;
}

static void __used siw_touch_mon_free_thread(struct siw_ts *ts)
{
	if (!(touch_flags(ts) & TOUCH_USE_MON_THREAD)) {
		return;
	}

	if (!siw_touch_mon_chk_thread(ts)) {
		return;
	}

	siw_touch_mon_clr_thread(ts);
}
#else	/* __SIW_SUPPORT_MON_THREAD */
void siw_touch_mon_pause(struct device *dev)
{

}

void siw_touch_mon_resume(struct device *dev)
{

}

static inline int __used siw_touch_mon_init_thread(struct siw_ts *ts)
{
	return 0;
}

static inline void __used siw_touch_mon_free_thread(struct siw_ts *ts)
{

}
#endif	/* __SIW_SUPPORT_MON_THREAD */

static int __used siw_touch_init_works(struct siw_ts *ts)
{
	ts->wq = create_singlethread_workqueue("touch_wq");
	if (!ts->wq) {
		t_dev_err(ts->dev, "failed to create workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&ts->init_work, siw_touch_init_work_func);
	INIT_DELAYED_WORK(&ts->upgrade_work, siw_touch_upgrade_work_func);

	return 0;
}

static void __used siw_touch_free_works(struct siw_ts *ts)
{
	if (ts->wq) {
		cancel_delayed_work(&ts->upgrade_work);
		cancel_delayed_work(&ts->init_work);

		destroy_workqueue(ts->wq);
		ts->wq = NULL;
	}
}

static irqreturn_t __used siw_touch_irq_handler(int irq, void *dev_id)
{
	struct siw_ts *ts = (struct siw_ts *)dev_id;
	struct device *dev = ts->dev;

	t_dev_dbg_irq(dev, "irq_handler\n");

	if (atomic_read(&ts->state.pm) >= DEV_PM_SUSPEND) {
		t_dev_info(dev, "interrupt in suspend[%d]\n",
				atomic_read(&ts->state.pm));
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

static int _siw_touch_do_irq_thread(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int ret = 0;

	ts->intr_status = 0;

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		goto out;
	}

	ret = siw_ops_irq_handler(ts);
	if (ret < 0) {
		t_dev_dbg_irq(dev, "Err in irq_handler of %s, %d",
				touch_chip_name(ts), ret);
		if (ret == -ERESTART) {
			if (t_dbg_flag & DBG_FLAG_SKIP_IRQ_RESET) {
				return ret;
			}

			siw_touch_reset(ts);
		}
		return ret;
	}

	siw_touch_event_ctrl(ts);

out:
	return ret;
}

static irqreturn_t __used siw_touch_irq_thread(int irq, void *dev_id)
{
	struct siw_ts *ts = (struct siw_ts *)dev_id;
	struct device *dev = ts->dev;
	int ret = 0;

	t_dev_dbg_irq(dev, "irq_thread %d\n", irq);

	if (t_dbg_flag & DBG_FLAG_SKIP_IRQ) {
		goto out;
	}

	if (ts->ops->irq_dbg_handler) {
		ret = ts->ops->irq_dbg_handler(dev);
		goto out;
	}

	mutex_lock(&ts->lock);
	ret = _siw_touch_do_irq_thread(ts);
	mutex_unlock(&ts->lock);

out:
	return IRQ_HANDLED;
}

static int __used siw_touch_verify_pdata(struct siw_ts *ts)
{
	struct siw_touch_operations *ops = ts->ops;

	if (0 ||
		!ops->probe ||
		!ops->remove ||
		!ops->suspend ||
		!ops->resume ||
		!ops->init ||
		!ops->reset ||
		!ops->ic_info ||
		!ops->tc_driving ||
		!ops->chk_status ||
		!ops->irq_handler ||
		!ops->power ||
		!ops->upgrade ||
		0)
		return -EPERM;

	return 0;
}

static struct siw_touch_pdata *siw_touch_probe_common(struct siw_ts *ts)
{
	struct siw_touch_pdata *pdata = NULL;
	struct device *dev = ts->dev;
	int ret = 0;

	pdata = (struct siw_touch_pdata *)ts->pdata;
	if (!pdata) {
		t_dev_err(dev, "NULL core pdata\n");
		goto out;
	}

	t_dev_info(dev, "%s quirks = 0x%08X\n",
			touch_chip_name(ts), (u32)touch_get_quirks(ts));

	atomic_set(&ts->state.core, CORE_EARLY_PROBE);

	if (!ts->ops) {
		t_dev_warn(dev, "%s ops is NULL : default ops selected\n",
				touch_chip_name(ts));
		siw_setup_operations(ts, siw_hal_get_default_ops(0));
	}

	siw_touch_pm_ops_setup(ts);

	ret = siw_ops_early_probe(ts);
	if (ret) {
		t_dev_err(dev, "failed to early_probe, %d\n", ret);
		goto out;
	}

	atomic_set(&ts->state.core, CORE_PROBE);

	ret = siw_touch_verify_pdata(ts);
	if (ret) {
		t_dev_err(dev, "failed to check functions, %d\n", ret);
		goto out;
	}

	ret = siw_touch_parse_data(ts);
	if (ret < 0) {
		t_dev_err(dev, "failed to parse touch data, %d\n", ret);
		goto out;
	}

	ret = siw_touch_bus_init(ts);
	if (ret < 0) {
		t_dev_err(dev, "failed to setup bus, %d\n", ret);
		goto out;
	}

	siw_touch_init_locks(ts);

	ret = siw_touch_init_works(ts);
	if (ret) {
		t_dev_err(dev, "failed to initialize works, %d\n", ret);
		goto out_init_works;
	}

	ret = siw_touch_init_input(ts);
	if (ret) {
		t_dev_err(dev, "failed to register input device, %d\n", ret);
		goto out_init_input;
	}

	ret = siw_touch_init_sysfs(ts);
	if (ret) {
		t_dev_err(dev, "failed to initialize sysfs, %d\n", ret);
		goto out_init_sysfs;
	}

	return pdata;

out_init_sysfs:
	siw_touch_free_input(ts);

out_init_input:
	siw_touch_free_works(ts);

out_init_works:
	siw_touch_free_locks(ts);

	siw_touch_bus_free(ts);

out:
	return NULL;
}

static void siw_touch_remove_common(struct siw_ts *ts)
{
//	struct siw_touch_pdata *pdata = (struct siw_touch_pdata *)ts->pdata;

	siw_touch_free_sysfs(ts);

	siw_touch_free_input(ts);

	siw_touch_free_works(ts);

	siw_touch_free_locks(ts);

	siw_touch_bus_free(ts);
}

static int __siw_touch_probe_init(void *data)
{
	struct siw_ts *ts = data;
	struct device *dev = ts->dev;
	const char *irq_name = NULL;
	int ret = 0;

	ret = siw_ops_probe(ts);
	if (ret) {
		t_dev_err(dev, "failed to probe, %d\n", ret);
		goto out;
	}

	ret = siw_touch_add_sysfs(ts);
	if (ret) {
		t_dev_err(dev, "failed to add sysfs, %d\n", ret);
		goto out_add_sysfs;
	}

	ret = siw_touch_init_pm(ts);
	if (ret) {
		t_dev_err(dev, "failed to init pm\n");
		goto out_init_pm;
	}

	if (ts->is_charger) {
		goto skip_charger;
	}

#if defined(__SIW_TEST_IRQ_OFF)
	ts->irq = 0;
#else	/* __SIW_TEST_IRQ_OFF */
	irq_name = (const char *)touch_drv_name(ts);
	irq_name = (irq_name) ? irq_name : SIW_TOUCH_NAME;
	ret = siw_touch_request_irq(ts,
						siw_touch_irq_handler,
						siw_touch_irq_thread,
						touch_irqflags(ts),
						irq_name);
	if (ret) {
		t_dev_err(dev, "failed to request thread irq(%d), %d\n",
					ts->irq, ret);
		goto out_request_irq;
	}
//	irq_set_irq_type(ts->irq, IRQF_ONESHOT | IRQ_TYPE_EDGE_FALLING);
#endif	/* __SIW_TEST_IRQ_OFF */

	siw_touch_disable_irq(dev, ts->irq);
//	t_dev_dbg_irq(dev, "disable irq until init completed\n");

	ret = siw_touch_mon_init_thread(ts);
	if (ret) {
		t_dev_err(dev, "failed to create thread\n");
		goto out_init_thread;
	}

	siw_touch_qd_init_work_now(ts);

skip_charger:
	ts->init_late_done = 1;

	return 0;

out_init_thread:
	siw_touch_free_irq(ts);

out_request_irq:
	siw_touch_free_pm(ts);

out_init_pm:
	siw_touch_del_sysfs(ts);

out_add_sysfs:
	siw_ops_remove(ts);

out:
	return ret;
}

static void __siw_touch_probe_free(void *data)
{
	struct siw_ts *ts = data;

	if (!ts->init_late_done) {
		return;
	}

	if (ts->is_charger) {
		goto skip_charger;
	}

	cancel_delayed_work_sync(&ts->upgrade_work);
	cancel_delayed_work_sync(&ts->init_work);

	siw_touch_mon_free_thread(ts);

	siw_touch_free_irq(ts);

skip_charger:
	siw_touch_free_pm(ts);

	siw_touch_del_sysfs(ts);

	siw_ops_remove(ts);
}

static int siw_touch_probe_init(void *data)
{
	struct siw_ts *ts = data;
	int ret = 0;

	mutex_lock(&ts->probe_lock);
	ret = __siw_touch_probe_init(data);
	mutex_unlock(&ts->probe_lock);

	return ret;
}

static void siw_touch_probe_free(void *data)
{
	struct siw_ts *ts = data;

	mutex_lock(&ts->probe_lock);
	__siw_touch_probe_free(ts);
	mutex_unlock(&ts->probe_lock);
}

/*
 * [Summary of init_late control]
 *
 * [Activation]
 * <touch_xxxxxxx.c>
 * #define CHIP_FLAGS     (0 |	\
 *                        ...
 *                        TOUCH_USE_PROBE_INIT_LATE |	\
 *                        ...
 *                        0)
 *
 * [Basic - sysfs or notifier]
 * Send echo command to sysfs after probe done
 * $ echo 0x55AA > /sys/device/virtual/input/siw_touch_input/init_late
 * or
 * Use notifier in other driver
 * {
 *     int value = 0x1234;
 *     siw_touch_atomic_notifier_call(LCD_EVENT_TOUCH_INIT_LATE, (void *)&value);
 * }
 *
 * [Ext - auto-execution via work queue, triggered by entry configuration]
 * <touch_xxxxxxx.c>
 * static const struct siw_touch_pdata chip_pdata = {
 *        ...
 *        //init_late_try = 2
 *        //init_late_time = 1000 msec
 *        .init_late_time = (2<<24) | 1000,
 * };
 *
 * [Ext - sysfs & work queue combination]
 * This case doesn't yet support notifier control.
 * For 'init_late_try = 2' & 'init_late_time = 1000 msec'
 * $ echo 0x55AA 1000 2 > /sys/device/virtual/input/siw_touch_input/init_late
 *
 */
enum {
	INIT_LATE_RETRY_POS		= 24,
	INIT_LATE_DELAY_MASK	= 0x00FFFFFF,
	/* */
	INIT_LATE_SIG_WQ		= 0x5A5A,
	INIT_LATE_SIG_DONE		= 0xAA55,
};

static void siw_touch_init_late_work_func(struct work_struct *work)
{
	struct siw_ts *ts =
			container_of(to_delayed_work(work),
						struct siw_ts, init_late_work);
	struct device *dev = ts->dev;
	int sig = 0;
	int retry = 0;
	int ret = 0;

	mutex_lock(&ts->lock);

	if (ts->init_late == NULL) {
		goto out;
	}

	sig = ts->init_late_sig;
	retry = ts->init_late_retry;

	ret = siw_touch_init_late(ts, sig);
	if (ret < 0) {
		goto out;
	}

	t_dev_info(ts->dev, "init_late_work done\n");

	retry = 0;

out:
	if (retry) {
		t_dev_info(dev, "retry init_late queue(%d)\n", retry);

		ts->init_late_retry--;

		queue_delayed_work(ts->wq,
			&ts->init_late_work,
			msecs_to_jiffies(ts->init_late_time));
	}

	ts->init_late_run = !!retry;

	mutex_unlock(&ts->lock);
}

static void siw_touch_init_late_work_run(struct siw_ts *ts, int delay)
{
	unsigned long queue_delay = 0;

	ts->init_late_run = 1;

	queue_delay = msecs_to_jiffies((delay) ? delay : ts->init_late_time);

	queue_delayed_work(ts->wq, &ts->init_late_work, queue_delay);

	t_dev_info(ts->dev,
		"init_late_work triggered, %d msec(%d, %Xh)\n",
		ts->init_late_time, ts->init_late_retry, ts->init_late_sig);
}

#if defined(__SIW_SUPPORT_PROBE_POST_RETRY)
#define LATE_WORK_SET_DELAY		200
#else	/* __SIW_SUPPORT_PROBE_POST_RETRY */
#define LATE_WORK_SET_DELAY		0
#endif	/* __SIW_SUPPORT_PROBE_POST_RETRY */

static int siw_touch_init_late_work_set(struct siw_ts *ts)
{
	mutex_lock(&ts->lock);

	if (t_dbg_flag & DBG_FLAG_SKIP_INIT_LATE_WORK) {
		ts->init_late_sig = 0;
		ts->init_late_retry = 0;
		ts->init_late_time = 0;
	}
	if (t_dbg_flag & DBG_FLAG_TEST_INIT_LATE_WORK) {
		ts->init_late_retry = 0;
		ts->init_late_time = 5000;
	}

	INIT_DELAYED_WORK(&ts->init_late_work, siw_touch_init_late_work_func);

	if (!ts->init_late_time) {
		goto out;
	}

	ts->init_late_sig = INIT_LATE_SIG_WQ;

	siw_touch_init_late_work_run(ts, LATE_WORK_SET_DELAY);

out:
	mutex_unlock(&ts->lock);

	return 0;
}

static void siw_touch_init_late_work_clr(struct siw_ts *ts)
{
	if (!(touch_flags(ts) & TOUCH_USE_PROBE_INIT_LATE)) {
		return;
	}

	if (!ts->init_late_time) {
		return;
	}

	t_dev_info(ts->dev, "init_late_work canceled\n");
	cancel_delayed_work_sync(&ts->init_late_work);
}

static int siw_touch_probe_post(struct siw_ts *ts)
{
#if defined(__SIW_SUPPORT_PROBE_POST_RETRY)
#define PROBE_POST_RETRY_DELAY	500

	ts->flags |= TOUCH_USE_PROBE_INIT_LATE;
	ts->init_late_retry = ~0;
	if (!ts->init_late_time) {
		ts->init_late_time = PROBE_POST_RETRY_DELAY;
	}
#else	/* __SIW_SUPPORT_PROBE_POST_RETRY */
	ts->init_late_retry = ts->pdata->init_late_time>>INIT_LATE_RETRY_POS;
	ts->init_late_time = ts->pdata->init_late_time & INIT_LATE_DELAY_MASK;
#endif	/* __SIW_SUPPORT_PROBE_POST_RETRY */

	if (t_dbg_flag & DBG_FLAG_SKIP_INIT_LATE) {
		ts->flags &= ~TOUCH_USE_PROBE_INIT_LATE;
	}
	if (t_dbg_flag & DBG_FLAG_TEST_INIT_LATE) {
		ts->flags |= TOUCH_USE_PROBE_INIT_LATE;
	}

	if (touch_flags(ts) & TOUCH_USE_PROBE_INIT_LATE) {
		/*
		 * Postpone actual init control
		 * This is related to LCD_EVENT_TOUCH_INIT_LATE and
		 * shall be controlled by MIPI via notifier
		 */
		ts->init_late = siw_touch_probe_init;

		siw_touch_init_late_work_set(ts);
		return 0;
	}

	ts->init_late = NULL;

	touch_msleep(200);

	return siw_touch_probe_init(ts);
}

static void siw_touch_remove_post(struct siw_ts *ts)
{
	siw_touch_init_late_work_clr(ts);

	siw_touch_probe_free(ts);
}

int siw_touch_probe(struct siw_ts *ts)
{
	struct siw_touch_pdata *pdata = NULL;
	struct device *dev = ts->dev;
	int ret = 0;

	t_dev_info(dev, "SiW Touch Probe\n");

	ts->is_charger = !!(siw_touch_get_boot_mode() == SIW_TOUCH_CHARGER_MODE);

	if (ts->is_charger) {
		t_dev_info(dev, "Probe - Charger mode\n");
	}

	pdata = siw_touch_probe_common(ts);
	if (!pdata) {
		return -EINVAL;
	}

	ret = siw_touch_probe_post(ts);
	if (ret) {
		goto out_probe_post;
	}

	t_dev_info(dev, "probe(%s) done%s\n",
		(ts->is_charger) ? "charger" : "normal",
		(touch_flags(ts) & TOUCH_USE_PROBE_INIT_LATE) ? " (init_late on)": "");

	return 0;

out_probe_post:
	siw_touch_remove_common(ts);

	return ret;
}

int siw_touch_remove(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	if (ts->is_charger) {
		t_dev_info(dev, "Remove - Charger mode\n");
	}

	siw_touch_remove_post(ts);

	siw_touch_remove_common(ts);

	t_dev_info(dev, "SiW Touch Removed\n");

	return 0;
}

int siw_touch_shutdown(struct siw_ts *ts)
{
	struct device *dev = NULL;

	if (ts == NULL)
		return 0;

	dev = ts->dev;

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	siw_touch_init_late_work_clr(ts);

	cancel_delayed_work_sync(&ts->upgrade_work);
	cancel_delayed_work_sync(&ts->init_work);

	t_dev_info(dev, "SiW Touch shutdown\n");

	return 0;
}

int siw_touch_init_late(struct siw_ts *ts, int value)
{
	struct device *dev = ts->dev;
	int ret = 0;

	if (!value) {
		goto out;
	}

	if (!(touch_flags(ts) & TOUCH_USE_PROBE_INIT_LATE)) {
		t_dev_info(dev, "init_late not enabled\n");
		goto out;
	}

	if (ts->init_late == NULL) {
		goto out;
	}

	t_dev_info(dev, "trigger init_late(%Xh)\n", value);

	ret = siw_touch_power_state(dev);
	if (ret < 0) {
		t_dev_err(dev, "init_late canceled by power state\n");
		goto out;
	}

	siw_touch_power_lock(dev, 1);
	ret = ts->init_late(ts);
	siw_touch_power_lock(dev, 0);
	if (ret < 0) {
		atomic_set(&ts->state.core, CORE_PROBE);
		t_dev_err(dev, "init_late failed, %d\n", ret);
		goto out;
	}

	t_dev_info(dev, "init_late done(%Xh)\n", value);

	ts->init_late = NULL;

out:
	return ret;
}

int siw_touch_init_late_queue(struct device *dev,
		int sig, int time, int retry)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (!(touch_flags(ts) & TOUCH_USE_PROBE_INIT_LATE)) {
		t_dev_info(dev, "init_late not enabled\n");
		return 0;
	}

	mutex_lock(&ts->lock);

	if (ts->init_late == NULL) {
		t_dev_err(dev, "queue init_late: no init_late\n");
		ret = -EFAULT;
		goto out;
	}

	if (ts->init_late_run) {
		t_dev_warn(dev, "queue init_late: already activated\n");
		ret = -EPERM;
		goto out;
	}

	if (!sig) {
		t_dev_err(dev, "queue init_late: zero sig\n");
		ret = -EINVAL;
		goto out;
	}

	t_dev_info(dev, "queue init_late: sig %Xh, time %d, retry %d\n",
		sig, time, retry);

	ts->init_late_sig = sig;
	ts->init_late_retry = retry;
	ts->init_late_time = time;

	siw_touch_init_late_work_run(ts, 0);

out:
	mutex_unlock(&ts->lock);

	return ret;
}


#if defined(CONFIG_TOUCHSCREEN_SIWMON) || defined(CONFIG_TOUCHSCREEN_SIWMON_MODULE)

struct siw_mon_operations *siw_mon_ops;
EXPORT_SYMBOL(siw_mon_ops);

int siw_mon_register(struct siw_mon_operations *ops)
{
	if (siw_mon_ops)
		return -EBUSY;

	siw_mon_ops = ops;
	t_pr_dbg(DBG_BASE, "siw mon ops assigned\n");
	mb();
	return 0;
}
EXPORT_SYMBOL_GPL(siw_mon_register);

void siw_mon_deregister(void)
{
	if (siw_mon_ops == NULL) {
		t_pr_err("monitor was not registered\n");
		return;
	}
	siw_mon_ops = NULL;
	t_pr_dbg(DBG_BASE, "siw mon ops released\n");
	mb();
}
EXPORT_SYMBOL_GPL(siw_mon_deregister);

#endif	/* CONFIG_TOUCHSCREEN_SIWMON */

__siw_setup_u32("siw_pr_dbg_mask=", siw_setup_pr_dbg_mask, t_pr_dbg_mask);
__siw_setup_u32("siw_dev_dbg_mask=", siw_setup_dev_dbg_mask, t_dev_dbg_mask);



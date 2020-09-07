/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * SiW touch core driver
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#ifndef __SIW_TOUCH_H
#define __SIW_TOUCH_H

#include "siw_touch_cfg.h"

#include <linux/kernel.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/atomic.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/pinctrl-state.h>
#include <linux/pinctrl/consumer.h>

#if defined(__SIW_SUPPORT_PM_WAKEUP)
#include <linux/pm_wakeup.h>
#endif	/* __SIW_SUPPORT_PM_WAKEUP */

#include "siw_touch_hal_reg.h"

#include "siw_touch_dbg.h"


#define SIW_DRV_VERSION		"vA0.01b-f2"

enum {
	SIW_TOUCH_NORMAL_MODE	= 0,
	SIW_TOUCH_CHARGER_MODE	= 1,
};

enum {
	SIW_TOUCH_BUF_MARGIN		= 32,
	SIW_TOUCH_MAX_BUF_SIZE		= (32<<10),	//(64<<10),
	SIW_TOUCH_MAX_BUF_IDX		= 4,
	/* */
	SIW_TOUCH_MAX_XFER_COUNT	= 10,
};

enum _SIW_BUS_IF {
	BUS_IF_I2C = 0,
	BUS_IF_SPI,
	BUS_IF_MAX,
};

enum {
	DRIVER_FREE = 0,
	DRIVER_INIT = 1,
};

enum {
	POWER_OFF = 0,
	POWER_SLEEP,
	POWER_WAKE,
	POWER_ON,
	POWER_SLEEP_STATUS,
	POWER_HW_RESET,
};

enum {
	DEV_PM_RESUME = 0,
	DEV_PM_SUSPEND,
};

enum {
	FB_RESUME = 0,
	FB_SUSPEND,
};

/* Deep Sleep or not */
enum {
	IC_NORMAL = 0,
	IC_DEEP_SLEEP,
};

enum { /* Command lists */
	CMD_VERSION,
	CMD_ATCMD_VERSION,
};

enum {
	INTERRUPT_DISABLE = 0,
	INTERRUPT_ENABLE,
};

enum {
	CORE_NONE = 0,
	CORE_EARLY_PROBE,
	CORE_PROBE,
	CORE_CHARGER_LOGO,
	CORE_MFTS,
	CORE_UPGRADE,
	CORE_NORMAL,
};

struct state_info {
	atomic_t core;
	atomic_t pm;
	atomic_t fb;
	atomic_t sleep;
	atomic_t irq_enable;
	atomic_t hw_reset;
	atomic_t mon_ignore;
};

enum {
	PWR_TYPE_NA		= 0,
	PWR_TYPE_REG,	//regulator
	PWR_TYPE_GPIO,
	PWR_TYPE_MAX,
	PWR_TYPE_MIN	= (PWR_TYPE_NA + 1),
};

enum {
	PWR_IDX_NA		= 0,
	PWR_IDX_VDD,
	PWR_IDX_VIO,
	PWR_IDX_MAX,
	PWR_IDX_MIN		= (PWR_IDX_NA + 1),
};

struct touch_pwr_cfg {
	int type;
	int pin;
	char *name;
	void *reg;
	int voltage;
	int load;
};

struct touch_pwr_con {
	char *name;
	struct touch_pwr_cfg *cfg;
	int (*init)(struct device *dev, struct touch_pwr_con *con);
	int (*free)(struct device *dev, struct touch_pwr_con *con);
	void (*ctrl)(struct device *dev, int value, struct touch_pwr_con *con);
};

struct touch_pins {
	int reset_pin;
	int reset_pin_pol;
	int irq_pin;
	/* */
	int vdd_pin;
	int vio_pin;
	struct touch_pwr_cfg vdd_cfg;
	struct touch_pwr_cfg vio_cfg;
};

struct touch_device_caps {
	u32 max_x;
	u32 max_y;
	u32 max_pressure;
	u32 max_width;
	u32 max_orientation;
	u32 max_id;
	int mt_slots_flags;
//	u32 button_support;
//	u32 number_of_button;
//	u32 button_name[MAX_BUTTON];
	/* */
	u32 hw_reset_delay;
	u32 sw_reset_delay;
};

struct touch_operation_role {
	u32 use_cancel_opt;
	u32 use_palm_opt;
	u32 use_firmware;
	u32 use_fw_upgrade;
	u32 use_fw_pwr_rst;
	u32 use_fw_ver_diff;
	u32 use_fw_skip_pid;
};

struct touch_pinctrl {
	struct pinctrl *ctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *suspend;
};

struct touch_data {
	u16 id;
	u16 x;
	u16 y;
	u16 width_major;
	u16 width_minor;
	s16 orientation;
	u16 pressure;
	/* finger, palm, pen, glove, hover */
	u16 type;
	u16 event;
};

typedef int (*siw_mon_handler_t)(struct device *dev, u32 opt);

struct siw_touch_operations {
	/* Register Map */
	struct siw_hal_reg *reg;
	/* Func. */
	int (*early_probe)(struct device *dev);
	int (*probe)(struct device *dev);
	int (*remove)(struct device *dev);
	int (*suspend)(struct device *dev);
	int (*resume)(struct device *dev);
	int (*init)(struct device *dev);
	int (*reset)(struct device *dev, int ctrl);
	int (*ic_info)(struct device *dev);
	int (*tc_con)(struct device *dev, u32 code, void *param);
	int (*tc_driving)(struct device *dev, int mode);
	int (*chk_status)(struct device *dev);
	int (*irq_dbg_handler)(struct device *dev);
	int (*irq_handler)(struct device *dev);
	int (*power)(struct device *dev, int power_mode);
	int (*upgrade)(struct device *dev);
	int (*set)(struct device *dev, u32 cmd, void *buf);
	int (*get)(struct device *dev, u32 cmd, void *buf);
	/* */
	int (*sysfs)(struct device *dev, int on_off);
	/* */
	siw_mon_handler_t mon_handler;
	int mon_interval;
	/* */
	int (*prd_sysfs)(struct device *dev, int on_off);
};

enum {
	MON_FLAG_RST_ONLY	= (1<<8),
};

struct siw_touch_bus_info {
	u32 bus_type;
	u32 buf_size;
	/* */
	u32 chip_select;
	u32 spi_mode;
	u32 bits_per_word;
	u32 max_freq;
	/* */
	int	bus_tx_hdr_size;
	int	bus_rx_hdr_size;
	int bus_tx_dummy_size;
	int bus_rx_dummy_size;
};

struct siw_touch_buf {
	u8 *buf;
	dma_addr_t dma;
	int size;
};

struct siw_touch_second_screen {
	int bound_i;
	int bound_j;
};

struct siw_touch_fquirks {	//function quirks
	int (*parse_dts)(void *data);
	/* */
	int (*gpio_init)(struct device *dev, int pin, const char *name);
	int (*gpio_free)(struct device *dev, int pin);
	int (*gpio_dir_input)(struct device *dev, int pin);
	int (*gpio_dir_output)(struct device *dev, int pin, int value);
	int (*gpio_set_pull)(struct device *dev, int pin, int value);
	int (*gpio_set_value)(struct device *dev, int pin, int value);
	int (*gpio_get_value)(struct device *dev, int pin);
	/* */
	int (*power_init)(struct device *dev);
	int (*power_free)(struct device *dev);
	int (*power_vdd)(struct device *dev, int value);
	int (*power_vio)(struct device *dev, int value);
	/* */
	int (*init_pre)(struct device *dev);
	int (*init_config_pre)(struct device *dev);
	int (*init_config_post)(struct device *dev);
	/* */
	int (*fwup_check)(struct device *dev, u8 *fw_buf);
	int (*fwup_upgrade)(struct device *dev, u8 *fw_buf, int fw_size, int retry);
	/* */
	int (*mon_handler)(struct device *dev, u32 opt);
	int mon_interval;
	/* */
	int (*boot_status)(struct device *dev, u32 *boot_st);
	/* */
	int (*flash_wp)(struct device *dev, int wp);
	/* */
	int (*gpio_init_reset)(struct device *dev);
	int (*gpio_free_reset)(struct device *dev);
	int (*gpio_set_reset)(struct device *dev, int val);
	/* */
	int (*gpio_init_irq)(struct device *dev);
	int (*gpio_free_irq)(struct device *dev);
	/* */
	int (*tc_driving)(struct device *dev, int mode);
	int (*tc_driving_post)(struct device *dev, int mode);
	int (*tc_driving_end)(struct device *dev, int mode);
	/* __SIW_SUPPORT_ALIVE_DETECTION */
	int (*alive_setup)(struct device *dev);
	int (*alive_pause_set)(struct device *dev, int level);
	int (*alive_check)(struct device *dev, int irq_type);
	int (*alive_mon_init)(struct device *dev);
	int (*alive_mon_free)(struct device *dev);
	int (*alive_mon_run)(struct device *dev, int restart);
	int (*alive_mon_stop)(struct device *dev);
	int (*alive_mon_check)(struct device *dev, int irq_type);
	/* */
	void *sysfs_group;
};

enum {
	TOUCH_IRQ_NONE			= 0,
	TOUCH_IRQ_FINGER		= BIT(0),
	//
	TOUCH_IRQ_ERROR			= BIT(31),
};

enum {
	ABS_MODE = 0,
};

struct siw_touch_fw_bin {
	u8 *fw_data;
	int fw_size;
};

struct siw_touch_pdata {
	/* Config. */
	char *compatible;		//compatible name
	char *chip_id;			//chip id(fixed)
	char *chip_name;		//chip name
	char *drv_name;			//driver name
	char *idrv_name;		//input driver name
	struct module *owner;
	u32 max_finger;
	u32 chip_type;
	u32 mode_allowed;
	u32 fw_size;		//Pure F/W size, not include config data(1K)

	u32 flags;
	unsigned long irqflags;

	unsigned long quirks;

	struct siw_touch_bus_info bus_info;

#if defined(__SIW_CONFIG_OF)
	const struct of_device_id *of_match_table;
#else
	struct touch_pins pins;
	struct touch_device_caps caps;
#endif

	/* Input Device ID */
	struct input_id i_id;

	/* Hal operations */
	struct siw_touch_operations *ops;

	void *reg_quirks;

	struct siw_touch_fquirks fquirks;

	void *fw_bin;

	/*
	 * Auto-execution for ts->init_late_work
	 * 0: disabled, non-zero: auto execution time delay(msec)
	 * To ts->init_late_time
	 *
	 * [31:24] - init_late retry count for failure case (0 means no retry)
	 * [23:00] - init_late delay time (msec)
	 */
	int init_late_time;
};

struct siw_touch_chip_data {
	const struct siw_touch_pdata *pdata;
	void *bus_drv;
};

static inline unsigned long pdata_get_quirks(struct siw_touch_pdata *pdata)
{
	return pdata->quirks;
}

static inline unsigned long pdata_test_quirks(struct siw_touch_pdata *pdata,
			unsigned long quirk_bit)
{
	return (pdata_get_quirks(pdata) & quirk_bit);
}

static inline char *pdata_compatible(struct siw_touch_pdata *pdata)
{
	return pdata->compatible;
}

static inline char *pdata_chip_id(struct siw_touch_pdata *pdata)
{
	return pdata->chip_id;
}

static inline char *pdata_chip_name(struct siw_touch_pdata *pdata)
{
	return pdata->chip_name;
}

static inline char *pdata_drv_name(struct siw_touch_pdata *pdata)
{
	return pdata->drv_name;
}

static inline char *pdata_idrv_name(struct siw_touch_pdata *pdata)
{
	return pdata->idrv_name;
}

static inline u32 pdata_max_finger(struct siw_touch_pdata *pdata)
{
	return (pdata->max_finger) ? pdata->max_finger : MAX_FINGER;
}

static inline u32 pdata_chip_type(struct siw_touch_pdata *pdata)
{
	return pdata->chip_type;
}

static inline u32 pdata_mode_allowed(struct siw_touch_pdata *pdata, u32 mode)
{
	return (pdata->mode_allowed & BIT(mode));
}

static inline u32 pdata_fw_size(struct siw_touch_pdata *pdata)
{
	return pdata->fw_size;
}

static inline u32 pdata_flags(struct siw_touch_pdata *pdata)
{
	return pdata->flags;
}

static inline unsigned long pdata_irqflags(struct siw_touch_pdata *pdata)
{
	return pdata->irqflags;
}

static inline u32 pdata_bus_type(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.bus_type;
}

static inline u32 pdata_buf_size(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.buf_size;
}

static inline u32 pdata_chip_select(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.chip_select;
}

static inline u32 pdata_spi_mode(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.spi_mode;
}

static inline u32 pdata_bits_per_word(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.bits_per_word;
}

static inline u32 pdata_max_freq(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.max_freq;
}

static inline int pdata_tx_hdr_size(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.bus_tx_hdr_size;
}

static inline int pdata_rx_hdr_size(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.bus_rx_hdr_size;
}

static inline int pdata_tx_dummy_size(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.bus_tx_dummy_size;
}

static inline int pdata_rx_dummy_size(struct siw_touch_pdata *pdata)
{
	return pdata->bus_info.bus_rx_dummy_size;
}

static inline struct siw_touch_fquirks *pdata_fquirks(
							struct siw_touch_pdata *pdata)
{
	return &pdata->fquirks;
}

static inline struct siw_touch_fw_bin *pdata_fw_bin(
								struct siw_touch_pdata *pdata)
{
	return pdata->fw_bin;
}

enum {
	TS_THREAD_OFF = 0,
	TS_THREAD_ON,
	TS_THREAD_PAUSE,
	/* */
	MON_INTERVAL_DEFAULT = 5000,	//msec
};

struct siw_mon_thread {
	struct delayed_work __work;
	struct delayed_work *work;
	atomic_t state;
	int interval;
	siw_mon_handler_t handler;
	struct mutex lock;
};

enum {
	DEFAULT_NAME_SZ = PATH_MAX,
};

struct siw_touch_pm_ops {
	void (*suspend)(struct device *dev);
	void (*resume)(struct device *dev);
	/* __SIW_CONFIG_PM_BUS */
	void (*suspend_bus)(struct device *dev);
	void (*resume_bus)(struct device *dev);
};

struct siw_ts {
//	struct platform_device *pdev;

	size_t addr;

	char *chip_id;
	char *chip_name;		//chip name
	char *drv_name; 		//driver name
	char *idrv_name;		//input driver name
	int max_finger;
	int chip_type;
	u32 mode_allowed;

	int irq;
	unsigned long irqflags;
	unsigned long irqflags_curr;

	void *bus_dev;				/* i2c or spi */
	struct device *dev;			/* client device : i2c->dev or spi->dev */
	void *dev_data;				/* chip */

	struct input_dev *input;
	struct siw_touch_pdata *pdata;

	struct kobject kobj;

	struct state_info state;

	struct touch_pins pins;
	struct touch_device_caps caps;
	struct touch_operation_role role;
	struct touch_pinctrl pinctrl;

	struct touch_pwr_con pwr_con_vdd;
	struct touch_pwr_con pwr_con_vio;

	u32 intr_status;
	u16 event_cnt_finger;
	/* */
	u32 new_mask;
	u32 old_mask;
	int tcount;
	struct touch_data tdata[MAX_FINGER];
	int is_cancel;

	int def_fwcnt;
	const char *def_fwpath[4];
	char test_fwpath[DEFAULT_NAME_SZ];
	const char *panel_spec;
	const char *panel_spec_mfts;
	u32 force_fwup;
#define _FORCE_FWUP_CLEAR		0
#define _FORCE_FWUP_ON			(1<<0)
#define _FORCE_FWUP_SYS_SHOW	(1<<2)
#define _FORCE_FWUP_SYS_STORE	(1<<3)
/* */
#define _FORCE_FWUP_SKIP_PID	(1<<7)

	/* __SIW_SUPPORT_PRD */
	const char *prd_in_file_path;
	const char *prd_in_file_m_path;
	const char *prd_out_file_path;
	const char *prd_out_file_mo_aat_path;
	const char *prd_out_file_mo_mfo_path;
	const char *prd_out_file_mo_mfl_path;
	const char *prd_out_file_mo_mcv_path;
	/* */

	int buf_size;
	struct siw_touch_buf tx_buf[SIW_TOUCH_MAX_BUF_IDX];
	struct siw_touch_buf rx_buf[SIW_TOUCH_MAX_BUF_IDX];
	int tx_buf_idx;
	int rx_buf_idx;

	struct mutex lock;
	struct mutex reset_lock;
	struct mutex probe_lock;
	struct mutex power_lock;
	struct workqueue_struct *wq;
	struct delayed_work init_work;
	struct delayed_work upgrade_work;

	struct siw_touch_pm_ops pm_ops;

	struct siw_mon_thread mon_thread;

	int vdd_id;
	int vdd_vol;
	int vio_id;
	int vio_vol;

	void *prd;

	//__SIW_SUPPORT_MISC
	void *misc;

	u32 flags;
#define _TOUCH_BUS_USE_DMA			(1UL<<7)

#define _TOUCH_USE_MON_THREAD		(1UL<<8)
#define _TOUCH_USE_PINCTRL			(1UL<<9)
#define _TOUCH_USE_PWRCTRL			(1UL<<10)
//__SIW_CONFIG_CLOCK_RECOVERY && CONFIG_PINCTRL
#define _TOUCH_USE_CR_PINCTRL		(1UL<<11)	/* for Clock Recovery */

#define _TOUCH_USE_VBLANK			(1UL<<12)

#define _TOUCH_USE_INPUT_PARENT		(1UL<<15)

#define _TOUCH_USE_DRV_NAME_SYSFS	(1UL<<17)
#define _TOUCH_USE_FW_BINARY		(1UL<<18)

#define _TOUCH_USE_PROBE_INIT_LATE	(1UL<<24)

#define _TOUCH_SKIP_RESET_PIN		(1UL<<29)

#define _TOUCH_IGNORE_DT_FLAGS		(1UL<<31)

	/* Input Device ID */
	struct input_id i_id;

	/* Hal operations */
	struct siw_touch_operations *ops_ext;
	struct siw_touch_operations ops_in;
	struct siw_touch_operations *ops;
	struct siw_hal_reg __reg;

	/* */
	int (*bus_init)(struct device *dev);
	int (*bus_read)(struct device *dev, void *msg);
	int (*bus_write)(struct device *dev, void *msg);
	int	bus_tx_hdr_size;
	int	bus_rx_hdr_size;
	int bus_tx_dummy_size;
	int bus_rx_dummy_size;

	/* */
	atomic_t recur_chk;

	/* */
	int is_charger;

	/* */
	int (*init_late)(void *data);
	int init_late_done;
	int init_late_run;
	int init_late_sig;
	int init_late_retry;
	int init_late_time;
	struct delayed_work init_late_work;
};

enum {
	FORCE_FWUP_CLEAR		= _FORCE_FWUP_CLEAR,
	FORCE_FWUP_ON			= _FORCE_FWUP_ON,
	FORCE_FWUP_SYS_SHOW		= _FORCE_FWUP_SYS_SHOW,
	FORCE_FWUP_SYS_STORE	= _FORCE_FWUP_SYS_STORE,
	/* */
	FORCE_FWUP_SKIP_PID		= _FORCE_FWUP_SKIP_PID,
};

enum {
	TOUCH_BUS_USE_DMA			= _TOUCH_BUS_USE_DMA,
	/* */
	TOUCH_USE_MON_THREAD		= _TOUCH_USE_MON_THREAD,
	TOUCH_USE_PINCTRL			= _TOUCH_USE_PINCTRL,
	TOUCH_USE_PWRCTRL			= _TOUCH_USE_PWRCTRL,
	//__SIW_CONFIG_CLOCK_RECOVERY && CONFIG_PINCTRL
	TOUCH_USE_CR_PINCTRL		= _TOUCH_USE_CR_PINCTRL,
	/* */
	TOUCH_USE_VBLANK			= _TOUCH_USE_VBLANK,
	TOUCH_USE_INPUT_PARENT		= _TOUCH_USE_INPUT_PARENT,
	/* */
	TOUCH_USE_DRV_NAME_SYSFS	= _TOUCH_USE_DRV_NAME_SYSFS,
	TOUCH_USE_FW_BINARY			= _TOUCH_USE_FW_BINARY,
	/* */
	TOUCH_USE_PROBE_INIT_LATE	= _TOUCH_USE_PROBE_INIT_LATE,
	/* */
	TOUCH_SKIP_RESET_PIN		= _TOUCH_SKIP_RESET_PIN,
	/* */
	TOUCH_IGNORE_DT_FLAGS		= _TOUCH_IGNORE_DT_FLAGS,
};


/* goes to siw_touch_init_work_func */
#define __siw_touch_qd_init_work(_ts, _delay)	\
		queue_delayed_work(_ts->wq, &_ts->init_work, _delay)
/* goes to siw_touch_upgrade_work_func */
#define __siw_touch_qd_upgrade_work(_ts, _delay)	\
		queue_delayed_work(_ts->wq, &_ts->upgrade_work, _delay)

#define siw_touch_qd_init_work_now(_ts)	\
		__siw_touch_qd_init_work(_ts, 0)
#define siw_touch_qd_init_work_jiffies(_ts, _jiffies)	\
		__siw_touch_qd_init_work(_ts, msecs_to_jiffies(_jiffies))
#define siw_touch_qd_init_work_sw(_ts)	\
		__siw_touch_qd_init_work(_ts,	\
			msecs_to_jiffies(_ts->caps.sw_reset_delay))
#define siw_touch_qd_init_work_hw(_ts)	\
		__siw_touch_qd_init_work(_ts,	\
			msecs_to_jiffies(_ts->caps.hw_reset_delay))

#define siw_touch_qd_upgrade_work_now(_ts)	\
		__siw_touch_qd_upgrade_work(_ts, 0)
#define siw_touch_qd_upgrade_work_jiffies(_ts, _jiffies)	\
		__siw_touch_qd_upgrade_work(_ts, msecs_to_jiffies(_jiffies))

#ifndef __BIN_ATTR	/* for low version compatibility */
#define __BIN_ATTR(_name, _mode, _read, _write, _size) {		\
	.attr = { .name = __stringify(_name), .mode = _mode },		\
	.read   = _read,											\
	.write  = _write,											\
	.size   = _size,											\
}
#endif

struct siw_touch_attribute {
	struct attribute attr;
	ssize_t (*show)(struct device *dev, char *buf);
	ssize_t (*store)(struct device *dev, const char *buf, size_t count);
};

#define __TOUCH_ATTR(_name, _attr, _show, _store)		\
			struct siw_touch_attribute touch_attr_##_name	\
			= __ATTR(_name, _attr, _show, _store)

#define __TOUCH_BIN_ATTR(_name, _attr, _show, _store, _size)	\
			struct bin_attribute touch_bin_attr_##_name	\
			= __BIN_ATTR(_name, _attr, _show, _store, _size)

#if defined(__SIW_ATTR_PERMISSION_ALL)
#define __TOUCH_DEFAULT_PERM	(S_IRUGO | S_IWUGO)
#else
#define __TOUCH_DEFAULT_PERM	(S_IRUGO | S_IWUSR | S_IWGRP)
#endif

#define TOUCH_ATTR(_name, _show, _store)	\
			__TOUCH_ATTR(_name, __TOUCH_DEFAULT_PERM, _show, _store)

#define TOUCH_BIN_ATTR(_name, _show, _store, _size)	\
			__TOUCH_BIN_ATTR(_name, __TOUCH_DEFAULT_PERM, _show, _store, _size)


static inline void touch_set_dev_data(struct siw_ts *ts, void *data)
{
//	t_dev_info(ts->dev, "data : 0x%P\n", data);
	ts->dev_data = data;
}

static inline void *touch_get_dev_data(struct siw_ts *ts)
{
	return ts->dev_data;
}

static inline struct siw_ts *to_touch_core(struct device *dev)
{
	return dev ? (struct siw_ts *)dev_get_drvdata(dev) : NULL;
}

static inline unsigned long touch_get_quirks(struct siw_ts *ts)
{
	return pdata_get_quirks(ts->pdata);
}

static inline unsigned long touch_test_quirks(struct siw_ts *ts,
			unsigned long quirk_bit)
{
	return pdata_test_quirks(ts->pdata, quirk_bit);
}

static inline char *touch_chip_id(struct siw_ts *ts)
{
	return ts->chip_id;
}

static inline char *touch_chip_name(struct siw_ts *ts)
{
	return ts->chip_name;
}

static inline char *touch_drv_name(struct siw_ts *ts)
{
	return ts->drv_name;
}

static inline char *touch_idrv_name(struct siw_ts *ts)
{
	return ts->idrv_name;
}

static inline u32 touch_max_finger(struct siw_ts *ts)
{
	return ts->max_finger;
}

static inline u32 touch_chip_type(struct siw_ts *ts)
{
	return ts->chip_type;
}

static inline u32 touch_mode_allowed(struct siw_ts *ts, u32 mode)
{
	return (ts->mode_allowed & BIT(mode));
}

static inline u32 touch_mode_not_allowed(struct siw_ts *ts, u32 mode)
{
	int ret;

	ret = !touch_mode_allowed(ts, mode);
	if (ret) {
		t_dev_warn(ts->dev, "target mode(%d) not supported\n", mode);
	}

	return ret;
}

static inline u32 touch_fw_size(struct siw_ts *ts)
{
	return pdata_fw_size(ts->pdata);
}

static inline u32 touch_flags(struct siw_ts *ts)
{
	return ts->flags;
}

static inline unsigned long touch_irqflags(struct siw_ts *ts)
{
	return ts->irqflags;
}

static inline u32 touch_bus_type(struct siw_ts *ts)
{
	return pdata_bus_type(ts->pdata);
}

static inline u32 touch_buf_size(struct siw_ts *ts)
{
	return pdata_buf_size(ts->pdata);
}

static inline u32 touch_chip_select(struct siw_ts *ts)
{
	return pdata_chip_select(ts->pdata);
}

static inline u32 touch_spi_mode(struct siw_ts *ts)
{
	return pdata_spi_mode(ts->pdata);
}

static inline u32 touch_bits_per_word(struct siw_ts *ts)
{
	return pdata_bits_per_word(ts->pdata);
}

static inline u32 touch_max_freq(struct siw_ts *ts)
{
	return pdata_max_freq(ts->pdata);
}

static inline int touch_tx_hdr_size(struct siw_ts *ts)
{
	return ts->bus_tx_hdr_size;
}

static inline int touch_rx_hdr_size(struct siw_ts *ts)
{
	return ts->bus_rx_hdr_size;
}

static inline int touch_tx_dummy_size(struct siw_ts *ts)
{
	return ts->bus_tx_dummy_size;
}

static inline int touch_rx_dummy_size(struct siw_ts *ts)
{
	return ts->bus_rx_dummy_size;
}

static inline struct siw_touch_fquirks *touch_fquirks(struct siw_ts *ts)
{
	return pdata_fquirks(ts->pdata);
}

static inline struct siw_touch_fw_bin *touch_fw_bin(
								struct siw_ts *ts)
{
	return pdata_fw_bin(ts->pdata);
}

static inline u32 touch_get_act_buf_size(struct siw_ts *ts)
{
	return ts->buf_size;
}

static inline void touch_set_act_buf_size(struct siw_ts *ts, int size)
{
	ts->buf_size = size;
}

static inline void touch_set_caps(struct siw_ts *ts,
					struct touch_device_caps *caps_src)
{
	struct touch_device_caps *caps = &ts->caps;

	caps->max_x = caps_src->max_x;
	caps->max_y = caps_src->max_y;
	caps->max_pressure = caps_src->max_pressure;
	caps->max_width = caps_src->max_width;
	caps->max_orientation = caps_src->max_orientation;
	caps->max_id = caps_src->max_id;
	caps->mt_slots_flags = caps_src->mt_slots_flags;

	caps->hw_reset_delay = caps_src->hw_reset_delay;
	caps->sw_reset_delay = caps_src->sw_reset_delay;
}

static inline void touch_set_pins(struct siw_ts *ts,
					struct touch_pins *pins_src)
{
	struct touch_pins *pins = &ts->pins;

	pins->reset_pin = pins_src->reset_pin;
	pins->reset_pin_pol = pins_src->reset_pin_pol;
	pins->irq_pin = pins_src->irq_pin;
	/* Power */
	pins->vdd_pin = pins_src->vdd_pin;
	pins->vio_pin = pins_src->vio_pin;
	memcpy(&pins->vdd_cfg, &pins_src->vdd_cfg, sizeof(struct touch_pwr_cfg));
	memcpy(&pins->vio_cfg, &pins_src->vio_cfg, sizeof(struct touch_pwr_cfg));
}

static inline int touch_reset_pin(struct siw_ts *ts)
{
	return ts->pins.reset_pin;
}

static inline int touch_reset_pin_pol(struct siw_ts *ts)
{
	return ts->pins.reset_pin_pol;
}

static inline int touch_irq_pin(struct siw_ts *ts)
{
	return ts->pins.irq_pin;
}

static inline int touch_vdd_pin(struct siw_ts *ts)
{
	return ts->pins.vdd_pin;
}

static inline int touch_vio_pin(struct siw_ts *ts)
{
	return ts->pins.vio_pin;
}


static inline void *siw_ops_reg(struct siw_ts *ts)
{
	return ts->ops->reg;
}

static inline void siw_ops_set_irq_handler(struct siw_ts *ts, void *handler)
{
	ts->ops->irq_handler = handler;
}

static inline void siw_ops_restore_irq_handler(struct siw_ts *ts)
{
	ts->ops->irq_handler = ts->ops_ext->irq_handler;
}


#define siw_ops_is_null(_ts, _ops)	!!(_ts->ops->_ops == NULL)

#define siw_ops_xxx(_ops, _ret, _ts, args...)	\
		({	int _r = 0;	\
			do {	\
				if (_ts->ops->_ops == NULL) {	\
					if ((_ret) < 0) {	\
						t_dev_err(ts->dev, "%s isn't assigned\n", #_ops);	\
						_r = _ret;	\
					}	\
					break;	\
				}	\
				_r = _ts->ops->_ops(_ts->dev, ##args);	\
			} while (0);	\
			_r;	\
		})

#define siw_ops_early_probe(_ts, args...)	siw_ops_xxx(early_probe, 0, _ts, ##args)
#define siw_ops_probe(_ts, args...)			siw_ops_xxx(probe, -ESRCH, _ts, ##args)
#define siw_ops_remove(_ts, args...)		siw_ops_xxx(remove, -ESRCH, _ts, ##args)
#define siw_ops_suspend(_ts, args...)		siw_ops_xxx(suspend, -ESRCH, _ts, ##args)
#define siw_ops_resume(_ts, args...)		siw_ops_xxx(resume, -ESRCH, _ts, ##args)
#define siw_ops_init(_ts, args...)			siw_ops_xxx(init, -ESRCH, _ts, ##args)
#define siw_ops_reset(_ts, args...)			siw_ops_xxx(reset, -ESRCH, _ts, ##args)
#define siw_ops_ic_info(_ts, args...)		siw_ops_xxx(ic_info, -ESRCH, _ts, ##args)
#define siw_ops_tc_driving(_ts, args...)	siw_ops_xxx(tc_driving, -ESRCH, _ts, ##args)
#define siw_ops_chk_status(_ts, args...)	siw_ops_xxx(chk_status, -ESRCH, _ts, ##args)
#define siw_ops_irq_handler(_ts, args...)	siw_ops_xxx(irq_handler, -ESRCH, _ts, ##args)
#define siw_ops_power(_ts, args...)			siw_ops_xxx(power, -ESRCH, _ts, ##args)
#define siw_ops_upgrade(_ts, args...)		siw_ops_xxx(upgrade, -ESRCH, _ts, ##args)
#define siw_ops_set(_ts, args...)			siw_ops_xxx(set, 0, _ts, ##args)
#define siw_ops_get(_ts, args...)			siw_ops_xxx(get, 0, _ts, ##args)
#define siw_ops_sysfs(_ts, args...)			siw_ops_xxx(sysfs, 0, _ts, ##args)
#define siw_ops_mon_handler(_ts, args...)	siw_ops_xxx(mon_handler, -ESRCH, _ts, ##args)

#define siw_ops_prd_sysfs(_ts, args...)		siw_ops_xxx(prd_sysfs, 0, _ts, ##args)


static inline void touch_msleep(unsigned int msecs)
{
	if (!msecs)
		return;

	if (msecs >= 20)
		msleep(msecs);
	else
		usleep_range(msecs * 1000, msecs * 1000);
}

static inline void *touch_kzalloc(struct device *dev, size_t size, gfp_t gfp)
{
	return devm_kzalloc(dev, size, gfp);
}

static inline void touch_kfree(struct device *dev, void *p)
{
	devm_kfree(dev, p);
}

static inline void *touch_getname(void)
{
	void *name = __getname();
	if (name != NULL) {
		memset(name, 0, PATH_MAX);
	}
	return name;
}

static inline void touch_putname(void *name)
{
	if (name != NULL) {
		__putname(name);
	}
}

static inline int touch_str_to_lower(char *dst, char *src)
{
	int len = strlen(src);
	int ret = len;

	while (len--) {
		*dst++ = tolower(*src++);
	}

	return ret;
}


struct siw_op_dbg {
	char		*name;
	int			(*func)(void *data);
	void		(*debug)(void *op, void *data);
	int			flags;
};

#define _SIW_OP_DBG(_type, _func, _debug, _flags)	\
	[_type] = {		\
		.name = #_func,		\
		.func = _func,		\
		.debug = _debug,	\
		.flags = _flags,	\
	}

static inline int __siw_touch_op_dbg(struct siw_op_dbg *op,
						void *data)
{
	int ret = 0;

	ret = op->func(data);
	if (op->debug) {
		op->debug(op, data);
	}

	return ret;
}

#define __siw_snprintf(_buf, _buf_max, _size, _fmt, _args...) \
		({	\
			int _n_size = 0;	\
			if (_size < _buf_max)	\
				_n_size = snprintf(_buf + _size, _buf_max - _size,\
								(const char *)_fmt, ##_args);	\
			_n_size;	\
		})


#define siw_snprintf(_buf, _size, _fmt, _args...) \
		__siw_snprintf(_buf, PAGE_SIZE, _size, _fmt, ##_args)


extern int siw_setup_params(struct siw_ts *ts, struct siw_touch_pdata *pdata);

extern void *siw_setup_operations(struct siw_ts *ts, struct siw_touch_operations *ops_ext);

extern int siw_touch_set(struct device *dev, u32 cmd, void *buf);
extern int siw_touch_get(struct device *dev, u32 cmd, void *buf);

extern void siw_touch_core_wake_lock(struct device *dev, int timeout);
extern void siw_touch_core_wake_unlock(struct device *dev);

extern int siw_touch_power_state(struct device *dev);
extern int siw_touch_power_lock(struct device *dev, int set);

extern void siw_touch_mon_pause(struct device *dev);
extern void siw_touch_mon_resume(struct device *dev);

extern int siw_touch_probe(struct siw_ts *ts);
extern int siw_touch_remove(struct siw_ts *ts);
extern int siw_touch_shutdown(struct siw_ts *ts);

extern int siw_touch_init_late(struct siw_ts *ts, int value);
extern int siw_touch_init_late_queue(struct device *dev,
		int sig, int time, int retry);


#if defined(CONFIG_TOUCHSCREEN_SIWMON) || defined(CONFIG_TOUCHSCREEN_SIWMON_MODULE)

struct touch_bus_msg;

struct siw_mon_operations {
	void (*submit_bus)(struct device *dev, char *dir, void *data, int ret);
	void (*submit_evt)(struct device *dev, char *type, int type_v, char *code, int code_v, int value, int ret);
	void (*submit_ops)(struct device *dev, char *ops, void *data, int size, int ret);
};

extern struct siw_mon_operations *siw_mon_ops;

static inline void siwmon_submit_bus(struct device *dev, char *dir, void *data, int ret)
{
	if (siw_mon_ops && siw_mon_ops->submit_bus)
		(*siw_mon_ops->submit_bus)(dev, dir, data, ret);
}

static inline void siwmon_submit_evt(struct device *dev, char *type, int type_v, char *code, int code_v, int value, int ret)
{
	if (siw_mon_ops && siw_mon_ops->submit_evt)
		(*siw_mon_ops->submit_evt)(dev, type, type_v, code, code_v, value, ret);
}

static inline void siwmon_submit_ops(struct device *dev, char *ops, void *data, int size, int ret)
{
	if (siw_mon_ops && siw_mon_ops->submit_ops)
		(*siw_mon_ops->submit_ops)(dev, ops, data, size, ret);
}

extern int siw_mon_register(struct siw_mon_operations *ops);
extern void siw_mon_deregister(void);

#else	/* CONFIG_TOUCHSCREEN_SIWMON */

static inline void siwmon_submit_bus(struct device *dev, char *dir, void *data, int ret){ }
static inline void siwmon_submit_evt(struct device *dev, char *type, int type_v, char *code, int code_v, int value, int ret){ }
static inline void siwmon_submit_ops(struct device *dev, char *ops, void *data, int size, int ret){ }

#endif	/* CONFIG_TOUCHSCREEN_SIWMON */

#define siwmon_submit_ops_wh_name(_dev, _fmt, _name, _val, _size, _ret)	\
		do {	\
			char _mstr[64];	\
			snprintf(_mstr, sizeof(_mstr), _fmt, _name);	\
			siwmon_submit_ops(_dev, _mstr, _val, _size, _ret);	\
		} while (0)

#define siwmon_submit_ops_step(_dev, _ops)	\
		siwmon_submit_ops(_dev, _ops, NULL, 0, 0)

#define siwmon_submit_ops_step_core(_dev, _ops, _ret)	\
		siwmon_submit_ops(_dev, "[S] " _ops, NULL, 0, _ret)

#define siwmon_submit_ops_step_chip(_dev, _ops, _ret)	\
		siwmon_submit_ops(_dev, "(c) " _ops, NULL, 0, _ret)

#define siwmon_submit_ops_step_chip_wh_name(_dev, _fmt, _name, _ret)	\
		do {	\
			char _mstr[64];	\
			snprintf(_mstr, sizeof(_mstr), "(c) " _fmt, _name);	\
			siwmon_submit_ops(_dev, _mstr, NULL, 0, _ret);	\
		} while (0)

#if !defined(MODULE)
#define __siw_setup_u32(_name, _fn, _var)	\
		static int __init _fn(char *in_str)	\
		{	\
			_var = (u32)simple_strtol(in_str, NULL, 0);	\
			return 1;	\
		}	\
		__setup(_name, _fn)

#define __siw_setup_str(_name, _fn, _var)	\
		static int __init _fn(char *in_str)	\
		{	\
			strlcpy(_var, in_str, sizeof(_var));	\
			return 1;	\
		}	\
		__setup(_name, _fn)
#else	/* MODULE */
#define __siw_setup_u32(_name, _fn, _var)
#define __siw_setup_str(_name, _fn, _var)
#endif	/* MODULE */

static inline void siw_logo(void)
{
#if defined(__SIW_SHOW_LOGO)
	const char *logo[] = {
		" _____ _ _    _   _____                _     ",
		"/  ___(_) |  | | |_   _|              | |    ",
		"\\ `--. _| |  | |   | | ___  _   _  ___| |__  ",
		" `--. \\ | |/\\| |   | |/ _ \\| | | |/ __| '_ \\ ",
		"/\\__/ / \\  /\\  /   | | (_) | |_| | (__| | | |",
		"\\____/|_|\\/  \\/    \\_/\\___/ \\__,_|\\___|_| |_|"
	};
#if 0
	int i;

	for (i = 0; i < ARRAY_SIZE(logo); i++) {
		t_pr_info("%s \n", logo[i]);
	}
	t_pr_info("\n");
#else
	t_pr_info("\n%s \n%s \n%s \n%s \n%s \n%s \n\n",
		logo[0], logo[1], logo[2], logo[3], logo[4], logo[5]);
#endif
#endif
}

#define siw_chip_module_init(_name, _data, _desc, _author)	\
		static int __init siw_touch_driver_init(void)\
		{	\
			siw_logo();	\
			t_pr_info("%s: %s driver init - %s\n",	\
				_data.pdata->drv_name, _name, SIW_DRV_VERSION);	\
			return siw_touch_bus_add_driver(&_data);	\
		}	\
		static void __exit siw_touch_driver_exit(void)	\
		{	\
			(void)siw_touch_bus_del_driver(&_data);\
			t_pr_info("%s: %s driver exit - %s\n",	\
				_data.pdata->drv_name, _name, SIW_DRV_VERSION);	\
			siw_logo();	\
		}	\
		module_init(siw_touch_driver_init);	\
		module_exit(siw_touch_driver_exit);	\
		MODULE_AUTHOR(_author);	\
		MODULE_DESCRIPTION(_desc);	\
		MODULE_VERSION(SIW_DRV_VERSION);	\
		MODULE_LICENSE("GPL");

#endif /* __SIW_TOUCH_H */



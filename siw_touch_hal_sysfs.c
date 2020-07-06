/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_hal_sysfs.c - SiW touch hal driver
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
#include "siw_touch_event.h"
#include "siw_touch_gpio.h"
#include "siw_touch_irq.h"
#include "siw_touch_sys.h"


#define siw_hal_sysfs_err_invalid_param(_dev)	\
		t_dev_err(_dev, "Invalid param\n");

#define _reg_snprintf(_buf, _size, _reg, _element)	\
		siw_snprintf(_buf, _size, "# 0x%04X [%s]\n", _reg->_element, #_element)

static int __show_reg_list(struct device *dev, char *buf, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;

	size += siw_snprintf(buf, size, "# Reg. Map List\n");

	size += _reg_snprintf(buf, size, reg, spr_chip_id);
	size += _reg_snprintf(buf, size, reg, spr_rst_ctl);
	size += _reg_snprintf(buf, size, reg, spr_boot_ctl);
	size += _reg_snprintf(buf, size, reg, spr_sram_ctl);
	size += _reg_snprintf(buf, size, reg, spr_boot_status);
	size += _reg_snprintf(buf, size, reg, spr_subdisp_status);
	size += _reg_snprintf(buf, size, reg, spr_code_offset);
	size += _reg_snprintf(buf, size, reg, tc_ic_status);
	size += _reg_snprintf(buf, size, reg, tc_status);
	size += _reg_snprintf(buf, size, reg, tc_version);
	size += _reg_snprintf(buf, size, reg, tc_product_id1);
	size += _reg_snprintf(buf, size, reg, tc_product_id2);
	size += _reg_snprintf(buf, size, reg, tc_version_ext);
	size += _reg_snprintf(buf, size, reg, info_chip_version);
	size += _reg_snprintf(buf, size, reg, info_lot_num);
	size += _reg_snprintf(buf, size, reg, info_serial_num);
	size += _reg_snprintf(buf, size, reg, info_date);
	size += _reg_snprintf(buf, size, reg, info_time);
	size += _reg_snprintf(buf, size, reg, code_access_addr);
	size += _reg_snprintf(buf, size, reg, data_i2cbase_addr);
	size += _reg_snprintf(buf, size, reg, prd_tcm_base_addr);
	size += _reg_snprintf(buf, size, reg, tc_device_ctl);
	size += _reg_snprintf(buf, size, reg, tc_interrupt_ctl);
	size += _reg_snprintf(buf, size, reg, tc_interrupt_status);
	size += _reg_snprintf(buf, size, reg, tc_drive_ctl);
	/* */
	size += _reg_snprintf(buf, size, reg, tc_tsp_test_ctl);
	size += _reg_snprintf(buf, size, reg, tc_tsp_test_status);
	size += _reg_snprintf(buf, size, reg, tc_tsp_test_pf_result);
	size += _reg_snprintf(buf, size, reg, tc_flash_dn_status);
	size += _reg_snprintf(buf, size, reg, tc_confdn_base_addr);
	size += _reg_snprintf(buf, size, reg, tc_flash_dn_ctl);
	size += _reg_snprintf(buf, size, reg, serial_data_offset);

	/* __SIW_SUPPORT_PRD */
	size += _reg_snprintf(buf, size, reg, prd_serial_tcm_offset);
	size += _reg_snprintf(buf, size, reg, prd_tc_mem_sel);
	size += _reg_snprintf(buf, size, reg, prd_tc_test_mode_ctl);
	size += _reg_snprintf(buf, size, reg, prd_m1_m2_raw_offset);
	size += _reg_snprintf(buf, size, reg, prd_tune_result_offset);
	size += _reg_snprintf(buf, size, reg, prd_open3_short_offset);
	size += _reg_snprintf(buf, size, reg, prd_ic_ait_start_reg);
	size += _reg_snprintf(buf, size, reg, prd_ic_ait_data_readystatus);

	return size;
}

static ssize_t _show_reg_list(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 bootmode = 0;
	u32 boot_chk_offset = 0;
	int size = 0;
	int ret = 0;

	ret = siw_hal_get_boot_status(dev, &bootmode);
	if (ret < 0) {
		return (ssize_t)ret;
	}

	size += __show_reg_list(dev, buf, size);

	size += siw_snprintf(buf, size, "\n");

	if (fw->version_ext) {
		size += siw_snprintf(buf, size,
					">> version    : %08X, chip : %u, protocol : %u\n",
					fw->version_ext,
					fw->v.version.chip,
					fw->v.version.protocol);
	} else {
		size += siw_snprintf(buf, size,
					">> version    : v%u.%02u, chip : %u, protocol : %u\n",
					fw->v.version.major,
					fw->v.version.minor,
					fw->v.version.chip,
					fw->v.version.protocol);
	}

	size += siw_snprintf(buf, size,
				">> revision   : %d\n",
				fw->revision);

	size += siw_snprintf(buf, size,
				">> product id : %s\n",
				fw->product_id);

	boot_chk_offset = siw_hal_boot_sts_pos_busy(chip);
	size += siw_snprintf(buf, size,
				">> flash boot : %s(%s), crc %s  (0x%08X)\n",
				(bootmode >> (boot_chk_offset) & 0x1) ? "BUSY" : "idle",
				(bootmode >> (boot_chk_offset + 1) & 0x1) ? "done" : "booting",
				(bootmode >> (boot_chk_offset + 2) & 0x1) ? "ERROR" : "ok",
				bootmode);

	size += siw_snprintf(buf, size,
				">> status     : type %d[%08Xh, %08Xh, %08Xh]\n",
				chip->status_type,
				chip->status_mask_normal,
				chip->status_mask_logging,
				chip->status_mask_reset);

	size += siw_snprintf(buf, size, "\n");

	return (ssize_t)size;
}

#define REG_BURST_MAX			512
#define REG_BURST_COL_PWR		4

static int __show_reg_ctrl_log_history(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg_log *reg_log = chip->reg_log;
	char *dir_name;
	int reg_burst;
	int reg_err;
	int i;
	int size = 0;

	size += siw_snprintf(buf, size, "[Test History]\n");
	for (i = 0; i < REG_LOG_MAX; i++) {
		if (reg_log->dir) {
			dir_name = !!((reg_log->dir & REG_DIR_MASK) == REG_DIR_WR) ?
						"wr" : "rd";
			reg_burst = !!(reg_log->dir & (REG_DIR_ERR<<1));
			reg_err = !!(reg_log->dir & REG_DIR_ERR);
		} else {
			dir_name = "__";
			reg_burst = 0;
			reg_err = 0;
		}

		if (reg_burst) {
			size += siw_snprintf(buf, size, " %s: reg[0x%04X] = (burst) %s\n",
						dir_name, reg_log->addr,
						(reg_err) ? "(err)" : "");
		} else {
			size += siw_snprintf(buf, size, " %s: reg[0x%04X] = 0x%08X %s\n",
						dir_name, reg_log->addr, reg_log->data,
						(reg_err) ? "(err)" : "");
		}

		reg_log++;
	}

	return size;
}

static ssize_t _show_reg_ctrl(struct device *dev, char *buf)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int size = 0;

	size += __show_reg_ctrl_log_history(dev, buf);

	size += siw_snprintf(buf, size, "\n[Usage]\n");
	size += siw_snprintf(buf, size, " echo wr 0x1234 {value} > reg_ctrl\n");
	size += siw_snprintf(buf, size, " echo rd 0x1234 > reg_ctrl\n");
	size += siw_snprintf(buf, size, " (burst access)\n");
	size += siw_snprintf(buf, size, " echo rd 0x1234 0x111 > reg_ctrl, 0x111 is size(max 0x%X)\n",
		REG_BURST_MAX);

	return (ssize_t)size;
}

static void __store_reg_ctrl_log_add(struct device *dev,
				struct siw_hal_reg_log *new_log)

{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg_log *reg_log = chip->reg_log;

	memmove(&reg_log[1], reg_log, sizeof(*reg_log) * (REG_LOG_MAX - 1));
	memcpy(reg_log, new_log, sizeof(*reg_log));
}

static inline void __store_reg_ctrl_rd_burst_log(struct device *dev,
					u8 *row_buf, int row, int col)
{
	if (col)
		t_dev_info(dev, "rd: [%3Xh] %*ph\n", row, col, row_buf);
}

static int __store_reg_ctrl_rd_burst(struct device *dev, u32 addr, int size, int burst)
{
	u8 *rd_buf, *row_buf;
	int col_power = REG_BURST_COL_PWR;
	int col_width = (1<<col_power);
	int row_curr, col_curr;
	int ret = 0;

	size = min(size, REG_BURST_MAX);

	rd_buf = (u8 *)kzalloc(size, GFP_KERNEL);
	if (rd_buf == NULL) {
		t_dev_err(dev, "failed to allocate rd_buf\n");
		return -ENOMEM;
	}

	if (burst) {
		ret = siw_hal_reg_read(dev, addr, rd_buf, size);
	} else {
		ret = siw_hal_reg_read_single(dev, addr, rd_buf, size);
	}
	if (ret < 0) {
		goto out;
	}

	t_dev_info(dev, "rd: addr %04Xh, size %Xh %s\n", addr, size,
		(burst) ? "(burst)" : "");

	row_buf = rd_buf;
	row_curr = 0;
	while (size) {
		col_curr = min(col_width, size);

		__store_reg_ctrl_rd_burst_log(dev, row_buf, row_curr, col_curr);

		row_buf += col_curr;
		row_curr += col_curr;
		size -= col_curr;
	}

out:
	kfree(rd_buf);

	return ret;
}

static ssize_t _store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
//	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_reg_log reg_log;
	char command[6] = {0};
	u32 reg = 0;
	u32 data = 1;
	u32 reg_addr;
	int wr = -1;
	int rd = -1;
	int value = 0;
	int ret = 0;

	if (sscanf(buf, "%5s %X %X", command, &reg, &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	if (!strcmp(command, "wr") || !strcmp(command, "write")) {
		wr = 1;
	} else if (!strcmp(command, "rd") || !strcmp(command, "read")) {
		rd = 1;		/* single */
	} else if (!strcmp(command, "rdb") || !strcmp(command, "readb")) {
		rd = 2;		/* burst */
	}

	reg_addr = reg;
	if (wr != -1) {
		data = value;
		ret = siw_hal_write_value(dev,
					reg_addr,
					data);
		reg_log.dir = REG_DIR_WR;
		reg_log.addr = reg_addr;
		reg_log.data = data;
		if (ret >= 0) {
			t_dev_info(dev,
				"wr: reg[0x%04X] = 0x%08X\n",
				reg_addr, data);
		} else {
			reg_log.dir |= REG_DIR_ERR;
		}
		__store_reg_ctrl_log_add(dev, &reg_log);
		goto out;
	}

	if (rd != -1) {
		reg_log.dir = REG_DIR_RD;
		reg_log.addr = reg_addr;
		if (value <= 4) {
			ret = siw_hal_read_value(dev,
						reg_addr,
						&data);
			reg_log.data = data;
			if (ret >= 0) {
				t_dev_info(dev,
					"rd: reg[0x%04X] = 0x%08X\n",
					reg_addr, data);
			}
		} else {
			reg_log.dir |= (REG_DIR_ERR<<1);
			ret = __store_reg_ctrl_rd_burst(dev, reg_addr, value, (rd == 2));
		}
		if (ret < 0) {
			reg_log.dir |= REG_DIR_ERR;
		}
		__store_reg_ctrl_log_add(dev, &reg_log);
		goto out;
	}

	t_dev_info(dev, "[Usage]\n");
	t_dev_info(dev, " echo wr 0x1234 {value} > reg_ctrl\n");
	t_dev_info(dev, " echo rd 0x1234 > reg_ctrl\n");
	t_dev_info(dev, " (burst access)\n");
	t_dev_info(dev, " echo rd 0x1234 0x111 > reg_ctrl, 0x111 is size(max 0x%X)\n",
		REG_BURST_MAX);

out:
	return count;
}

static ssize_t _show_lcd_mode(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int size = 0;

	size += siw_snprintf(buf, size, "current driving mode is %s\n",
				siw_lcd_driving_mode_str(chip->lcd_mode));

	return size;
}

static ssize_t _store_lcd_mode(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	t_dev_info(dev, "try to change driving mode: %s -> %s\n",
			siw_lcd_driving_mode_str(chip->lcd_mode),
			siw_lcd_driving_mode_str(value));
	siw_hal_change_mode(dev, value);

	return count;
}

static ssize_t _show_reset_ctrl(struct device *dev, char *buf)
{
	int size = 0;

	size += siw_snprintf(buf, size, "Reset Control Usage\n");
	size += siw_snprintf(buf, size,
				" SW Reset        : echo %d > hal_reset_ctrl\n",
				SW_RESET);
	size += siw_snprintf(buf, size,
				" HW Reset(Async) : echo %d > hal_reset_ctrl\n",
				HW_RESET_ASYNC);
	size += siw_snprintf(buf, size,
				" HW Reset(Sync)  : echo %d > hal_reset_ctrl\n",
				HW_RESET_SYNC);

	size += siw_snprintf(buf, size,
				" HW Reset(Cond)  : echo 0x%X > hal_reset_ctrl\n",
				HW_RESET_COND);

	return size;
}

static ssize_t _store_reset_xxx(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (type == HW_RESET_COND) {
		siw_ops_mon_handler(ts, MON_FLAG_RST_ONLY);
		goto out;
	}

	siw_ops_reset(ts, type);

out:
	return 0;
}

static ssize_t _store_reset_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	int value = 0;

	if (sscanf(buf, "%X", &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	_store_reset_xxx(dev, value);

	return count;
}

#if defined(__SIW_ATTR_RST_BY_READ)
static ssize_t _show_reset_sw(struct device *dev, char *buf)
{
	return _store_reset_xxx(dev, SW_RESET);
}

static ssize_t _show_reset_hw(struct device *dev, char *buf)
{
	return _store_reset_xxx(dev, HW_RESET_SYNC);
}
#endif

static const char *__debug_hal_delay_str[] = {
	[HAL_DBG_DLY_TC_DRIVING_0]	= "(Group - TC Driving)",
	[HAL_DBG_DLY_FW_0]			= "(Group - FW)",
	[HAL_DBG_DLY_HW_RST_0]		= "(Group - Reset)",
	[HAL_DBG_DLY_NOTIFY]		= "(Group - Notify)",
};

static ssize_t _show_debug_hal(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_debug *dbg = &chip->dbg;
	char *str;
	int size = 0;
	int i;

	for (i = 0; i < HAL_DBG_DLY_MAX; i++) {
		switch (i) {
		case HAL_DBG_DLY_TC_DRIVING_0:
		case HAL_DBG_DLY_FW_0:
		case HAL_DBG_DLY_HW_RST_0:
		case HAL_DBG_DLY_NOTIFY:
			str = (char *)__debug_hal_delay_str[i];
			break;
		default:
			str = "";
		}
		size += siw_snprintf(buf, size,
			 "debug_hal: delay[%d] = %Xh %s\n",
			 i, dbg->delay[i], str);
	}

	return (ssize_t)size;
}

static int _store_debug_hal_delay(struct device *dev, int sel, int val, int opt)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_debug *dbg = &chip->dbg;

	if (sel < HAL_DBG_DLY_MAX) {
		u32 *delay = &dbg->delay[sel];

		if ((*delay) != val) {
			t_dev_info(dev, "debug_hal: delay[%d] changed: %Xh -> %Xh\n",
				sel, (*delay), val);
			(*delay) = val;
		}
	}

	return 0;
}

static ssize_t _store_debug_hal(struct device *dev,
				const char *buf, size_t count)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int code = 0;
	int sel = 0;
	int val = 0;
	int opt = 0;
	int ret;

	if (sscanf(buf, "%X %X %X %X", &code, &sel, &val, &opt) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	t_dev_info(dev, "debug_hal: code %Xh, sel %Xh, val %Xh, opt %Xh\n",
			code, sel, val, opt);

	switch (code) {
	case HAL_DBG_GRP_0:
		ret = _store_debug_hal_delay(dev, sel, val, opt);
		break;
	default:
		break;
	}

	return count;
}

static ssize_t _show_debug_tc_cmd(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	char *mode_str, *ext_str;
	int *tc_cmd_table = chip->tc_cmd_table;
	int size = 0;
	int ctrl, i;

	size += siw_snprintf(buf, size,
		"[%s tc cmd set] (mode bit %04Xh)\n",
		touch_chip_name(ts), ts->mode_allowed);

	for (i = 0; i < LCD_MODE_MAX; i++) {
		mode_str = (char *)siw_lcd_driving_mode_str(i);
		ctrl = tc_cmd_table[i];

		if (ctrl < 0) {
			ext_str = "(not granted)";
		} else if (!touch_mode_allowed(ts, i)) {
			ext_str = "(not allowed)";
		} else {
			ext_str = "";
		}

		size += siw_snprintf(buf, size,
			" %04Xh [%-13s] %s\n",
			ctrl, mode_str, ext_str);
	}

	size += siw_snprintf(buf, size, "\n");

	return (ssize_t)size;
}

static ssize_t _store_debug_tc_cmd(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int *tc_cmd_table = chip->tc_cmd_table;
	char *mode_str;
	int mode, value;
	int ctrl;

	if (sscanf(buf, "%X %X", &mode, &value) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	if (mode >= LCD_MODE_MAX) {
		t_dev_err(dev, "Invalid mode: %d >= LCD_MODE_MAX(%d)\n",
			mode, LCD_MODE_MAX);
		return count;
	}

	mode_str = (char *)siw_lcd_driving_mode_str(mode);
	ctrl = tc_cmd_table[mode];

	if (!touch_mode_allowed(ts, mode)) {
		t_dev_info(dev, "%s(%d) is not allowed\n", mode_str, mode);
		goto out;
	}

	if (value >= 0xFFFF) {
		value = -1;
	}

	if (value == ctrl) {
		goto out;
	}

	tc_cmd_table[mode] = value;

	t_dev_info(dev, "%s(%d) changed: %04Xh -> %04Xh\n",
		mode_str, mode, ctrl, value);

out:
	return count;
}

static ssize_t _show_debug_power(struct device *dev, char *buf)
{
	int size = 0;

	size += siw_snprintf(buf, size, "[Usage]\n");
	size += siw_snprintf(buf, size, " off : echo 0 > debug_power\n");
	size += siw_snprintf(buf, size, " on  : echo 1 > debug_power\n");
	size += siw_snprintf(buf, size, "\n");

	return (ssize_t)size;
}

static ssize_t _store_debug_power(struct device *dev,
					const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ctrl = 0;

	if (sscanf(buf, "%d", &ctrl) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	t_dev_info(dev, "debug_power: %s\n",
		(ctrl) ? "on" : "off");

	if (!ctrl) {
		siw_hal_alive_pause_set(dev, ALIVE_PAUSE_PWR);
		siw_touch_irq_control(dev, INTERRUPT_DISABLE);
	}

	siw_ops_power(ts, (ctrl) ? POWER_ON : POWER_OFF);

	if (ctrl) {
		siw_touch_qd_init_work_hw(ts);
	}

	return count;
}

static ssize_t _show_debug_voltage(struct device *dev, char *buf)
{
	int size = 0;

	size += siw_snprintf(buf, size, "[Usage]\n");
	size += siw_snprintf(buf, size, " <vio control>\n");
	size += siw_snprintf(buf, size, " off : echo 0 0 > debug_voltage\n");
	size += siw_snprintf(buf, size, " on  : echo 0 1 > debug_voltage\n");
	size += siw_snprintf(buf, size, " <vdd control>\n");
	size += siw_snprintf(buf, size, " off : echo 1 0 > debug_voltage\n");
	size += siw_snprintf(buf, size, " on  : echo 1 1 > debug_voltage\n");
	size += siw_snprintf(buf, size, "\n");

	return (ssize_t)size;
}

static ssize_t _store_debug_voltage(struct device *dev,
					const char *buf, size_t count)
{
	int index = 0;
	int ctrl = 0;

	if (sscanf(buf, "%d %d", &index, &ctrl) <= 0) {
		siw_hal_sysfs_err_invalid_param(dev);
		return count;
	}

	index = !!index;
	ctrl = !!ctrl;

	t_dev_info(dev, "debug_voltage: %s %s\n",
		(index) ? "vdd" : "vio",
		(ctrl) ? "on" : "off");

	if (index) {
		siw_touch_power_vdd(dev, ctrl);
	} else {
		siw_touch_power_vio(dev, ctrl);
	}

	return count;
}

static ssize_t _show_fwup_status(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int fwup_status = siw_hal_get_fwup_status(chip);
	const char *str = siw_fwup_status_str(fwup_status);
	int size = 0;

	if (fwup_status != FWUP_STATUS_BUSY) {
		t_dev_info(dev, "fwup_status: %d(%s)\n", fwup_status, str);
	}

	size += siw_snprintf(buf, size, "%d\n", fwup_status);

	return (ssize_t)size;
}


#define SIW_TOUCH_HAL_ATTR(_name, _show, _store)	\
		TOUCH_ATTR(_name, _show, _store)

#define _SIW_TOUCH_HAL_ATTR_T(_name)	\
		touch_attr_##_name

static SIW_TOUCH_HAL_ATTR(reg_list, _show_reg_list, NULL);
static SIW_TOUCH_HAL_ATTR(reg_ctrl, _show_reg_ctrl, _store_reg_ctrl);
static SIW_TOUCH_HAL_ATTR(lcd_mode, _show_lcd_mode, _store_lcd_mode);
static SIW_TOUCH_HAL_ATTR(reset_ctrl, _show_reset_ctrl, _store_reset_ctrl);
#if defined(__SIW_ATTR_RST_BY_READ)
static SIW_TOUCH_HAL_ATTR(reset_sw, _show_reset_sw, NULL);
static SIW_TOUCH_HAL_ATTR(reset_hw, _show_reset_hw, NULL);
#endif
static SIW_TOUCH_HAL_ATTR(debug_hal, _show_debug_hal, _store_debug_hal);
static SIW_TOUCH_HAL_ATTR(debug_tc_cmd, _show_debug_tc_cmd, _store_debug_tc_cmd);
static SIW_TOUCH_HAL_ATTR(debug_power, _show_debug_power, _store_debug_power);
static SIW_TOUCH_HAL_ATTR(debug_voltage, _show_debug_voltage, _store_debug_voltage);
static SIW_TOUCH_HAL_ATTR(fwup_status, _show_fwup_status, NULL);

#if defined(__SIW_SUPPORT_ALIVE_DETECTION)
extern u32 t_alive_dbg_mask;

static ssize_t _show_alive_debug(struct device *dev, char *buf)
{
	u32 mask = t_alive_dbg_mask;
	int size = 0;

	size += siw_snprintf(buf, size,
				"alive_dbg_mask %08Xh\n", mask);

	size += siw_snprintf(buf, size,
				"Usage:\n");
	size += siw_snprintf(buf, size,
				" echo {mask_value} > alive_debug\n");

	return (ssize_t)size;
}

static ssize_t _store_alive_debug(struct device *dev,
				const char *buf, size_t count)
{
	u32 old_value = 0;
	u32 new_value = 0;

	if (sscanf(buf, "%X", &new_value) <= 0) {
		t_dev_err(dev, "Invalid param\n");
		return count;
	}

	old_value = t_alive_dbg_mask;
	t_alive_dbg_mask = new_value;
	t_dev_info(dev, "alive_dbg_mask changed : %08Xh -> %08Xh\n",
		old_value, new_value);

	return count;
}

static ssize_t _show_alive_count(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int size = 0;

	size += siw_snprintf(buf, size,
				"irq_count : alive %d, report %d, others %d\n",
				chip->irq_count_alive,
				chip->irq_count_report,
				chip->irq_count_others);

	return (ssize_t)size;
}

static ssize_t _store_alive_count(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	char string[32] = {0, };

	if (sscanf(buf, "%s", string) <= 0) {
		t_dev_err(dev, "Invalid param\n");
		return count;
	}

	mutex_lock(&ts->lock);

	if (!strcmp(string, "reset")) {
		chip->irq_count_alive = 0;
		chip->irq_count_report = 0;
		chip->irq_count_others = 0;
	}

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t _show_alive_level(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int level = chip->alive_level;
	int min = chip->alive_level_min;
	int def = chip->alive_level_def;
	int max = chip->alive_level_max;
	int size = 0;

	if (!siw_hal_alive_is_active(dev)) {
		t_dev_info(dev, "alive not supported\n");
	} else {
		t_dev_info(dev, "alive_level : %s (%d <= x <= %d, default %d)\n",
				__alive_level_str(level),
				min, max, def);
	}

	size += siw_snprintf(buf, size, "%d\n", chip->alive_level);

	return size;
}

static ssize_t _store_alive_level(struct device *dev,
			const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int level = 0;

	if (sscanf(buf, "%d", &level) <= 0) {
		t_dev_err(dev, "Invalid param\n");
		return count;
	}

	if (!siw_hal_alive_is_active(dev)) {
		t_dev_info(dev, "alive not supported\n");
	}

	mutex_lock(&ts->lock);

	siw_hal_alive_level_set(dev, level);

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t _show_alive_pause(struct device *dev, char *buf)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
	int level = siw_hal_alive_pause_get(dev);
	int size = 0;

	if (!siw_hal_alive_is_active(dev)) {
		t_dev_info(dev, "alive not supported\n");
	} else {
		t_dev_info(dev, "alive %s(%d)\n", __alive_pause_str(level), level);
	}

	size += siw_snprintf(buf, size, "%d\n", level);

	return size;
}

static ssize_t _store_alive_pause(struct device *dev,
			const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int set = 0;

	if (sscanf(buf, "%d", &set) <= 0) {
		t_dev_err(dev, "Invalid param\n");
		return count;
	}

	if (!siw_hal_alive_is_active(dev)) {
		t_dev_info(dev, "alive not supported\n");
	}

	mutex_lock(&ts->lock);

	siw_hal_alive_pause_set(dev, (set) ? ALIVE_PAUSE_EXT : ALIVE_RESUME);

	mutex_unlock(&ts->lock);

	return count;
}

static SIW_TOUCH_HAL_ATTR(alive_debug, _show_alive_debug, _store_alive_debug);
static SIW_TOUCH_HAL_ATTR(alive_count, _show_alive_count, _store_alive_count);
static SIW_TOUCH_HAL_ATTR(alive_level, _show_alive_level, _store_alive_level);
static SIW_TOUCH_HAL_ATTR(alive_pause, _show_alive_pause, _store_alive_pause);
#endif	/* __SIW_SUPPORT_ALIVE_DETECTION */


static struct attribute *siw_hal_attribute_list[] = {
	&_SIW_TOUCH_HAL_ATTR_T(reg_list).attr,
	&_SIW_TOUCH_HAL_ATTR_T(reg_ctrl).attr,
	&_SIW_TOUCH_HAL_ATTR_T(lcd_mode).attr,
	&_SIW_TOUCH_HAL_ATTR_T(reset_ctrl).attr,
#if defined(__SIW_ATTR_RST_BY_READ)
	&_SIW_TOUCH_HAL_ATTR_T(reset_sw).attr,
	&_SIW_TOUCH_HAL_ATTR_T(reset_hw).attr,
#endif
	&_SIW_TOUCH_HAL_ATTR_T(debug_hal).attr,
	&_SIW_TOUCH_HAL_ATTR_T(debug_tc_cmd).attr,
	&_SIW_TOUCH_HAL_ATTR_T(debug_power).attr,
	&_SIW_TOUCH_HAL_ATTR_T(debug_voltage).attr,
	&_SIW_TOUCH_HAL_ATTR_T(fwup_status).attr,
	/* */
#if defined(__SIW_SUPPORT_ALIVE_DETECTION)
	&_SIW_TOUCH_HAL_ATTR_T(alive_debug).attr,
	&_SIW_TOUCH_HAL_ATTR_T(alive_count).attr,
	&_SIW_TOUCH_HAL_ATTR_T(alive_level).attr,
	&_SIW_TOUCH_HAL_ATTR_T(alive_pause).attr,
#endif	/* __SIW_SUPPORT_ALIVE_DETECTION */
	NULL,
};

static const struct attribute_group siw_hal_attribute_group = {
	.attrs = siw_hal_attribute_list,
};

static int __siw_hal_sysfs_add_prd(struct device *dev, int on_off)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	ret = siw_ops_prd_sysfs(ts, on_off);
	if ((on_off == DRIVER_INIT) && (ret < 0)) {
		t_dev_err(dev, "%s prd sysfs register failed\n",
			touch_chip_name(ts));
	}

	return ret;
}

static int siw_hal_create_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct kobject *kobj = &ts->kobj;
	int ret = 0;

	if (chip->sysfs_done)
		return 0;

	ret = sysfs_create_group(kobj, &siw_hal_attribute_group);
	if (ret < 0) {
		t_dev_err(dev, "%s sysfs register failed\n",
				touch_chip_name(ts));
		goto out;
	}

	t_dev_info(dev, "%s sysfs registered\n",
			touch_chip_name(ts));

	chip->sysfs_done = 1;

	return 0;

out:

	return ret;
}

static void siw_hal_remove_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (!chip->sysfs_done)
		return;

	sysfs_remove_group(&ts->kobj, &siw_hal_attribute_group);

	t_dev_info(dev, "%s sysfs unregistered\n",
			touch_chip_name(ts));

	chip->sysfs_done = 0;
}

int siw_hal_sysfs(struct device *dev, int on_off)
{
	if (on_off == DRIVER_INIT) {
		return siw_hal_create_sysfs(dev);
	}

	siw_hal_remove_sysfs(dev);
	return 0;
}

int siw_hal_sysfs_post(struct device *dev, int on_off)
{
	__siw_hal_sysfs_add_prd(dev, on_off);

	return 0;
}


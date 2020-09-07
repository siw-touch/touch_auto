/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_hal.c - SiW touch hal driver
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

#ifndef __weak
#define __weak __attribute__((weak))
#endif


extern int siw_hal_sysfs(struct device *dev, int on_off);
extern int siw_hal_sysfs_post(struct device *dev, int on_off);

/*
 * weak(dummy) function for PRD control
 * These are deactivated by enabling __SIW_SUPPORT_PRD
 * and the actual functions can be found in siw_touch_hal_prd.c
 */
int __weak siw_hal_prd_sysfs(struct device *dev, int on_off)
{
	t_dev_info_once(dev, "PRD disabled\n");
	return 0;
}

static int siw_hal_reset_ctrl(struct device *dev, int ctrl);

static int siw_hal_tc_driving(struct device *dev, int mode);

static int siw_hal_lpwg_mode(struct device *dev);

static void siw_hal_power_init(struct device *dev)
{
	siw_touch_power_init(dev);
}

static void siw_hal_power_free(struct device *dev)
{
	siw_touch_power_free(dev);
}

static void siw_hal_power_vdd(struct device *dev, int value)
{
	siw_touch_power_vdd(dev, value);
}

static void siw_hal_power_vio(struct device *dev, int value)
{
	siw_touch_power_vio(dev, value);
}

#define SIW_HAL_GPIO_RST		"siw_hal_reset"
#define SIW_HAL_GPIO_IRQ		"siw_hal_irq"

static int __siw_hal_gpio_skip_reset(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int reset_pin = touch_reset_pin(ts);

	if (touch_flags(ts) & TOUCH_SKIP_RESET_PIN) {
		return 1;
	}

	if (!gpio_is_valid(reset_pin)) {
		t_dev_err(dev, "reset_pin invalid, %d\n", reset_pin);
		return 1;
	}

	return 0;
}

static void __siw_hal_init_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);
	int ret = 0;

	if (__siw_hal_gpio_skip_reset(ts)) {
		return;
	}

	ret = siw_touch_gpio_init(dev,
			reset_pin,
			SIW_HAL_GPIO_RST);
	if (ret)
		return;

	siw_touch_gpio_direction_output(dev,
			reset_pin, GPIO_OUT_ONE);
	t_dev_dbg_gpio(dev, "set %s(%d) as output\n",
			SIW_HAL_GPIO_RST, reset_pin);

	siw_touch_gpio_set_pull(dev,
			reset_pin, GPIO_PULL_UP);
	t_dev_dbg_gpio(dev, "set %s(%d) as pull-up(%d)\n",
			SIW_HAL_GPIO_RST,
			reset_pin, GPIO_NO_PULL);
}

static void __siw_hal_free_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);

	if (__siw_hal_gpio_skip_reset(ts)) {
		return;
	}

	siw_touch_gpio_free(dev, reset_pin);
}

static void __siw_hal_set_gpio_reset(struct device *dev, int val)
{
	struct siw_ts *ts = to_touch_core(dev);
	int reset_pin = touch_reset_pin(ts);

	if (__siw_hal_gpio_skip_reset(ts)) {
		return;
	}

	siw_touch_gpio_direction_output(dev,
			reset_pin, !!(val));
	t_dev_dbg_gpio(dev, "set %s(%d) : %d\n",
			SIW_HAL_GPIO_RST,
			reset_pin, !!(val));
}

static void siw_hal_init_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_init_reset) {
		ret = fquirks->gpio_init_reset(dev);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_init_gpio_reset(dev);
}

static void siw_hal_free_gpio_reset(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_free_reset) {
		ret = fquirks->gpio_free_reset(dev);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_free_gpio_reset(dev);
}

static void siw_hal_set_gpio_reset(struct device *dev, int val)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_set_reset) {
		ret = fquirks->gpio_set_reset(dev, val);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_set_gpio_reset(dev, val);
}

static void siw_hal_trigger_gpio_reset(struct device *dev, int delay)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);
	touch_msleep(chip->drv_reset_low + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_0));
	siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);

	t_dev_info(dev, "trigger gpio reset\n");

	touch_msleep((delay) ? delay : ts->caps.hw_reset_delay);
}

static int __siw_hal_gpio_skip_irq(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	int irq_pin = touch_irq_pin(ts);

	if (!gpio_is_valid(irq_pin)) {
		t_dev_err(dev, "irq_pin invalid, %d\n", irq_pin);
		return 1;
	}

	return 0;
}

static void __siw_hal_init_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int irq_pin = touch_irq_pin(ts);
	int ret = 0;

	if (__siw_hal_gpio_skip_irq(ts)) {
		return;
	}

	ret = siw_touch_gpio_init(dev,
			irq_pin,
			SIW_HAL_GPIO_IRQ);
	if (ret)
		return;

	siw_touch_gpio_direction_input(dev,
			irq_pin);
	t_dev_dbg_gpio(dev, "set %s(%d) as input\n",
			SIW_HAL_GPIO_IRQ,
			irq_pin);

	siw_touch_gpio_set_pull(dev,
			irq_pin, GPIO_PULL_UP);
	t_dev_dbg_gpio(dev, "set %s(%d) as pull-up(%d)\n",
			SIW_HAL_GPIO_IRQ,
			irq_pin, GPIO_PULL_UP);
}

static void __siw_hal_free_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int irq_pin = touch_irq_pin(ts);

	if (__siw_hal_gpio_skip_irq(ts)) {
		return;
	}

	siw_touch_gpio_free(dev, irq_pin);
}

static void siw_hal_init_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_init_irq) {
		ret = fquirks->gpio_init_irq(dev);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_init_gpio_irq(dev);
}

static void siw_hal_free_gpio_irq(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_free_irq) {
		ret = fquirks->gpio_free_irq(dev);
		if (ret != -EAGAIN) {
			return;
		}
	}

	__siw_hal_free_gpio_irq(dev);
}

static void siw_hal_init_gpios(struct device *dev)
{
	siw_hal_init_gpio_reset(dev);

	siw_hal_init_gpio_irq(dev);
}

static void siw_hal_free_gpios(struct device *dev)
{
	siw_hal_free_gpio_reset(dev);

	siw_hal_free_gpio_irq(dev);
}

u32 t_bus_dbg_mask = 0;

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/bus_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko bus_dbg_mask=<value>
 */
module_param_named(bus_dbg_mask, t_bus_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);

#define SIW_HAL_BUS_TAG 		"hal(bus): "
#define SIW_HAL_BUS_TAG_ERR 	"hal(bus)(E): "
#define SIW_HAL_BUS_TAG_WARN	"hal(bus)(W): "
#define SIW_HAL_BUS_TAG_DBG		"hal(bus)(D): "

#if 1
#define t_hal_bus_info(_dev, fmt, args...)	\
		__t_dev_info(_dev, SIW_HAL_BUS_TAG fmt, ##args)

#define t_hal_bus_warn(_abt, fmt, args...)	\
		__t_dev_warn(_dev, SIW_HAL_BUS_TAG_WARN fmt, ##args)
#else
#define t_hal_bus_info(_dev, fmt, args...)	__t_dev_none(_dev, fmt, ##args)
#define t_hal_bus_warn(_dev, fmt, args...)	__t_dev_none(_dev, fmt, ##args)
#endif

#define t_hal_bus_err(_dev, fmt, args...)	\
		__t_dev_err(_dev, SIW_HAL_BUS_TAG_ERR fmt, ##args)

#define t_hal_bus_dbg(condition, _dev, fmt, args...)			\
		do {							\
			if (unlikely(t_bus_dbg_mask & (condition)))	\
				__t_dev_info(_dev, SIW_HAL_BUS_TAG_DBG fmt, ##args);	\
		} while (0)

#define t_hal_bus_dbg_base(_dev, fmt, args...)	\
		t_hal_bus_dbg(DBG_BASE, _dev, fmt, ##args)

#define t_hal_bus_dbg_trace(_dev, fmt, args...)	\
		t_hal_bus_dbg(DBG_TRACE, _dev, fmt, ##args)

#define DBG_BUS_ERR_TRACE	DBG_GET_DATA

static void __siw_hal_bus_err(struct device *dev,
		u32 addr, u8 *buf, int size, int wr)
{
	int prt_len = 0;
	int prt_idx = 0;
	int prd_sz = size;

	if (!unlikely(t_bus_dbg_mask & DBG_BUS_ERR_TRACE)) {
		return;
	}

	while (size) {
		prt_len = min(size, 16);

		t_hal_bus_err(dev,
				"%s 0x%04X, 0x%04X buf[%3d~%3d] %*ph\n",
				(wr) ? "wr" : "rd",
				(u32)addr, (u32)prd_sz,
				prt_idx, prt_idx + prt_len - 1,
				prt_len, &buf[prt_idx]);

		size -= prt_len;
		prt_idx += prt_len;
	}
}

static void __siw_hal_bus_dbg(struct device *dev,
		u32 addr, u8 *buf, int size, int wr)
{
	int prt_len = 0;
	int prt_idx = 0;
	int prd_sz = size;

	if (!unlikely(t_bus_dbg_mask & DBG_TRACE)) {
		return;
	}

	while (size) {
		prt_len = min(size, 16);

		t_hal_bus_dbg_trace(dev,
				"%s 0x%04X, 0x%04X buf[%3d~%3d] %*ph\n",
				(wr) ? "wr" : "rd",
				(u32)addr, (u32)prd_sz,
				prt_idx, prt_idx + prt_len - 1,
				prt_len, &buf[prt_idx]);

		size -= prt_len;
		prt_idx += prt_len;
	}
}

static void *__siw_hal_get_curr_buf(struct siw_ts *ts, dma_addr_t *dma, int tx)
{
	struct siw_touch_buf *t_buf;
	int *idx;
	void *buf = NULL;

	idx = (tx) ? &ts->tx_buf_idx : &ts->rx_buf_idx;
	t_buf = (tx) ? &ts->tx_buf[(*idx)] : &ts->rx_buf[(*idx)];

	buf = t_buf->buf;
	if (dma)
		*dma = t_buf->dma;

	(*idx)++;
	(*idx) %= SIW_TOUCH_MAX_BUF_IDX;

	return buf;
}

static int __siw_hal_reg_quirk_addr(struct device *dev,
			u32 addr, int size, int wr)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int t_rw_opt = chip->opt.t_rw_opt;
	int bus_type = touch_bus_type(ts);
	int last = addr + size - 1;
	int invalid_s = 0;
	int invalid_e = 0;
	int detected = 0;

	switch (t_rw_opt) {
	case 1:
		invalid_s = (bus_type == BUS_IF_I2C) ? 0x400 : 0x200;
		invalid_e = invalid_s + 0x200;

		if ((addr >= invalid_s) && (addr < invalid_e)) {
			detected |= 0x1;
		}
		if ((last >= invalid_s) && (last < invalid_e)) {
			detected |= (0x1<<1);
		}

		if (detected) {
			t_dev_info(dev,
				"invalid access(%s) : %04Xh, %04Xh (%X, %04Xh, %04Xh)\n",
				(wr) ? "wr" : "rd",
				addr, last,
				detected, invalid_s, invalid_e);
			return -EINVAL;
		}
		break;
	}

	return 0;
}

static int __siw_hal_reg_quirk_rd(struct device *dev,
			u32 *addr, int *size,
			int *hdr_sz, int *dummy_sz, int *hdr_flag)
{
	int ret;

	ret = __siw_hal_reg_quirk_addr(dev, (*addr), (*size), 0);
	if (ret < 0) {
		return -EINVAL;
	}

	return 0;
}

static int __siw_hal_reg_quirk_wr(struct device *dev,
			u32 *addr, int *size, int *hdr_sz)
{
	int ret = 0;

	ret = __siw_hal_reg_quirk_addr(dev, (*addr), (*size), 1);
	if (ret < 0) {
		return -EINVAL;
	}

	return 0;
}

//#define __SIW_CONFIG_CLR_RX_BUFFER

static int __used __siw_hal_do_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int bus_tx_hdr_size = touch_tx_hdr_size(ts);
	int bus_rx_hdr_size = touch_rx_hdr_size(ts);
//	int bus_tx_dummy_size = touch_tx_dummy_size(ts);
	int bus_rx_dummy_size = touch_rx_dummy_size(ts);
	int bus_rd_hdr_flag = 0;
	struct touch_bus_msg _msg = {0, };
	struct touch_bus_msg *msg = &_msg;
	int tx_size = bus_tx_hdr_size;
	u8 *tx_buf;
	u8 *rx_buf;
	dma_addr_t tx_dma;
	dma_addr_t rx_dma;
	int ret = 0;

	ret = __siw_hal_reg_quirk_rd(dev, &addr, &size,
		&bus_rx_hdr_size, &bus_rx_dummy_size, &bus_rd_hdr_flag);
	if (ret < 0) {
		return -EINVAL;
	}

//	t_dev_info(dev, "addr %04Xh, size %d\n", addr, size);

	tx_buf = __siw_hal_get_curr_buf(ts, &tx_dma, 1);
	rx_buf = __siw_hal_get_curr_buf(ts, &rx_dma, 0);

#if defined(__SIW_CONFIG_CLR_RX_BUFFER)
	if (touch_bus_type(ts) == BUS_IF_I2C) {
		memset(&rx_buf[bus_rx_hdr_size], 0xFF, min(8, size));
	}
#endif

	switch (chip->opt.t_bus_opt) {
	case 1:
		tx_buf[0] = ((addr >> 8) & 0xff);
		tx_buf[1] = (addr & 0xff);
		break;
	default:
		tx_buf[0] = bus_rd_hdr_flag | ((size > 4) ? 0x20 : 0x00);
		tx_buf[0] |= ((addr >> 8) & 0x0f);
		tx_buf[1] = (addr & 0xff);
		break;
	}

//	while (bus_tx_dummy_size--) {
	while (bus_rx_dummy_size--) {
		tx_buf[tx_size++] = 0;
	}

	msg->tx_buf = tx_buf;
	msg->tx_size = tx_size;
	msg->rx_buf = rx_buf;
	msg->rx_size = bus_rx_hdr_size + size;
	msg->tx_dma = tx_dma;
	msg->rx_dma = rx_dma;
	msg->bits_per_word = 8;
	msg->priv = 0;

	ret = siw_touch_bus_read(dev, msg);
	if (ret < 0) {
		t_hal_bus_err(dev, "read reg error(0x%04X, 0x%04X, %*ph), %d\n",
				(u32)addr, (u32)size,
				tx_size, tx_buf,
				ret);
		__siw_hal_bus_err(dev, addr, (u8 *)msg->rx_buf, msg->rx_size, 0);
		return ret;
	}

	memcpy(data, &rx_buf[bus_rx_hdr_size], size);

	__siw_hal_bus_dbg(dev, addr, (u8 *)data, size, 0);

	return size;
}

static int __used __siw_hal_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	if (!size) {
		t_hal_bus_dbg_base(dev, "rd: size zero\n");
		return 0;
	}

	if (siw_addr_is_skip(addr)) {
		t_hal_bus_dbg_base(dev, "rd: skip by ADDR_SKIP_MASK\n");
		return 0;
	}

	if (!data) {
		t_dev_err(dev, "rd: NULL data(0x%04X, 0x%04X)\n", addr, size);
		return -EFAULT;
	}

	mutex_lock(&chip->bus_lock);
	ret = __siw_hal_do_reg_read(dev, addr, data, size);
	mutex_unlock(&chip->bus_lock);

	return ret;
}

static int __used __siw_hal_do_reg_write(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int bus_tx_hdr_size = touch_tx_hdr_size(ts);
//	int bus_rx_hdr_size = touch_rx_hdr_size(ts);
	struct touch_bus_msg _msg = {0, };
	struct touch_bus_msg *msg = &_msg;
	u8 *tx_buf;
	dma_addr_t tx_dma;
	int is_spi = !!(touch_bus_type(ts) == BUS_IF_SPI);
	int ret = 0;

	ret = __siw_hal_reg_quirk_wr(dev, &addr, &size, &bus_tx_hdr_size);
	if (ret < 0) {
		return -EINVAL;
	}

	tx_buf = __siw_hal_get_curr_buf(ts, &tx_dma, 1);

	switch (chip->opt.t_bus_opt) {
	case 1:
		tx_buf[0] = ((addr >> 8) & 0xff);
		tx_buf[1] = (addr & 0xff);
		break;
	default:
		tx_buf[0] = (is_spi || (size > 4)) ? 0x60 : 0x40;
		tx_buf[0] |= ((addr >> 8) & 0x0f);
		tx_buf[1] = (addr & 0xff);
		break;
	}

	msg->tx_buf = tx_buf;
	msg->tx_size = bus_tx_hdr_size + size;
	msg->rx_buf = NULL;
	msg->rx_size = 0;
	msg->tx_dma = tx_dma;
	msg->rx_dma = 0;
	msg->bits_per_word = 8;
	msg->priv = 0;

	memcpy(&tx_buf[bus_tx_hdr_size], data, size);

	ret = siw_touch_bus_write(dev, msg);
	if (ret < 0) {
		t_hal_bus_err(dev, "write reg error(0x%04X, 0x%04X, %*ph - %*ph%s), %d\n",
				(u32)addr, (u32)size,
				bus_tx_hdr_size, tx_buf,
				min(size, 8), data,
				(size > 8) ? " ..." : "",
				ret);
		__siw_hal_bus_err(dev, addr, (u8 *)data, size, 1);
		return ret;
	}

	__siw_hal_bus_dbg(dev, addr, (u8 *)data, size, 1);

	return size;
}

static int __used __siw_hal_reg_write(struct device *dev, u32 addr, void *data, int size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	if (!size) {
		t_hal_bus_dbg_base(dev, "wr: size zero\n");
		return 0;
	}

	if (siw_addr_is_skip(addr)) {
		t_hal_bus_dbg_base(dev, "wr: skip by ADDR_SKIP_MASK\n");
		return 0;
	}

	if (!data) {
		t_dev_err(dev, "wr: NULL data(0x%04X, 0x%04X)\n", addr, size);
		return -EFAULT;
	}

	mutex_lock(&chip->bus_lock);
	ret = __siw_hal_do_reg_write(dev, addr, data, size);
	mutex_unlock(&chip->bus_lock);

	return ret;
}

int siw_hal_read_value(struct device *dev, u32 addr, u32 *value)
{
	return __siw_hal_reg_read(dev, addr, value, sizeof(u32));
}

int siw_hal_write_value(struct device *dev, u32 addr, u32 value)
{
	return __siw_hal_reg_write(dev, addr, &value, sizeof(u32));
}

int siw_hal_reg_read(struct device *dev, u32 addr, void *data, int size)
{
	return __siw_hal_reg_read(dev, addr, data, size);
}

int siw_hal_reg_write(struct device *dev, u32 addr, void *data, int size)
{
	return __siw_hal_reg_write(dev, addr, data, size);
}

int siw_hal_reg_read_single(struct device *dev, u32 addr, void *data, int size)
{
	u32 *__data = (u32 *)data;
	int __size;
	int ret = 0;

	while (size) {
		__size = min(4, size);
		ret = siw_hal_reg_read(dev, addr, __data, __size);
		if (ret < 0) {
			break;
		}

		addr++;
		__data++;
		size -= __size;
	}

	return ret;
}

int siw_hal_reg_write_single(struct device *dev, u32 addr, void *data, int size)
{
	u32 *__data = (u32 *)data;
	int __size;
	int ret = 0;

	while (size) {
		__size = min(4, size);
		ret = siw_hal_reg_write(dev, addr, __data, __size);
		if (ret < 0) {
			break;
		}

		addr++;
		__data++;
		size -= __size;
	}

	return ret;
}

int siw_hal_reg_rw_multi(struct device *dev,
		struct siw_hal_rw_multi *multi, char *title)
{
	int (*func)(struct device *dev, u32 addr, void *data, int size);
	int ret = 0;

	while (1) {
		if ((multi->wr == -1) ||
			(multi->addr == -1) ||
			(multi->data == NULL)) {
			break;
		}

		func = (multi->wr) ? siw_hal_reg_write : siw_hal_reg_read;

		ret = func(dev, multi->addr, multi->data, multi->size);
		if (ret < 0) {
			t_dev_err(dev, "%s: %s %s failed, %d\n",
				title, (multi->name) ? multi->name : "",
				(multi->wr) ? "write" : "read",
				ret);
			break;
		}

		multi++;
	}

	return ret;
}

static int __siw_hal_reg_bit_mask(struct device *dev, u32 addr, u32 *value, u32 mask, int set)
{
	const char *str = (set) ? "set" : "clr";
	u32 rdata = 0;
	u32 data = 0;
	int ret = 0;

	ret = siw_hal_read_value(dev, addr, &data);
	if (ret < 0) {
		t_dev_err(dev, "bit %s failed, read addr 0x%04X, mask %X\n",
			str, addr, mask);
		return ret;
	}

	rdata = data;

	if (set) {
		data |= mask;
	} else {
		data &= ~mask;
	}

	ret = siw_hal_write_value(dev, addr, data);
	if (ret < 0) {
		t_dev_err(dev, "bit %s failed, write addr 0x%04X, rdata 0x%08X, mask %X\n",
			str, addr, rdata, mask);
		return ret;
	}

	if (value)
		*value = data;

	return 0;
}

int siw_hal_reg_bit_set(struct device *dev, u32 addr, u32 *value, u32 mask)
{
	return __siw_hal_reg_bit_mask(dev, addr, value, mask, 1);
}

int siw_hal_reg_bit_clr(struct device *dev, u32 addr, u32 *value, u32 mask)
{
	return __siw_hal_reg_bit_mask(dev, addr, value, mask, 0);
}

static int siw_hal_condition_wait(struct device *dev,
				    u16 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry)
{
	u32 data = 0;
	int ret = 0;

	do {
		touch_msleep(delay);

		ret = siw_hal_read_value(dev, addr, &data);
		switch (expect) {
		case FLASH_CODE_DNCHK_VALUE:
		case FLASH_CONF_DNCHK_VALUE:
			t_dev_dbg_base(dev,
				"wait read: addr[%04Xh] data[%08Xh], "
				"mask[%08Xh], expect[%08Xh], %d\n",
				addr, data, mask, expect, retry);
			break;
		}
		if ((ret >= 0) && ((data & mask) == expect)) {
			if (value)
				*value = data;
		#if 0
			t_dev_info(dev,
				"wait done: addr[%04Xh] data[%08Xh], "
				"mask[%08Xh], expect[%08Xh], %d\n",
				addr, data, mask, expect, retry);
		#endif
			return 0;
		}
	} while (--retry);

	if (value)
		*value = data;

	t_dev_err(dev,
		"wait fail: addr[%04Xh] data[%08Xh], "
		"mask[%08Xh], expect[%08Xh]\n",
		addr, data, mask, expect);

	return -EPERM;
}

static int siw_hal_flash_wp(struct device *dev, int wp)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->flash_wp) {
		ret = fquirks->flash_wp(dev, wp);
		t_dev_info(dev, "flash_wp(%s) %s\n",
			touch_chip_name(ts), (wp) ? "enabled" : "disabled");
	}

	return ret;
}

int siw_hal_enable_flash_wp(struct device *dev)
{
	return siw_hal_flash_wp(dev, 1);
}

int siw_hal_disable_flash_wp(struct device *dev)
{
	return siw_hal_flash_wp(dev, 0);
}

int siw_hal_access_not_allowed(struct device *dev, char *title, int skip_flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	char *sub = "";

	if (!(skip_flag & HAL_ACCESS_CHK_SKIP_SLEEP)) {
		if (atomic_read(&ts->state.sleep) != IC_NORMAL) {
			sub = "not IC_NORMAL";
			goto out;
		}
	}

	if (!(skip_flag & HAL_ACCESS_CHK_SKIP_FB)) {
		if (atomic_read(&ts->state.fb) != FB_RESUME) {
			sub = "not FB_RESUME";
			goto out;
		}
	}

	if (!(skip_flag & HAL_ACCESS_CHK_SKIP_PM)) {
		if (atomic_read(&ts->state.pm) != DEV_PM_RESUME) {
			sub = "not DEV_PM_RESUME";
			goto out;
		}
	}

	if (!(skip_flag & HAL_ACCESS_CHK_SKIP_INIT)) {
		if (atomic_read(&chip->init) != IC_INIT_DONE) {
			sub = "not IC_INIT_DONE";
			goto out;
		}
	}

	return 0;

out:
	if (title) {
		t_dev_warn(dev, "%s: %s\n", title, sub);
	}

	return 1;
}

static int siw_hal_tc_not_allowed(struct device *dev, char *title)
{
#if defined(__SIW_CONFIG_SYSTEM_PM)
	int skip_flag = 0;
#else	/* __SIW_CONFIG_SYSTEM_PM */
	int skip_flag = HAL_ACCESS_CHK_SKIP_PM | HAL_ACCESS_CHK_SKIP_FB;
#endif	/* __SIW_CONFIG_SYSTEM_PM */

	return siw_hal_access_not_allowed(dev, title, skip_flag);
}

static void siw_hal_init_works(struct siw_touch_chip *chip)
{

}

static void siw_hal_free_works(struct siw_touch_chip *chip)
{

}

static void siw_hal_init_locks(struct siw_touch_chip *chip)
{
	mutex_init(&chip->bus_lock);
}

static void siw_hal_free_locks(struct siw_touch_chip *chip)
{
	mutex_destroy(&chip->bus_lock);
}

static u32 siw_hal_get_subdisp_sts(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 rdata = LCD_MODE_U3;	//dummy value

	siw_hal_read_value(dev, reg->spr_subdisp_status, &rdata);

	return rdata;
}

int siw_hal_get_boot_status(struct device *dev, u32 *boot_st)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	struct siw_hal_reg *reg = chip->reg;
	u32 bootmode = 0;
	int ret = 0;

	if (fquirks->boot_status) {
		ret = fquirks->boot_status(dev, boot_st);
		if (ret != -EAGAIN) {
			return ret;
		}
	}

	ret = siw_hal_read_value(dev, reg->spr_boot_status, &bootmode);
	if (ret < 0) {
		return ret;
	}

	if (boot_st) {
		*boot_st = bootmode;
	}

	return 0;
}

static const char *siw_hal_pwr_name[] = {
	[POWER_OFF]		= "Power off",
	[POWER_SLEEP]	= "Power sleep",
	[POWER_WAKE]	= "Power wake",
	[POWER_ON]		= "Power on",
};

static int siw_hal_power_core(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int pwr_s_delay = chip->pwr_s_delay;
	int pwr_g_delay = chip->pwr_g_delay;

	switch (ctrl) {
	case POWER_OFF:
		t_dev_dbg_pm(dev, "power core: power off\n");
		atomic_set(&chip->init, IC_INIT_NEED);

		touch_msleep(pwr_s_delay);

		siw_hal_power_vio(dev, 0);

		touch_msleep(pwr_s_delay);

		siw_hal_power_vdd(dev, 0);

		touch_msleep(chip->drv_reset_low + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_0));
		break;

	case POWER_ON:
		t_dev_dbg_pm(dev, "power core: power on\n");

		touch_msleep(pwr_s_delay);

		siw_hal_power_vdd(dev, 1);

		touch_msleep(pwr_s_delay);

		siw_hal_power_vio(dev, 1);

		touch_msleep(pwr_g_delay);
		break;

	case POWER_SLEEP:
		break;

	case POWER_WAKE:
		break;

	case POWER_HW_RESET:
		break;
	}

	return 0;
}

static int siw_hal_power(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if ((ctrl < 0) || (ctrl > POWER_ON)) {
		t_dev_err(dev, "power ctrl: wrong ctrl value, %d\n", ctrl);
		return -EINVAL;
	}

	t_dev_dbg_pm(dev, "power ctrl: %s - %s\n",
			touch_chip_name(ts), siw_hal_pwr_name[ctrl]);

	switch (ctrl) {
	case POWER_OFF:
		t_dev_dbg_pm(dev, "power ctrl: power off\n");
		atomic_set(&chip->init, IC_INIT_NEED);

		siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);

		siw_hal_power_core(dev, ctrl);
		break;

	case POWER_ON:
		t_dev_dbg_pm(dev, "power ctrl: power on\n");

		siw_hal_power_core(dev, ctrl);

		siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);
		break;

	case POWER_SLEEP:
		t_dev_dbg_pm(dev, "power ctrl: sleep\n");
		break;

	case POWER_WAKE:
		t_dev_dbg_pm(dev, "power ctrl: wake\n");
		break;

	case POWER_HW_RESET:
		t_dev_info(dev, "power ctrl: reset\n");
		siw_hal_reset_ctrl(dev, HW_RESET_ASYNC);
		break;
	}

	return 0;
}

enum {
	BOOT_CHK_SKIP = (1<<16),
};

static int siw_hal_chk_boot_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	u32 boot_failed = 0;
	u32 bootmode = 0;
	u32 boot_chk_offset_busy = siw_hal_boot_sts_pos_busy(chip);
	u32 boot_chk_offset_err = siw_hal_boot_sts_pos_dump_err(chip);
	u32 boot_chk_empty_mask = siw_hal_boot_sts_mask_empty(chip);
	int ret = 0;

	ret = siw_hal_get_boot_status(dev, &bootmode);
	if (ret < 0) {
		return ret;
	}

	/* maybe nReset is low state */
	if (!bootmode || (bootmode == ~0)) {
		return BOOT_CHK_SKIP;
	}

	/* booting... need to wait */
	if ((bootmode >> boot_chk_offset_busy) & 0x1) {
		return BOOT_CHK_SKIP;
	}

	boot_failed |= !!((bootmode >> boot_chk_offset_err) & 0x1);	/* CRC error */
	boot_failed |= (!!(bootmode & boot_chk_empty_mask))<<1;

	if (boot_failed) {
		t_dev_err(dev, "boot fail: boot sts  = %08Xh(%02Xh)\n",
			bootmode, boot_failed);
	}

	return boot_failed;
}

static int siw_hal_chk_boot_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_status_mask_bit *mask_bit = &chip->status_mask_bit;
	u32 boot_failed = 0;
	u32 tc_status = 0;
	u32 valid_cfg_crc_mask = 0;
	u32 valid_code_crc_mask = 0;
	int ret = 0;

	valid_cfg_crc_mask = mask_bit->valid_cfg_crc;
	valid_code_crc_mask = mask_bit->valid_code_crc;

	ret = siw_hal_read_value(dev,
			reg->tc_status,
			&tc_status);
	if (ret < 0) {
		return ret;
	}

	/* maybe nReset is low state */
	if (!tc_status || (tc_status == ~0)) {
		return BOOT_CHK_SKIP;
	}

	if (valid_cfg_crc_mask && !(tc_status & valid_cfg_crc_mask)) {
		boot_failed |= (1<<5);
	}
	if (valid_code_crc_mask && !(tc_status & valid_code_crc_mask)) {
		boot_failed |= (1<<4);
	}
	if (boot_failed) {
		t_dev_err(dev, "boot fail: tc_status = %08Xh(%02Xh)\n",
			tc_status, boot_failed);
	}

	return boot_failed;
}

enum {
	BOOT_CHK_MODE_RETRY = 2,
	BOOT_CHK_STS_RETRY	= 2,
	/* */
	BOOT_CHK_MODE_DELAY	= 10,
	BOOT_CHK_STS_DELAY	= 10,
};

static int siw_hal_chk_boot(struct device *dev)
{
	u32 boot_failed = 0;
	int retry;
	int ret = 0;

	retry = BOOT_CHK_MODE_RETRY;
	while (retry--) {
		ret = siw_hal_chk_boot_mode(dev);
		if (ret < 0) {
			return ret;
		}
		if (ret == BOOT_CHK_SKIP) {
			return 0;
		}
		if (!ret) {
			break;
		}
		touch_msleep(BOOT_CHK_MODE_DELAY);
	}
	boot_failed |= ret;

	retry = BOOT_CHK_STS_RETRY;
	while (retry--) {
		ret = siw_hal_chk_boot_status(dev);
		if (ret < 0) {
			return ret;
		}
		if (ret == BOOT_CHK_SKIP) {
			return boot_failed;
		}
		if (!ret) {
			break;
		}
		touch_msleep(BOOT_CHK_STS_DELAY);
	}
	boot_failed |= ret;

	return boot_failed;
}

#if defined(__SIW_SUPPORT_ALIVE_DETECTION)
u32 t_alive_dbg_mask = 0;

module_param_named(alive_dbg_mask, t_alive_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);

#define SIW_HAL_ALIVE_TAG_DBG		"alive(D): "

#define t_alive_dbg(condition, _dev, fmt, args...)			\
		do {							\
			if (unlikely(t_alive_dbg_mask & (condition)))	\
				__t_dev_info(_dev, SIW_HAL_ALIVE_TAG_DBG fmt, ##args);	\
		} while (0)

#define t_alive_dbg_base(_dev, fmt, args...)	\
		t_alive_dbg(DBG_BASE, _dev, fmt, ##args)

#define t_alive_dbg_trace(_dev, fmt, args...)	\
		t_alive_dbg(DBG_TRACE, _dev, fmt, ##args)

#define ALIVE_MON_TIME_MSEC		2000
#define ALIVE_MON_TIME_ADD		1000

static void __alive_mon_do_stop(struct device *dev, int log)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	cancel_delayed_work(&chip->alive_mon_work);

	chip->alive_mon_run = 0;

	if (log) {
		t_dev_info(dev, "alive mon stops\n");
	}
}

static void __alive_mon_stop(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (!chip->alive_mon || !chip->alive_mon_init)
		return;

	if (!chip->alive_mon_run)
		return;

	mutex_lock(&chip->alive_mon_lock);

	__alive_mon_do_stop(dev, 1);

	mutex_unlock(&chip->alive_mon_lock);
}

static void __alive_mon_do_run(struct device *dev, int log)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int mon_time = ALIVE_MON_TIME_MSEC;

	/*
	 *  500 ~ 1660 ms : 2000 ms
	 * 1820 ~ 2000 ms : 3000 ms
	 */
	mon_time += (chip->alive_level >= 9) ? ALIVE_MON_TIME_ADD : 0;

	queue_delayed_work(ts->wq, &chip->alive_mon_work, msecs_to_jiffies(mon_time));

	chip->alive_mon_run = 1;

	if (log) {
		t_dev_info(dev, "alive mon[%d msec for Lv.%d] begins\n",
			mon_time, chip->alive_level);
	}
}

static void __alive_mon_run(struct device *dev, int restart)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (!chip->alive_mon || !chip->alive_mon_init)
		return;

	if (!restart) {
		if (chip->alive_mon_run)
			return;
	}

	mutex_lock(&chip->alive_mon_lock);

	if (restart) {
		if (chip->alive_mon_run)
			__alive_mon_do_stop(dev, 1);
	}

	chip->alive_irq_detect = 0;

	__alive_mon_do_run(dev, 1);

	mutex_unlock(&chip->alive_mon_lock);
}

static int __alive_mon_work_skip(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (siw_hal_access_not_allowed(dev, "alive mon", 0))
		return 1;

	if (chip->lcd_mode != LCD_MODE_U3) {
		return 1;
	}

	if (chip->driving_mode != LCD_MODE_U3) {
		return 1;
	}

	return 0;
}

static void __alive_mon_work(struct work_struct *work)
{
	struct siw_touch_chip *chip =
			container_of(to_delayed_work(work),
						struct siw_touch_chip, alive_mon_work);
	struct device *dev = chip->dev;
	int do_reset = 0;

	if (!chip->alive_mon || !chip->alive_mon_init)
		return;

	if (__alive_mon_work_skip(dev))
		return;

	mutex_lock(&chip->alive_mon_lock);

	do_reset = !chip->alive_irq_detect;

	chip->alive_irq_detect = 0;

	if (do_reset) {
		t_dev_err(dev, "alive mon detects no irq (%d), trigger reset\n",
			chip->driving_mode);

		mutex_unlock(&chip->alive_mon_lock);

		siw_hal_reset_ctrl(dev, HW_RESET_ASYNC);
		return;
	}

	t_alive_dbg_base(dev, "alive mon ok\n");

	__alive_mon_do_run(dev, 0);

	mutex_unlock(&chip->alive_mon_lock);
}

static void __alive_mon_check(struct device *dev, int irq_type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (!chip->alive_mon || !chip->alive_mon_init)
		return;

#if 0
	if (chip->alive_pause != ALIVE_RESUME) {
		/* pause state for a specific operation */
		return;
	}
#endif

	switch (irq_type) {
	case TC_STS_IRQ_TYPE_ABNORMAL:
		__alive_mon_stop(dev);
		break;

	case TC_STS_IRQ_TYPE_ALIVE:
	case TC_STS_IRQ_TYPE_REPORT:
		mutex_lock(&chip->alive_mon_lock);
		chip->alive_irq_detect = 1;
		mutex_unlock(&chip->alive_mon_lock);
		break;
	}
}

static void __alive_mon_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (!chip->alive_mon)
		return;

	if (chip->alive_mon_init)
		return;

	mutex_init(&chip->alive_mon_lock);

	INIT_DELAYED_WORK(&chip->alive_mon_work, __alive_mon_work);

	chip->alive_mon_run = 0;

	chip->alive_mon_init = 1;

	chip->alive_irq_detect = 0;

	t_dev_info(dev, "alive mon init\n");
}

static void __alive_mon_free(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (!chip->alive_mon)
		return;

	if (!chip->alive_mon_init)
		return;

	__alive_mon_stop(dev);

	mutex_destroy(&chip->alive_mon_lock);

	chip->alive_mon_init = 0;

	t_dev_info(dev, "alive mon free\n");
}

static void siw_hal_alive_mon_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);

	if (fquirks->alive_mon_init) {
		/* disable default mon */
		chip->alive_mon = 0;

		fquirks->alive_mon_init(dev);
	} else {
		__alive_mon_init(dev);
	}
}

static void siw_hal_alive_mon_free(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);

	if (fquirks->alive_mon_free) {
		fquirks->alive_mon_free(dev);
	} else {
		__alive_mon_free(dev);
	}
}

static void siw_hal_alive_mon_run(struct device *dev, int restart)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);

	if (fquirks->alive_mon_run) {
		fquirks->alive_mon_run(dev, restart);
	} else {
		__alive_mon_run(dev, restart);
	}

}

static void siw_hal_alive_mon_stop(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);

	if (fquirks->alive_mon_stop) {
		fquirks->alive_mon_stop(dev);
	} else {
		__alive_mon_stop(dev);
	}
}

static void siw_hal_alive_mon_check(struct device *dev, int irq_type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);

	if (fquirks->alive_mon_check) {
		fquirks->alive_mon_check(dev, irq_type);
	} else {
		__alive_mon_check(dev, irq_type);
	}
}

static void __alive_setup(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	struct siw_hal_fw_info *fw = &chip->fw;

	if (fw->invalid_pid)
		return;

	if (chip->alive_reg)
		return;

	if (fquirks->alive_setup) {
		fquirks->alive_setup(dev);
	}
}

static int __alive_is_valid(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (!chip->alive_reg)
		return 0;

	if (!chip->alive_level_min)
		return 0;

	if (!chip->alive_level_def)
		return 0;

	if (!chip->alive_level_max)
		return 0;

	return 1;
}

int siw_hal_alive_is_active(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	return (chip->alive_init_done && chip->alive_is_active);
}

int siw_hal_alive_pause_get(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	return chip->alive_pause;
}

int siw_hal_alive_pause_set(struct device *dev, int pause)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int init_done = chip->alive_init_done;
	int curr = chip->alive_pause;
	const char *str = NULL;
	int ret = 0;

	if (pause == ALIVE_NA) {
		/* exception */
	} else if (!siw_hal_alive_is_active(dev)) {
		return 0;
	}

	if (fquirks->alive_pause_set) {
		fquirks->alive_pause_set(dev, pause);
	}

	if (pause == curr) {
		if (pause != ALIVE_RESUME) {
			t_dev_warn(dev, "already alive pause[%d] state\n", pause);
		}

		ret = -EINVAL;
		goto out;
	}

	str = __alive_pause_str(pause);
	if (!strcmp(str, PAUSE_INVALID_STR)) {
		t_dev_warn(dev, "invald pause state, %d\n", pause);
		return -EINVAL;
	}

	chip->alive_pause = pause;

	t_dev_info(dev, "alive %s(%d)\n", str, pause);

out:
	/*
	 * Post action following pause level transition
	 */
	if (pause == ALIVE_RESUME) {
		/* Run alive monitoring */
		siw_hal_alive_mon_run(dev, 0);
	} else {
		/* Stop alive monitoring including its timeout detection */
		siw_hal_alive_mon_stop(dev);
	}

	if (!init_done) {
		return ret;
	}

	/* TBD */
	if (pause == ALIVE_RESUME) {

	} else {

	}

	return ret;
}

static void __alive_na(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	chip->alive_is_active = 0;
	chip->alive_pause = 0;
	chip->alive_level = 0;

	siw_hal_alive_pause_set(dev, ALIVE_NA);
}

static int __alive_level_set(struct device *dev, int level, int set)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	u32 data[2];
	u32 addr = chip->alive_reg;
	int min_level = chip->alive_level_min;
	int def_level = chip->alive_level_def;
	int max_level = chip->alive_level_max;
	int cur_level = 0;
	int update = 0;
	int ret = 0;

	if (!__alive_is_valid(dev)) {
		return -EPERM;
	}

	/*
	 * set new level
	 */
	if (level == -1) {
		level = def_level;
	}

	ret = siw_hal_read_value(dev, addr, &cur_level);
	if (ret < 0) {
		return ret;
	}

	if ((cur_level < min_level) || (cur_level > max_level)) {
		t_dev_info(dev, "alive not supported\n");
		return -EPERM;
	}

	if ((level < min_level) || (level > max_level)) {
		t_dev_err(dev, "alive invalid level, %d\n", level);
		return -EINVAL;
	}

	update = (level != cur_level);

	data[0] = level;
	data[1] = 1;
	ret = siw_hal_reg_write(dev, addr, data, sizeof(data));
	if (ret < 0) {
		return ret;
	}

	chip->alive_level = level;

	t_dev_info(dev, "alive level: %s%s%s\n",
		__alive_level_str(level),
		(set && update) ? " <- " : "",
		(set && update) ? __alive_level_str(cur_level) : "");

	return update;
}

int siw_hal_alive_level_set(struct device *dev, int level)
{
	int ret = 0;

	if (!siw_hal_alive_is_active(dev)) {
		return 0;
	}

	ret = __alive_level_set(dev, level, 1);
	if (ret == -EPERM) {
		__alive_na(dev);
	}

	/* level changed */
	if (ret == 1) {
		siw_hal_alive_mon_run(dev, 1);
	}

	return ret;
}

static int siw_hal_alive_check(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	char *title = NULL;
	u32 status = 0;
	u32 irq_type = 0;
	int count = 0;
	int ret = 0;

	if (!__alive_is_valid(dev))
		return 0;

	status = siw_report_tc_status(chip);

	if (!(status & chip->status_mask_bit.valid_irq_pin)) {
		return 0;
	}

	irq_type = siw_tc_sts_irq_type(status);

	/*
	 * Operation for each interrupt type.
	 */
	switch (irq_type) {
	/* Alive */
	case TC_STS_IRQ_TYPE_ALIVE:
		count = ++(chip->irq_count_alive);
		title = "irq_count_alive";
		break;
	/* Touch report */
	case TC_STS_IRQ_TYPE_REPORT:
		count = ++(chip->irq_count_report);
		title = "irq_count_report";
		break;
	/* The others */
	default:
		count = ++(chip->irq_count_others);
		title = "irq_count_others";
		break;
	}

	t_alive_dbg_trace(dev, "%s: %d\n", title, count);

	if (!chip->alive_is_active)
		return 0;

	siw_hal_alive_mon_check(dev, irq_type);

	if (fquirks->alive_check) {
		/* When touch reset required, return -ERESTART. */
		ret = fquirks->alive_check(dev, irq_type);
	}

	if (ret < 0) {
		t_dev_err(dev, "irq mon failed, %d\n", ret);
		if (ret == -ERESTART) {
			if (touch_flags(ts) & TOUCH_SKIP_RESET_PIN) {
				siw_touch_irq_control(dev, INTERRUPT_ENABLE);
				/*
				 * When reset is handled by external component,
				 * some delay is required to guarantee bus access safety.
				 */
				touch_msleep(20);
			}
		}
		return ret;
	}

	return 0;
}

static int siw_hal_alive_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int is_probe = (atomic_read(&ts->state.core) == CORE_PROBE);
	int is_stop = (chip->lcd_mode == LCD_MODE_STOP);
	int level = 0;
	int pause_level = (is_stop) ? ALIVE_PAUSE_TC_STOP : ALIVE_RESUME;
	int ret = 0;

	__alive_setup(dev);

	if (!__alive_is_valid(dev)) {
		__alive_na(dev);
		return 0;
	}

	level = (is_probe) ? chip->alive_level_def : chip->alive_level;
	if (!level) {
		level = chip->alive_level_def;
	}

	ret = __alive_level_set(dev, level, 0);
	if (ret == -EPERM) {
		__alive_na(dev);

		t_dev_info(dev, "alive setup: 0x%X, %d, %d, %d, %d\n",
			chip->alive_reg,
			chip->alive_level_min,
			chip->alive_level_def,
			chip->alive_level_max,
			chip->alive_mon);
		return 0;
	} else if (ret < 0) {
		return ret;
	}

	siw_hal_alive_mon_init(dev);

	chip->alive_is_active = 1;

	chip->alive_init_done = 1;

	siw_hal_alive_pause_set(dev, pause_level);

	if (touch_flags(ts) & TOUCH_USE_MON_THREAD) {
		t_dev_dbg_base(dev, "alive is active : mon thread is suspended\n");
	}

	t_dev_info(dev, "alive init [0x%X, %d, %d, %d, %d]\n",
		chip->alive_reg,
		chip->alive_level_min,
		chip->alive_level_def,
		chip->alive_level_max,
		chip->alive_mon);

	return 0;
}

static void siw_hal_alive_free(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (!__alive_is_valid(dev))
		return;

	siw_hal_alive_mon_free(dev);

	chip->alive_is_active = 0;

	chip->alive_init_done = 0;

	t_dev_info(dev, "alive free\n");
}
#else	/* !__SIW_SUPPORT_ALIVE_DETECTION */
int siw_hal_alive_is_active(struct device *dev)
{
	return 0;
}

int siw_hal_alive_level_set(struct device *dev, int level)
{
	return 0;
}

int siw_hal_alive_pause_get(struct device *dev)
{
	return 0;
}

int siw_hal_alive_pause_set(struct device *dev, int level)
{
	return 0;
}

static inline int siw_hal_alive_check(struct device *dev)
{
	return 0;
}

static inline int siw_hal_alive_init(struct device *dev)
{
	return 0;
}

static inline void siw_hal_alive_free(struct device *dev)
{

}
#endif	/* __SIW_SUPPORT_ALIVE_DETECTION */

//#define __SIW_SUPPORT_STATUS_OPT_IGNORE_ABNORMAL
//#define __SIW_SUPPORT_STATUS_OPT_IGNORE_DISP_ERR

enum {
	IC_DEBUG_SIZE		= 16,	// byte
	//
	IC_CHK_LOG_MAX		= (1<<9),
	//
	INT_IC_ABNORMAL_STATUS	= (1<<0),
	//
	INT_IC_ERROR_STATUS = ((1<<5) | (1<<3)),
};

static const struct siw_hal_status_filter status_filter_type_1[] = {
	_STS_FILTER(STS_ID_VALID_DEV_CTL, 1, STS_POS_VALID_DEV_CTL,
		0, "device ctl not set"),
	_STS_FILTER(STS_ID_VALID_CODE_CRC, 1, STS_POS_VALID_CODE_CRC,
		0, "code crc invalid"),
	_STS_FILTER(STS_ID_VALID_CFG_CRC, 1, STS_POS_VALID_CFG_CRC,
		0, "cfg crc invalid"),
	_STS_FILTER(STS_ID_ERROR_ABNORMAL, 1, STS_POS_ERROR_ABNORMAL,
		STS_FILTER_FLAG_TYPE_ERROR | STS_FILTER_FLAG_CHK_FAULT,
		"abnormal status detected"),
	_STS_FILTER(STS_ID_ERROR_SYSTEM, 1, STS_POS_ERROR_SYSTEM,
		STS_FILTER_FLAG_TYPE_ERROR,
		"system error detected"),
	_STS_FILTER(STS_ID_ERROR_MISMTACH, 1, STS_POS_ERROR_MISMTACH,
		STS_FILTER_FLAG_TYPE_ERROR,
		"display mode mismatch"),
	_STS_FILTER(STS_ID_VALID_IRQ_PIN, 1, STS_POS_VALID_IRQ_PIN,
		0, "irq pin invalid"),
	_STS_FILTER(STS_ID_VALID_IRQ_EN, 1, STS_POS_VALID_IRQ_EN,
		0, "irq status invalid"),
	_STS_FILTER(STS_ID_VALID_TC_DRV, 1, STS_POS_VALID_TC_DRV,
		0, "driving invalid"),
	/* end mask */
	_STS_FILTER(STS_ID_NONE, 0, 0, 0, NULL),
};

static const struct siw_hal_status_filter status_filter_type_2[] = {
	_STS_FILTER(STS_ID_VALID_DEV_CTL, 1, STS_POS_VALID_DEV_CTL,
		0, "device ctl not set"),
	_STS_FILTER(STS_ID_ERROR_ABNORMAL, 1, STS_POS_ERROR_ABNORMAL,
		STS_FILTER_FLAG_TYPE_ERROR | STS_FILTER_FLAG_CHK_FAULT,
		"re-init required"),
	_STS_FILTER(STS_ID_VALID_IRQ_PIN, 1, STS_POS_VALID_IRQ_PIN,
		0, "irq pin invalid"),
	_STS_FILTER(STS_ID_VALID_IRQ_EN, 1, STS_POS_VALID_IRQ_EN,
		0, "irq status invalid"),
	_STS_FILTER(STS_ID_VALID_TC_DRV, 1, STS_POS_VALID_TC_DRV,
		0, "driving invalid"),
	/* end mask */
	_STS_FILTER(STS_ID_NONE, 0, 0, 0, NULL),
};

static u32 siw_hal_get_status_mask(struct device *dev, int id)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_status_filter *filter = chip->status_filter;
	u32 mask = 0;

	if (filter == NULL)
		goto out;

	while (1) {
		if (!filter->id || !filter->width) {
			break;
		}

		if (filter->id == id) {
			mask = ((1<<filter->width)-1)<<filter->pos;
			break;
		}
		filter++;
	}

out:
	return mask;
}

static int siw_hal_chk_report_type(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	int invalid_pid = fw->invalid_pid;

	if (chip->report_type) {
		return 0;
	}

	if (invalid_pid) {
		return -EINVAL;
	}

	switch (touch_chip_type(ts)) {
	default:
		chip->report_type = CHIP_REPORT_TYPE_0;
		break;
	}

	switch (chip->report_type) {
	default:
		chip->report_info = &chip->info_grp.info;
		chip->report_data = chip->info_grp.info.data;
		chip->report_size = sizeof(chip->info_grp.info);
		break;
	}

	t_dev_info(dev, "report type  : %d\n", chip->report_type);

	return 0;
}

static void siw_hal_chk_status_type_quirks(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;

	switch (touch_chip_type(ts)) {
	case CHIP_SW42103:
		if (!strncmp(fw->product_id, "LA145WF1", 8)) {
			chip->status_filter++;
			t_dev_info(dev, "%s[%s] status quirk adopted\n",
				touch_chip_name(ts), fw->product_id);
		}
		break;
	}
}

static int siw_hal_chk_status_type(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_status_mask_bit *mask_bit = &chip->status_mask_bit;
	int t_sts_mask = chip->opt.t_sts_mask;
	int invalid_pid = fw->invalid_pid;

	siw_hal_chk_report_type(dev);

	if (chip->status_type) {
		return 0;
	}

	if (invalid_pid) {
		return -EINVAL;
	}

	switch (touch_chip_type(ts)) {
	case CHIP_SW42101:
		chip->status_type = CHIP_STATUS_TYPE_2;
		break;
	default:
		chip->status_type = CHIP_STATUS_TYPE_1;
		break;
	}

	switch (chip->status_type) {
	case CHIP_STATUS_TYPE_2:
		chip->status_filter = (struct siw_hal_status_filter *)status_filter_type_2;
		break;
	default:
		chip->status_filter = (struct siw_hal_status_filter *)status_filter_type_1;
		break;
	}

	siw_hal_chk_status_type_quirks(dev);

	mask_bit->valid_dev_ctl = siw_hal_get_status_mask(dev, STS_ID_VALID_DEV_CTL);
	mask_bit->valid_code_crc = siw_hal_get_status_mask(dev, STS_ID_VALID_CODE_CRC);
	mask_bit->valid_cfg_crc = siw_hal_get_status_mask(dev, STS_ID_VALID_CFG_CRC);;
	mask_bit->error_abnormal = siw_hal_get_status_mask(dev, STS_ID_ERROR_ABNORMAL);
	mask_bit->error_system = siw_hal_get_status_mask(dev, STS_ID_ERROR_SYSTEM);
	mask_bit->error_mismtach = siw_hal_get_status_mask(dev, STS_ID_ERROR_MISMTACH);
	mask_bit->valid_irq_pin = siw_hal_get_status_mask(dev, STS_ID_VALID_IRQ_PIN);
	mask_bit->valid_irq_en = siw_hal_get_status_mask(dev, STS_ID_VALID_IRQ_EN);
	mask_bit->error_mem = siw_hal_get_status_mask(dev, STS_ID_ERROR_MEM);
	mask_bit->valid_tv_drv = siw_hal_get_status_mask(dev, STS_ID_VALID_TC_DRV);
	mask_bit->error_disp = siw_hal_get_status_mask(dev, STS_ID_ERROR_DISP);

#if defined(__SIW_SUPPORT_STATUS_OPT_IGNORE_ABNORMAL)
	mask_bit->error_abnormal = 0;
	mask_bit->error_system = 0;
#endif

#if defined(__SIW_SUPPORT_STATUS_OPT_IGNORE_DISP_ERR)
	mask_bit->error_disp = 0;
#endif

	t_dev_dbg_base(dev, "mask[v_dev]  : %08Xh\n", mask_bit->valid_dev_ctl);
	t_dev_dbg_base(dev, "mask[v_code] : %08Xh\n", mask_bit->valid_code_crc);
	t_dev_dbg_base(dev, "mask[v_cfg]  : %08Xh\n", mask_bit->valid_cfg_crc);
	t_dev_dbg_base(dev, "mask[e_abn]  : %08Xh\n", mask_bit->error_abnormal);
	t_dev_dbg_base(dev, "mask[e_sys]  : %08Xh\n", mask_bit->error_system);
	t_dev_dbg_base(dev, "mask[e_mis]  : %08Xh\n", mask_bit->error_mismtach);
	t_dev_dbg_base(dev, "mask[v_i_p]  : %08Xh\n", mask_bit->valid_irq_pin);
	t_dev_dbg_base(dev, "mask[v_i_e]  : %08Xh\n", mask_bit->valid_irq_en);
	t_dev_dbg_base(dev, "mask[e_mem]  : %08Xh\n", mask_bit->error_mem);
	t_dev_dbg_base(dev, "mask[v_tc]   : %08Xh\n", mask_bit->valid_tv_drv);
	t_dev_dbg_base(dev, "mask[e_disp] : %08Xh\n", mask_bit->error_disp);

	chip->status_mask_normal = mask_bit->valid_dev_ctl |
						mask_bit->valid_code_crc |
						mask_bit->valid_cfg_crc |
						mask_bit->valid_irq_pin |
						mask_bit->valid_irq_en |
						mask_bit->valid_tv_drv |
						0;

	chip->status_mask_logging = mask_bit->error_mismtach |
						mask_bit->valid_irq_pin |
						mask_bit->valid_irq_en |
						mask_bit->valid_tv_drv |
						0;

	chip->status_mask_reset = mask_bit->valid_dev_ctl |
						mask_bit->valid_code_crc |
						mask_bit->valid_cfg_crc |
						mask_bit->error_abnormal |
						mask_bit->error_system |
						mask_bit->error_mem |
						0;

	chip->status_mask = chip->status_mask_normal |
						chip->status_mask_logging |
						chip->status_mask_reset |
						0;

	chip->status_mask_ic_abnormal = INT_IC_ABNORMAL_STATUS;
	chip->status_mask_ic_error = INT_IC_ERROR_STATUS;
	chip->status_mask_ic_disp_err = 0;

	switch (t_sts_mask) {
	case 1:
		chip->status_mask_ic_valid = 0xFFFF;
		chip->status_mask_ic_disp_err = (0x3<<6);
		break;
	case 4:
	case 2:
		chip->status_mask_ic_abnormal |= (0x3<<1);
		chip->status_mask_ic_error = ((1<<7) | (1<<5));
		chip->status_mask_ic_valid = (t_sts_mask == 4) ? 0x1FFFFF : 0x7FFFF;
		chip->status_mask_ic_disp_err = (0x3<<8);
		break;
	case 5:
	case 3:
		chip->status_mask_ic_abnormal = 0;
		chip->status_mask_ic_error = ((1<<3) | (1<<1));
		chip->status_mask_ic_valid = (t_sts_mask == 5) ? 0x3FF : 0x7FFFF;
		break;
	default:
		chip->status_mask_ic_valid = 0xFF;
		break;
	}

	chip->status_mask_ic_normal = chip->status_mask_ic_valid;
	chip->status_mask_ic_normal &= ~chip->status_mask_ic_abnormal;
	chip->status_mask_ic_normal &= ~chip->status_mask_ic_error;
	chip->status_mask_ic_normal &= ~chip->status_mask_ic_disp_err;

#if defined(__SIW_SUPPORT_STATUS_OPT_IGNORE_ABNORMAL)
	chip->status_mask_ic_abnormal = 0;
#endif

#if defined(__SIW_SUPPORT_STATUS_OPT_IGNORE_DISP_ERR)
	chip->status_mask_ic_disp_err = 0;
#endif

	t_dev_info(dev, "status type  : %d\n", chip->status_type);
	t_dev_info(dev, "status mask  : %08Xh\n", chip->status_mask);
	t_dev_info(dev, " normal      : %08Xh\n", chip->status_mask_normal);
	t_dev_info(dev, " logging     : %08Xh\n", chip->status_mask_logging);
	t_dev_info(dev, " reset       : %08Xh\n", chip->status_mask_reset);
	t_dev_info(dev, " ic normal   : %08Xh\n", chip->status_mask_ic_normal);
	t_dev_info(dev, " ic abnormal : %08Xh\n", chip->status_mask_ic_abnormal);
	t_dev_info(dev, " ic error    : %08Xh\n", chip->status_mask_ic_error);
	t_dev_info(dev, " ic valid    : %08Xh\n", chip->status_mask_ic_valid);
	t_dev_info(dev, " ic disp err : %08Xh\n", chip->status_mask_ic_disp_err);

	return 0;
}

struct siw_ic_info_chip_proto {
	int chip_type;
	int vchip;
	int vproto;
};

static const struct siw_ic_info_chip_proto siw_ic_info_chip_protos[] = {
	{ CHIP_SW1828, 9, 4 },
	{ CHIP_SW42101, 15, 4 },
	{ CHIP_SW42103, 17, 4 },
	{ CHIP_SW17700, 18, 4 },
	{ CHIP_NONE, 0, 0 },	//End mark
};

static int siw_hal_ic_info_ver_check(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	const struct siw_ic_info_chip_proto *chip_proto = siw_ic_info_chip_protos;
	u32 vchip = fw->v.version.chip;
	u32 vproto = fw->v.version.protocol;

	while (1) {
		if (chip_proto->chip_type == CHIP_NONE) {
			break;
		}

		if (touch_chip_type(ts) == chip_proto->chip_type) {
			if ((chip_proto->vchip != vchip) ||
				(chip_proto->vproto != vproto)) {
				break;
			}

			t_dev_info(dev, "[%s] IC info is good: %d, %d\n",
					touch_chip_name(ts), vchip, vproto);

			return 0;
		}

		chip_proto++;
	}

	t_dev_err(dev, "[%s] IC info is abnormal: %d, %d\n",
			touch_chip_name(ts), vchip, vproto);

	return -EINVAL;
}

static int siw_hal_ic_info_boot(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int boot_fail_cnt = chip->boot_fail_cnt;
	int ret = 0;

	ret = siw_hal_chk_boot(dev);
	if (ret < 0) {
		return ret;
	}

	if (ret) {
		atomic_set(&chip->boot, IC_BOOT_FAIL);

		/* Limit to avoid infinite repetition */
		if (boot_fail_cnt >= BOOT_FAIL_RECOVERY_MAX) {
			t_dev_err(dev, "Boot fail can't be recovered(%d) - %02Xh\n",
				boot_fail_cnt, ret);
			return -EFAULT;
		}

		t_dev_err(dev, "Boot fail detected(%d) - %02Xh\n",
			boot_fail_cnt, ret);

		chip->boot_fail_cnt++;

		/* return special flag to let the core layer know */
		return -ETDBOOTFAIL;
	}
	chip->boot_fail_cnt = 0;

	return 0;
}

static int siw_hal_hw_reset_quirk(struct device *dev, int pwr_con, int delay);

static int siw_hal_ic_info_quirk(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;

	switch (touch_chip_type(ts)) {
	case CHIP_SW17700:
		if (atomic_read(&ts->state.core) != CORE_PROBE) {
			break;
		}

		if (touch_flags(ts) & TOUCH_SKIP_RESET_PIN) {
			break;
		}

		if (fw->revision) {
			break;
		}

		if (chip->ops_quirk.hw_reset != NULL) {
			break;
		}

		if (touch_fquirks(ts)->gpio_set_reset != NULL) {
			break;
		}

		chip->ops_quirk.hw_reset = siw_hal_hw_reset_quirk;

		t_dev_info(dev, "[%s] reset quirk activated\n",
			touch_chip_name(ts));

		break;
	}

	return 0;
}

static int siw_hal_do_ic_info(struct device *dev, int prt_on)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 product[2] = {0};
	u32 chip_id = 0;
	u32 version = 0;
	u32 version_ext = 0;
	u32 revision = 0;
	u32 bootmode = 0;
	u32 boot_chk_offset = 0;
	int invalid_pid = 0;
	struct siw_hal_rw_multi multi[] = {
		{ 0, reg->spr_chip_id, &chip_id, sizeof(chip_id), "chip_id" },
		{ 0, reg->tc_version, &version, sizeof(version), "version" },
		{ 0, reg->info_chip_version, &revision, sizeof(revision), "revision" },
		{ 0, reg->tc_product_id1, product, sizeof(product), "product_id" },
		{ -1, -1, NULL, },
	};
	int ret = 0;

	ret = siw_hal_reg_rw_multi(dev, multi, "ic_info(1)");
	if (ret < 0) {
		return ret;
	}

	if (chip->opt.f_ver_ext) {
		ret = siw_hal_read_value(dev, reg->tc_version_ext, &version_ext);
		if (ret < 0) {
			t_dev_err(dev, "ic_info(1): version_ext failed, %d\n", ret);
			return ret;
		}
	}

	ret = siw_hal_get_boot_status(dev, &bootmode);
	if (ret < 0) {
		t_dev_err(dev, "ic_info(1): failed to get boot status, %d\n", ret);
		return ret;
	}

	siw_hal_fw_set_chip_id(fw, chip_id);
	siw_hal_fw_set_version(fw, version, version_ext);
	siw_hal_fw_set_revision(fw, revision);
	siw_hal_fw_set_prod_id(fw, (u8 *)product, sizeof(product));

	invalid_pid = fw->invalid_pid;
	if (invalid_pid) {
		t_dev_err(dev, "[info] invalid PID - \"%s\" (%03Xh)\n",
			fw->product_id, invalid_pid);
	}

	siw_hal_chk_status_type(dev);

	if (fw->version_ext) {
		int ferr;

		ferr = siw_hal_fw_chk_version_ext(fw->version_ext,
									fw->v.version.ext);
		t_dev_info_sel(dev, prt_on,
				"[T] chip id %s, version %08X(%u.%02u) (0x%02X) %s\n",
				chip->fw.chip_id,
				fw->version_ext,
				fw->v.version.major, fw->v.version.minor,
				fw->revision,
				(ferr < 0) ? "(invalid)" : "");
	} else {
		t_dev_info_sel(dev, prt_on,
				"[T] chip id %s, version v%u.%02u (0x%08X, 0x%02X)\n",
				fw->chip_id,
				fw->v.version.major, fw->v.version.minor,
				version, fw->revision);
	}

	boot_chk_offset = siw_hal_boot_sts_pos_busy(chip);
	t_dev_info_sel(dev, prt_on,
			"[T] product id %s, flash boot %s(%s), crc %s (0x%08X)\n",
			fw->product_id,
			((bootmode >> boot_chk_offset) & 0x1) ? "BUSY" : "idle",
			((bootmode >> (boot_chk_offset + 1)) & 0x1) ? "done" : "booting",
			((bootmode >> (boot_chk_offset + 2)) & 0x1) ? "ERROR" : "ok",
			bootmode);

	ret = siw_hal_ic_info_boot(dev);
	if (ret) {
		return ret;
	}

	if (strcmp(fw->chip_id, touch_chip_id(ts))) {
		t_dev_err(dev, "Invalid chip id(%s), shall be %s\n",
			fw->chip_id, touch_chip_id(ts));
		return -EINVAL;
	}

	ret = siw_hal_ic_info_ver_check(dev);
	if (ret < 0) {
		return ret;
	}

	siw_hal_ic_info_quirk(dev);

	if (invalid_pid) {
		return -EINVAL;
	}

	return 0;
}

static int siw_hal_ic_info(struct device *dev)
{
	return siw_hal_do_ic_info(dev, 1);
}

static int siw_hal_init_reg_set_pre(struct device *dev)
{
	return 0;
}

static int siw_hal_init_reg_set_post(struct device *dev)
{
	return 0;
}

static int siw_hal_init_reg_set(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
//	u32 data = 1;
	int ret = 0;

	siw_hal_init_reg_set_pre(dev);

	ret = siw_hal_write_value(dev,
				reg->tc_device_ctl,
				1);
	if (ret < 0) {
		t_dev_err(dev, "failed to start chip, %d\n", ret);
		goto out;
	}

	ret = siw_hal_write_value(dev,
				reg->tc_interrupt_ctl,
				1);
	if (ret < 0) {
		t_dev_err(dev, "failed to start chip irq, %d\n", ret);
		goto out;
	}

	siw_hal_init_reg_set_post(dev);

out:
	return ret;
}

enum {
	IC_TEST_ADDR_NOT_VALID = 0x8000,
};

int siw_hal_ic_test_unit(struct device *dev, u32 data)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data_rd;
	int ret;

	if (!reg->spr_chip_test) {
		t_dev_warn(dev, "ic test addr not valid, skip\n");
		return IC_TEST_ADDR_NOT_VALID;
	}

	ret = siw_hal_write_value(dev,
				reg->spr_chip_test,
				data);
	if (ret < 0) {
		t_dev_err(dev, "ic test wr err, %08Xh, %d\n", data, ret);
		goto out;
	}

	ret = siw_hal_read_value(dev,
				reg->spr_chip_test,
				&data_rd);
	if (ret < 0) {
		t_dev_err(dev, "ic test rd err: %08Xh, %d\n", data, ret);
		goto out;
	}

	if (data != data_rd) {
		t_dev_err(dev, "ic test cmp err, %08Xh, %08Xh\n", data, data_rd);
		ret = -EFAULT;
		goto out;
	}

out:
	return ret;
}

static int siw_hal_ic_test(struct device *dev)
{
	u32 data[] = {
		0x5A5A5A5A,
		0xA5A5A5A5,
		0xF0F0F0F0,
		0x0F0F0F0F,
		0xFF00FF00,
		0x00FF00FF,
		0xFFFF0000,
		0x0000FFFF,
		0xFFFFFFFF,
		0x00000000,
	};
	int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		ret = siw_hal_ic_test_unit(dev, data[i]);
		if ((ret == IC_TEST_ADDR_NOT_VALID) || (ret < 0)) {
			break;
		}
	}

	if (ret >= 0) {
		t_dev_dbg_base(dev, "ic bus r/w test done\n");
	}

	return ret;
}

static void siw_hal_init_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (chip->ops_quirk.hw_reset != NULL) {
		chip->ops_quirk.hw_reset(dev, 1, 0);
		return;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);
	siw_hal_power(dev, POWER_OFF);
	siw_hal_power(dev, POWER_ON);
	touch_msleep(ts->caps.hw_reset_delay);
}

static int siw_hal_init_pre(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->init_pre) {
		ret = fquirks->init_pre(dev);
		if (ret < 0) {
			goto out;
		}
	}

	ret = siw_hal_ic_test(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_init_config(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->init_config_pre) {
		ret = fquirks->init_config_pre(dev);
		if (ret < 0) {
			return ret;
		}
	}

	siw_hal_init_reg_set(dev);

	if (fquirks->init_config_post) {
		ret = fquirks->init_config_post(dev);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int siw_hal_init_charger(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	if (!ts->is_charger) {
		return 0;
	}

	ret = siw_hal_init_pre(dev);
	if (ret < 0) {
		return ret;
	}

	ret = siw_hal_tc_driving(dev, LCD_MODE_STOP);
	if (ret < 0) {
		return ret;
	}
	touch_msleep(100);

	return 0;
}

static int siw_hal_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int is_probe = !!(atomic_read(&ts->state.core) == CORE_PROBE);
	int init_retry = (is_probe) ? CHIP_INIT_RETRY_PROBE : CHIP_INIT_RETRY_MAX;
	int i;
	int ret = 0;

	siw_hal_alive_pause_set(dev, ALIVE_PAUSE_INIT);

	atomic_set(&chip->boot, IC_BOOT_DONE);

	ret = siw_hal_init_pre(dev);
	if (ret < 0) {
		siw_hal_init_reset(dev);
		goto out;
	}

	for (i = 0; i < init_retry; i++) {
		ret = siw_hal_ic_info(dev);
		if (ret >= 0) {
			break;
		}
		/*
		 * When boot fail detected
		 *
		 * 1. At the fisrt detection,
		 *    it sends esd noti for LCD recovery(full reset procedure)
		 *    and skip fw_upgrade.
		 * 2. LCD driver is suppsed to send lcd mode notifier
		 *    back to touch side after its recovery.
		 * 3. The lcd mode notifier restarts init work again
		 *    via siw_touch_resume.
		 * 4. If boot fail detected again(counted by boot_fail_cnt)
		 *    it goes to fw_upgrade stage.
		 * (See siw_touch_init_work_func in siw_touch.c)
		 */
		if (ret == -ETDBOOTFAIL) {
			/* For the probe stage */
			if (atomic_read(&ts->state.core) == CORE_PROBE) {
				break;
			}

			/* At the first boot fail */
			if (chip->boot_fail_cnt > 1) {
				break;
			}
		}

		t_dev_dbg_base(dev, "retry getting ic info (%d)\n", i);

		siw_hal_init_reset(dev);
	}
	if (ret < 0) {
		goto out;
	}

	siw_hal_init_config(dev);

	atomic_set(&chip->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);

	ret = siw_hal_lpwg_mode(dev);
	if (ret < 0) {
		t_dev_err(dev, "failed to set lpwg control, %d\n", ret);
		goto out;
	}

	siw_hal_sysfs_post(dev, DRIVER_INIT);

	siw_hal_alive_init(dev);

out:
	if (ret < 0) {
		t_dev_err(dev, "%s init failed, %d\n",
			touch_chip_name(ts), ret);
	} else {
		t_dev_info(dev, "%s init done\n",
			touch_chip_name(ts));
	}

	siwmon_submit_ops_step_chip_wh_name(dev, "%s init done",
			touch_chip_name(ts), ret);

	return ret;
}

static int siw_hal_reinit(struct device *dev,
					int pwr_con,
					int delay,
					int irq_enable,
					int (*do_call)(struct device *dev))
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;
	int skip_quirk = !!(pwr_con & 0x10);

	pwr_con &= 0x0F;

	if (!skip_quirk && (chip->ops_quirk.hw_reset != NULL)) {
		ret = chip->ops_quirk.hw_reset(dev, pwr_con, delay);
		goto reset_done;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	atomic_set(&chip->init, IC_INIT_NEED);

	if (pwr_con) {
		siw_hal_power(dev, POWER_OFF);
		siw_hal_power(dev, POWER_ON);
	} else {
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ZERO);
		touch_msleep(chip->drv_reset_low + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_0));
		siw_hal_set_gpio_reset(dev, GPIO_OUT_ONE);
	}

	touch_msleep(delay);
reset_done:

	if (do_call) {
		ret = do_call(dev);
	}

	if (irq_enable)
		siw_touch_irq_control(dev, INTERRUPT_ENABLE);

	return ret;
}

#define SIW_SW_RST_TYPE_NONE	0x0F
#define SIW_SW_RST_TYPE_MAX		5

static int siw_hal_sw_reset_type_5(struct device *dev)
{
	return 0;
}

static int siw_hal_sw_reset_type_4(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = 0;
	u32 value = 0;

	addr = reg->spr_boot_ctl;
	t_dev_dbg_trace(dev, "spr_boot_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	addr = 0x081;
	value = 0xF83;
	t_dev_dbg_trace(dev, "spr_clk_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	addr = reg->spr_rst_ctl;
	value = 0x13;
	t_dev_dbg_trace(dev, "spr_rst_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	touch_msleep(10);

	return 0;
}

static int siw_hal_sw_reset_type_3(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = 0;
	u32 value = 0x03;

	if (chip->ops_quirk.hw_reset != NULL) {
		value |= 0x08;
	}

	if (value & 0x08) {
		addr = reg->spr_boot_ctl;
		t_dev_dbg_trace(dev, "spr_boot_ctl[%04Xh] = %08Xh\n", addr, 0);
		siw_hal_write_value(dev, addr, 0);
	}

	addr = reg->spr_rst_ctl;
	t_dev_dbg_trace(dev, "spr_rst_ctl[%04Xh] = %08Xh\n", addr, value);
	siw_hal_write_value(dev, addr, value);

	touch_msleep(10);

	return 0;
}

static int siw_hal_sw_reset_type_2(struct device *dev)
{
	return 0;
}

static int siw_hal_sw_reset_type_1(struct device *dev)
{
	return 0;
}

static int siw_hal_sw_reset_default(struct device *dev)
{
	return 0;
}

static int __siw_hal_sw_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int type = chip->opt.t_sw_rst;
	int ret = 0;

	if (type > SIW_SW_RST_TYPE_MAX) {
		t_dev_warn(dev, "sw reset not supported\n");
		ret = -EPERM;
		goto out;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	atomic_set(&chip->init, IC_INIT_NEED);

	t_dev_info(dev, "SW Reset(%d)\n", type);

	touch_msleep(chip->drv_reset_low + 10);

	switch (type) {
	case 5:
		ret = siw_hal_sw_reset_type_5(dev);
		break;
	case 4:
		ret = siw_hal_sw_reset_type_4(dev);
		break;
	case 3:
		ret = siw_hal_sw_reset_type_3(dev);
		break;
	case 2:
		ret = siw_hal_sw_reset_type_2(dev);
		break;
	case 1:
		ret = siw_hal_sw_reset_type_1(dev);
		break;
	case 0:
		ret = siw_hal_sw_reset_default(dev);
		break;
	default:
		t_dev_warn(dev, "unknown sw reset type, %d\n", type);
		ret = -ESRCH;
		break;
	}

	if (chip->ops_quirk.sw_reset_post) {
		chip->ops_quirk.sw_reset_post(dev);
	}

out:
	return ret;
}

static int siw_hal_sw_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	ret = __siw_hal_sw_reset(dev);

	touch_msleep(hal_dbg_delay(chip, HAL_DBG_DLY_SW_RST_1));

	if (chip->ops_quirk.hw_reset != NULL) {
		siw_touch_qd_init_work_hw(ts);
	} else {
		siw_touch_qd_init_work_sw(ts);
	}

	return ret;
}

static int siw_hal_hw_reset_quirk(struct device *dev, int pwr_con, int delay)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	t_dev_info(dev, "run sw reset (reset gpio deactivated)\n");

	if (pwr_con) {
		siw_touch_irq_control(dev, INTERRUPT_DISABLE);

		atomic_set(&chip->init, IC_INIT_NEED);

		siw_hal_power_core(dev, POWER_OFF);
		siw_hal_power_core(dev, POWER_ON);
		touch_msleep((delay) ? delay : ts->caps.hw_reset_delay);
	}

	ret = __siw_hal_sw_reset(dev);

	touch_msleep((delay) ? delay : ts->caps.hw_reset_delay);

	return ret;
}

static int siw_hal_hw_reset(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int pwr_con = 0;

	pwr_con = !!(ctrl & 0x80);
	pwr_con |= (ctrl & 0x10);
	ctrl &= 0x0F;

	t_dev_info(dev, "HW Reset(%s)\n",
		(ctrl == HW_RESET_ASYNC) ? "Async" : "Sync");

	if (ctrl == HW_RESET_ASYNC) {
		siw_hal_reinit(dev, pwr_con, hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_1), 0, NULL);
		siw_touch_qd_init_work_hw(ts);
		return 0;
	}

	siw_hal_reinit(dev, pwr_con, ts->caps.hw_reset_delay, 1, siw_hal_init);

	return 0;
}

static int siw_hal_reset_ctrl(struct device *dev, int ctrl)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ctrl_low = ctrl & 0xF;
	int ret = -EINVAL;

	mutex_lock(&ts->reset_lock);

	siw_hal_alive_pause_set(dev, ALIVE_PAUSE_RESET);

	t_dev_info(dev, "%s reset control(0x%X)\n",
			touch_chip_name(ts), ctrl);

	switch (ctrl_low) {
	case SW_RESET:
		ret = siw_hal_sw_reset(dev);
		break;
	case HW_RESET_ASYNC:
	case HW_RESET_SYNC:
		ret = siw_hal_hw_reset(dev, ctrl);
		break;

	default:
		t_dev_err(dev, "unknown reset type, 0x%X\n", ctrl);
		break;
	}

	mutex_unlock(&ts->reset_lock);

	return ret;
}

static u32 siw_hal_fw_act_buf_size(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int buf_size = (touch_get_act_buf_size(ts) - SIW_TOUCH_BUF_MARGIN) & (~0x3FF);

	return buf_size;
}

static int siw_hal_fw_rd_value(struct device *dev,
				u32 addr, u32 *value)
{
	u32 data;
	int ret;

	ret = siw_hal_read_value(dev, addr, &data);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_fwup(dev, "FW upgrade: reg rd: addr[%04Xh], value[%08Xh], %d\n",
			addr, data, ret);

	if (value)
		*value = data;

	return 0;
}

static int siw_hal_fw_wr_value(struct device *dev,
				u32 addr, u32 value)
{
	int ret;

	ret = siw_hal_write_value(dev, addr, value);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_fwup(dev, "FW upgrade: reg wr: addr[%04Xh], value[%08Xh], %d\n",
			addr, value, ret);

	return 0;
}

static int siw_hal_fw_wr_seq(struct device *dev,
				u32 addr, u8 *data, int size)
{
	int ret;

	ret = siw_hal_reg_write(dev, addr, (void *)data, size);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_fwup(dev, "FW upgrade: reg wr: addr[%04Xh], data[%02X ...], %d\n",
			addr, data[0], ret);

	return 0;
}

static int siw_hal_fw_wr_data(struct device *dev,
				u32 addr, u8 *dn_buf, int dn_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 offset = reg->serial_data_offset;
	u32 data_access = reg->data_i2cbase_addr;
	int ret = 0;

	if (!dn_size)
		goto out;

	ret = siw_hal_fw_wr_value(dev, offset, addr);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_wr_seq(dev, data_access, (void *)dn_buf, dn_size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_sram_wr_enable(struct device *dev, int onoff)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data;
	int ret = 0;

#if 0
	ret = siw_hal_fw_rd_value(dev, reg->spr_sram_ctl, &data);
	if (ret < 0) {
		goto out;
	}

	if (onoff)
		data |= 0x01;
	else
		data &= ~0x01;

	ret = siw_hal_fw_wr_value(dev, reg->spr_sram_ctl, data);
	if (ret < 0) {
		goto out;
	}
#else
//	data = !!onoff;
	data = (onoff) ? 0x03 : 0x00;
	ret = siw_hal_fw_wr_value(dev, reg->spr_sram_ctl, data);
	if (ret < 0) {
		goto out;
	}
#endif

out:
	return ret;
}

static int siw_hal_fw_upgrade_fw_core(struct device *dev, u8 *dn_buf, int dn_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int is_i2c = !!(touch_bus_type(ts) == BUS_IF_I2C);
	u8 *fw_data;
	int fw_size;
	int fw_pos, curr_size;
	int fw_size_org = dn_size;
	int fw_dn_size = 0, fw_dn_percent;
	int buf_size = min(MAX_RW_SIZE, (int)siw_hal_fw_act_buf_size(dev));
	int ret = 0;

	fw_data = dn_buf;
	fw_size = dn_size;
	fw_pos = 0;
	while (fw_size) {
		t_dev_dbg_fwup(dev, "FW upgrade: fw_pos[%06Xh ...] = %02X %02X %02X %02X ...\n",
				fw_pos,
				fw_data[0], fw_data[1], fw_data[2], fw_data[3]);

		curr_size = min(fw_size, buf_size);

		/* code sram base address write */
		ret = siw_hal_fw_wr_value(dev, reg->spr_code_offset, fw_pos>>2);
		if (ret < 0) {
			goto out;
		}

		ret = siw_hal_fw_wr_seq(dev, reg->code_access_addr,
					(void *)fw_data, curr_size);
		if (ret < 0) {
			goto out;
		}

		fw_data += curr_size;
		fw_pos += curr_size;
		fw_size -= curr_size;

		/*
		 * Show progress log for slow I2C case
		 */
		if (!is_i2c) {
			continue;
		}

		fw_dn_size += curr_size;
		if (!fw_size || !(fw_dn_size & (FW_DN_LOG_UNIT-1))) {
			fw_dn_percent = (fw_dn_size * 100);
			fw_dn_percent /= fw_size_org;

			t_dev_info(dev, "FW upgrade: downloading...(%d%c)\n",
				fw_dn_percent, '%');
		}
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf_core(struct device *dev,
				u32 addr, u8 *dn_buf, int dn_size)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int buf_size = (int)siw_hal_fw_act_buf_size(dev);
	int ret = 0;

	if (dn_size > buf_size) {
		t_dev_err(dev, "FW upgrade: buffer overflow, dn_size %d > %d\n",
			dn_size, buf_size);
		ret = -EOVERFLOW;
		goto out;
	}

	ret = siw_hal_fw_wr_data(dev, addr, dn_buf, dn_size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

#if defined(__SIW_FW_TYPE_1)
static int siw_hal_fw_upgrade_fw_post_quirk(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 chk_resp, data;
	u32 boot_code_addr = chip->fw.boot_code_addr;
	int ret;

	ret = siw_hal_fw_wr_value(dev, boot_code_addr, FW_BOOT_LOADER_INIT);
	if (ret < 0) {
		goto out;
	}

	/* Set Serial Dump Done */
	ret = siw_hal_fw_wr_value(dev, reg->spr_boot_ctl, 1);
	if (ret < 0) {
		goto out;
	}

	/* Release CM3 core */
	ret = siw_hal_fw_wr_value(dev, reg->spr_rst_ctl, 0);
	if (ret < 0) {
		goto out;
	}

	/* firmware boot done check */
	chk_resp = FW_BOOT_LOADER_CODE;
	ret = siw_hal_condition_wait(dev, boot_code_addr, &data,
				chk_resp, ~0,
				FW_POST_QUIRK_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_0),
				FW_POST_QUIRK_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - boot check(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: boot check done\n");

out:
	return ret;
}

static u32 siw_hal_fw_upgrade_conf_pre_quirk(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	return (chip->fw.conf_c_addr<<16);
}

static int siw_hal_fw_upgrade_conf_quirk(struct device *dev,
			     u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max;
	u32 conf_index = chip->fw.conf_index;
	u32 conf_dn_addr;
	u32 data;
	int ret = 0;

	if (!conf_index) {
		goto out;
	}

	fw_size_max = touch_fw_size(ts);

	data = chip->fw.conf_s_addr;
	if (data) {
		goto done_conf_dn_addr;
	}

	conf_dn_addr = chip->fw.conf_dn_addr;
	ret = siw_hal_fw_rd_value(dev, conf_dn_addr, &data);
	if (ret < 0) {
		goto out;
	}

done_conf_dn_addr:
	conf_dn_addr = (data & 0xFFFF);
	t_dev_dbg_fwup(dev, "FW upgrade: s_conf_dn_addr %04Xh (%08Xh)\n",
			conf_dn_addr, data);

	data = fw_size_max +	\
		(NUM_C_CONF<<POW_C_CONF) +	\
		((conf_index - 1)<<POW_S_CONF);
	ret = siw_hal_fw_upgrade_conf_core(dev, conf_dn_addr,
				(u8 *)&fw_buf[data], FLASH_CONF_SIZE);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static void siw_hal_fw_var_init(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;

	fw->conf_index = 0;
	fw->conf_idx_addr = FW_S_CONF_IDX_ADDR;
	fw->conf_dn_addr = FW_S_CONF_DN_ADDR;
	fw->boot_code_addr = FW_BOOT_CODE_ADDR;
	fw->conf_c_addr = 0;
	fw->conf_s_addr = 0;
	fw->conf_skip = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_SW42103:
		fw->boot_code_addr = 0x03F;
		break;
	case CHIP_SW17700:
		fw->conf_idx_addr = 0x246;
		fw->conf_dn_addr = 0x24D;
		fw->boot_code_addr = 0x0BB;
		break;
	}

	t_dev_info(dev, "FW upgrade: idx %Xh, dn %Xh, code %Xh\n",
		fw->conf_idx_addr, fw->conf_dn_addr, fw->boot_code_addr);
}

static int siw_hal_fw_size_check(struct device *dev, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	int fw_size_max = touch_fw_size(ts);
	int size_min = (fw_size_max + (NUM_C_CONF<<POW_C_CONF) + (MIN_S_CONF<<POW_S_CONF));
	int size_max = (fw_size_max + (NUM_C_CONF<<POW_C_CONF) + (MAX_S_CONF<<POW_S_CONF));

	siw_hal_fw_var_init(dev);

	switch (touch_chip_type(ts)) {
	case CHIP_SW42103:
		fw->conf_skip = !!(fw_size == fw_size_max);

		if (!strncmp(fw->product_id, "LA145WF1", 8)) {
			fw->conf_skip = 1;
		}
		break;
	case CHIP_SW17700:
		fw->conf_skip = !!(fw_size == fw_size_max);

		if (!strncmp(fw->product_id, "LA103WF5", 8)) {
			fw->conf_skip = 1;
		}

		if (!strncmp(fw->product_id, "LA123WF7", 8)) {
			fw->conf_skip = 1;
		}
		break;
	}

	if (fw->conf_skip) {
		return 0;
	}

	if ((fw_size < size_min) || (fw_size > size_max)) {
		t_dev_err(dev, "FW upgrade: wrong file size - %Xh,\n",
			fw_size);
		t_dev_err(dev, "			shall be '%Xh <= x <= %Xh'\n",
			size_min, size_max);
		return -EFAULT;
	}

	return 0;
}

static int siw_hal_fw_size_check_post(struct device *dev, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max = touch_fw_size(ts);
	int required_size;
	u32 index = 0;
	u32 conf_idx_addr = chip->fw.conf_idx_addr;
	int ret = 0;

	if (chip->fw.conf_skip) {
		return 0;
	}

#if (S_CFG_DBG_IDX != 0)
	index = S_CFG_DBG_IDX;
	t_dev_warn(dev, "FW upgrade: conf_index fixed for debugging: %d\n", index);
#else
	ret = siw_hal_fw_rd_value(dev, conf_idx_addr, &index);
#endif
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - conf_index(%04Xh) read, %d\n",
			conf_idx_addr, ret);
		return ret;
	}
	if ((index < MIN_S_CONF_IDX) || (index > MAX_S_CONF_IDX)) {
		t_dev_err(dev, "FW upgrade: failed - wrong cfg index, %d\n", index);
		return -EFAULT;
	}
	t_dev_info(dev, "FW upgrade: conf_index: %d\n", index);

	required_size = fw_size_max + (NUM_C_CONF<<POW_C_CONF) + (index<<POW_S_CONF);
	if (fw_size < required_size) {
		t_dev_err(dev, "FW upgrade: wrong file size - %Xh < %Xh,\n",
			fw_size, required_size);
		return -EFAULT;
	}

	chip->fw.conf_index = index;

	return 0;
}
#else	/* __SIW_FW_TYPE_1 */
static int siw_hal_fw_upgrade_fw_post_quirk(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 chk_resp, data;
	int ret;

	/* Release CM3 core */
	ret = siw_hal_fw_wr_value(dev, reg->spr_rst_ctl, 0);
	if (ret < 0) {
		goto out;
	}

	/* Set Serial Dump Done */
	ret = siw_hal_fw_wr_value(dev, reg->spr_boot_ctl, 1);
	if (ret < 0) {
		goto out;
	}

	/* firmware boot done check */
	chk_resp = FLASH_BOOTCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, ~0,
				FW_POST_QUIRK_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_0),
				FW_POST_QUIRK_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - boot check(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: boot check done\n");

out:
	return ret;
}

static u32 siw_hal_fw_upgrade_conf_pre_quirk(struct device *dev)
{
	return 0;
}

static int siw_hal_fw_upgrade_conf_quirk(struct device *dev,
			     u8 *fw_buf, int fw_size)
{
	return 0;
}

static int siw_hal_fw_size_check(struct device *dev, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max = touch_fw_size(ts);

	chip->fw.conf_index = 0;

	if ((fw_size != fw_size_max) &&
		(fw_size != (fw_size_max + FLASH_CONF_SIZE)))
	{
		t_dev_err(dev, "FW upgrade: wrong file size - %Xh,\n",
			fw_size);
		t_dev_err(dev, "            shall be '%Xh' or '%Xh + %Xh'\n",
			fw_size_max, fw_size_max, FLASH_CONF_SIZE);
		return -EFAULT;
	}

	return 0;
}

static int siw_hal_fw_size_check_post(struct device *dev, int fw_size)
{
	return 0;
}
#endif	/* __SIW_FW_TYPE_1 */

static int siw_hal_fw_compare(struct device *dev, u8 *fw_buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	struct siw_hal_tc_version_bin *bin_ver;
	int fw_max_size = touch_fw_size(ts);
	u32 bin_ver_offset = 0;
	u32 bin_ver_ext_offset = 0;
	u32 bin_pid_offset = 0;
	u32 dev_major = 0;
	u32 dev_minor = 0;
	u32 dev_build = 0;
	char pid[12] = {0, };
	u32 bin_major = 0;
	u32 bin_minor = 0;
	u32 bin_build = 0;
//	u32 bin_raw = 0;
	u32 bin_raw_ext = 0;
	int bin_diff = 0;
	int update = 0;
	int use_fw_ver_diff = !!ts->role.use_fw_ver_diff;
	int use_fw_skip_pid = !!ts->role.use_fw_skip_pid;
	int boot_failed = !!(atomic_read(&chip->boot) == IC_BOOT_FAIL);
//	int ret = 0;

	if (fquirks->fwup_check) {
		update = fquirks->fwup_check(dev, fw_buf);
		if (update != -EAGAIN) {
			if (update < 0) {
				return update;
			}
			goto out;
		}
		update = 0;
	}

	if (!boot_failed) {
		if (fw->version_ext) {
			dev_major = fw->version_ext >> 8;
			dev_minor = fw->version_ext & 0xFF;
		} else {
			dev_major = fw->v.version.major;
			dev_minor = fw->v.version.minor;
			dev_build = fw->v.version.build;
		}

		if (!dev_major && !dev_minor){
			t_dev_err(dev, "fw can not be 0.0!! Check your panel connection!!\n");
			return 0;
		}
	}

	bin_ver_offset = *((u32 *)&fw_buf[BIN_VER_OFFSET_POS]);
	if (!bin_ver_offset) {
		t_dev_err(dev, "FW compare: zero ver offset\n");
		return -EINVAL;
	}

	if (chip->opt.f_ver_ext) {
		bin_ver_ext_offset = *((u32 *)&fw_buf[BIN_VER_EXT_OFFSET_POS]);
	} else {
		bin_ver_ext_offset = 0;
	}

	if (!boot_failed) {
		if ((fw->version_ext && !bin_ver_ext_offset) ||
			(!fw->version_ext && bin_ver_ext_offset)) {
			if (!ts->force_fwup) {
				t_dev_warn(dev,
					"FW compare: different version format, "
					"use force update %s",
					(fw->version_ext) ? "(ext)" : "");
				return -EINVAL;
			}
			bin_diff = 1;
		}
	}

	bin_pid_offset = *((u32 *)&fw_buf[BIN_PID_OFFSET_POS]);
	if (!bin_pid_offset) {
		t_dev_err(dev, "FW compare: zero pid offset\n");
		return -EINVAL;
	}

	if (((bin_ver_offset + 4) > fw_max_size) ||
		((bin_ver_ext_offset + 4) > fw_max_size) ||
		((bin_pid_offset + 8) > fw_max_size)) {
		t_dev_err(dev, "FW compare: invalid offset - ver %06Xh, ver_ext %06Xh pid %06Xh, max %06Xh\n",
			bin_ver_offset, bin_ver_ext_offset, bin_pid_offset, fw_max_size);
		return -EINVAL;
	}

	t_dev_dbg_fwup(dev, "ver %06Xh, ver_ext %06Xh, pid %06Xh\n",
			bin_ver_offset, bin_ver_ext_offset, bin_pid_offset);

	memcpy(pid, &fw_buf[bin_pid_offset], 8);
	t_dev_dbg_fwup(dev, "pid %s\n", pid);

	if (siw_hal_fw_check_pid(pid)) {
		t_dev_err(dev, "[fw-bin] invalid pid - \"%s\"\n", pid);
		return -EINVAL;
	}

	if (boot_failed) {
		update |= BIT(7);
		goto out;
	}

	bin_ver = (struct siw_hal_tc_version_bin *)&fw_buf[bin_ver_offset];
	bin_major = bin_ver->major;
	bin_minor = bin_ver->minor;
	bin_build = bin_ver->build;

	if (bin_ver_ext_offset) {
		if (!bin_ver->ext) {
			t_dev_err(dev, "FW compare: (no ext flag in binary)\n");
			return -EINVAL;
		}

		memcpy(&bin_raw_ext, &fw_buf[bin_ver_ext_offset], sizeof(bin_raw_ext));
		bin_major = bin_raw_ext >> 8;
		bin_minor = bin_raw_ext & 0xFF;
		bin_build = 0;

		t_dev_info(dev,
			"FW compare: bin-ver: %08X (%s)(%d)\n",
			bin_raw_ext, pid, bin_diff);

		if (siw_hal_fw_chk_version_ext(bin_raw_ext,
					bin_ver->ext) < 0) {
			t_dev_err(dev, "FW compare: (invalid extension in binary)\n");
			return -EINVAL;
		}
	} else {
		t_dev_info(dev,
			"FW compare: bin-ver: %d.%02d(%d) (%s)(%d)\n",
			bin_major, bin_minor, bin_build, pid, bin_diff);
	}

	if (fw->version_ext) {
		t_dev_info(dev, "FW compare: dev-ver: %08X (%s)\n",
				fw->version_ext, fw->product_id);
	} else {
		t_dev_info(dev, "FW compare: dev-ver: %d.%02d(%d) (%s)\n",
				dev_major, dev_minor, dev_build, fw->product_id);
	}

	if (ts->force_fwup) {
		update |= BIT(0);
	} else {
		if (use_fw_ver_diff) {
			u32 bin_ver_val = (bin_major<<16) | (bin_minor<<8) | bin_build;
			u32 dev_ver_val = (dev_major<<16) | (dev_minor<<8) | dev_build;
			if (bin_ver_val != dev_ver_val) {
				update |= BIT(8);
			}
		} else {
			update |= siw_hal_fw_ver_cmp(bin_major, bin_minor, bin_build,
						dev_major, dev_minor, dev_build);
		}
	}

	use_fw_skip_pid |= !!(ts->force_fwup & FORCE_FWUP_SKIP_PID);
	if (use_fw_skip_pid) {
		t_dev_warn(dev, "FW compare: skip pid check\n");
		goto out;
	}

	if (memcmp(pid, fw->product_id, 8)) {
		if (fw->invalid_pid) {
			t_dev_err(dev,
				"FW compare: bin-pid[%s], dev-pid invalid, halted (up %02X, fup %02X)\n",
				pid, update, ts->force_fwup);
			return -EINVAL;

		}

		t_dev_err(dev,
			"FW compare: bin-pid[%s] != dev-pid[%s], halted (up %02X, fup %02X)\n",
			pid, fw->product_id, update, ts->force_fwup);
		return -EINVAL;
	}

out:
	t_dev_info(dev,
		"FW compare: up %02X, fup %02X\n",
		update, ts->force_fwup);

	return update;
}

static int siw_hal_fw_upgrade_fw_pre(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ret;

	/* Reset CM3 core */
	ret = siw_hal_fw_wr_value(dev, reg->spr_rst_ctl, 2);
	if (ret < 0) {
		goto out;
	}

	/* Disable SRAM write protection */
	ret = siw_hal_fw_sram_wr_enable(dev, 1);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade_fw_post(struct device *dev, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dn_cmd, chk_resp, data;
	int ret;

	/* Enable SRAM write protection */
	ret = siw_hal_fw_sram_wr_enable(dev, 0);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_upgrade_fw_post_quirk(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_size_check_post(dev, fw_size);
	if (ret < 0) {
		goto out;
	}

	/* Firmware Download Start */
	dn_cmd = (FLASH_KEY_CODE_CMD << 16) | 1;
	ret = siw_hal_fw_wr_value(dev, reg->tc_flash_dn_ctl, dn_cmd);
	if (ret < 0) {
		goto out;
	}

	touch_msleep(ts->caps.hw_reset_delay);

	/* download check */
	chk_resp = FLASH_CODE_DNCHK_VALUE;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, 0xFFFF,
				FW_POST_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_1),
				FW_POST_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - code check(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}
	t_dev_info(dev, "FW upgrade: code check done\n");

out:
	return ret;
}

static int siw_hal_fw_upgrade_fw(struct device *dev,
				u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	u8 *fw_data;
	int fw_size_max;
	int ret = 0;

	/*
	 * Stage 1-1 : download code data
	 */
	fw_size_max = touch_fw_size(ts);

	ret = siw_hal_fw_upgrade_fw_pre(dev);
	if (ret < 0) {
		goto out;
	}

	/*
	 * [Caution]
	 * The size for F/W upgrade is fw_size_max, not fw->size
	 * because the fw file can have config area.
	 */
	fw_data = fw_buf;
	ret = siw_hal_fw_upgrade_fw_core(dev, fw_data, fw_size_max);
	if (ret < 0) {
		goto out;
	}

	/*
	 * Stage 1-2: upgrade code data
	 */
	ret = siw_hal_fw_upgrade_fw_post(dev, fw_size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf_pre(struct device *dev, u32 *value)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 data = 0;
	int ret = 0;

	data = siw_hal_fw_upgrade_conf_pre_quirk(dev);
	if (data) {
		goto out;
	}

	ret = siw_hal_fw_rd_value(dev, reg->tc_confdn_base_addr, &data);
	if (ret < 0) {
		goto out;
	}

out:
	if (value)
		*value = data;

	return ret;
}

static int siw_hal_fw_upgrade_release_cm3(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ret;

	/* Release & Reset CM3 */
	ret = siw_hal_fw_wr_value(dev, reg->spr_rst_ctl, 1);

	return ret;
}

static int siw_hal_fw_upgrade_conf_post(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dn_cmd, chk_resp, data;
	int ret;

	/* Conf Download Start */
	dn_cmd = (FLASH_KEY_CONF_CMD << 16) | 2;
	ret = siw_hal_fw_wr_value(dev, reg->tc_flash_dn_ctl, dn_cmd);
	if (ret < 0) {
		goto out;
	}

	/* Conf check */
	chk_resp = FLASH_CONF_DNCHK_VALUE_TYPE_X;
	ret = siw_hal_condition_wait(dev, reg->tc_flash_dn_status, &data,
				chk_resp, 0xFFFF,
				CONF_POST_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_2),
				CONF_POST_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - conf check(%Xh), %X\n",
			chk_resp, data);
		ret = -EPERM;
		goto out;
	}

	t_dev_info(dev, "FW upgrade: conf check done\n");

out:
	return ret;
}

static int siw_hal_fw_upgrade_conf(struct device *dev,
			     u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int fw_size_max;
	u32 conf_dn_addr;
	u32 data;
	int ret;

	fw_size_max = touch_fw_size(ts);

	/*
	 * Stage 2-1: download config data
	 */
	ret = siw_hal_fw_upgrade_conf_pre(dev, &data);
	if (ret < 0) {
		goto out;
	}

	conf_dn_addr = ((data >> 16) & 0xFFFF);

	t_dev_dbg_fwup(dev, "FW upgrade: conf_dn_addr %04Xh (%08Xh)\n",
		conf_dn_addr, data);
#if 0
	if (conf_dn_addr >= (0x1200) || conf_dn_addr < (0x8C0)) {
		t_dev_err(dev, "FW upgrade: failed - conf base invalid\n");
		ret = -EPERM;
		goto out;
	}
#endif

	/* C_CFG */
	ret = siw_hal_fw_upgrade_conf_core(dev, conf_dn_addr,
				(u8 *)&fw_buf[fw_size_max], FLASH_CONF_SIZE_TYPE_X);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_fw_upgrade_conf_quirk(dev, fw_buf, fw_size);
	if (ret < 0) {
		goto out;
	}

	/*
	 * Stage 2-2: upgrade config data
	 */
	ret = siw_hal_fw_upgrade_conf_post(dev);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int siw_hal_fw_upgrade(struct device *dev,
				u8 *fw_buf, int fw_size, int retry)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int fw_size_max;
	u32 include_conf;
	int ret = 0;

	t_dev_info(dev, "===== FW upgrade: start (%d) =====\n", retry);

	if (fquirks->fwup_upgrade) {
		ret = fquirks->fwup_upgrade(dev, fw_buf, fw_size, retry);
		if (ret < 0) {
			goto out;
		}
		goto out_done;
	}

	fw_size_max = touch_fw_size(ts);

	ret = siw_hal_fw_size_check(dev, fw_size);
	if (ret < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_CHK);
		goto out;
	}

	include_conf = (chip->fw.conf_skip) ? 0 : !!(fw_size > fw_size_max);
	t_dev_info(dev, "FW upgrade:%s include conf data\n",
			(include_conf) ? "" : " not");

	t_dev_dbg_fwup(dev, "FW upgrade: fw size %08Xh, fw_size_max %08Xh\n",
			fw_size, fw_size_max);

	ret = siw_hal_fw_upgrade_fw(dev, fw_buf, fw_size);
	if (ret < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_CODE);
		goto out;
	}

	if (include_conf) {
		ret = siw_hal_fw_upgrade_conf(dev, fw_buf, fw_size);
		if (ret < 0) {
			siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_CFG);
			goto out;
		}
	}

	ret = siw_hal_fw_upgrade_release_cm3(dev);
	if (ret < 0) {
		goto out;
	}

out_done:
	t_dev_info(dev, "===== FW upgrade: done (%d) =====\n", retry);

out:
	return ret;
}

static int siw_hal_fw_do_get_fw_abs(const struct firmware **fw_p,
				const char *name,
                struct device *dev)
{
//	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct firmware *fw = NULL;
	struct file *filp = NULL;
	char *buf = NULL;
	loff_t offset = 0;
	loff_t size;
	int rd_size;
	int ret = 0;

	fw = kzalloc(sizeof(*fw), GFP_KERNEL);
	if (fw == NULL) {
		dev_err(dev, "can't allocate fw(struct firmware)\n");
		return -ENOMEM;
	}

	filp = filp_open(name, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		dev_err(dev, "can't open %s\n", name);
		kfree(fw);
		return (int)PTR_ERR(filp);
	}

	size = vfs_llseek(filp, 0, SEEK_END);
	if (size < 0)	 {
		t_dev_err(dev, "invalid file size, %d\n", (int)size);
		ret = -EINVAL;
		goto out;
	}

	buf = kzalloc((size_t)size, GFP_KERNEL);
	if (buf == NULL) {
		t_dev_err(dev, "can't allocate firm buf\n");
		ret = -ENOMEM;
		goto out;
	}

	rd_size = touch_kernel_read(filp, buf, (size_t)size, &offset);
	if (rd_size != (int)size) {
		t_dev_err(dev, "can't read[%d], %d\n",
			(int)size, (int)rd_size);
		ret = (rd_size < 0) ? rd_size : -EFAULT;
		goto out;
	}

	fw->data = buf;
	fw->size = size;

	if (fw_p) {
		*fw_p = fw;
	}

	filp_close(filp, NULL);

	return 0;

out:
	if (buf)
		kfree(buf);

	if (fw)
		kfree(fw);

	filp_close(filp, NULL);

	return ret;
}

static int siw_hal_fw_do_get_file(const struct firmware **fw_p,
				const char *name,
                struct device *dev,
                int abs_path)
{
	if (abs_path) {
		return siw_hal_fw_do_get_fw_abs(fw_p, name, dev);
	}

	return request_firmware(fw_p, name, dev);
}

static int siw_hal_fw_get_file(const struct firmware **fw_p,
				char *fwpath,
				struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	const struct firmware *fw = NULL;
	char *src_path;
	int src_len;
	int abs_path = 0;
	int ret = 0;

	if (ts->test_fwpath[0]) {
		src_path = (char *)ts->test_fwpath;
	} else if (ts->def_fwcnt) {
		src_path = (char *)ts->def_fwpath[0];
	} else {
		t_dev_err(dev, "no target fw defined\n");
		ret = -ENOENT;
		goto out;
	}

	/*
	 * Absolute path option
	 * ex) echo {root}/.../target_fw_img > fw_upgrade
	 *          ++++++~~~~~~~~~~~~~~~~~~
	 *          flag  |
	 *                absolute path
	 */
	src_len = strlen(src_path);
	if (strncmp(src_path, "{root}", 6) == 0) {
		abs_path = 1;
		src_path += 6;
		src_len -= 6;
	}
	chip->fw_abs_path = abs_path;

	strncpy(fwpath, src_path, src_len);
	fwpath[src_len] = 0;

	t_dev_info(dev, "target fw: %s (%s)\n",
		fwpath,
		(abs_path) ? "abs" : "rel");

	ret = siw_hal_fw_do_get_file(&fw,
				(const char *)fwpath,
				dev, abs_path);
	if (ret < 0) {
		if (ret == -ENOENT) {
			t_dev_err(dev, "can't find fw: %s\n", fwpath);
		} else {
			t_dev_err(dev, "can't %s fw: %s, %d\n",
				(abs_path) ? "read" : "request",
				fwpath, ret);
		}
		goto out;
	}

	if (fw_p) {
		*fw_p = fw;
	}

out:
	return ret;
}

static void siw_hal_fw_release_firm(struct device *dev,
			const struct firmware *fw)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;

	if (fw == NULL) {
		return;
	}

	if (chip->fw_abs_path) {
		chip->fw_abs_path = 0;
		kfree(fw->data);
		kfree(fw);
		return;
	}

	release_firmware(fw);
}

/*
 * FW upgrade option
 *
 * 1. If TOUCH_USE_FW_BINARY used
 * 1-1 Default upgrade (through version comparison)
 *     do upgarde using binary header link
 * 1-2 echo {bin} > fw_upgrade
 *     do force-upgrade using binary header link (same as 1-1)
 * 1-3 echo /.../fw_img > fw_upgrade
 *     do force-upgrade using request_firmware (relative path)
 * 1-4 echo {root}/.../fw_img > fw_upgrade
 *     do force-upgrade using normal file open control (absolute path)
 *
 * 2. Else
 * 2-1 Default upgrade (through version comparison)
 *     do upgarde using request_firmware (relative path)
 * 2-2 echo /.../fw_img > fw_upgrade
 *     do force-upgrade using request_firmware (relative path)
 * 2-3 echo {root}/.../fw_img > fw_upgrade
 *     do force-upgrade using normal file open control (absolute path)
 */
static int siw_hal_upgrade_not_allowed(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (atomic_read(&chip->boot) == IC_BOOT_FAIL) {
		return 0;
	}

	if (chip->lcd_mode != LCD_MODE_U3) {
		t_dev_warn(dev, "FW upgrade: not U3 mode, %s(%d)\n",
			siw_lcd_driving_mode_str(chip->lcd_mode),
			chip->lcd_mode);
		return 1;
	}

	if (siw_hal_access_not_allowed(dev, "FW_Upgrade", HAL_ACCESS_CHK_SKIP_INIT)) {
		return 1;
	}

	return 0;
}

static int siw_hal_upgrade_pre(struct device * dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int ctrl = chip->tc_cmd_table[LCD_MODE_STOP];
	u32 rdata;
	int ret = 0;

	if ((ctrl < 0) || !touch_mode_allowed(ts, LCD_MODE_STOP)) {
		goto out;
	}

	/*
	 * TC_STOP before fw upgrade
	 * to avoid unexpected IRQ drop by internal watchdog
	 */
	rdata = reg->tc_drive_ctl;

	ret = siw_hal_write_value(dev, rdata, ctrl);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: TC stop(%04Xh, 0x%08X) failed\n",
				rdata, ctrl);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: TC stop(%04Xh, 0x%08X)\n",
			rdata, ctrl);

	rdata = chip->drv_delay + hal_dbg_delay(chip, HAL_DBG_DLY_TC_DRIVING_1);
	touch_msleep(rdata);

out:
	return ret;
}

static int siw_hal_upgrade(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fw_bin *fw_bin = NULL;
	const struct firmware *fw = NULL;
	char *fwpath = NULL;
	u8 *fw_buf = NULL;
	int fw_max_size = touch_fw_size(ts);
	int fw_size = 0;
	int fw_up_binary = 0;
	int i = 0;
	int ret_val = 0;
	int ret = 0;

	siw_hal_set_fwup_status(chip, FWUP_STATUS_BUSY);

	chip->fw_abs_path = 0;

	if (siw_hal_upgrade_not_allowed(dev)) {
		t_dev_warn(dev, "FW upgrade: not granted\n");
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_OP);
		return EACCES;
	}

	t_dev_info(dev, "fw type: %s\n", FW_TYPE_STR);

	fwpath = touch_getname();
	if (fwpath == NULL) {
		t_dev_err(dev, "failed to allocate name buffer - fwpath\n");
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_OP);
		return -ENOMEM;
	}

	if (touch_flags(ts) & TOUCH_USE_FW_BINARY) {
		fw_up_binary = 1;

		if (ts->force_fwup & FORCE_FWUP_SYS_STORE) {
			switch (ts->test_fwpath[0]) {
			case 0:
				/* fall through */
			case ' ':	/* ignore space */
				break;

			default:
				/* if target string is not "{bin}" */
				if (strncmp(ts->test_fwpath, "{bin}", 5) != 0) {
					fw_up_binary = 0;
				}
				break;
			}
		}
	}

	if (fw_up_binary) {
		t_dev_info(dev, "getting fw from binary header data\n");
		fw_bin = touch_fw_bin(ts);
		if (fw_bin != NULL) {
			fw_buf = fw_bin->fw_data;
			fw_size = fw_bin->fw_size;
		} else {
			t_dev_warn(dev, "empty fw info\n");
		}
	} else {
		t_dev_info(dev, "getting fw from file\n");
		ret = siw_hal_fw_get_file(&fw, fwpath, dev);
		if (ret < 0) {
			siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_OPEN);
			goto out;
		}
		fw_buf = (u8 *)fw->data;
		fw_size = (int)fw->size;
	}

//	ret = -EINVAL;
	ret = -EPERM;

	if ((fw_buf == NULL) || !fw_size) {
		t_dev_err(dev, "invalid fw info\n");
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_CHK);
		goto out_fw;
	}

	if (fw_size < fw_max_size) {
		t_dev_err(dev, "invalid fw size: %Xh < %Xh\n",
			fw_size, fw_max_size);
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_CHK);
		goto out_fw;
	}

	t_dev_info(dev, "fw size: %d\n", fw_size);

	ret_val = siw_hal_fw_compare(dev, fw_buf);
	if (ret_val < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_F_CHK);
		ret = ret_val;
		goto out_fw;
	}

	if (!ret_val) {
		goto out_fw;
	}

	ret_val = siw_hal_upgrade_pre(dev);
	if (ret_val < 0) {
		siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_IO);
		ret = ret_val;
		goto out_fw;
	}

	siw_hal_alive_pause_set(dev, ALIVE_PAUSE_FWUP);

	touch_msleep(100);

	siw_hal_disable_flash_wp(dev);
	for (i = 0; (i < 2) && (ret < 0); i++) {
		ret = siw_hal_fw_upgrade(dev, fw_buf, fw_size, i);
	}
	siw_hal_enable_flash_wp(dev);

out_fw:
	siw_hal_fw_release_firm(dev, fw);

out:
	if (ret < 0) {
		siwmon_submit_ops_step_chip_wh_name(dev, "%s - FW upgrade halted",
				touch_chip_name(ts), ret);
		if (siw_hal_get_fwup_status(chip) == FWUP_STATUS_BUSY) {
			siw_hal_set_fwup_status(chip, FWUP_STATUS_NG_IO);
		}
	} else {
		siwmon_submit_ops_step_chip_wh_name(dev, "%s - FW upgrade done",
				touch_chip_name(ts), ret);
		siw_hal_set_fwup_status(chip, FWUP_STATUS_OK);
	}

	touch_putname(fwpath);

	return ret;
}

static void siw_hal_chk_dbg_report(struct device *dev, u32 status, int irq)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 addr = reg->tc_ic_status;
	u32 irq_type = siw_tc_sts_irq_type(status);
	u32 ic_debug[4];
	u32 debug_info = 0;
	u32 debug_len = 0;
	u32 debug_type = 0;
	int ret = 0;

	if (!chip->opt.f_dbg_report) {
		return;
	}

	switch (irq_type) {
	case TC_STS_IRQ_TYPE_ABNORMAL:
	case TC_STS_IRQ_TYPE_DEBUG:
		break;
	default:
		return;
	}

	addr += ((0x100>>2) - 2);

	ret = siw_hal_reg_read(dev, addr, ic_debug, sizeof(ic_debug));
	if (ret < 0) {
		return;
	}

	debug_info = ic_debug[3];
	debug_len = ((debug_info>>24) & 0xFF);
	debug_type = (debug_info & ((1<<24)-1));

	t_dev_info(dev,
			"[%d] ic debug: s %08Xh / m %Xh, l %Xh, t %Xh (%08Xh)\n",
			irq, status, irq_type, debug_len, debug_type, debug_info);

	t_dev_info(dev,
		"[%d] ic debug: log %08Xh %08Xh %08Xh\n",
		irq, ic_debug[0], ic_debug[1], ic_debug[2]);
}

static int siw_hal_tc_driving_end(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->tc_driving_end) {
		ret = fquirks->tc_driving_end(dev, mode);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int siw_hal_tc_driving_post(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->tc_driving_post) {
		ret = fquirks->tc_driving_post(dev, mode);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int siw_hal_tc_driving_cmd(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	int ctrl = 0;

	if ((mode < LCD_MODE_U0) || (mode >= LCD_MODE_MAX)) {
		t_dev_err(dev, "invalid mode, %d\n", mode);
		return -EINVAL;
	}

	ctrl = chip->tc_cmd_table[mode];
	if (ctrl < 0) {
		t_dev_err(dev, "%s(%d) not granted\n",
			siw_lcd_driving_mode_str(mode), mode);
		return -ESRCH;
	}

	chip->driving_ctrl = ctrl;

	return ctrl;
}

static int siw_hal_tc_driving_quirk(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->tc_driving) {
		ret = fquirks->tc_driving(dev, mode);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int siw_hal_tc_driving(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 tc_status = 0;
	u32 running_status = 0;
	u32 rdata;
	int ctrl = 0;
	int re_init = 0;
	int ret = 0;

	if (siw_hal_tc_not_allowed(dev, "tc driving not allowed")) {
		return 0;
	}

	if (touch_mode_not_allowed(ts, mode)) {
		return -EPERM;
	}

	if (atomic_read(&ts->recur_chk)) {
		/* keep the last value for retry case */
		mode = chip->driving_mode;
		t_dev_info(dev, "keep the last mode(%d) for retry\n", mode);
	}

	siw_hal_tc_driving_quirk(dev, mode);

	ctrl = siw_hal_tc_driving_cmd(dev, mode);
	if (ctrl < 0) {
		return ctrl;
	}

	chip->driving_mode = mode;

	touch_msleep(hal_dbg_delay(chip, HAL_DBG_DLY_TC_DRIVING_0));

	t_dev_info(dev, "current driving mode is %s\n",
			siw_lcd_driving_mode_str(mode));

	rdata = siw_hal_get_subdisp_sts(dev);
	t_dev_info(dev, "DDI Display Mode[%04Xh] = 0x%08X\n",
			reg->spr_subdisp_status, rdata);

	rdata = reg->tc_drive_ctl;

	ret = siw_hal_write_value(dev, rdata, ctrl);
	if (ret < 0) {
		t_dev_err(dev, "TC Driving[%04Xh](0x%08X) failed, %d\n",
				rdata, ctrl, ret);
		return ret;
	}
	t_dev_info(dev, "TC Driving[%04Xh] wr 0x%08X\n",
			rdata, ctrl);

	rdata = chip->drv_delay + hal_dbg_delay(chip, HAL_DBG_DLY_TC_DRIVING_1);
	touch_msleep(rdata);
	t_dev_dbg_base(dev, "waiting %d msecs\n", rdata);

	if (mode == LCD_MODE_U3_PARTIAL) {
		goto out;
	}

	siw_hal_tc_driving_post(dev, mode);

	ret = siw_hal_read_value(dev,
				reg->tc_status,
				&tc_status);
	if (ret < 0) {
		t_dev_err(dev, "failed to get tc_status\n");
		atomic_set(&ts->recur_chk, 0);
		return ret;
	}

	siw_hal_chk_dbg_report(dev, tc_status, 0);

	running_status = siw_tc_sts_running_sts(tc_status);

	re_init = 0;
	if (mode == LCD_MODE_STOP) {
		re_init = !!running_status;
	} else {
		if (!running_status ||
			(running_status == 0x10) ||
			(running_status == 0x0F)){
			re_init = 1;
		}
	}

	if (re_init) {
		int delay = ts->caps.hw_reset_delay + hal_dbg_delay(chip, HAL_DBG_DLY_HW_RST_2);

		rdata = siw_hal_get_subdisp_sts(dev);

		if (ts->is_charger || atomic_read(&ts->recur_chk)) {
			t_dev_err(dev, "command failed: mode %d, tc_status %08Xh, DDI %08Xh\n",
				mode, tc_status, rdata);
			atomic_set(&ts->recur_chk, 0);
			return -EFAULT;
		}

		t_dev_err(dev, "command missed: mode %d, tc_status %08Xh, DDI %08Xh\n",
			mode, tc_status, rdata);

		atomic_set(&ts->recur_chk, 1);

		ret = siw_hal_reinit(dev, 1, delay, 1, siw_hal_init);
		if (ret < 0) {
			return ret;
		}
	} else {
		t_dev_info(dev, "command done: mode %d, running_sts %02Xh\n",
			mode, running_status);

		siw_hal_tc_driving_end(dev, mode);
	}

out:
	atomic_set(&ts->recur_chk, 0);

	return 0;
}

static int siw_hal_lpwg_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int mode = chip->lcd_mode;
	int ret = 0;

	if (atomic_read(&chip->init) == IC_INIT_NEED) {
		t_dev_warn(dev, "Not Ready, Need IC init (lpwg_mode)\n");
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		mode = LCD_MODE_STOP;
	}

	ret = siw_hal_tc_driving(dev, mode);

	t_dev_info(dev, "tc driving(%d): lcd_mode %d, driving_mode %d\n",
			ret, mode, chip->driving_mode);

	return ret;
}


#define siw_chk_sts_snprintf(_dev, _buf, _buf_max, _size, _fmt, _args...) \
		({	\
			int _n_size = 0;	\
			_n_size = __siw_snprintf(_buf, _buf_max, _size, _fmt, ##_args);	\
			t_dev_dbg_trace(_dev, _fmt, ##_args);	\
			_n_size;	\
		})

static int siw_hal_check_status_type_x(struct device *dev,
				u32 status, u32 ic_status, int irq)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_status_filter *filter = chip->status_filter;
	u32 check_mask, detected;
	int type_error;
	u32 irq_type = siw_tc_sts_irq_type(status);
	u32 log_flag = 0;
	u32 err_send = 0;
	u32 ic_abnormal, ic_error, ic_disp_err;
	int log_max = IC_CHK_LOG_MAX;
	char log[IC_CHK_LOG_MAX] = {0, };
	int len = 0;
	int ret = 0;

	if (filter == NULL) {
		return -EINVAL;
	}

	if (ic_status & ~chip->status_mask_ic_valid) {
		t_dev_err(dev, "[%d] status %08Xh, ic_status %08Xh, ic_status invalid\n",
			irq, status, ic_status);

		return -ERESTART;
	}

	while (1) {
		if (!filter->id || !filter->width) {
			break;
		}

		type_error = !!(filter->flag & STS_FILTER_FLAG_TYPE_ERROR);

		check_mask = ((1<<filter->width)-1)<<filter->pos;

		detected = (type_error) ? (status & check_mask) : !(status & check_mask);

		if (check_mask && detected) {
			log_flag |= check_mask;

			len += siw_chk_sts_snprintf(dev, log, log_max, len,
						"[b%d] %s ", filter->pos, filter->str);
		}

		filter++;
	}

	if (log_flag) {
		t_dev_err(dev, "[%d] status %08Xh, ic_status %08Xh, (%08Xh) %s\n",
			irq, status, ic_status, log_flag, log);
	}

	ic_abnormal = ic_status & chip->status_mask_ic_abnormal;
	ic_error = ic_status & chip->status_mask_ic_error;
	ic_disp_err = ic_status & chip->status_mask_ic_disp_err;

	if (ic_abnormal || ic_error || ic_disp_err) {
		u32 err_val[3] = { ic_abnormal, ic_error, ic_disp_err };
		char *err_str[3] = {
			"esd",
			"watchdog",
			"dic"
		};
		int log_add = !log_flag;
		int err_pre, i;

		len = siw_chk_sts_snprintf(dev, log, log_max, 0,
					"[%d] ", irq);

		err_pre = 0;
		for (i = 0; i < ARRAY_SIZE(err_val) ; i++) {
			if (!err_val[i]) {
				continue;
			}

			if (err_pre) {
				len += siw_chk_sts_snprintf(dev, log, log_max, len, " & ");
			}
			len += siw_chk_sts_snprintf(dev, log, log_max, len, "%s", err_str[i]);
			err_pre |= err_val[i];
		}

		if (log_add) {
			len += siw_chk_sts_snprintf(dev, log, log_max, len,
							" - ");

			len += siw_chk_sts_snprintf(dev, log, log_max, len,
						"status %08Xh, ic_status %08Xh%s",
						status, ic_status,
						(log_add & 0x02) ? ", " : " ");
		}

		t_dev_err(dev, "%s\n", log);

		err_send |= (ic_abnormal | ic_disp_err);
	}

	if (err_send) {
		ret = -ERESTART;
	}

	if (ret == -ERESTART) {
		return ret;
	}

	/*
	 * Check interrupt_type[19:16] in TC_STATUS
	 */
	switch (irq_type) {
	case TC_STS_IRQ_TYPE_INIT_DONE:
		t_dev_info(dev, "[%d] TC Driving OK\n", irq);
		ret = -ERANGE;
		break;
	case TC_STS_IRQ_TYPE_REPORT:	/* Touch report */
		break;
	default:
		t_dev_dbg_trace(dev, "[%d] irq_type %Xh\n",
			irq, irq_type);
		ret = -ERANGE;
		break;
	}

	return ret;
}

#define STS_RET_ERR		ERESTART

static int siw_hal_do_check_status(struct device *dev,
				u32 status, u32 ic_status, int irq)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	u32 reset_clr_bit = 0;
	u32 logging_clr_bit = 0;
	u32 int_norm_mask = 0;
	u32 status_mask = 0;
	int skip_trace = (irq & 0x80);
	int ret_pre = 0;
	int ret = 0;

	irq &= 0x01;

	if (!status && !ic_status) {
		t_dev_err(dev, "[%d] all low detected\n", irq);
		return -STS_RET_ERR;
	}
	if ((status == ~0) && (ic_status == ~0)) {
		t_dev_err(dev, "[%d] all high detected\n", irq);
		return -STS_RET_ERR;
	}

	reset_clr_bit = chip->status_mask_reset;
	logging_clr_bit = chip->status_mask_logging;
	int_norm_mask = chip->status_mask_normal;

	status_mask = status ^ int_norm_mask;

	if (!skip_trace) {
		t_dev_dbg_trace(dev, "[%d] h/w:%Xh, f/w:%Xh(%Xh)\n",
				irq, ic_status, status, status_mask);
	}

	if (status_mask & reset_clr_bit) {
		t_dev_err(dev,
			"[%d] need reset : status %08Xh, ic_status %08Xh, chk %08Xh (%08Xh)\n",
			irq, status, ic_status, status_mask & reset_clr_bit, reset_clr_bit);
		ret_pre = -ERESTART;
	} else if (status_mask & logging_clr_bit) {
		t_dev_err(dev,
			"[%d] need logging : status %08Xh, ic_status %08Xh, chk %08Xh (%08Xh)\n",
			irq, status, ic_status, status_mask & logging_clr_bit, logging_clr_bit);
		ret_pre = -ERANGE;
	}

	switch (chip->status_type) {
	case CHIP_STATUS_TYPE_2:
	case CHIP_STATUS_TYPE_1:
	case CHIP_STATUS_TYPE_0:
		ret = siw_hal_check_status_type_x(dev, status, ic_status, irq);
		siw_hal_chk_dbg_report(dev, status, irq);
		break;
	default:
		t_dev_warn(dev, "unknown status type, %d\n", chip->status_type);
		break;
	}

	if (ret == -ETDSENTESDIRQ) {
		return ret;
	}

	if (ret_pre) {
		if (ret != -ERESTART) {
			ret = ret_pre;
		}
	}

	return ret;
}

static int siw_hal_check_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	u32 ic_status = siw_report_ic_status(chip);
	u32 status = siw_report_tc_status(chip);

	return siw_hal_do_check_status(dev, status, ic_status, 1);
}

#define PALM_DETECTED	1

static int siw_hal_irq_abs_palm_chk(struct device *dev, int track_id, int event)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (ts->role.use_palm_opt)
		return 0;

	if (track_id != PALM_ID) {
		return 0;
	}

	if (event == TOUCHSTS_DOWN) {
		ts->is_cancel = 1;
		t_dev_info(dev, "Palm Detected\n");
	} else if (event == TOUCHSTS_UP) {
		ts->is_cancel = 0;
		t_dev_info(dev, "Palm Released\n");
	}
	ts->tcount = 0;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return PALM_DETECTED;
}

static int siw_hal_irq_abs_data_default(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_touch_info *info = siw_report_info(chip);
	struct siw_hal_touch_data *data = siw_report_data(chip);
	struct touch_data *tdata = NULL;
	u32 palm_bit = (ts->role.use_palm_opt & BIT(1)) ? info->palm_bit : 0;
	u32 touch_count = info->touch_cnt;
	int finger_index = 0;
	int i = 0;
	int ret = 0;

	ts->new_mask = 0;

	/* check if palm detected */
	ret = siw_hal_irq_abs_palm_chk(dev, data->track_id, data->event);
	if (ret == PALM_DETECTED) {
		goto out;
	}

	for (i = 0; i < touch_count; i++, data++) {
		if (data->track_id >= touch_max_finger(ts)) {
			continue;
		}

		if (palm_bit & BIT(i)) {
			continue;
		}

		if ((data->event == TOUCHSTS_DOWN) ||
			(data->event == TOUCHSTS_MOVE)) {
			ts->new_mask |= (1 << data->track_id);
			tdata = ts->tdata + data->track_id;

			tdata->id = data->track_id;
			tdata->type = data->tool_type;
			tdata->event = data->event;
			tdata->x = data->x;
			tdata->y = data->y;
			tdata->pressure = data->pressure;
			tdata->width_major = data->width_major;
			tdata->width_minor = data->width_minor;

			if (data->width_major == data->width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = (s8)data->angle;

			finger_index++;

			t_dev_dbg_abs(dev,
					"touch data [id %d, t %d, e %d, x %d, y %d, z %d - %d, %d, %d]\n",
					tdata->id,
					tdata->type,
					tdata->event,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);
		}
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

out:
	return ret;
}

static int siw_hal_irq_abs_pre_default(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_touch_info *info = siw_report_info(chip);
	struct siw_hal_touch_data *data = siw_report_data(chip);
	int touch_cnt = info->touch_cnt;

	/* check if touch cnt is valid */
	if (!touch_cnt || (touch_cnt > ts->caps.max_id)) {
		t_dev_dbg_abs(dev, "Invalid touch count, %d(%d)\n",
			touch_cnt, ts->caps.max_id);

		/* debugging */
		t_dev_dbg_abs(dev, "t %d, ev %d, id %d, x %d, y %d, p %d, a %d, w %d %d\n",
			data->tool_type, data->event, data->track_id,
			data->x, data->y, data->pressure, data->angle,
			data->width_major, data->width_minor);

		return -ERANGE;
	}

	return 0;
}

static int siw_hal_irq_abs_pre(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	char *str = "";
	int ret = 0;

	switch (chip->report_type) {
	default:
		ret = siw_hal_irq_abs_pre_default(dev);
		break;
	}

	if (ret < 0) {
		t_dev_err(dev, "irq_abs_pre%s failed, %d\n", str, ret);
	}

	return ret;
}

static int siw_hal_irq_abs_data(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	char *str = "";
	int ret = 0;

	switch (chip->report_type) {
	default:
		ret = siw_hal_irq_abs_data_default(dev);
		break;
	}

	if (ret < 0) {
		t_dev_err(dev, "irq_abs_data%s failed, %d\n", str, ret);
	}

	return ret;
}

static int siw_hal_irq_abs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	if (siw_report_info(chip) == NULL) {
		t_dev_err(dev, "irq_abs: report_info not defined\n");
		return -ESRCH;
	}

	ret = siw_hal_irq_abs_pre(dev);
	if (ret < 0) {
		return ret;
	}

	ret = siw_hal_irq_abs_data(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

#define GET_REPORT_BASE_PKT		(1)
#define GET_REPORT_BASE_HDR		(3)

static int siw_hal_irq_get_report(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int is_flex_report = chip->opt.f_flex_report;
	u32 addr = reg->tc_ic_status;
	char *buf = (char *)siw_report_info(chip);
	int size = 0;
	int pkt_unit = 0;
	int pkt_cnt = 0;
	int wakeup_type = 0;
	int touch_cnt = 0;
	int ret = 0;

	if (buf == NULL) {
		t_dev_err(dev, "get_report: report_info not defined\n");
		return -ESRCH;
	}

	pkt_unit = sizeof(struct siw_hal_touch_data);
	pkt_cnt = GET_REPORT_BASE_PKT;
	touch_cnt = touch_max_finger(ts) - pkt_cnt;

	size = (GET_REPORT_BASE_HDR<<2);
	size += (pkt_unit * pkt_cnt);

	/*
	 * Dynamic read access
	 */
	if (is_flex_report) {
		t_dev_dbg_irq(dev, "get dynamic report, %d\n", size);

		ret = siw_hal_reg_read(dev, addr, (void *)buf, size);
		if (ret < 0) {
			return ret;
		}

		wakeup_type = siw_report_info_wakeup_type(chip);
		touch_cnt = siw_report_info_touch_cnt(chip);

		if (wakeup_type != ABS_MODE) {
			/* No need to read more */
			return 0;
		}

		if ((touch_cnt <= pkt_cnt) || (touch_cnt > ts->caps.max_id)) {
			/* No need to read more */
			return 0;
		}

		addr += (size>>2);
		buf += size;
		size = 0;

		touch_cnt -= pkt_cnt;
	}

	size += (pkt_unit * touch_cnt);

	ret = siw_hal_reg_read(dev, addr, (void *)buf, size);

	return ret;
}

static int siw_hal_irq_exception(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	char *title = NULL;

	if (atomic_read(&chip->init) == IC_INIT_NEED) {
		title = "Not Ready, Need IC init (irq)";
		goto out;
	}

	if (!chip->status_type) {
		title = "No status type";
		goto out;
	}

	if (!chip->report_type) {
		title = "No report type";
		goto out;
	}

	return 0;

out:
	t_dev_warn(dev, "%s\n", title);

	return 1;
}

static int siw_hal_irq_skip_event(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	char *title = NULL;

	switch (chip->driving_mode) {
	case LCD_MODE_STOP:
		title = "stop state";
		goto out;
	}

	return 0;

out:
	t_dev_info(dev, "skip event - %s\n", title);
	siw_touch_report_all_event(ts);

	return 1;
}

static int siw_hal_irq_handler(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	int wakeup_type = 0;
	int touch_cnt = 0;
	int ret = 0;

	if (siw_hal_irq_exception(dev)) {
		return 0;
	}

	ret = siw_hal_irq_get_report(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_alive_check(dev);
	if (ret < 0) {
		goto out;
	}

	ret = siw_hal_check_status(dev);
	if (ret < 0) {
		goto out;
	}

	wakeup_type = siw_report_info_wakeup_type(chip);
	touch_cnt = siw_report_info_touch_cnt(chip);

	t_dev_dbg_irq(dev, "hal irq handler: wakeup_type %d\n", wakeup_type);

	if (siw_hal_irq_skip_event(dev)) {
		goto out;
	}

	if (wakeup_type == ABS_MODE) {
		ret = siw_hal_irq_abs(dev);
		if (ret) {
			t_dev_err(dev, "siw_hal_irq_abs failed(%d), %d\n",
				touch_cnt, ret);
			goto out;
		}
	} else {
		t_dev_dbg_irq(dev, "non-abs type, %d\n", wakeup_type);
	}

out:
	return ret;
}

static int siw_hal_lcd_mode(struct device *dev, u32 mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	if (touch_mode_not_allowed(ts, mode)) {
		return -EPERM;
	}

	if (chip->lcd_mode == mode)
		return 0;

	chip->prev_lcd_mode = chip->lcd_mode;
	chip->lcd_mode = mode;

	t_dev_info(dev, "lcd_mode: %d (prev: %d)\n",
		mode, chip->prev_lcd_mode);

	return 0;
}

void siw_hal_change_mode(struct device *dev, int value)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret = 0;

	ret = siw_hal_lcd_mode(dev, value);
	if (ret < 0) {
		return;
	}

	if (chip->lcd_mode == LCD_MODE_STOP) {
		ret = siw_hal_lpwg_mode(dev);
		if (ret >= 0) {
			return;
		}
		t_dev_err(dev, "stop control failed, retry init\n");
		siw_hal_reset_ctrl(dev, HW_RESET_ASYNC);
		return;
	}

	siw_touch_qd_init_work_now(ts);
}

enum {
	SIW_GET_CHIP_NAME	= (1<<0),
	SIW_GET_VERSION		= (1<<1),
	SIW_GET_REVISION	= (1<<2),
	SIW_GET_PRODUCT		= (1<<3),
	/* */
	SIW_GET_OPT1		= (1<<8),
	/* */
	SIW_GET_VER_SIMPLE	= (1<<16),
	/* */
	SIW_GET_ALL			= 0xFFFF,
};

static int siw_hal_get_cmd_version(struct device *dev, char *buf, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	int offset = 0;
//	int ret = 0;

	if (!flag)
		return 0;

	if (flag & SIW_GET_CHIP_NAME) {
		offset += siw_snprintf(buf, offset,
					"chip : %s\n",
					touch_chip_name(ts));
	}

	if (flag & (SIW_GET_VERSION|SIW_GET_VER_SIMPLE)) {
		char *ver_tag = (flag & SIW_GET_VER_SIMPLE) ? "" : "version : ";
		if (fw->version_ext) {
			offset += siw_snprintf(buf, offset,
						"%s%08X(%u.%02u)\n",
						ver_tag,
						fw->version_ext,
						fw->v.version.major, fw->v.version.minor);
		} else {
			offset += siw_snprintf(buf, offset,
						"%sv%u.%02u\n",
						ver_tag,
						fw->v.version.major, fw->v.version.minor);
		}
	}

	if (flag & SIW_GET_REVISION) {
		if (chip->fw.revision == 0xFF) {
			offset += siw_snprintf(buf, offset,
						"revision : Flash Erased(0xFF)\n");
		} else {
			offset += siw_snprintf(buf, offset,
						"revision : %d\n", fw->revision);
		}
	}

	if (flag & SIW_GET_PRODUCT) {
		offset += siw_snprintf(buf, offset,
					"product id : %s\n", fw->product_id);
	}

	if (flag & SIW_GET_OPT1) {

	}

	return offset;
}

static int siw_hal_set(struct device *dev, u32 cmd, void *buf)
{
	return 0;
}

static int siw_hal_get(struct device *dev, u32 cmd, void *buf)
{
	int ret = 0;

	t_dev_dbg_base(dev, "cmd %d\n", cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = siw_hal_get_cmd_version(dev, (char *)buf, SIW_GET_ALL);
		break;

	case CMD_ATCMD_VERSION:
		ret = siw_hal_get_cmd_version(dev, (char *)buf, SIW_GET_VER_SIMPLE);
		break;

	default:
		break;
	}

	return ret;
}

u32 t_mon_dbg_mask = 0;

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/mon_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko mon_dbg_mask=<value>
 */
module_param_named(mon_dbg_mask, t_mon_dbg_mask, uint, S_IRUGO|S_IWUSR|S_IWGRP);

#define SIW_HAL_MON_TAG 		"mon: "
#define SIW_HAL_MON_TAG_ERR 	"mon(E): "
#define SIW_HAL_MON_TAG_WARN	"mon(W): "
#define SIW_HAL_MON_TAG_DBG		"mon(D): "

#if 1
#define t_mon_info(_dev, fmt, args...)		__t_dev_info(_dev, SIW_HAL_MON_TAG fmt, ##args)
#define t_mon_warn(_dev, fmt, args...)		__t_dev_warn(_dev, SIW_HAL_MON_TAG_WARN fmt, ##args)
#else
#define t_mon_info(_dev, fmt, args...)		__t_dev_none(_dev, fmt, ##args)
#define t_mon_warn(_dev, fmt, args...)		__t_dev_none(_dev, fmt, ##args)
#endif

#define t_mon_err(_dev, fmt, args...)		__t_dev_err(_dev, SIW_HAL_MON_TAG_ERR fmt, ##args)

#define t_mon_dbg(condition, _dev, fmt, args...)			\
		do {							\
			if (unlikely(t_mon_dbg_mask & (condition)))	\
				__t_dev_info(_dev, SIW_HAL_MON_TAG_DBG fmt, ##args);	\
		} while (0)

#define t_mon_dbg_base(_dev, fmt, args...)	\
		t_mon_dbg(DBG_BASE, _dev, fmt, ##args)

#define t_mon_dbg_trace(_dev, fmt, args...)	\
		t_mon_dbg(DBG_TRACE, _dev, fmt, ##args)

#if defined(__SIW_SUPPORT_MON_THREAD)
static int siw_hal_mon_handler_chk_frame(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 frame_addr = 0;
	u32 frame_s = 0;
	u32 frame_e = 0;
	u32 delay = 1;
	int cnt = 20;
	int i;
	int ret = 0;

	switch (chip->opt.t_chk_frame) {
	case 3:
		frame_addr = 0x272;
		break;
	case 2:
		if (!strncmp(fw->product_id, "LA145WF1", 8)) {
			if (fw->v.version.major || (fw->v.version.minor > 2)) {
				frame_addr = 0x271;
			}
		}
		break;
	case 1:
		frame_addr = 0x24F;
		break;
	default:
		break;
	}

	if (!frame_addr) {
		return 0;
	}

	ret = siw_hal_read_value(dev,
				frame_addr,
				(void *)&frame_s);
	if (ret < 0){
		goto out;
	}

	for (i = 0; i < cnt; i++) {
		touch_msleep(delay);

		ret = siw_hal_read_value(dev,
					frame_addr,
					(void *)&frame_e);
		if (ret < 0){
			goto out;
		}

		if (frame_e != frame_s) {
			t_mon_dbg_trace(dev, "frame ok: %d(%d), %d x %d ms\n",
				frame_e, frame_s, i, delay);
			return 0;
		}
	}

	t_mon_err(dev, "frame not changed: %d, %d x %d ms\n",
			frame_s, cnt, delay);

	ret = -ERESTART;

out:
	return ret;
}

static int siw_hal_mon_handler_chk_status(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 dbg_mask = (t_mon_dbg_mask & DBG_TRACE) ? 0 : 0x80;
	u32 ic_status;
	u32 status;
	int ret = 0;

	ret = siw_hal_reg_read(dev,
				reg->tc_ic_status,
				(void *)&ic_status, sizeof(ic_status));
	if (ret < 0){
		goto out;
	}

	ret = siw_hal_reg_read(dev,
				reg->tc_status,
				(void *)&status, sizeof(status));
	if (ret < 0){
		goto out;
	}

	status |= 0x8000;	//Valid IRQ
	ret = siw_hal_do_check_status(dev, status, ic_status, dbg_mask);
	if (ret < 0) {
		if (ret == -ERESTART) {
			goto out;
		}
		ret = 0;
	}

out:
	return ret;
}

static int siw_hal_mon_handler_chk_id(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	u32 chip_id;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->spr_chip_id,
				&chip_id);
	if (ret < 0) {
		goto out;
	}

	if (fw->chip_id_raw != chip_id) {
		ret = -ERESTART;
		goto out;
	}

out:
	return ret;
}

struct siw_mon_hanlder_op {
	unsigned int step;
	unsigned int delay;	//msec
	unsigned int retry;
	char *name;
	int (*func)(struct device *dev);
};

#define SIW_MON_HANDLER_OP_SET(_step, _delay, _retry, _name, _func)	\
	[_step] = {	\
		.step = _step,	\
		.delay = _delay,	\
		.retry = _retry,	\
		.name = _name,	\
		.func = _func,	\
	}

static const struct siw_mon_hanlder_op siw_mon_hanlder_ops[] = {
	SIW_MON_HANDLER_OP_SET(0, 10, 3, "id", siw_hal_mon_handler_chk_id),
	SIW_MON_HANDLER_OP_SET(1, 10, 3, "status", siw_hal_mon_handler_chk_status),
	SIW_MON_HANDLER_OP_SET(2, 10, 3, "frame", siw_hal_mon_handler_chk_frame),
};

static int siw_hal_mon_hanlder_do_op(struct device *dev,
				const struct siw_mon_hanlder_op *op, char *p_name)
{
	unsigned int delay = op->delay;
	unsigned int retry = op->retry;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < retry; i++) {
		ret = op->func(dev);
		if (ret >= 0) {
			t_mon_dbg_trace(dev,
				"%s : [%d] %s check done\n",
				p_name, op->step, op->name);
			break;
		}

		t_mon_err(dev,
			"%s : [%d] %s check failed(%d), %d (%d)\n",
			p_name, op->step, op->name, i, ret, op->delay);

		touch_msleep(delay);
	}

	return ret;
}

static int siw_hal_mon_handler_skip(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;

	if (siw_hal_access_not_allowed(dev, NULL, 0)) {
		return 1;
	}

	if (chip->lcd_mode != LCD_MODE_U3) {
		return 1;
	}

	if (chip->driving_mode != LCD_MODE_U3) {
		return 1;
	}

	if (!chip->status_type || !chip->report_type) {
		return 1;
	}

	if (chip->fw.invalid_pid) {
		return 1;
	}

	if (siw_hal_alive_is_active(dev)) {
		return 1;
	}

	return 0;
}

static void siw_hal_mon_handler_self_reset(struct device *dev, char *title)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	const struct siw_mon_hanlder_op *ops = siw_mon_hanlder_ops;
	unsigned int ops_num = ARRAY_SIZE(siw_mon_hanlder_ops);
	char name[32];
	int step;
	int ret = 0;

	mutex_lock(&ts->lock);

	mutex_lock(&ts->reset_lock);

	if (siw_hal_mon_handler_skip(dev)) {
		mutex_unlock(&ts->reset_lock);
		mutex_unlock(&ts->lock);
		return;
	}

	snprintf(name, sizeof(name), "%s self-reset", title);

	for (step = 0 ; step<ops_num ; step++, ops++) {
		if ((ops->step >= ops_num) ||
			(ops->name == NULL) ||
			(ops->func == NULL)) {
			break;
		}

		ret = siw_hal_mon_hanlder_do_op(dev, ops, name);
		if (ret < 0){
			break;
		}
	}

	mutex_unlock(&ts->reset_lock);

	if (ret < 0) {
		t_mon_err(dev,
			"%s : recovery begins(hw reset)\n",
			name);

		siw_hal_reset_ctrl(dev, HW_RESET_ASYNC);
	} else {
		t_mon_dbg_trace(dev,
			"%s : check ok\n",
			name);
	}

	mutex_unlock(&ts->lock);
}

static int siw_hal_mon_handler(struct device *dev, u32 opt)
{
	char *name;

	name = (opt & MON_FLAG_RST_ONLY) ? "reset cond" : "mon handler";

	t_mon_dbg_trace(dev, "%s begins\n", name);

	siw_hal_mon_handler_self_reset(dev, name);

	if (opt & MON_FLAG_RST_ONLY) {
		goto out;
	}

	/*
	 * For other controls
	 */

out:
	t_mon_dbg_trace(dev, "%s ends\n", name);

	return 0;
}
#else	/* __SIW_SUPPORT_MON_THREAD */
static int siw_hal_mon_handler(struct device *dev, u32 opt)
{
	t_mon_dbg_trace(dev, "mon handler(noop)\n");
	return 0;
}
#endif	/* __SIW_SUPPORT_MON_THREAD */

static int siw_hal_early_probe(struct device *dev)
{
	return 0;
}

static void siw_hal_tc_cmd_set_default(struct siw_touch_chip *chip)
{
	struct siw_ts *ts = chip->ts;
	int *tc_cmd_table = chip->tc_cmd_table;
	int ctrl;

	ctrl = (TC_DRIVE_CTL_DISP_U3 | TC_DRIVE_CTL_MODE_6LHB | TC_DRIVE_CTL_START);

	switch (touch_chip_type(ts)) {
	case CHIP_SW17700:
		ctrl &= ~TC_DRIVE_CTL_DISP_U3;
		ctrl |= (1<<7);
		break;
	}

	if (touch_flags(ts) & TOUCH_USE_VBLANK)
		ctrl &= ~TC_DRIVE_CTL_MODE_6LHB;

	tc_cmd_table[LCD_MODE_U0] = -1;
	tc_cmd_table[LCD_MODE_U2] = -1;
	tc_cmd_table[LCD_MODE_U2_UNBLANK] = -1;
	tc_cmd_table[LCD_MODE_U3] = ctrl;
	tc_cmd_table[LCD_MODE_U3_PARTIAL] = -1;
	tc_cmd_table[LCD_MODE_U3_QUICKCOVER] = -1;
	tc_cmd_table[LCD_MODE_STOP] = TC_DRIVE_CTL_STOP;
}

static void siw_hal_tc_cmd_set(struct siw_touch_chip *chip)
{
	int t_tc_cmd = chip->opt.t_tc_cmd;

	switch (t_tc_cmd) {
	default:
		siw_hal_tc_cmd_set_default(chip);
		break;
	}
}

static void siw_hal_show_tc_cmd_set(struct siw_touch_chip *chip)
{
	struct siw_ts *ts = chip->ts;
	struct device *dev = chip->dev;
	int *tc_cmd_table = chip->tc_cmd_table;
	char *mode_str = NULL;
	char *ext_str = NULL;
	int ctrl = 0;
	int i = 0;

	t_dev_info(dev, "[tc cmd set] (mode bit %04Xh)\n",
		ts->mode_allowed);

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

		t_dev_info(dev, " %04Xh [%-13s] %s\n",
			ctrl, mode_str, ext_str);
	}
}

static void siw_hal_chipset_option(struct siw_touch_chip *chip)
{
//	struct device *dev = chip->dev;
	struct siw_touch_chip_opt *opt = &chip->opt;
	struct siw_ts *ts = chip->ts;

	chip->drv_reset_low = 10;
	chip->drv_delay = 20;

	opt->t_sw_rst = SIW_SW_RST_TYPE_NONE;

	switch (touch_chip_type(ts)) {
	case CHIP_SW42101:
		opt->t_bus_opt = 1;
		break;

	case CHIP_SW1828 :
		opt->f_ver_ext = 1;
		opt->f_dbg_report = 1;
		opt->t_chk_frame = 1;
		break;

	case CHIP_SW42103:
		if (touch_bus_type(ts) == BUS_IF_I2C) {
			opt->f_flex_report = 1;
		}

		opt->t_boot_mode = 1;
		opt->t_sts_mask = 3;
		opt->t_sw_rst = 3;
		opt->t_chk_frame = 2;
		opt->t_rw_opt = 1;
		break;

	case CHIP_SW17700:
		if (touch_bus_type(ts) == BUS_IF_I2C) {
			opt->f_flex_report = 1;
		}
		opt->t_boot_mode = 2;
		opt->t_sts_mask = 5;

		opt->t_sw_rst = 4;
		opt->t_chk_frame = 3;
		break;
	}

	if (!chip->pwr_s_delay) {
		chip->pwr_s_delay = 1;
	}

	if (!chip->pwr_g_delay) {
		chip->pwr_g_delay = 1;
	}

	siw_hal_tc_cmd_set(chip);
}

static void siw_hal_show_chipset_option(struct siw_touch_chip *chip)
{
	struct device *dev = chip->dev;
	struct siw_touch_chip_opt *opt = &chip->opt;
	struct chip_options {
		const char *fmt;
		int value;
		int chk;
	} *options, lists[] = {
		{	" f_ver_ext       : %d\n", opt->f_ver_ext, 0	},
		{	" f_dbg_report    : %d\n", opt->f_dbg_report, 0	},
		{	" f_flex_report   : %d\n", opt->f_flex_report, 0	},
		/* */
		{	" t_boot_mode     : %d\n", opt->t_boot_mode, 0	},
		{	" t_sts_mask      : %d\n", opt->t_sts_mask, 0	},
		{	" t_sw_rst        : %d\n", opt->t_sw_rst, SIW_SW_RST_TYPE_NONE	},
		{	" t_chk_frame     : %d\n", opt->t_chk_frame, 0	},
		/* */
		{	" t_tc_cmd        : %d\n", opt->t_tc_cmd, 0	},
		/* */
		{	" t_bus_opt       : %d\n", opt->t_bus_opt, 0	},
		{	" t_rw_opt        : %d\n", opt->t_rw_opt, 0	},
		{ NULL, 0, 0	},
	};

	options = lists;

	t_dev_info(dev, "[opt summary]\n");
	while (options->fmt != NULL) {
		if (options->value != options->chk)
			t_dev_info(dev, options->fmt, options->value);

		options++;
	}

	t_dev_info(dev, " pwr_s_delay     : %d ms\n", chip->pwr_s_delay);
	t_dev_info(dev, " pwr_g_delay     : %d ms\n", chip->pwr_g_delay);

	t_dev_info(dev, " drv_reset_low   : %d ms\n", chip->drv_reset_low);
	t_dev_info(dev, " drv_delay       : %d ms\n", chip->drv_delay);

	siw_hal_show_tc_cmd_set(chip);
}

static void siw_hal_chipset_quirk_reset(struct siw_touch_chip *chip)
{
	struct siw_ts *ts = chip->ts;
	struct device *dev = chip->dev;

	if (!(touch_flags(ts) & TOUCH_SKIP_RESET_PIN)) {
		return;
	}

	if (touch_fquirks(ts)->gpio_set_reset != NULL) {
		return;
	}

	t_dev_info(dev, "hw_reset_quirk activated\n");

	chip->ops_quirk.hw_reset = siw_hal_hw_reset_quirk;
}

static void siw_hal_chipset_quirks(struct siw_touch_chip *chip)
{
	siw_hal_chipset_quirk_reset(chip);
}

static void __siw_hal_do_remove(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_power(dev, POWER_OFF);

	siw_hal_power_free(dev);
	siw_hal_free_gpios(dev);

	siw_hal_free_works(chip);
	siw_hal_free_locks(chip);

	touch_set_dev_data(ts, NULL);

	touch_kfree(dev, chip);
}

static int siw_hal_probe(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_chip *chip = NULL;
	char log_str[64] = {0, };
	int ret = 0;

	chip = touch_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		t_dev_err(dev, "failed to allocate %s data\n",
				touch_chip_name(ts));
		return -ENOMEM;
	}

	chip->dev = dev;
	chip->reg = siw_ops_reg(ts);
	chip->ts = ts;

	touch_set_dev_data(ts, chip);

	siw_hal_chipset_quirks(chip);

	siw_hal_chipset_option(chip);

	siw_hal_init_locks(chip);
	siw_hal_init_works(chip);

	siw_hal_init_gpios(dev);
	siw_hal_power_init(dev);

	siw_hal_power(dev, POWER_ON);
	siw_hal_trigger_gpio_reset(dev, 0);

	siw_hal_show_chipset_option(chip);

	if (ts->is_charger) {
		ret = siw_hal_init_charger(dev);
		goto out;
	}

	chip->driving_mode = LCD_MODE_U3;
	chip->lcd_mode = LCD_MODE_U3;

out:
	snprintf(log_str, sizeof(log_str), "%s hal probe %s%s",
		touch_chip_name(ts),
		(ret < 0) ? "failed" : "done",
		(ts->is_charger) ? " (charger)" : "");

	t_dev_dbg_base(dev, "%s\n", log_str);

	siwmon_submit_ops_step_chip_wh_name(dev, "%s", log_str, 0);

	if (ret < 0) {
		__siw_hal_do_remove(dev);
	}

	return ret;
}

static int siw_hal_remove(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_alive_free(dev);

	siw_hal_sysfs_post(dev, DRIVER_FREE);

	__siw_hal_do_remove(dev);

	t_dev_dbg_base(dev, "%s remove done\n",
				touch_chip_name(ts));

	return 0;
}

static int siw_hal_do_suspend(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);

	if (ts->is_charger)
		return -EPERM;

	siw_hal_alive_pause_set(dev, ALIVE_PAUSE_SUSPEND);

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	siw_hal_power(dev, POWER_OFF);

	return 0;
}

static int siw_hal_do_resume(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	siw_hal_power(dev, POWER_ON);

	siw_hal_trigger_gpio_reset(dev, 0);	//Double check for reset

	if (ts->is_charger) {
		siw_hal_init_charger(dev);
		return -EPERM;
	}

	siw_hal_lcd_mode(dev, LCD_MODE_U3);

	return 0;
}

static int siw_hal_suspend(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		return -EPERM;
	}

	ret = siw_hal_do_suspend(dev);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_pm(dev, "%s suspend done\n",
			touch_chip_name(ts));

	return ret;
}

static int siw_hal_resume(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int ret = 0;

	if (atomic_read(&ts->state.core) != CORE_NORMAL) {
		return -EPERM;
	}

	ret = siw_hal_do_resume(dev);
	if (ret < 0) {
		return ret;
	}

	t_dev_dbg_pm(dev, "%s resume done\n",
			touch_chip_name(ts));

	return ret;
}

static const struct siw_hal_reg siw_touch_default_reg = {
	.spr_chip_test				= SPR_CHIP_TEST,
	.spr_chip_id				= SPR_CHIP_ID,
	.spr_rst_ctl				= SPR_RST_CTL,
	.spr_boot_ctl				= SPR_BOOT_CTL,
	.spr_sram_ctl				= SPR_SRAM_CTL,
	.spr_boot_status			= SPR_BOOT_STS,
	.spr_subdisp_status			= SPR_SUBDISP_STS,
	.spr_code_offset			= SPR_CODE_OFFSET,
	.tc_ic_status				= TC_IC_STATUS,
	.tc_status					= TC_STS,
	.tc_version					= TC_VERSION,
	.tc_product_id1				= TC_PRODUCT_ID1,
	.tc_product_id2				= TC_PRODUCT_ID2,
	.tc_version_ext				= TC_VERSION_EXT,
	.info_chip_version			= INFO_CHIP_VERSION,
	.code_access_addr			= CODE_ACCESS_ADDR,
	.data_i2cbase_addr			= DATA_I2CBASE_ADDR,
	.prd_tcm_base_addr			= PRD_TCM_BASE_ADDR,
	.tc_device_ctl				= TC_DEVICE_CTL,
	.tc_interrupt_ctl			= TC_INTERRUPT_CTL,
	.tc_interrupt_status		= TC_INTERRUPT_STS,
	.tc_drive_ctl				= TC_DRIVE_CTL,
	/* */
	.tc_tsp_test_ctl			= TC_TSP_TEST_CTL,
	.tc_tsp_test_status			= TC_TSP_TEST_STS,
	.tc_tsp_test_pf_result		= TC_TSP_TEST_PF_RESULT,
	.tc_flash_dn_status			= TC_FLASH_DN_STS,
	.tc_confdn_base_addr		= TC_CONFDN_BASE_ADDR,
	.tc_flash_dn_ctl			= TC_FLASH_DN_CTL,
	.serial_data_offset			= SERIAL_DATA_OFFSET,
	/* __SIW_SUPPORT_PRD */
	.prd_serial_tcm_offset		= PRD_SERIAL_TCM_OFFSET,
	.prd_tc_mem_sel				= PRD_TC_MEM_SEL,
	.prd_tc_test_mode_ctl		= PRD_TC_TEST_MODE_CTL,
	.prd_m1_m2_raw_offset		= PRD_M1_M2_RAW_OFFSET,
	.prd_tune_result_offset		= PRD_TUNE_RESULT_OFFSET,
	.prd_open3_short_offset		= PRD_OPEN3_SHORT_OFFSET,
	.prd_ic_ait_start_reg		= PRD_IC_AIT_START_REG,
	.prd_ic_ait_data_readystatus= PRD_IC_AIT_DATA_READYSTATUS,
};

enum {
	HAL_MON_INTERVAL_DEFAULT = MON_INTERVAL_DEFAULT,
};

static const struct siw_touch_operations siw_touch_default_ops = {
	/* Register Map */
	.reg				= (void *)&siw_touch_default_reg,
	/* Functions */
	.early_probe		= siw_hal_early_probe,
	.probe				= siw_hal_probe,
	.remove				= siw_hal_remove,
	.suspend			= siw_hal_suspend,
	.resume				= siw_hal_resume,
	.init				= siw_hal_init,
	.reset				= siw_hal_reset_ctrl,
	.ic_info			= siw_hal_ic_info,
	.tc_driving			= siw_hal_tc_driving,
	.chk_status			= siw_hal_check_status,
	.irq_handler		= siw_hal_irq_handler,
	.power				= siw_hal_power,
	.upgrade			= siw_hal_upgrade,
	.set				= siw_hal_set,
	.get				= siw_hal_get,
	/* */
	.sysfs				= siw_hal_sysfs,
	/* */
	.mon_handler		= siw_hal_mon_handler,
	.mon_interval		= HAL_MON_INTERVAL_DEFAULT,
	/* */
	.prd_sysfs			= siw_hal_prd_sysfs,
};

struct siw_hal_reg *siw_hal_get_default_reg(int opt)
{
	return (struct siw_hal_reg *)&siw_touch_default_reg;
}

struct siw_touch_operations *siw_hal_get_default_ops(int opt)
{
	return (struct siw_touch_operations *)&siw_touch_default_ops;
}



/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_bus_spi.c - SiW touch bus spi driver
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
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include "siw_touch.h"
#include "siw_touch_bus.h"
#include "siw_touch_irq.h"

#if defined(CONFIG_SPI_MASTER)

#define siwmon_submit_bus_spi_read(_spi, _data, _ret)	\
		siwmon_submit_bus(&_spi->dev, "SPI_R", _data, _ret)

#define siwmon_submit_bus_spi_write(_spi, _data, _ret)	\
		siwmon_submit_bus(&_spi->dev, "SPI_W", _data, _ret)

static int __is_dma_mapped(struct spi_transfer *x)
{
	int is_dma_mapped = !!(x->tx_dma);

	is_dma_mapped &= (x->rx_buf != NULL) ? !!(x->rx_dma) : 1;

	return is_dma_mapped;
}

static void siw_touch_spi_message_init(struct spi_device *spi,
						struct spi_message *m)
{
	spi_message_init(m);
}

static void siw_touch_spi_message_add_tail(struct spi_device *spi,
						struct spi_transfer *x,
						struct spi_message *m)
{
	m->is_dma_mapped = __is_dma_mapped(x);

	spi_message_add_tail(x, m);
}

static int siw_touch_spi_sync(struct spi_device *spi,
				struct spi_message *m)
{
	return spi_sync(spi, m);
}

static int siw_touch_spi_check(struct spi_device *spi,
					u32 bits, u32 mode, u32 freq)
{
	struct device *dev = &spi->dev;

	if (bits == ~0) {
		t_dev_err(dev, "spi check: wrong spi setup: bits_per_word %d\n",
			bits);
		return -EFAULT;
	}

	if (mode == ~0) {
		t_dev_err(dev, "spi check: wrong spi setup: spi_mode %d\n",
			mode);
		return -EFAULT;
	}

	if (spi_freq_out_of_range(freq)) {
		t_dev_err(dev, "spi check: wrong spi setup: max_freq %d.%d Mhz(%d)\n",
			freq_to_mhz_unit(freq),
			freq_to_khz_top(freq),
			freq);
		return -EFAULT;
	}

	return 0;
}

static int siw_touch_spi_init(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct siw_ts *ts = to_touch_core(dev);
	u32 new_bits = touch_bits_per_word(ts);
	u32 new_mode = touch_spi_mode(ts);
	u32 new_freq = touch_max_freq(ts);
	u32 old_bits = spi->bits_per_word;
	u32 old_mode = spi->mode;
	u32 old_freq = spi->max_speed_hz;
	int ret;

	ret = siw_touch_spi_check(spi, new_bits, new_mode, new_freq);
	if (ret < 0) {
		return ret;
	}

#if !defined(__SIW_CONFIG_OF)
	spi->chip_select = touch_chip_select(ts);
#endif
	spi->bits_per_word = new_bits;
	spi->mode = new_mode;
	spi->max_speed_hz = new_freq;

	ret = spi_setup(spi);
	if (ret < 0) {
		spi->bits_per_word = old_bits;
		spi->mode = old_mode;
		spi->max_speed_hz = old_freq;

		t_dev_err(dev, "failed to perform SPI setup\n");
		return ret;
	}

	t_dev_info(dev, "spi init: %d.%d Mhz, mode %d, bpw %d, cs %d (%s)\n",
			freq_to_mhz_unit(spi->max_speed_hz),
			freq_to_khz_top(spi->max_speed_hz),
			spi->mode,
			spi->bits_per_word,
			spi->chip_select,
			dev_name(&spi->master->dev));

	return 0;
}

static int siw_touch_spi_do_read(struct spi_device *spi,
							struct touch_bus_msg *msg)
{
	struct siw_ts *ts = spi_get_drvdata(spi);
	struct spi_transfer x = {
		.cs_change = 0,
		.bits_per_word = spi->bits_per_word,
		.delay_usecs = 0,
		.speed_hz = spi->max_speed_hz,
	};
	struct spi_message m;
	int max_buf_size = touch_get_act_buf_size(ts);
	int ret = 0;

	if ((msg->rx_size > max_buf_size) ||
		(msg->tx_size > max_buf_size)) {
		t_dev_err(&spi->dev, "spi rd: buffer overflow - rx %Xh, tx %Xh\n",
			msg->rx_size, msg->tx_size);
		return -EOVERFLOW;
	}

	siw_touch_spi_message_init(spi, &m);

	x.tx_buf = msg->tx_buf;
	x.rx_buf = msg->rx_buf;
	x.tx_dma = msg->tx_dma;
	x.rx_dma = msg->rx_dma;
	x.len = msg->rx_size;

	siw_touch_spi_message_add_tail(spi, &x, &m);

	ret = siw_touch_spi_sync(spi, &m);

	siwmon_submit_bus_spi_read(spi, msg, ret);

	return ret;
}

static int siw_touch_spi_read(struct device *dev, void *msg)
{
	return siw_touch_spi_do_read(to_spi_device(dev), (struct touch_bus_msg *)msg);
}

int siw_touch_spi_do_write(struct spi_device *spi,
						struct touch_bus_msg *msg)
{
	struct siw_ts *ts = spi_get_drvdata(spi);
	struct spi_transfer x = {
		.cs_change = 0,
		.bits_per_word = spi->bits_per_word,
		.delay_usecs = 0,
		.speed_hz = spi->max_speed_hz,
	};
	struct spi_message m;
	int max_buf_size = touch_get_act_buf_size(ts);
	int ret = 0;

	if (msg->tx_size > max_buf_size) {
		t_dev_err(&spi->dev, "spi wr: buffer overflow - tx %Xh\n",
			msg->tx_size);
		return -EOVERFLOW;
	}

	siw_touch_spi_message_init(spi, &m);

	x.tx_buf = msg->tx_buf;
	x.rx_buf = msg->rx_buf;
	x.tx_dma = msg->tx_dma;
	x.rx_dma = msg->rx_dma;
	x.len = msg->tx_size;

	siw_touch_spi_message_add_tail(spi, &x, &m);

	ret = siw_touch_spi_sync(spi, &m);

	siwmon_submit_bus_spi_write(spi, msg, ret);

	return ret;
}

static int siw_touch_spi_write(struct device *dev, void *msg)
{
	return siw_touch_spi_do_write(to_spi_device(dev), (struct touch_bus_msg *)msg);
}

static struct siw_ts *siw_touch_spi_alloc(
			struct spi_device *spi,
			struct siw_touch_bus_drv *bus_drv)
{
	struct device *dev = &spi->dev;
	struct siw_ts *ts = NULL;

	ts = siw_touch_bus_ts_alloc(dev, bus_drv,
			spi, (size_t)spi, spi->irq, "spi");
	if (ts == NULL) {
		goto out;
	}

	ts->bus_init = siw_touch_spi_init;
	ts->bus_read = siw_touch_spi_read;
	ts->bus_write = siw_touch_spi_write;

out:
	return ts;
}

static void siw_touch_spi_free(struct spi_device *spi)
{
	struct device *dev = &spi->dev;

	siw_touch_bus_ts_free(dev);
}

static int siw_touch_spi_probe(struct spi_device *spi)
{
	struct siw_touch_bus_drv *bus_drv = NULL;
	struct siw_ts *ts = NULL;
	struct device *dev = &spi->dev;
	int ret = 0;

	t_dev_info_bus_parent(dev);

	bus_drv = container_of(to_spi_driver(dev->driver),
					struct siw_touch_bus_drv, bus.spi_drv);
	if (bus_drv == NULL) {
		t_dev_err(dev, "NULL bus_drv\n");
		return -EINVAL;
	}

	ts = siw_touch_spi_alloc(spi, bus_drv);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	ret = siw_touch_probe(ts);
	if (ret)
		goto out_plat;

	return 0;

out_plat:
	siw_touch_spi_free(spi);

out:
	return ret;
}

static int siw_touch_spi_remove(struct spi_device *spi)
{
	struct siw_ts *ts = to_touch_core(&spi->dev);

	siw_touch_remove(ts);

	siw_touch_spi_free(spi);

	return 0;
}

void siw_touch_spi_shutdown(struct spi_device *spi)
{
	struct siw_ts *ts = to_touch_core(&spi->dev);

	siw_touch_shutdown(ts);
}

#if defined(CONFIG_PM_SLEEP)
static int siw_touch_spi_pm_suspend(struct device *dev)
{
	int ret = 0;

	ret = siw_touch_bus_pm_suspend(dev, 0);

	return ret;
}

static int siw_touch_spi_pm_resume(struct device *dev)
{
	int ret = 0;

	ret = siw_touch_bus_pm_resume(dev, 0);

	return ret;
}

#if defined(__SIW_CONFIG_FASTBOOT)
static int siw_touch_spi_pm_freeze(struct device *dev)
{
	int ret = 0;

	ret = siw_touch_bus_pm_suspend(dev, 1);

	return ret;
}

static int siw_touch_spi_pm_thaw(struct device *dev)
{
	int ret = 0;

	ret = siw_touch_bus_pm_resume(dev, 1);

	return ret;
}
#endif

static const struct dev_pm_ops siw_touch_spi_pm_ops = {
	.suspend	= siw_touch_spi_pm_suspend,
	.resume		= siw_touch_spi_pm_resume,
#if defined(__SIW_CONFIG_FASTBOOT)
	.freeze		= siw_touch_spi_pm_freeze,
	.thaw		= siw_touch_spi_pm_thaw,
	.poweroff	= siw_touch_spi_pm_freeze,
	.restore	= siw_touch_spi_pm_thaw,
#endif
};
#define DEV_PM_OPS	(&siw_touch_spi_pm_ops)
#else	/* CONFIG_PM_SLEEP */
#define DEV_PM_OPS	NULL
#endif	/* CONFIG_PM_SLEEP */

static struct spi_device_id siw_touch_spi_id[] = {
	{ "siw,reserved", 0 },
	{ SIW_TOUCH_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, siw_touch_spi_id);

int siw_touch_spi_add_driver(void *data)
{
	struct siw_touch_chip_data *chip_data = data;
	struct siw_touch_bus_drv *bus_drv = NULL;
	struct siw_touch_pdata *pdata = NULL;
	struct spi_driver *spi_drv = NULL;
	struct spi_device_id *id = siw_touch_spi_id;
	char __chip_name[SPI_NAME_SIZE] = {0, };
	char *chip_name = __chip_name;
	char *drv_name = NULL;
	int bus_type;
	int ret = 0;

	if (chip_data == NULL) {
		t_pr_err("NULL chip data\n");
		return -EINVAL;
	}

	if (chip_data->pdata == NULL) {
		t_pr_err("NULL chip pdata\n");
		return -EINVAL;
	}

	bus_type = pdata_bus_type((struct siw_touch_pdata *)chip_data->pdata);

	bus_drv = siw_touch_bus_create_bus_drv(bus_type);
	if (bus_drv == NULL) {
		return -ENOMEM;
	}

	pdata = bus_drv->pdata;

	memcpy(pdata, chip_data->pdata, sizeof(*pdata));

	drv_name = pdata_drv_name(pdata);

	spi_drv = &bus_drv->bus.spi_drv;
	spi_drv->driver.name = drv_name;
	spi_drv->driver.owner = pdata->owner;
#if defined(__SIW_CONFIG_OF)
	spi_drv->driver.of_match_table = pdata->of_match_table;
#endif
	spi_drv->driver.pm = DEV_PM_OPS;

	spi_drv->probe = siw_touch_spi_probe;
	spi_drv->remove = siw_touch_spi_remove;
	spi_drv->shutdown = siw_touch_spi_shutdown;
	spi_drv->id_table = siw_touch_spi_id;

	/*
	 * for non-DTS case
	 * : change siw_touch_spi_id[0].name to compatible name
	 */
	memset((void *)id->name, 0, SPI_NAME_SIZE);
	if (pdata_compatible(pdata)) {
		chip_name = pdata_compatible(pdata);
	} else {
		touch_str_to_lower(chip_name, pdata_chip_name(pdata));
	}
	snprintf((char *)id->name, SPI_NAME_SIZE, "siw,%s", chip_name);
	id->driver_data = (typeof(id->driver_data))pdata;

	ret = spi_register_driver(spi_drv);
	if (ret) {
		t_pr_err("spi_register_driver[%s] failed, %d\n",
				drv_name, ret);
		goto out;
	}

	chip_data->bus_drv = bus_drv;

	return 0;

out:
	siw_touch_bus_free_bus_drv(bus_drv);

	return ret;
}

int siw_touch_spi_del_driver(void *data)
{
	struct siw_touch_chip_data *chip_data = data;
	struct siw_touch_bus_drv *bus_drv = NULL;
//	struct siw_touch_pdata *pdata = NULL;

	if (chip_data == NULL) {
		t_pr_err("NULL touch chip data\n");
		return -ENODEV;
	}

	bus_drv = (void *)chip_data->bus_drv;
	if (bus_drv == NULL) {
		return 0;
	}

	spi_unregister_driver(&bus_drv->bus.spi_drv);

	siw_touch_bus_free_bus_drv(bus_drv);

	chip_data->bus_drv = NULL;

	return 0;
}
#endif	/* CONFIG_SPI_MASTER */



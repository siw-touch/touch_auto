/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_bus_i2c.c - SiW touch bus i2c driver
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
#include <linux/i2c.h>

#include "siw_touch.h"
#include "siw_touch_bus.h"
#include "siw_touch_irq.h"

#if defined(CONFIG_I2C)

#define siwmon_submit_bus_i2c_read(_client, _data, _ret)	\
		siwmon_submit_bus(&_client->dev, "I2C_R", _data, _ret)

#define siwmon_submit_bus_i2c_write(_client, _data, _ret)	\
		siwmon_submit_bus(&_client->dev, "I2C_W", _data, _ret)


static int siw_touch_i2c_init(struct device *dev)
{
	return 0;
}

static int siw_touch_do_i2c_read(struct i2c_client *client,
							struct touch_bus_msg *msg)
{
	struct siw_ts *ts = (struct siw_ts *)i2c_get_clientdata(client);
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = msg->tx_size,
			.buf = msg->tx_buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = msg->rx_size,
			.buf = msg->rx_buf,
		},
	};
	int max_buf_size = touch_get_act_buf_size(ts);
	int ret = 0;

	if ((msg->rx_size > max_buf_size) ||
		(msg->tx_size > max_buf_size)) {
		t_dev_err(&client->dev, "i2c rd: buffer overflow - rx %Xh, tx %Xh\n",
			msg->rx_size, msg->tx_size);
		return -EOVERFLOW;
	}

	ret = i2c_transfer(client->adapter, msgs, 2);

	siwmon_submit_bus_i2c_read(client, msg, ret);

	return ret;
}

static int siw_touch_i2c_read(struct device *dev, void *msg)
{
	return siw_touch_do_i2c_read(to_i2c_client(dev), (struct touch_bus_msg *)msg);
}

int siw_touch_do_i2c_write(struct i2c_client *client,
						struct touch_bus_msg *msg)
{
	struct siw_ts *ts = (struct siw_ts *)i2c_get_clientdata(client);
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = msg->tx_size,
			.buf = msg->tx_buf,
		},
	};
	int max_buf_size = touch_get_act_buf_size(ts);
	int ret = 0;

	if (msg->tx_size > max_buf_size) {
		t_dev_err(&client->dev, "i2c wr: buffer overflow - tx %Xh\n",
			msg->tx_size);
		return -EOVERFLOW;
	}

	ret = i2c_transfer(client->adapter, msgs, 1);

	siwmon_submit_bus_i2c_write(client, msg, ret);

	return ret;
}

static int siw_touch_i2c_write(struct device *dev, void *msg)
{
	return siw_touch_do_i2c_write(to_i2c_client(dev), (struct touch_bus_msg *)msg);
}

static struct siw_ts *siw_touch_i2c_alloc(
			struct i2c_client *i2c,
			struct siw_touch_bus_drv *bus_drv)
{
	struct device *dev = &i2c->dev;
	struct siw_ts *ts = NULL;

	ts = siw_touch_bus_ts_alloc(dev, bus_drv,
			i2c, (size_t)i2c->addr, i2c->irq, "i2c");
	if (ts == NULL) {
		goto out;
	}

	ts->bus_init = siw_touch_i2c_init;
	ts->bus_read = siw_touch_i2c_read;
	ts->bus_write = siw_touch_i2c_write;

out:
	return ts;
}

static void siw_touch_i2c_free(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;

	siw_touch_bus_ts_free(dev);
}

static int siw_touch_i2c_probe(struct i2c_client *i2c,
					const struct i2c_device_id *id)
{
	struct siw_touch_bus_drv *bus_drv = NULL;
	struct siw_ts *ts = NULL;
	struct device *dev = &i2c->dev;
	int ret = 0;

	t_dev_info_bus_parent(dev);

	bus_drv = container_of(to_i2c_driver(dev->driver),
					struct siw_touch_bus_drv, bus.i2c_drv);
	if (bus_drv == NULL) {
		t_dev_err(dev, "NULL bus_drv info\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		t_dev_err(dev, "i2c func not Supported\n");
		return -EIO;
	}

//	t_dev_dbg(DBG_BUS, dev, "i2c->dev.platform_data = %p\n", dev->platform_data);

	ts = siw_touch_i2c_alloc(i2c, bus_drv);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	ret = siw_touch_probe(ts);
	if (ret)
		goto out_plat;

	return 0;

out_plat:
	siw_touch_i2c_free(i2c);

out:
	return ret;
}

static int siw_touch_i2c_remove(struct i2c_client *i2c)
{
	struct siw_ts *ts = to_touch_core(&i2c->dev);

	siw_touch_remove(ts);

	siw_touch_i2c_free(i2c);

	return 0;
}

void siw_touch_i2c_shutdown(struct i2c_client *i2c)
{
	struct siw_ts *ts = to_touch_core(&i2c->dev);

	siw_touch_shutdown(ts);
}

#if defined(CONFIG_PM_SLEEP)
static int siw_touch_i2c_pm_suspend(struct device *dev)
{
	int ret = 0;

	ret = siw_touch_bus_pm_suspend(dev, 0);

	return ret;
}

static int siw_touch_i2c_pm_resume(struct device *dev)
{
	int ret = 0;

	ret = siw_touch_bus_pm_resume(dev, 0);

	return ret;
}

#if defined(__SIW_CONFIG_FASTBOOT)
static int siw_touch_i2c_pm_freeze(struct device *dev)
{
	int ret = 0;

	ret = siw_touch_bus_pm_suspend(dev, 1);

	return ret;
}

static int siw_touch_i2c_pm_thaw(struct device *dev)
{
	int ret = 0;

	ret = siw_touch_bus_pm_resume(dev, 1);

	return ret;
}
#endif

static const struct dev_pm_ops siw_touch_i2c_pm_ops = {
	.suspend 		= siw_touch_i2c_pm_suspend,
	.resume 		= siw_touch_i2c_pm_resume,
#if defined(__SIW_CONFIG_FASTBOOT)
	.freeze			= siw_touch_i2c_pm_freeze,
	.thaw			= siw_touch_i2c_pm_thaw,
	.poweroff		= siw_touch_i2c_pm_freeze,
	.restore		= siw_touch_i2c_pm_thaw,
#endif
};
#define DEV_PM_OPS	(&siw_touch_i2c_pm_ops)
#else	/* CONFIG_PM_SLEEP */
#define DEV_PM_OPS	NULL
#endif	/* CONFIG_PM_SLEEP */

static struct i2c_device_id siw_touch_i2c_id[] = {
	{ "siw,reserved", 0 },
	{ SIW_TOUCH_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, siw_touch_i2c_id);

int siw_touch_i2c_add_driver(void *data)
{
	struct siw_touch_chip_data *chip_data = data;
	struct siw_touch_bus_drv *bus_drv = NULL;
	struct siw_touch_pdata *pdata = NULL;
	struct i2c_driver *i2c_drv = NULL;
	struct i2c_device_id *id = siw_touch_i2c_id;
	char __chip_name[I2C_NAME_SIZE] = {0, };
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

	i2c_drv = &bus_drv->bus.i2c_drv;
	i2c_drv->driver.name = drv_name;
	i2c_drv->driver.owner = pdata->owner;
#if defined(__SIW_CONFIG_OF)
	i2c_drv->driver.of_match_table = pdata->of_match_table;
#endif
	i2c_drv->driver.pm = DEV_PM_OPS;

	i2c_drv->probe = siw_touch_i2c_probe;
	i2c_drv->remove = siw_touch_i2c_remove;
	i2c_drv->shutdown = siw_touch_i2c_shutdown;
	i2c_drv->id_table = siw_touch_i2c_id;

	/*
	 * for non-DTS case
	 * : change siw_touch_i2c_id[0].name to compatible name
	 */
	memset((void *)id->name, 0, I2C_NAME_SIZE);
	if (pdata_compatible(pdata)) {
		chip_name = pdata_compatible(pdata);
	} else {
		touch_str_to_lower(chip_name, pdata_chip_name(pdata));
	}
	snprintf((char *)id->name, I2C_NAME_SIZE, "siw,%s", chip_name);
	id->driver_data = (typeof(id->driver_data))pdata;

	ret = i2c_add_driver(i2c_drv);
	if (ret) {
		t_pr_err("i2c_register_driver[%s] failed, %d\n",
				drv_name, ret);
		goto out;
	}

	chip_data->bus_drv = bus_drv;

	return 0;

out:
	siw_touch_bus_free_bus_drv(bus_drv);

	return ret;
}

int siw_touch_i2c_del_driver(void *data)
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

	i2c_del_driver(&bus_drv->bus.i2c_drv);

	siw_touch_bus_free_bus_drv(bus_drv);

	chip_data->bus_drv = NULL;

	return 0;
}
#endif	/* CONFIG_I2C */



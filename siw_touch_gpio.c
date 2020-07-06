/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * siw_touch_gpio.c - SiW touch gpio driver
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
/*#include <linux/regulator/machine.h>*/
#include <linux/regulator/consumer.h>

#include "siw_touch.h"
#include "siw_touch_gpio.h"
#include "siw_touch_sys.h"

static int siw_touch_gpio_do_init(struct device *dev,
							int pin, const char *name)
{
	int ret = 0;

	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "gpio_request: pin %d, name %s\n", pin, name);
		ret = gpio_request(pin, name);
		if (ret) {
			t_dev_err(dev, "gpio_request[%s] failed, %d",
				name, ret);
		}
	}

	return ret;
}

static void siw_touch_gpio_do_free(struct device *dev, int pin)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "gpio_free: pin %d\n", pin);
		gpio_free(pin);
	}
}

static void siw_touch_gpio_do_dir_input(struct device *dev, int pin)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) input mode", pin);
		gpio_direction_input(pin);
	}
}

static void siw_touch_gpio_do_dir_output(struct device *dev, int pin, int value)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) output mode(%d)", pin, value);
		gpio_direction_output(pin, value);
	}
}

static void siw_touch_gpio_do_set_pull(struct device *dev, int pin, int value)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) pull(%d)", pin, value);

		siw_touch_sys_gpio_set_pull(pin, value);
	}
}

static void siw_touch_gpio_do_set_value(struct device *dev, int pin, int value)
{
	if (gpio_is_valid(pin)) {
		t_dev_dbg_gpio(dev, "set pin(%d) value, %d", pin, value);

		gpio_set_value(pin, value);
	}
}

static int siw_touch_gpio_do_get_value(struct device *dev, int pin)
{
	int value = -1;

	if (gpio_is_valid(pin)) {
		value= gpio_get_value(pin);

		t_dev_dbg_gpio(dev, "get pin(%d) value, %d", pin, value);
	}

	return value;
}

int siw_touch_gpio_init(struct device *dev,
							int pin, const char *name)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->gpio_init) {
		ret = fquirks->gpio_init(dev, pin, name);
		if (ret != -EAGAIN) {
			goto out;
		}
	}

	ret = siw_touch_gpio_do_init(dev, pin, name);

out:
	return ret;
}

void siw_touch_gpio_free(struct device *dev, int pin)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret;

	if (fquirks->gpio_free) {
		ret = fquirks->gpio_free(dev, pin);
		if (ret != -EAGAIN) {
			return;
		}
	}

	siw_touch_gpio_do_free(dev, pin);
}

void siw_touch_gpio_direction_input(struct device *dev, int pin)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret;

	if (fquirks->gpio_dir_input) {
		ret = fquirks->gpio_dir_input(dev, pin);
		if (ret != -EAGAIN) {
			return;
		}
	}

	siw_touch_gpio_do_dir_input(dev, pin);
}

void siw_touch_gpio_direction_output(struct device *dev, int pin, int value)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret;

	if (fquirks->gpio_dir_output) {
		ret = fquirks->gpio_dir_output(dev, pin, value);
		if (ret != -EAGAIN) {
			return;
		}
	}

	siw_touch_gpio_do_dir_output(dev, pin, value);
}

void siw_touch_gpio_set_pull(struct device *dev, int pin, int value)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret;

	if (fquirks->gpio_set_pull) {
		ret = fquirks->gpio_set_pull(dev, pin, value);
		if (ret != -EAGAIN) {
			return;
		}
	}

	siw_touch_gpio_do_set_pull(dev, pin, value);
}

void siw_touch_gpio_set_value(struct device *dev, int pin, int value)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret;

	if (fquirks->gpio_set_value) {
		ret = fquirks->gpio_set_value(dev, pin, value);
		if (ret != -EAGAIN) {
			return;
		}
	}

	siw_touch_gpio_do_set_value(dev, pin, value);
}

int siw_touch_gpio_get_value(struct device *dev, int pin)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);

	if (fquirks->gpio_get_value) {
		return fquirks->gpio_get_value(dev, pin);
	}

	return siw_touch_gpio_do_get_value(dev, pin);
}

#if defined(__SIW_SUPPORT_PWRCTRL)
#define PWR_VDD_REG_NAME		"vdd"
#define PWR_VIO_REG_NAME		"vio"

/*
 * [DTS example 1]
 * chip_flags = <0x400>;	//TOUCH_USE_PWRCTL
 * vdd-type = <1>;
 * vdd-supply = <&ldo4_reg>;	//Indirect lookup
 * vdd-voltage = <3200000>;
 *
 * [DTS example 2]
 * chip_flags = <0x400>;
 * vdd-type = <1>;
 * vdd-name = "vdd_ldo4";		//Direct lookup for regulator-name
 * vdd-voltage = <3200000>;
 *
 */
static int __siw_touch_regulator_set(struct device *dev,
			void **reg, char *name, int voltage, int load)
{
//	struct siw_ts *ts = to_touch_core(dev);
	struct regulator *regulator = NULL;
	int n_voltages;
	int ret = 0;

	regulator = regulator_get(dev, name);	//lookup for {name}-supply
	if (IS_ERR_OR_NULL(regulator)) {
		regulator = regulator_get(NULL, name);	//lookup for {name}
		if (IS_ERR_OR_NULL(regulator)) {
			t_dev_err(dev, "power reg %s not exist\n", name);
			return -ENXIO;
		}
	}

	n_voltages = regulator_count_voltages(regulator);
	if (n_voltages <= 0) {
		t_dev_info(dev, "power reg %s has no selector, skip set\n", name);
		goto out_set;
	}

	if (voltage) {
		ret = regulator_set_voltage(regulator, voltage, voltage);
		if (ret < 0) {
			t_dev_err(dev, "power reg %s set voltage(%d) failed, %d\n",
				name, voltage, ret);
			goto out;
		}
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
	if (load) {
		ret = regulator_set_load(regulator, load);
		if (ret < 0) {
			t_dev_err(dev, "power reg %s set load(%d) failed, %d\n",
				name, load, ret);
			goto out;
		}
	}
#endif

out_set:
	if (reg) {
		*reg = regulator;
	}

	return 0;

out:
	regulator_put(regulator);

	return ret;
}

static int __siw_touch_power_do_init_reg(struct device *dev, struct touch_pwr_con *con)
{
	struct touch_pwr_cfg *cfg = NULL;
	char *name = NULL;
	void *reg = NULL;
	int voltage = 0;
	int load = 0;
	int ret = 0;

	if (con == NULL) {
		t_dev_err(dev, "power reg init con NULL\n");
		return -EINVAL;
	}

	cfg = con->cfg;

	if (cfg == NULL) {
		t_dev_err(dev, "power reg init cfg NULL\n");
		return -EINVAL;
	}

	name = con->name;
	voltage = cfg->voltage;
	load = cfg->load;

	if (name == NULL) {
		t_dev_err(dev, "power reg init cfg not configured\n");
		return -EINVAL;
	}

	ret = __siw_touch_regulator_set(dev, &reg, name, voltage, load);
	if (ret < 0) {
		return ret;
	}

	cfg->reg = reg;

	t_dev_info(dev, "power reg %s init[%d, %d]\n", name, voltage, load);

	return 0;
}

static int __siw_touch_power_do_free_reg(struct device *dev, struct touch_pwr_con *con)
{
	struct touch_pwr_cfg *cfg = NULL;
	char *name = NULL;
	void *reg = NULL;

	if (con == NULL) {
		return 0;
	}

	cfg = con->cfg;

	if (cfg == NULL) {
		return 0;
	}

	name = con->name;
	reg = cfg->reg;

	if ((name == NULL) || IS_ERR_OR_NULL(reg)) {
		return 0;
	}

	regulator_disable(reg);
	regulator_put(reg);

	cfg->reg = NULL;

	t_dev_info(dev, "power reg %s free\n", name);

	return 0;
}

static void __siw_touch_power_do_ctrl_reg(struct device *dev, int value, struct touch_pwr_con *con)
{
	struct touch_pwr_cfg *cfg = NULL;
	char *name = NULL;
	void *reg = NULL;
	int ret = 0;

	if (con == NULL) {
		return;
	}

	cfg = con->cfg;

	if (cfg == NULL) {
		return;
	}

	name = con->name;
	reg = cfg->reg;

	if (IS_ERR_OR_NULL(reg)) {
		return;
	}

	if (value) {
		ret = regulator_enable(reg);
	} else {
		ret = regulator_disable(reg);
	}

	if (ret) {
		t_dev_err(dev, "power reg %s %s failed, %d\n",
			name, (value) ? "enable" : "disable", ret);
		return;
	}

	t_dev_info(dev, "power reg %s %s\n",
		name, (regulator_is_enabled(reg)) ? "enabled" : "disabled");
}

#define PWR_VDD_PIN_NAME		"touch-vdd"
#define PWR_VIO_PIN_NAME		"touch-vio"

static int __siw_touch_power_do_init_pin(struct device *dev, struct touch_pwr_con *con)
{
	struct touch_pwr_cfg *cfg = NULL;
	char *name = NULL;
	int pin = -1;

	if (con == NULL) {
		t_dev_err(dev, "power pin init con NULL\n");
		return -EINVAL;
	}

	cfg = con->cfg;

	if (cfg == NULL) {
		t_dev_err(dev, "power pin init cfg NULL\n");
		return -EINVAL;
	}

	name = con->name;
	pin = cfg->pin;

	if (name == NULL) {
		t_dev_err(dev, "power pin init cfg not configured\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pin)) {
		t_dev_err(dev, "power pin %s invalid %d\n", name, pin);
		return -ENXIO;
	}

	gpio_request(pin, name);

	t_dev_info(dev, "power pin %s init[%d]\n", name, pin);

	return 0;
}

static int __siw_touch_power_do_free_pin(struct device *dev, struct touch_pwr_con *con)
{
	struct touch_pwr_cfg *cfg = NULL;
	char *name = NULL;
	int pin = -1;

	if (con == NULL) {
		return 0;
	}

	cfg = con->cfg;

	if (cfg == NULL) {
		return 0;
	}

	name = con->name;
	pin = cfg->pin;

	if ((name == NULL) || !gpio_is_valid(pin)) {
		return 0;
	}

	gpio_free(pin);

	t_dev_info(dev, "power pin %s free\n", name);

	return 0;
}

static void __siw_touch_power_do_ctrl_pin(struct device *dev, int value, struct touch_pwr_con *con)
{
	struct touch_pwr_cfg *cfg = NULL;
	char *name = NULL;
	int pin = -1;
	int ret = 0;

	if (con == NULL) {
		return;
	}

	cfg = con->cfg;

	if (cfg == NULL) {
		return;
	}

	name = con->name;
	pin = cfg->pin;

	if ((name == NULL) || !gpio_is_valid(pin)) {
		return;
	}

	ret = gpio_direction_output(pin, !!value);

	if (ret) {
		t_dev_err(dev, "power pin %s %s failed, %d\n",
			name, (value) ? "enable" : "disable", ret);
		return;
	}

	t_dev_info(dev, "power pin %s %s\n",
		name, (value) ? "enabled" : "disabled");
}

static int __siw_touch_power_do_init(struct device *dev, int index)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct touch_pwr_con *con = NULL;
	int ret = 0;

	if (!(touch_flags(ts) & TOUCH_USE_PWRCTRL)) {
		return 0;
	}

	switch (index) {
	case PWR_IDX_VDD:
		con = &ts->pwr_con_vdd;
		break;
	case PWR_IDX_VIO:
		con = &ts->pwr_con_vio;
		break;
	}

	if (con && con->init) {
		ret = con->init(dev, con);
	}

	return ret;
}

static int __siw_touch_power_do_free(struct device *dev, int index)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct touch_pwr_con *con = NULL;
	int ret = 0;

	if (!(touch_flags(ts) & TOUCH_USE_PWRCTRL)) {
		return 0;
	}

	switch (index) {
	case PWR_IDX_VDD:
		con = &ts->pwr_con_vdd;
		break;
	case PWR_IDX_VIO:
		con = &ts->pwr_con_vio;
		break;
	}

	if (con && con->free) {
		ret = con->free(dev, con);
	}

	return ret;
}

static void __siw_touch_power_do_ctrl(struct device *dev, int value, int index)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct touch_pwr_con *con = NULL;

	if (!(touch_flags(ts) & TOUCH_USE_PWRCTRL)) {
		return;
	}

	switch (index) {
	case PWR_IDX_VDD:
		con = &ts->pwr_con_vdd;
		break;
	case PWR_IDX_VIO:
		con = &ts->pwr_con_vio;
		break;
	}

	if (con && con->ctrl) {
		con->ctrl(dev, value, con);
	}
}

static void siw_touch_power_data(struct device *dev,
			struct touch_pwr_cfg *cfg, char *name)
{
	if (cfg->type > 0) {
		t_dev_info(dev, "power %s type = %d\n", name, cfg->type);
	}
	if (cfg->pin > 0) {
		t_dev_info(dev, "power %s pin  = %d\n", name, cfg->pin);
	}
	if (cfg->name) {
		t_dev_info(dev, "power %s name = %s\n", name, cfg->name);
	}
	if (cfg->voltage > 0) {
		t_dev_info(dev, "power %s vol  = %d\n", name, cfg->voltage);
	}
	if (cfg->load > 0) {
		t_dev_info(dev, "power %s load = %d\n", name, cfg->load);
	}
}

static void siw_touch_power_cfg(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct touch_pins *pins = &ts->pins;
	struct touch_pwr_con *con = NULL;
	struct touch_pwr_cfg *cfg = NULL;

	/*
	 * vdd setup
	 */
	cfg = &ts->pins.vdd_cfg;

	cfg->pin = pins->vdd_pin;

	siw_touch_power_data(dev, cfg, "vdd");

	con = &ts->pwr_con_vdd;

	con->cfg = cfg;

	switch (cfg->type) {
	case PWR_TYPE_REG:
		con->name = (cfg->name) ? cfg->name : PWR_VDD_REG_NAME;
		con->init = __siw_touch_power_do_init_reg;
		con->free = __siw_touch_power_do_free_reg;
		con->ctrl = __siw_touch_power_do_ctrl_reg;

		if (!cfg->voltage) {
			cfg->voltage = 3300000;
		}
		break;
	case PWR_TYPE_GPIO:
		con->name = PWR_VDD_PIN_NAME;
		con->init = __siw_touch_power_do_init_pin;
		con->free = __siw_touch_power_do_free_pin;
		con->ctrl = __siw_touch_power_do_ctrl_pin;
		break;
	}

	/*
	 * vio setup
	 */
	cfg = &ts->pins.vio_cfg;

	cfg->pin = pins->vio_pin;

	siw_touch_power_data(dev, cfg, "vio");

	con = &ts->pwr_con_vio;

	con->cfg = cfg;

	switch (cfg->type) {
	case PWR_TYPE_REG:
		con->name = (cfg->name) ? cfg->name : PWR_VIO_REG_NAME;
		con->init = __siw_touch_power_do_init_reg;
		con->free = __siw_touch_power_do_free_reg;
		con->ctrl = __siw_touch_power_do_ctrl_reg;

		if (!cfg->voltage) {
		#if defined(__SIW_PANEL_CLASS_AUTO)
			cfg->voltage = 3300000;
		#else	/* !__SIW_PANEL_CLASS_AUTO */
			cfg->voltage = 1800000;
		#endif	/* __SIW_PANEL_CLASS_AUTO */
		}
		break;
	case PWR_TYPE_GPIO:
		con->name = PWR_VIO_PIN_NAME;
		con->init = __siw_touch_power_do_init_pin;
		con->free = __siw_touch_power_do_free_pin;
		con->ctrl = __siw_touch_power_do_ctrl_pin;
		break;
	}
}

static int siw_touch_power_do_init(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int i = 0;
	int ret = 0;

	if (!(touch_flags(ts) & TOUCH_USE_PWRCTRL)) {
		return 0;
	}

	t_dev_info(dev, "power init\n");

	siw_touch_power_cfg(dev);

	for (i = PWR_IDX_MIN; i < PWR_IDX_MAX; i++) {
		ret = __siw_touch_power_do_init(dev, i);
		if (ret < 0) {
			break;
		}
	}

	if (ret < 0) {
		/* free in reverse */
		for (i = (PWR_IDX_MAX - 1); i >= PWR_IDX_MIN; i--) {
			__siw_touch_power_do_free(dev, i);
		}
	}

	return ret;
}

static int siw_touch_power_do_free(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	int i = 0;

	if (!(touch_flags(ts) & TOUCH_USE_PWRCTRL)) {
		return 0;
	}

	for (i = (PWR_IDX_MAX - 1); i >= PWR_IDX_MIN; i--) {
		__siw_touch_power_do_free(dev, i);
	}

	t_dev_info(dev, "power free\n");

	return 0;
}

static void siw_touch_power_do_vdd(struct device *dev, int value)
{
	__siw_touch_power_do_ctrl(dev, value, PWR_IDX_VDD);
}

static void siw_touch_power_do_vio(struct device *dev, int value)
{
	__siw_touch_power_do_ctrl(dev, value, PWR_IDX_VIO);
}
#else	/* __SIW_SUPPORT_PWRCTRL */
static int siw_touch_power_do_init(struct device *dev)
{
	t_dev_dbg_gpio(dev, "power init, nop ...\n");

	return 0;
}

static int siw_touch_power_do_free(struct device *dev)
{
	t_dev_dbg_gpio(dev, "power free, nop ...\n");

	return 0;
}

static void siw_touch_power_do_vdd(struct device *dev, int value)
{

}

static void siw_touch_power_do_vio(struct device *dev, int value)
{

}
#endif	/* __SIW_SUPPORT_PWRCTRL */

int siw_touch_power_init(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->power_init) {
		ret = fquirks->power_init(dev);
		if (ret != -EAGAIN) {
			goto out;
		}
	}

	ret = siw_touch_power_do_init(dev);

out:
	return ret;
}

int siw_touch_power_free(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->power_free) {
		ret = fquirks->power_free(dev);
		if (ret != -EAGAIN) {
			goto out;
		}
	}

	ret = siw_touch_power_do_free(dev);

out:
	return ret;
}


void siw_touch_power_vdd(struct device *dev, int value)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->power_vdd) {
		ret = fquirks->power_vdd(dev, value);
		if (ret != -EAGAIN) {
			return;
		}
	}

	siw_touch_power_do_vdd(dev, value);
}

void siw_touch_power_vio(struct device *dev, int value)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->power_vio) {
		ret = fquirks->power_vio(dev, value);
		if (ret != -EAGAIN) {
			return;
		}
	}

	siw_touch_power_do_vio(dev, value);
}




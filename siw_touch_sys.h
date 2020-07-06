/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * SiW touch system interface
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#ifndef __SIW_TOUCH_SYS_H
#define __SIW_TOUCH_SYS_H

extern int siw_touch_get_boot_mode(void);

extern int siw_touch_sys_gpio_set_pull(int pin, int value);

extern int siw_touch_sys_power_state(struct device *dev);
extern int siw_touch_sys_power_lock(struct device *dev, int set);

#endif	/* __SIW_TOUCH_SYS_H */


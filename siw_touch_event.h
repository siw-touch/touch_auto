/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * SiW touch event
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#ifndef __SIW_TOUCH_EVENT_H
#define __SIW_TOUCH_EVENT_H

#include <linux/input.h>
#include <linux/input/mt.h>

#define SIW_TOUCH_PHYS_NAME_SIZE	128

#if defined(CONFIG_ANDROID)
#define __SIW_CONFIG_INPUT_ANDROID
#endif

#define USE_CANCEL_EVENT_FINGER		(BIT(0) | BIT(1))

static inline int use_cancel_event_finger(u32 flag)
{
	return ((flag & USE_CANCEL_EVENT_FINGER) == USE_CANCEL_EVENT_FINGER);
}

#define __siw_input_report_key(_idev, _code, _value)	\
	do {	\
		if (t_dbg_flag & DBG_FLAG_SKIP_IEVENT) {	\
			t_dev_dbg_event(&_idev->dev, "skip input report: c %d, v %d\n", _code, _value);	\
		} else {	\
			input_report_key(_idev, _code, _value);	\
		}	\
	} while (0)

#define siw_input_report_key(_idev, _code, _value)	\
	do {	\
		__siw_input_report_key(_idev, _code, _value);	\
		siwmon_submit_evt(&_idev->dev, "EV_KEY", EV_KEY, #_code, _code, _value, 0);	\
	} while (0)

#define siw_input_report_key_name(_idev, _code, _value, _name)	\
	do {	\
		__siw_input_report_key(_idev, _code, _value);	\
		siwmon_submit_evt(&_idev->dev, "EV_KEY", EV_KEY, _name, _code, _value, 0); \
	} while (0)

#define __siw_input_report_abs(_idev, _code, _value)	\
	do {	\
		if (t_dbg_flag & DBG_FLAG_SKIP_IEVENT) {	\
			t_dev_dbg_event(&_idev->dev, "skip input report: c %d, v %d\n", _code, _value);	\
		} else {	\
			input_report_abs(_idev, _code, _value);	\
		}	\
	} while (0)

#define siw_input_report_abs(_idev, _code, _value)	\
	do {	\
		__siw_input_report_abs(_idev, _code, _value);	\
		siwmon_submit_evt(&_idev->dev, "EV_ABS", EV_ABS, #_code, _code, _value, 0);	\
	} while (0)

#define siw_input_report_abs_name(_idev, _code, _value, _name)	\
	do {	\
		__siw_input_report_abs(_idev, _code, _value);	\
		siwmon_submit_evt(&_idev->dev, "EV_ABS", EV_ABS, _name, _code, _value, 0); \
	} while (0)

#if defined(EV_KEY)
#define __EV_KEY			EV_KEY
#endif
#if defined(BTN_TOUCH)
#define __BTN_TOUCH			BTN_TOUCH
#endif
#if defined(BTN_TOOL_FINGER)
#define __BTN_TOOL_FINGER	BTN_TOOL_FINGER
#endif

static inline void siw_input_set_ev_key(struct input_dev *input)
{
#if defined(__EV_KEY)
	set_bit(__EV_KEY, input->evbit);
#endif
}

static inline void siw_input_set_btn_touch(struct input_dev *input)
{
#if defined(__BTN_TOUCH)
	set_bit(__BTN_TOUCH, input->keybit);
#endif
}

static inline void siw_input_set_btn_tool_finger(struct input_dev *input)
{
#if defined(__BTN_TOOL_FINGER)
	set_bit(__BTN_TOOL_FINGER, input->keybit);
#endif
}

static inline void siw_input_report_btn_touch(struct input_dev *input, int value)
{
#if defined(__BTN_TOUCH)
	siw_input_report_key(input, __BTN_TOUCH, value);
#endif
}

static inline void siw_input_report_btn_tool_finger(struct input_dev *input, int value)
{
#if defined(__BTN_TOOL_FINGER)
	siw_input_report_key(input, __BTN_TOOL_FINGER, value);
#endif
}

static inline void siw_input_set_prop_direct(struct input_dev *input)
{
#if defined(INPUT_PROP_DIRECT)
	set_bit(INPUT_PROP_DIRECT, input->propbit);
#endif
}

extern void siw_touch_report_event(void *ts_data);
extern void siw_touch_report_all_event(void *ts_data);
extern int siw_touch_init_input(void *ts_data);
extern void siw_touch_free_input(void *ts_data);

extern void siw_touch_event_ctrl(void *ts_data);

#endif	/* __SIW_TOUCH_EVENT_H */


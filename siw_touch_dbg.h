/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * SiW touch debug
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#ifndef __SIW_TOUCH_DBG_H
#define __SIW_TOUCH_DBG_H

#include <linux/errno.h>

#define __SIZE_POW(x)			(1<<(x))

enum _SIW_TOUCH_DBG_FLAG {
	DBG_NONE				= 0,
	DBG_BASE				= (1U << 0),
	DBG_TRACE				= (1U << 1),
	DBG_GET_DATA			= (1U << 2),
	DBG_ABS					= (1U << 3),
	//
	DBG_BUTTON				= (1U << 4),
	DBG_FW_UPGRADE			= (1U << 5),
	DBG_BUTTON_M			= (1U << 6),
	DBG_IRQ					= (1U << 7),
	//
	DBG_PM					= (1U << 8),
	DBG_JITTER				= (1U << 9),
	DBG_ACCURACY			= (1U << 10),
	DBG_BOUNCING			= (1U << 11),
	//
	DBG_GRIP				= (1U << 12),
	DBG_FILTER_RESULT		= (1U << 13),
	DBG_QUICKCOVER			= (1U << 14),
	DBG_RSVD15				= (1U << 15),
	//
	DBG_RSVD16				= (1U << 16),
	DBG_NOISE				= (1U << 17),
	DBG_WATCH				= (1U << 18),
	DBG_RSVD18				= (1U << 19),
	//
	DBG_RSVD20				= (1U << 20),
	DBG_RSVD21				= (1U << 21),
	DBG_RSVD22				= (1U << 22),
	DBG_RSVD23				= (1U << 23),
	//
	DBG_GPIO				= (1U << 24),
	DBG_RSVD25				= (1U << 25),
	DBG_NOTI				= (1U << 26),
	DBG_EVENT				= (1U << 27),
	//
	DBG_RSVD28				= (1U << 28),
	DBG_RSVD29				= (1U << 29),
	DBG_OF					= (1U << 30),
	DBG_ETC					= (1U << 31),
};

extern u32 t_pr_dbg_mask;
extern u32 t_dev_dbg_mask;

#define LOG_SIG_INFO		"(I) "
#define LOG_SIG_NOTI		"(N) "
#define LOG_SIG_WARN		"(W) "
#define LOG_SIG_ERR			"(E) "
#define LOG_SIG_TRACE		"(T) "
#define LOG_SIG_DBG			"(D) "

#define t_pr_info(fmt, args...)		pr_info(fmt, ##args)
#define t_pr_noti(fmt, args...)		pr_notice(fmt, ##args)
#define t_pr_warn(fmt, args...)		pr_warning(fmt, ##args)
#define t_pr_err(fmt, args...)		pr_err(fmt, ##args)

#define t_pr_dbg(condition, fmt, args...)			\
		do {							\
			if (unlikely(t_pr_dbg_mask & (condition)))	\
				pr_info(fmt, ##args);	\
		} while (0)


#define __t_dev_none(_dev, fmt, args...)	\
		do{	\
			if (0) dev_printk(KERN_DEBUG, _dev, fmt, ##args);	\
		} while (0)

#define __t_dev_info(_dev, fmt, args...)	dev_info(_dev, fmt, ##args)
#define __t_dev_warn(_dev, fmt, args...)	dev_warn(_dev, fmt, ##args)
#define __t_dev_err(_dev, fmt, args...)		dev_err(_dev, fmt, ##args)
#define __t_dev_trace(_dev, fmt, args...)	dev_info(_dev, fmt, ##args)
#define __t_dev_dbg(_dev, fmt, args...)		dev_info(_dev, fmt, ##args)

#define _t_dev_info(_dev, fmt, args...)		dev_info(_dev, fmt, ##args)
#define _t_dev_warn(_dev, fmt, args...)		dev_warn(_dev, LOG_SIG_WARN fmt, ##args)
#define _t_dev_err(_dev, fmt, args...)		dev_err(_dev, LOG_SIG_ERR fmt, ##args)
#define _t_dev_trace(_dev, fmt, args...)	dev_info(_dev, LOG_SIG_TRACE fmt, ##args)
#define _t_dev_dbg(_dev, fmt, args...)		dev_info(_dev, LOG_SIG_DBG fmt, ##args)

#define t_dev_info_once(_dev, fmt, args...)	\
		({						\
			static bool __dev_info_once = false;	\
			if (!__dev_info_once) {			\
				__dev_info_once = true;		\
				_t_dev_info(_dev, fmt, ##args);	\
			}					\
		})
#define t_dev_warn_once(_dev, fmt, args...)	\
		({						\
			static bool __dev_warn_once = false;	\
			if (!__dev_warn_once) { 		\
				__dev_warn_once = true; 	\
				_t_dev_warn(_dev, fmt, ##args); \
			}					\
		})

#define t_dev_trace_once(_dev, fmt, args...)	\
		({						\
			static bool __dev_trace_once = false;	\
			if (!__dev_trace_once) {		\
				__dev_trace_once = true;	\
				_t_dev_trace(_dev, fmt, ##args); \
			}					\
		})

#define t_dev_err_once(_dev, fmt, args...)	\
		({						\
			static bool __dev_err_once = false; \
			if (!__dev_err_once) {		\
				__dev_err_once = true;	\
				_t_dev_err(_dev, fmt, ##args); \
			}					\
		})

#if 1
#define t_dev_info(_dev, fmt, args...)		_t_dev_info(_dev, fmt, ##args)
#define t_dev_warn(_dev, fmt, args...)		_t_dev_warn(_dev, fmt, ##args)
#else
#define t_dev_info(_dev, fmt, args...)		__t_dev_none(_dev, fmt, ##args)
#define t_dev_warn(_dev, fmt, args...)		__t_dev_none(_dev, fmt, ##args)
#endif

#define t_dev_trace(_dev, fmt, args...)		_t_dev_trace(_dev, fmt, ##args)
#define t_dev_err(_dev, fmt, args...)		_t_dev_err(_dev, fmt, ##args)

#define t_dev_info_sel(_dev, _prt, fmt, args...)	\
		do {	\
			if (_prt)	\
				_t_dev_info(_dev, fmt, ##args);	\
		} while (0)

#define t_dev_trcf(_dev)					t_dev_trace(_dev, "[%s]\n", __func__);

#define t_dev_dbg(condition, _dev, fmt, args...)			\
		do {							\
			if (unlikely(t_dev_dbg_mask & (condition)))	\
				_t_dev_dbg(_dev, fmt, ##args);	\
		} while (0)

#define t_dev_dbg_base(_dev, fmt, args...)	\
		t_dev_dbg(DBG_BASE, _dev, fmt, ##args)

#define t_dev_dbg_trace(_dev, fmt, args...)	\
		t_dev_dbg(DBG_TRACE, _dev, fmt, ##args)

#define t_dev_dbg_getd(_dev, fmt, args...)	\
		t_dev_dbg(DBG_GET_DATA, _dev, fmt, ##args)

#define t_dev_dbg_abs(_dev, fmt, args...)	\
		t_dev_dbg(DBG_ABS, _dev, fmt, ##args)

#define t_dev_dbg_button(_dev, fmt, args...)	\
		t_dev_dbg(DBG_BUTTON, _dev, fmt, ##args)

#define t_dev_dbg_fwup(_dev, fmt, args...)	\
		t_dev_dbg(DBG_FW_UPGRADE, _dev, fmt, ##args)

#define t_dev_dbg_button_m(_dev, fmt, args...)	\
		t_dev_dbg(DBG_BUTTON_M, _dev, fmt, ##args)

#define t_dev_dbg_irq(_dev, fmt, args...)	\
		t_dev_dbg(DBG_IRQ, _dev, fmt, ##args)

#define t_dev_dbg_pm(_dev, fmt, args...)	\
		t_dev_dbg(DBG_PM, _dev, fmt, ##args)

#define t_dev_dbg_jitter(_dev, fmt, args...)	\
		t_dev_dbg(DBG_JITTER, _dev, fmt, ##args)

#define t_dev_dbg_acc(_dev, fmt, args...)	\
		t_dev_dbg(DBG_ACCURACY, _dev, fmt, ##args)

#define t_dev_dbg_bounce(_dev, fmt, args...)	\
		t_dev_dbg(DBG_BOUNCING, _dev, fmt, ##args)

#define t_dev_dbg_grip(_dev, fmt, args...)	\
		t_dev_dbg(DBG_GRIP, _dev, fmt, ##args)

#define t_dev_dbg_filter(_dev, fmt, args...)	\
		t_dev_dbg(DBG_FILTER_RESULT, _dev, fmt, ##args)

#define t_dev_dbg_qcover(_dev, fmt, args...)	\
		t_dev_dbg(DBG_QUICKCOVER, _dev, fmt, ##args)

#define t_dev_dbg_noise(_dev, fmt, args...)	\
		t_dev_dbg(DBG_NOISE, _dev, fmt, ##args)

#define t_dev_dbg_watch(_dev, fmt, args...)	\
		t_dev_dbg(DBG_WATCH, _dev, fmt, ##args)

#define t_dev_dbg_noti(_dev, fmt, args...)	\
		t_dev_dbg(DBG_NOTI, _dev, fmt, ##args)

#define t_dev_dbg_gpio(_dev, fmt, args...)	\
		t_dev_dbg(DBG_GPIO, _dev, fmt, ##args)

#define t_dev_dbg_event(_dev, fmt, args...)	\
		t_dev_dbg(DBG_EVENT, _dev, fmt, ##args)

#define t_dev_dbg_of(_dev, fmt, args...)	\
		t_dev_dbg(DBG_OF, _dev, fmt, ##args)

#define t_dev_dbg_etc(_dev, fmt, args...)	\
		t_dev_dbg(DBG_ETC, _dev, fmt, ##args)


extern u32 t_dbg_flag;

enum {
	DBG_FLAG_SKIP_IRQ				= (1<<0),
	DBG_FLAG_SKIP_IRQ_RESET			= (1<<1),
	DBG_FLAG_RSVD2					= (1<<2),
	DBG_FLAG_RSVD3					= (1<<3),
	DBG_FLAG_TEST_BUS_USE_DMA		= (1<<4),
	DBG_FLAG_SKIP_BUS_USE_DMA		= (1<<5),
	DBG_FLAG_RSVD6					= (1<<6),
	DBG_FLAG_RSVD7					= (1<<7),
	/* */
	DBG_FLAG_SKIP_IEVENT			= (1<<8),
	DBG_FLAG_SKIP_UEVENT			= (1<<9),
	DBG_FLAG_RSVD10					= (1<<10),
	DBG_FLAG_RSVD11					= (1<<11),
	DBG_FLAG_RSVD12					= (1<<12),
	DBG_FLAG_RSVD13					= (1<<13),
	DBG_FLAG_RSVD14					= (1<<14),
	DBG_FLAG_RSVD15					= (1<<15),
	/* */
	DBG_FLAG_SKIP_INIT_LATE			= (1<<16),
	DBG_FLAG_TEST_INIT_LATE			= (1<<17),
	DBG_FLAG_SKIP_INIT_LATE_WORK	= (1<<18),
	DBG_FLAG_TEST_INIT_LATE_WORK	= (1<<19),
	DBG_FLAG_RSVD20					= (1<<20),
	DBG_FLAG_RSVD21					= (1<<21),
	DBG_FLAG_RSVD22					= (1<<22),
	DBG_FLAG_RSVD23					= (1<<23),
	/* */
	DBG_FLAG_SKIP_MON_THREAD		= (1<<24),
	DBG_FLAG_TEST_MON_THREAD		= (1<<25),
	DBG_FLAG_RSVD26					= (1<<26),
	DBG_FLAG_RSVD27					= (1<<27),
	DBG_FLAG_RSVD28					= (1<<28),
	DBG_FLAG_RSVD29					= (1<<29),
	DBG_FLAG_RSVD30					= (1<<30),
	DBG_FLAG_RSVD31					= (1<<31),
};

#define ETDBOOTFAIL			((ENOMEDIUM<<3) + 0x00)
#define ETDSENTESD			((ENOMEDIUM<<3) + 0x01)
#define ETDSENTESDIRQ		((ENOMEDIUM<<3) + 0x0F)

#endif	/* __SIW_TOUCH_DBG_H */


/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * SiW touch hal driver - reg. map
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 */

#ifndef __SIW_TOUCH_HAL_REG_H
#define __SIW_TOUCH_HAL_REG_H

#include "siw_touch_cfg.h"

#define SPR_CHIP_TEST				(0x041)

#define SPR_CHIP_ID					(0x000)
#define SPR_RST_CTL					(0x006)
#define SPR_BOOT_CTL				(0x00F)
#define SPR_SRAM_CTL				(0x010)
#define SPR_BOOT_STS				(0x011)
#define SPR_SUBDISP_STS				(0x021)

#define SPR_CODE_OFFSET				(0x078)

#define TC_IC_STATUS				(0x200) 	/* siw_hal_touch_info base addr*/
#define TC_STS						(0x201)
#define TC_VERSION					(0x242)
#define TC_PRODUCT_ID1				(0x244)
#define TC_PRODUCT_ID2				(0x245)
#define TC_VERSION_EXT				(0x269)		/* YYMMDDXX format (BCD)*/

#define INFO_CHIP_VERSION			(0x27C)

#define CODE_ACCESS_ADDR			(0x300)
#define DATA_I2CBASE_ADDR			(0x301)
#define PRD_TCM_BASE_ADDR			(0x303)

#define TC_DEVICE_CTL				(0xC00)
#define TC_INTERRUPT_CTL			(0xC01)
#define TC_INTERRUPT_STS			(0xC02)
#define TC_DRIVE_CTL				(0xC03)

/* production test */
#define TC_TSP_TEST_CTL				(0xC04)
#define TC_TSP_TEST_STS				(0x265)
#define TC_TSP_TEST_PF_RESULT		(0x266)

/* Firmware control */
#define TC_FLASH_DN_STS				(0x247)
#define TC_CONFDN_BASE_ADDR			(0x2F9)
#define TC_FLASH_DN_CTL				(0xC05)

/* */
#define SERIAL_DATA_OFFSET			(0x07B)

/* __SIW_SUPPORT_PRD */
#define PRD_SERIAL_TCM_OFFSET		(0x07C)
#define PRD_TC_MEM_SEL				(0x457)
#define PRD_TC_TEST_MODE_CTL		(0xC6E)
#define PRD_M1_M2_RAW_OFFSET		(0x287)
#define PRD_TUNE_RESULT_OFFSET		(0x289)	//See 'chip_reg_quirks' in touch_lg4895.c
#define PRD_OPEN3_SHORT_OFFSET		(0x288)
#define PRD_IC_AIT_START_REG		(0xC6C) //AIT Algorithm Engine HandShake Reg
#define PRD_IC_AIT_DATA_READYSTATUS	(0xC64)

/* __SIW_SUPPORT_PRD */

struct siw_hal_reg {
	u32 spr_chip_test;
	u32 spr_chip_id;
	u32 spr_rst_ctl;
	u32 spr_boot_ctl;
	u32 spr_sram_ctl;
	u32 spr_boot_status;
	u32 spr_subdisp_status;
	u32 spr_code_offset;
	u32 tc_ic_status;
	u32 tc_status;
	u32 tc_version;
	u32 tc_product_id1;
	u32 tc_product_id2;
	u32 tc_version_ext;
	u32 info_fpc_type;
	u32 info_wfr_type;
	u32 info_chip_version;
	u32 info_cg_type;
	u32 info_lot_num;
	u32 info_serial_num;
	u32 info_date;
	u32 info_time;
	u32 code_access_addr;
	u32 data_i2cbase_addr;
	u32 prd_tcm_base_addr;
	u32 tc_device_ctl;
	u32 tc_interrupt_ctl;
	u32 tc_interrupt_status;
	u32 tc_drive_ctl;
	/* */
	u32 tc_tsp_test_ctl;
	u32 tc_tsp_test_status;
	u32 tc_tsp_test_pf_result;
	u32 tc_flash_dn_status;
	u32 tc_confdn_base_addr;
	u32 tc_flash_dn_ctl;
	u32 serial_data_offset;
	/* __SIW_SUPPORT_PRD */
	u32 prd_serial_tcm_offset;
	u32 prd_tc_mem_sel;
	u32 prd_tc_test_mode_ctl;
	u32 prd_m1_m2_raw_offset;
	u32 prd_tune_result_offset;
	u32 prd_open3_short_offset;
	u32 prd_ic_ait_start_reg;
	u32 prd_ic_ait_data_readystatus;
};

/* Reg. exchange */
struct siw_hal_reg_quirk {
	u32 old_addr;
	u32 new_addr;
};

#endif	/* __SIW_TOUCH_HAL_REG_H */



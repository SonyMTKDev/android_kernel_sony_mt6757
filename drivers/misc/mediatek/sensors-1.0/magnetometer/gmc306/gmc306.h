/*
* Copyright(C)2014 MediaTek Inc.
* Modification based on code covered by the below mentioned copyright
* and/or permission notice(S).
*/

/* akm09911.c - akm09911 compass driver
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef GMC306_H
#define GMC306_H

#include <linux/ioctl.h>

#define GMC306_I2C_NAME "gmc306"

#define GMC306_I2C_ADDRESS	0x18
#define GMC306_BUFSIZE		0x50

#define SENSOR_DATA_SIZE		9	/* Rx buffer size, i.e from ST1 to ST2 */
#define RWBUF_SIZE				16	/* Read/Write buffer size. */
#define CALIBRATION_DATA_SIZE	26

/*! \name GMC306 register address
*\anchor GMC306_REG
*Defines a register address of the GMC306.
*/
/*! @{*/
/* Device specific constant values */
#define GMC306
#ifdef GMC306

#define GMC306_REG_ST1			0x00
#define GMC306_REG_HXH			0x01
#define GMC306_REG_HXL			0x02
#define GMC306_REG_HYH			0x03
#define GMC306_REG_HYL			0x04
#define GMC306_REG_HZH			0x05
#define GMC306_REG_HZL			0x06
#define GMC306_REG_TMPS			0x07
#define GMC306_REG_ST2			0x08
#define GMC306_REG_CNTL1		0x10
#define GMC306_REG_CNTL2		0x11
#define GMC306_REG_CNTL3		0x12

#define GMC306_REG_CMPID		0x50
#define GMC306_REG_DEVID		0x51
#define GMC306_REG_INFO			0x52
/*! \name GMC306 fuse-rom address
*\anchor GMC306
*Defines a read-only address of the fuse ROM of the GMC306.
*/
#define GMC306_FUSE_ASAX		0x53
#define GMC306_FUSE_ASAY		0x54
#define GMC306_FUSE_ASAZ		0x55

/*! \name GMC306 operation mode
* \anchor GMC306_Mode
* Defines an operation mode of the GMC306.
*/
#define GMC306_MODE_SNG_MEASURE	(0xC1)
#define GMC306_MODE_SELF_TEST	(0xD0)
#define GMC306_MODE_FUSE_ACCESS	(0xDF)
#define GMC306_MODE_POWERDOWN	(0xC0)
#define GMC306_RESET_COMMAND	(0x01)
	
#define GMC306_CMPID_VALUE		0x13
#define GMC306_DEVID_VALUE		0x30
#define GMC306_RESET_DATA 		0x01
#endif


/* To avoid device dependency, convert to general name */
#define GME_I2C_NAME			"gmc306"
#define GME_MISCDEV_NAME		"gmc306_dev"
#define GME_SYSCLS_NAME			"compass"
#define GME_SYSDEV_NAME			"gmc306"
#define GME_REG_MODE			GMC306_REG_CNTL1
#define GME_REG_RESET			GMC306_REG_CNTL2
#define GME_REG_STATUS			GMC306_REG_ST1
#define GME_REG_RAW_DATA_START 	GMC306_REG_HXH

#define GME_MODE_SNG_MEASURE 	GMC306_MODE_SNG_MEASURE
#define GME_MODE_POWERDOWN 		GMC306_MODE_POWERDOWN
#define GME_MODE_SELF_TEST 		GMC306_MODE_SELF_TEST

#define GME_MEASURE_TIME_US		10000
#define GME_DRDY_IS_HIGH(x)		((x) & 0x01)
#define GME_SENSOR_INFO_SIZE	2
#define GME_SENSOR_CONF_SIZE	3
#define GME_SENSOR_DATA_SIZE	9


#define GME_YPR_DATA_SIZE		CALIBRATION_DATA_SIZE
#define GME_RWBUF_SIZE			RWBUF_SIZE
#define GME_FUSE_1ST_ADDR		GMC306_FUSE_ASAX
#define GME_RESET_DATA			GMC306_RESET_DATA

#define GME_MODE_SNG_MEASURE 	GMC306_MODE_SNG_MEASURE
#define GME_MODE_POWERDOWN 		GMC306_MODE_POWERDOWN
#define GME_MODE_SELF_TEST 		GMC306_MODE_SELF_TEST

#define GME_CMPID_VALUE 		GMC306_CMPID_VALUE
#define GME_REG_CMPID 			GMC306_REG_CMPID
#define GME_MODE_FUSE_ACCESS 	GMC306_MODE_FUSE_ACCESS
#define GME_ReadChipInfo			"GMC306"
#define GME_FUSE_ASAX 			GMC306_FUSE_ASAX



#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define FUSION_DATA_FLAG	2
#define AKM_NUM_SENSORS		3

#define ACC_DATA_READY		(1<<(ACC_DATA_FLAG))
#define MAG_DATA_READY		(1<<(MAG_DATA_FLAG))
#define FUSION_DATA_READY	(1<<(FUSION_DATA_FLAG))
/*! @}*/

/* conversion of magnetic data (for AK09911) to uT units */
/* #define CONVERT_M                   (1.0f*0.06f) */
/* conversion of orientation data to degree units */
/* #define CONVERT_O                   (1.0f/64.0f) */

#define CONVERT_M			6
#define CONVERT_M_DIV		100	/* 6/100 = CONVERT_M */
/* /#define CONVERT_O                    1 */
/* #define CONVERT_O_DIV                (64*6)           1/(64*6) = CONVERT_O */

/* /#define CONVERT_Q16                  1 */
/* #define CONVERT_Q16_DIV              65536                    1/64 = CONVERT_O */

#define CSPEC_SPI_USE			0
#define DBG_LEVEL0   0x0001	/* Critical */
#define DBG_LEVEL1   0x0002	/* Notice */
#define DBG_LEVEL2   0x0003	/* Information */
#define DBG_LEVEL3   0x0004	/* Debug */
#define DBGFLAG      DBG_LEVEL2

#define GME_AXIS_X	0
#define GME_AXIS_Y	1
#define GME_AXIS_Z	2

#define GMECS_ASA_CACULATE(x)	((x / 128) + 1)


/* IOCTLs for Msensor misc. device library */
#define MSENSOR						   0x83
/* IOCTLs for AKM library */
#define ECS_IOCTL_WRITE                 _IOW(MSENSOR, 0x0b, char*)
#define ECS_IOCTL_READ                  _IOWR(MSENSOR, 0x0c, char*)
#define ECS_IOCTL_RESET	        _IO(MSENSOR, 0x0d)	
#define ECS_IOCTL_SET_MODE              _IOW(MSENSOR, 0x0e, short)
#define ECS_IOCTL_GETDATA               _IOR(MSENSOR, 0x0f, char[SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR               _IOW(MSENSOR, 0x10, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS       _IOR(MSENSOR, 0x11, int)
#define ECS_IOCTL_GET_CLOSE_STATUS      _IOR(MSENSOR, 0x12, int)
#define ECS_IOCTL_GET_OSENSOR_STATUS	_IOR(MSENSOR, 0x13, int)
#define ECS_IOCTL_GET_DELAY             _IOR(MSENSOR, 0x14, short)
#define ECS_IOCTL_GET_PROJECT_NAME      _IOR(MSENSOR, 0x15, char[64])
#define ECS_IOCTL_GET_MATRIX            _IOR(MSENSOR, 0x16, short [4][3][3])
#define	ECS_IOCTL_GET_LAYOUT			_IOR(MSENSOR, 0x17, int[3])

#define ECS_IOCTL_GET_OUTBIT		_IOR(MSENSOR, 0x23, char)
#define ECS_IOCTL_GET_ACCEL		_IOR(MSENSOR, 0x24, short[3])

/* #define ECS_IOCTL_GET_INFO			_IOR(MSENSOR, 0x27, unsigned char[AKM_SENSOR_INFO_SIZE]) */
#define ECS_IOCTL_GET_CONF			_IOR(MSENSOR, 0x28, unsigned char[AKM_SENSOR_CONF_SIZE])
#define ECS_IOCTL_SET_YPR_09911               _IOW(MSENSOR, 0x29, int[26])
#define ECS_IOCTL_GET_DELAY_09911             _IOR(MSENSOR, 0x30, int64_t[3])
#define	ECS_IOCTL_GET_LAYOUT_09911			_IOR(MSENSOR, 0x31, char)

#ifndef DBGPRINT
#define DBGPRINT(level, format, ...) \
	((((level) != 0) && ((level) <= DBGFLAG))  \
	? (pr_debug((format), ##__VA_ARGS__)) \
	: (void)0)

#endif

struct akm09911_platform_data {
	char layout;
	char outbit;
	int gpio_DRDY;
	int gpio_RSTN;
};

/*** Limit of factory shipment test *******************************************/
#define TLIMIT_TN_REVISION_60X				""
#define TLIMIT_NO_RST_WIA1_60X				"1-3"
#ifdef GMC306
#define TLIMIT_TN_RST_WIA1_60X				"CMPID"
#define TLIMIT_LO_RST_WIA1_60X				0x13
#define TLIMIT_HI_RST_WIA1_60X				0x13
#define TLIMIT_NO_RST_WIA2_60X				"1-4"
#define TLIMIT_TN_RST_WIA2_60X				"DEVID"
#define TLIMIT_LO_RST_WIA2_60X				0x30
#define TLIMIT_HI_RST_WIA2_60X				0x30
#endif


#define TLIMIT_NO_ASAX_60X					"1-7"
#define TLIMIT_TN_ASAX_60X					"ASAX"
#define TLIMIT_LO_ASAX_60X					1
#define TLIMIT_HI_ASAX_60X					254
#define TLIMIT_NO_ASAY_60X					"1-8"
#define TLIMIT_TN_ASAY_60X					"ASAY"
#define TLIMIT_LO_ASAY_60X					1
#define TLIMIT_HI_ASAY_60X					254
#define TLIMIT_NO_ASAZ_60X					"1-9"
#define TLIMIT_TN_ASAZ_60X					"ASAZ"
#define TLIMIT_LO_ASAZ_60X					1
#define TLIMIT_HI_ASAZ_60X					254

#define TLIMIT_NO_SNG_ST1_60X				"2-3"
#define TLIMIT_TN_SNG_ST1_60X				"SNG_ST1"
#define TLIMIT_LO_SNG_ST1_60X				1
#define TLIMIT_HI_SNG_ST1_60X				1

#define TLIMIT_NO_SNG_HX_60X				"2-4"
#define TLIMIT_TN_SNG_HX_60X				"SNG_HX"
#define TLIMIT_LO_SNG_HX_60X				-8189
#define TLIMIT_HI_SNG_HX_60X				8189

#define TLIMIT_NO_SNG_HY_60X				"2-6"
#define TLIMIT_TN_SNG_HY_60X				"SNG_HY"
#define TLIMIT_LO_SNG_HY_60X				-8189
#define TLIMIT_HI_SNG_HY_60X				8189

#define TLIMIT_NO_SNG_HZ_60X				"2-8"
#define TLIMIT_TN_SNG_HZ_60X				"SNG_HZ"
#define TLIMIT_LO_SNG_HZ_60X				-8189
#define TLIMIT_HI_SNG_HZ_60X				8189

#define TLIMIT_NO_SNG_ST2_60X				"2-10"
#define TLIMIT_TN_SNG_ST2_60X				"SNG_ST2"
#define TLIMIT_LO_SNG_ST2_60X				0
#define TLIMIT_HI_SNG_ST2_60X				0

#define TLIMIT_NO_SLF_ST1_60X				"2-13"
#define TLIMIT_TN_SLF_ST1_60X				"SLF_ST1"
#define TLIMIT_LO_SLF_ST1_60X				1
#define TLIMIT_HI_SLF_ST1_60X				1
#ifdef GMC306

#define TLIMIT_NO_SLF_RVHX_60X				"2-14"
#define TLIMIT_TN_SLF_RVHX_60X				"SLF_REVSHX"
#define TLIMIT_LO_SLF_RVHX_60X				-40
#define TLIMIT_HI_SLF_RVHX_60X				40

#define TLIMIT_NO_SLF_RVHY_60X				"2-16"
#define TLIMIT_TN_SLF_RVHY_60X				"SLF_REVSHY"
#define TLIMIT_LO_SLF_RVHY_60X				-40
#define TLIMIT_HI_SLF_RVHY_60X				40

#define TLIMIT_NO_SLF_RVHZ_60X				"2-18"
#define TLIMIT_TN_SLF_RVHZ_60X				"SLF_REVSHZ"
#define TLIMIT_LO_SLF_RVHZ_60X				-320
#define TLIMIT_HI_SLF_RVHZ_60X				-80
#endif


#define TLIMIT_NO_SLF_ST2_60X				"2-20"
#define TLIMIT_TN_SLF_ST2_60X				"SLF_ST2"
#define TLIMIT_LO_SLF_ST2_60X				0
#define TLIMIT_HI_SLF_ST2_60X				0

/*** Limit of factory shipment test *******************************************/

#define TLIMIT_TN_REVISION				""
#define TLIMIT_NO_RST_WIA				"1-3"
#define TLIMIT_TN_RST_WIA				"RST_WIA"
#define TLIMIT_LO_RST_WIA				0x13
#define TLIMIT_HI_RST_WIA				0x13
#define TLIMIT_NO_RST_INFO				"1-4"
#define TLIMIT_TN_RST_INFO				"RST_INFO"
#define TLIMIT_LO_RST_INFO				0
#define TLIMIT_HI_RST_INFO				255
#define TLIMIT_NO_RST_ST1				"1-5"
#define TLIMIT_TN_RST_ST1				"RST_ST1"
#define TLIMIT_LO_RST_ST1				0
#define TLIMIT_HI_RST_ST1				0
#define TLIMIT_NO_RST_HXL				"1-6"
#define TLIMIT_TN_RST_HXL				"RST_HXL"
#define TLIMIT_LO_RST_HXL				0
#define TLIMIT_HI_RST_HXL				0
#define TLIMIT_NO_RST_HXH				"1-7"
#define TLIMIT_TN_RST_HXH				"RST_HXH"
#define TLIMIT_LO_RST_HXH				0
#define TLIMIT_HI_RST_HXH				0
#define TLIMIT_NO_RST_HYL				"1-8"
#define TLIMIT_TN_RST_HYL				"RST_HYL"
#define TLIMIT_LO_RST_HYL				0
#define TLIMIT_HI_RST_HYL				0
#define TLIMIT_NO_RST_HYH				"1-9"
#define TLIMIT_TN_RST_HYH				"RST_HYH"
#define TLIMIT_LO_RST_HYH				0
#define TLIMIT_HI_RST_HYH				0
#define TLIMIT_NO_RST_HZL				"1-10"
#define TLIMIT_TN_RST_HZL				"RST_HZL"
#define TLIMIT_LO_RST_HZL				0
#define TLIMIT_HI_RST_HZL				0
#define TLIMIT_NO_RST_HZH				"1-11"
#define TLIMIT_TN_RST_HZH				"RST_HZH"
#define TLIMIT_LO_RST_HZH				0
#define TLIMIT_HI_RST_HZH				0
#define TLIMIT_NO_RST_ST2				"1-12"
#define TLIMIT_TN_RST_ST2				"RST_ST2"
#define TLIMIT_LO_RST_ST2				0
#define TLIMIT_HI_RST_ST2				0
#define TLIMIT_NO_RST_CNTL				"1-13"
#define TLIMIT_TN_RST_CNTL				"RST_CNTL"
#define TLIMIT_LO_RST_CNTL				0
#define TLIMIT_HI_RST_CNTL				0
#define TLIMIT_NO_RST_ASTC				"1-14"
#define TLIMIT_TN_RST_ASTC				"RST_ASTC"
#define TLIMIT_LO_RST_ASTC				0
#define TLIMIT_HI_RST_ASTC				0
#define TLIMIT_NO_RST_I2CDIS			"1-15"
#define TLIMIT_TN_RST_I2CDIS			"RST_I2CDIS"
#define TLIMIT_LO_RST_I2CDIS_USEI2C		0
#define TLIMIT_HI_RST_I2CDIS_USEI2C		0
#define TLIMIT_LO_RST_I2CDIS_USESPI		1
#define TLIMIT_HI_RST_I2CDIS_USESPI		1
#define TLIMIT_NO_ASAX					"1-17"
#define TLIMIT_TN_ASAX					"ASAX"
#define TLIMIT_LO_ASAX					1
#define TLIMIT_HI_ASAX					254
#define TLIMIT_NO_ASAY					"1-18"
#define TLIMIT_TN_ASAY					"ASAY"
#define TLIMIT_LO_ASAY					1
#define TLIMIT_HI_ASAY					254
#define TLIMIT_NO_ASAZ					"1-19"
#define TLIMIT_TN_ASAZ					"ASAZ"
#define TLIMIT_LO_ASAZ					1
#define TLIMIT_HI_ASAZ					254
#define TLIMIT_NO_WR_CNTL				"1-20"
#define TLIMIT_TN_WR_CNTL				"WR_CNTL"
#define TLIMIT_LO_WR_CNTL				0x0F
#define TLIMIT_HI_WR_CNTL				0x0F

#define TLIMIT_NO_SNG_ST1				"2-3"
#define TLIMIT_TN_SNG_ST1				"SNG_ST1"
#define TLIMIT_LO_SNG_ST1				1
#define TLIMIT_HI_SNG_ST1				1

#define TLIMIT_NO_SNG_HX				"2-4"
#define TLIMIT_TN_SNG_HX				"SNG_HX"
#define TLIMIT_LO_SNG_HX				-32759
#define TLIMIT_HI_SNG_HX				32759

#define TLIMIT_NO_SNG_HY				"2-6"
#define TLIMIT_TN_SNG_HY				"SNG_HY"
#define TLIMIT_LO_SNG_HY				-32759
#define TLIMIT_HI_SNG_HY				32759

#define TLIMIT_NO_SNG_HZ				"2-8"
#define TLIMIT_TN_SNG_HZ				"SNG_HZ"
#define TLIMIT_LO_SNG_HZ				-32759
#define TLIMIT_HI_SNG_HZ				32759

#define TLIMIT_NO_SNG_ST2				"2-10"
#define TLIMIT_TN_SNG_ST2				"SNG_ST2"
#define TLIMIT_LO_SNG_ST2				0
#define TLIMIT_HI_SNG_ST2				0

#define TLIMIT_NO_SLF_ST1				"2-14"
#define TLIMIT_TN_SLF_ST1				"SLF_ST1"
#define TLIMIT_LO_SLF_ST1				1
#define TLIMIT_HI_SLF_ST1				1

#define TLIMIT_NO_SLF_RVHX				"2-15"
#define TLIMIT_TN_SLF_RVHX				"SLF_REVSHX"
#define TLIMIT_LO_SLF_RVHX				-200
#define TLIMIT_HI_SLF_RVHX				200

#define TLIMIT_NO_SLF_RVHY				"2-17"
#define TLIMIT_TN_SLF_RVHY				"SLF_REVSHY"
#define TLIMIT_LO_SLF_RVHY				-200
#define TLIMIT_HI_SLF_RVHY				200

#define TLIMIT_NO_SLF_RVHZ				"2-19"
#define TLIMIT_TN_SLF_RVHZ				"SLF_REVSHZ"
#define TLIMIT_LO_SLF_RVHZ				-3200
#define TLIMIT_HI_SLF_RVHZ				-800

#define TLIMIT_NO_SLF_ST2				"2-21"
#define TLIMIT_TN_SLF_ST2				"SLF_ST2"
#define TLIMIT_LO_SLF_ST2				0
#define TLIMIT_HI_SLF_ST2				0

#ifdef CONFIG_CUSTOM_KERNEL_MAGNETOMETER_MODULE
extern bool mag_success_Flag;
#endif

#endif

/*
* Copyright(C)2014 MediaTek Inc.
* Modification based on code covered by the below mentioned copyright
* and/or permission notice(S).
*/

/* gmc306.c - gmc306 compass driver
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

#include "cust_mag.h"
#include "gmc306.h"
#include "mag.h"

#define DEBUG 0
#define GMC306_DEV_NAME	"gmc306"
#define DRIVER_VERSION	 "1.0.1"
#define GMC306_DEBUG	1
#define GMC306_RETRY_COUNT	10
#define GMC306_DEFAULT_DELAY	100


#if GMC306_DEBUG
#define MAGN_TAG		 "[GMC306] "
#define MAGN_PR_ERR(fmt, args...)	pr_err(MAGN_TAG fmt, ##args)
//#define MAGN_LOG(fmt, args...)	pr_debug(MAGN_TAG fmt, ##args)
#define MAGN_LOG(fmt, args...)	pr_err(MAGN_TAG fmt, ##args)
#else
#define MAGN_TAG
#define MAGN_PR_ERR(fmt, args...)	do {} while (0)
#define MAGN_LOG(fmt, args...)	do {} while (0)
#endif

static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static short akmd_delay = GMC306_DEFAULT_DELAY;
static int factory_mode;
static int gmc306_init_flag;
static struct i2c_client *this_client;
static int8_t akm_device;

static uint8_t akm_fuse[3] = {0};
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id gmc306_i2c_id[] = { {GMC306_DEV_NAME, 0}, {} };

/* Maintain  cust info here */
struct mag_hw gmc306_mag_cust;
static struct mag_hw *hw = &gmc306_mag_cust;

/* For  driver get cust info */
struct mag_hw *gmc306_get_cust_mag(void)
{
	return &gmc306_mag_cust;
}

/*----------------------------------------------------------------------------*/
static int gmc306_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int gmc306_i2c_remove(struct i2c_client *client);
static int gmc306_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int gmc306_suspend(struct device *dev);
static int gmc306_resume(struct device *dev);
static int gmc306_local_init(void);
static int gmc306_remove(void);
static int gmc306_flush(void);

static struct mag_init_info gmc306_init_info = {
	.name = "gmc306",
	.init = gmc306_local_init,
	.uninit = gmc306_remove,
};


/*----------------------------------------------------------------------------*/
enum {
	GMC_FUN_DEBUG = 0x01,
	GMC_DATA_DEBUG = 0X02,
	GMC_HWM_DEBUG = 0X04,
	GMC_CTR_DEBUG = 0X08,
	GMC_I2C_DEBUG = 0x10,
} GMC_TRC;


/*----------------------------------------------------------------------------*/
struct gmc306_i2c_data {
	struct i2c_client *client;
	struct mag_hw *hw;
	atomic_t layout;
	atomic_t trace;
	struct hwmsen_convert cvt;
	bool flush;
	bool enable;
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops gmc306_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gmc306_suspend, gmc306_resume)
};
#endif

static struct i2c_driver gmc306_i2c_driver = {
	.driver = {
		   .name = GMC306_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm    = &gmc306_pm_ops,
#endif
#ifdef CONFIG_OF
		   .of_match_table = mag_of_match,
#endif
		   },
	.probe = gmc306_i2c_probe,
	.remove = gmc306_i2c_remove,
	.detect = gmc306_i2c_detect,
	.id_table = gmc306_i2c_id,
};


/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/

static DEFINE_MUTEX(gmc306_i2c_mutex);
#ifndef CONFIG_MTK_I2C_EXTENSION
static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	if (!client) {
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		MAGN_PR_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	mutex_lock(&gmc306_i2c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != 2) {
		MAGN_PR_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&gmc306_i2c_mutex);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{				/*because address also occupies one byte, the maximum length for write is 7 bytes */
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&gmc306_i2c_mutex);
	if (!client) {
		mutex_unlock(&gmc306_i2c_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&gmc306_i2c_mutex);
		MAGN_PR_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&gmc306_i2c_mutex);
		MAGN_PR_ERR("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&gmc306_i2c_mutex);
	return err;
}
#endif
static void gmc306_power(struct mag_hw *hw, unsigned int on)
{
}

static long GMI2C_RxData(char *rxData, int length)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr;

	if ((rxData == NULL) || (length < 1))
		return -EINVAL;
	addr = rxData[0];

	res = mag_i2c_read_block(client, addr, rxData, length);
	if (res < 0)
		return -1;
	return 0;
#else
	uint8_t loop_i = 0;
#if DEBUG
	int i = 0;
	struct i2c_client *client = this_client;
	struct gmc306_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];
#endif

	/* Caller should check parameter validity. */
	if ((rxData == NULL) || (length < 1))
		return -EINVAL;

	mutex_lock(&gmc306_i2c_mutex);
	for (loop_i = 0; loop_i < GMC306_RETRY_COUNT; loop_i++) {
		this_client->addr = this_client->addr & I2C_MASK_FLAG;
		this_client->addr = this_client->addr | I2C_WR_FLAG;
		if (i2c_master_send(this_client, (const char *)rxData, ((length << 0X08) | 0X01)))
			break;
		mdelay(10);
	}

	if (loop_i >= GMC306_RETRY_COUNT) {
		mutex_unlock(&gmc306_i2c_mutex);
		MAGN_PR_ERR("%s retry over %d\n", __func__, GMC306_RETRY_COUNT);
		return -EIO;
	}
	mutex_unlock(&gmc306_i2c_mutex);
#if DEBUG
	if (atomic_read(&data->trace) & GMC_I2C_DEBUG) {
		MAGN_LOG("RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for (i = 0; i < length; i++)
			MAGN_LOG(" %02x", rxData[i]);

		MAGN_LOG("\n");
	}
#endif

	return 0;
#endif
}

static long GMI2C_TxData(char *txData, int length)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr;
	u8 *buff;

	if ((txData == NULL) || (length < 2))
		return -EINVAL;
	addr = txData[0];
	buff = &txData[1];
	res = mag_i2c_write_block(client, addr, buff, (length - 1));
	if (res < 0)
		return -1;
	return 0;
#else
	uint8_t loop_i = 0;
#if DEBUG
	int i = 0;
	struct i2c_client *client = this_client;
	struct gmc306_i2c_data *data = i2c_get_clientdata(client);
#endif

	/* Caller should check parameter validity. */
	if ((txData == NULL) || (length < 2))
		return -EINVAL;
	mutex_lock(&gmc306_i2c_mutex);
	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for (loop_i = 0; loop_i < GMC306_RETRY_COUNT; loop_i++) {
		if (i2c_master_send(this_client, (const char *)txData, length) > 0)
			break;
		mdelay(10);
	}

	if (loop_i >= GMC306_RETRY_COUNT) {
		mutex_unlock(&gmc306_i2c_mutex);
		MAGN_PR_ERR("%s retry over %d\n", __func__, GMC306_RETRY_COUNT);
		return -EIO;
	}
	mutex_unlock(&gmc306_i2c_mutex);
#if DEBUG
	if (atomic_read(&data->trace) & GMC_I2C_DEBUG) {
		MAGN_LOG("TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for (i = 0; i < (length - 1); i++)
			MAGN_LOG(" %02x", txData[i + 1]);

		MAGN_LOG("\n");
	}
#endif

	return 0;
#endif
}

static long GMECS_SetMode_SngMeasure(void)
{
	char buffer[2];
	/* Set measure mode */
	buffer[0] = GME_REG_MODE;
	buffer[1] = GME_MODE_SNG_MEASURE;

	/* Set data */
	return GMI2C_TxData(buffer, 2);
}

/*static long GMECS_SetMode_SelfTest(void)
{
	char buffer[2];

	// Set measure mode 
	buffer[0] = GME_REG_MODE;
	buffer[1] = GME_MODE_SELF_TEST;

	return GMI2C_TxData(buffer, 2);
}*/

static long GMECS_SetMode_FUSEAccess(void)
{
	char buffer[2];

	/* Set measure mode */
	buffer[0] = GME_REG_MODE;
	buffer[1] = GME_MODE_FUSE_ACCESS;

	return GMI2C_TxData(buffer, 2);
}

static int GMECS_SetMode_PowerDown(void)
{
	char buffer[2];

	/* Set powerdown mode */
	buffer[0] = GME_REG_MODE;
	buffer[1] = GMC306_MODE_POWERDOWN;

	return GMI2C_TxData(buffer, 2);
}

static long GMECS_Reset(int hard)
{
	unsigned char buffer[2];
	long err = 0;

	if (hard != 0) {
		/*TODO change to board setting */
		/* gpio_set_value(akm->rstn, 0); */
		udelay(5);
		/* gpio_set_value(akm->rstn, 1); */
	} else {
		/* Set measure mode */

		buffer[0] = GME_REG_RESET;
		buffer[1] = GME_RESET_DATA;
		err = GMI2C_TxData(buffer, 2);
		if (err < 0)
			MAGN_LOG("%s: Can not set SRST bit.", __func__);
		else
			MAGN_LOG("Soft reset is done.");
	}

	/* Device will be accessible 300 us after */
	udelay(300);		/* 100 */

	return err;
}

static long GMECS_SetMode(char mode)
{	
	//long ret;
	char buffer[2];	/* Set measure mode */
	buffer[0] = GME_REG_MODE;/*checked*/
	buffer[1] = mode; 		/* Set data */	
	return GMI2C_TxData(buffer, 2);	
}

/*
static long GMECS_SetMode(char mode)
{
	long ret;

	switch (mode & 0x1F) {
	case AK09911_MODE_SNG_MEASURE:
		ret = GMECS_SetMode_SngMeasure();
		break;

	case AK09911_MODE_SELF_TEST:
	case AK8963_MODE_SELF_TEST:
		ret = GMECS_SetMode_SelfTest();
		break;

	case AK09911_MODE_FUSE_ACCESS:
	case AK8963_MODE_FUSE_ACCESS:
		ret = GMECS_SetMode_FUSEAccess();
		break;

	case AK09911_MODE_POWERDOWN:
		ret = GMECS_SetMode_PowerDown();
		break;

	default:
		MAGN_LOG("%s: Unknown mode(%d)", __func__, mode);
		return -EINVAL;
	}

	// wait at least 100us after changing mode 
	udelay(100);

	return ret;
}
*/
static int GMECS_ReadFuse(void)
{
	int ret = 0;

	ret = GMECS_SetMode_FUSEAccess();
	if (ret < 0) {
		MAGN_LOG("AKM set read fuse mode fail ret:%d\n", ret);
		return ret;
	}
	akm_fuse[0] = GME_FUSE_1ST_ADDR;
	ret = GMI2C_RxData(akm_fuse, 3);
	if (ret < 0) {
		MAGN_LOG("AKM read fuse fail ret:%d\n", ret);
		return ret;
	}
	ret = GMECS_SetMode_PowerDown();
	return ret;
}

static int GMECS_CheckDevice(void)
{
	char buffer[2];
	int ret;

	/* Set measure mode */
	buffer[0] = GME_REG_CMPID;

	/* Read data */
	ret = GMI2C_RxData(buffer, 2);
	if (ret < 0)
		return ret;

	/* Check read data */
	if (buffer[0] != GME_CMPID_VALUE)
		return -ENXIO;

	akm_device = 6;//GMC306
	ret = GMECS_ReadFuse();
	if (ret < 0) {
		MAGN_PR_ERR("GMC306 gmc306_probe: read fuse fail\n");
		return -ENXIO;
	}

	return 0;
}

static int GMECS_AxisInfoToPat(
	const uint8_t axis_order[3],
	const uint8_t axis_sign[3],
	int16_t *pat)
{
	/* check invalid input */
	if ((axis_order[0] < 0) || (axis_order[0] > 2) ||
	   (axis_order[1] < 0) || (axis_order[1] > 2) ||
	   (axis_order[2] < 0) || (axis_order[2] > 2) ||
	   (axis_sign[0] < 0) || (axis_sign[0] > 1) ||
	   (axis_sign[1] < 0) || (axis_sign[1] > 1) ||
	   (axis_sign[2] < 0) || (axis_sign[2] > 1) ||
	  ((axis_order[0] * axis_order[1] * axis_order[2]) != 0) ||
	  ((axis_order[0] + axis_order[1] + axis_order[2]) != 3)) {
		*pat = 0;
		return -1;
	}
	/* calculate pat
	 * BIT MAP
	 * [8] = sign_x
	 * [7] = sign_y
	 * [6] = sign_z
	 * [5:4] = order_x
	 * [3:2] = order_y
	 * [1:0] = order_z
	 */
	*pat = ((int16_t)axis_sign[0] << 8);
	*pat += ((int16_t)axis_sign[1] << 7);
	*pat += ((int16_t)axis_sign[2] << 6);
	*pat += ((int16_t)axis_order[0] << 4);
	*pat += ((int16_t)axis_order[1] << 2);
	*pat += ((int16_t)axis_order[2] << 0);
	return 0;
}

static int16_t GMECS_SetCert(void)
{
	struct i2c_client *client = this_client;
	struct gmc306_i2c_data *data = i2c_get_clientdata(client);
	uint8_t axis_sign[3] = {0};
	uint8_t axis_order[3] = {0};
	int16_t ret = 0;
	int i = 0;
	int16_t cert = 0x06;

	for (i = 0; i < 3; i++)
		axis_order[i] = (uint8_t)data->cvt.map[i];

	for (i = 0; i < 3; i++) {
		axis_sign[i] = (uint8_t)data->cvt.sign[i];
		if (axis_sign[i] > 0)
			axis_sign[i] = 0;
		else if (axis_sign[i] < 0)
			axis_sign[i] = 1;
	}
#if 0
	axis_order[0] = 0;
	axis_order[1] = 1;
	axis_order[2] = 2;
	axis_sign[0] = 0;
	axis_sign[1] = 0;
	axis_sign[2] = 0;
#endif
	ret = GMECS_AxisInfoToPat(axis_order, axis_sign, &cert);
	if (ret != 0)
		return 0;
	return cert;
}
/* M-sensor daemon application have set the sng mode */
static long GMECS_GetData(char *rbuf, int size)
{
	char temp;
	int loop_i, ret;
#if DEBUG
	struct i2c_client *client = this_client;
	struct gmc306_i2c_data *data = i2c_get_clientdata(client);
#endif

	if (size < SENSOR_DATA_SIZE) {
		MAGN_PR_ERR("buff size is too small %d!\n", size);
		return -1;
	}

	memset(rbuf, 0, SENSOR_DATA_SIZE);

	rbuf[0] = GME_REG_STATUS;


	for (loop_i = 0; loop_i < GMC306_RETRY_COUNT; loop_i++) {
		ret = GMI2C_RxData(rbuf, 1);
		if (ret) {
			MAGN_PR_ERR("read ST1 resigster failed!\n");
			return -1;
		}

		if ((rbuf[0] & 0x01) == 0x01)
			break;

		mdelay(2);

		rbuf[0] = GME_REG_STATUS;

	}

	if (loop_i >= GMC306_RETRY_COUNT) {
		MAGN_PR_ERR("Data read retry larger the max count!\n");
		if (factory_mode == 0)
			/* if return we can not get data at factory mode */
			return -1;
	}

	temp = rbuf[0];

	rbuf[1] = GME_REG_RAW_DATA_START;
	ret = GMI2C_RxData(&rbuf[1], SENSOR_DATA_SIZE - 1);
	if (ret < 0) {
		MAGN_PR_ERR("GMC306 gmc306_work_func: I2C failed\n");
		return -1;
	}
	rbuf[0] = temp;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int gmc306_ReadChipInfo(char *buf, int bufsize)
{
	if ((!buf) || (bufsize <= GMC306_BUFSIZE - 1))
		return -1;

	if (!this_client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "gmc306 Chip");
	return 0;
}

/*----------------------------shipment test------------------------------------------------*/
/*!
 *@return If @a testdata is in the range of between @a lolimit and @a hilimit,
 *the return value is 1, otherwise -1.
 *@param[in] testno   A pointer to a text string.
 *@param[in] testname A pointer to a text string.
 *@param[in] testdata A data to be tested.
 *@param[in] lolimit  The maximum allowable value of @a testdata.
 *@param[in] hilimit  The minimum allowable value of @a testdata.
 *@param[in,out] pf_total
 */
int GMC306_TEST_DATA(const char testno[], const char testname[], const int testdata,
	      const int lolimit, const int hilimit, int *pf_total)
{
	int pf;			/* Pass;1, Fail;-1 */

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		MAGN_LOG("--------------------------------------------------------------------\n");
		MAGN_LOG(" Test No. Test Name	Fail	Test Data	[	 Low	High]\n");
		MAGN_LOG("--------------------------------------------------------------------\n");
		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		MAGN_LOG("--------------------------------------------------------------------\n");
		if (*pf_total == 1)
			MAGN_LOG("Factory shipment test was passed.\n\n");
		else
			MAGN_LOG("Factory shipment test was failed.\n\n");

		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit))
			pf = 1;
		else
			pf = -1;

		/* display result */
		MAGN_LOG(" %7s  %-10s	 %c	%9d	[%9d	%9d]\n",
			 testno, testname, ((pf == 1) ? ('.') : ('F')), testdata, lolimit, hilimit);
	}

	/* Pass/Fail check */
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1))
			*pf_total = 1;	/* Pass */
		else
			*pf_total = -1;	/* Fail */
	}
	return pf;
}


/*!
 *Execute "Onboard Function Test" (NOT includes "START" and "END" command).
 *@retval 1 The test is passed successfully.
 *@retval -1 The test is failed.
 *@retval 0 The test is aborted by kind of system error.
 */
int FST_GMC306(void)
{
	int pf_total;		/* p/f flag for this subtest */
	char i2cData[16];
	int hdata[3];
	int asax;
	int asay;
	int asaz;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	pf_total = 1;

	/* *********************************************** */
	/* Step1 */
	/* *********************************************** */

	/* Reset device. */
	if (GMECS_Reset(0) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* Read values from WIA. */
	i2cData[0] = GME_REG_CMPID;
	if (GMI2C_RxData(i2cData, 2) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* TEST */
	GMC306_TEST_DATA(TLIMIT_NO_RST_WIA1_60X, TLIMIT_TN_RST_WIA1_60X, (int)i2cData[0],
		  TLIMIT_LO_RST_WIA1_60X, TLIMIT_HI_RST_WIA1_60X, &pf_total);
	GMC306_TEST_DATA(TLIMIT_NO_RST_WIA2_60X, TLIMIT_TN_RST_WIA2_60X, (int)i2cData[1],
		  TLIMIT_LO_RST_WIA2_60X, TLIMIT_HI_RST_WIA2_60X, &pf_total);

	/* Set to FUSE ROM access mode */
	if (GMECS_SetMode(GME_MODE_FUSE_ACCESS) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* Read values from ASAX to ASAZ */
	i2cData[0] = GME_FUSE_1ST_ADDR;
	if (GMI2C_RxData(i2cData, 3) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}
	asax = (int)i2cData[0];
	asay = (int)i2cData[1];
	asaz = (int)i2cData[2];

	/* TEST */
	GMC306_TEST_DATA(TLIMIT_NO_ASAX_60X, TLIMIT_TN_ASAX_60X, asax, TLIMIT_LO_ASAX_60X,
		  TLIMIT_HI_ASAX_60X, &pf_total);
	GMC306_TEST_DATA(TLIMIT_NO_ASAY_60X, TLIMIT_TN_ASAY_60X, asay, TLIMIT_LO_ASAY_60X,
		  TLIMIT_HI_ASAY_60X, &pf_total);
	GMC306_TEST_DATA(TLIMIT_NO_ASAZ_60X, TLIMIT_TN_ASAZ_60X, asaz, TLIMIT_LO_ASAZ_60X,
		  TLIMIT_HI_ASAZ_60X, &pf_total);

	/* Set to PowerDown mode */
	if (GMECS_SetMode(GME_MODE_POWERDOWN) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* *********************************************** */
	/* Step2 */
	/* *********************************************** */

	/* Set to SNG measurement pattern (Set CNTL register) */
	if (GMECS_SetMode(GME_MODE_SNG_MEASURE) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* Wait for DRDY pin changes to HIGH. */
	/* usleep(AKM_MEASURE_TIME_US); */
	/* Get measurement data from AK09911 */
	/* ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TEMP + ST2 */
	/* = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 + 1 = 9yte */
	/* if (AKD_GetMagneticData(i2cData) != AKD_SUCCESS) { */
	if (GMECS_GetData(i2cData, SENSOR_DATA_SIZE) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	hdata[0] = (int16_t) (i2cData[2] | (i2cData[1] << 8));
	hdata[1] = (int16_t) (i2cData[4] | (i2cData[3] << 8));
	hdata[2] = (int16_t) (i2cData[6] | (i2cData[5] << 8));

	/* TEST */
	i2cData[0] &= 0x7F;
	GMC306_TEST_DATA(TLIMIT_NO_SNG_ST1_60X, TLIMIT_TN_SNG_ST1_60X, (int)i2cData[0],
		  TLIMIT_LO_SNG_ST1_60X, TLIMIT_HI_SNG_ST1_60X, &pf_total);

	/* TEST */
	GMC306_TEST_DATA(TLIMIT_NO_SNG_HX_60X, TLIMIT_TN_SNG_HX_60X, hdata[0], TLIMIT_LO_SNG_HX_60X,
		  TLIMIT_HI_SNG_HX_60X, &pf_total);
	GMC306_TEST_DATA(TLIMIT_NO_SNG_HY_60X, TLIMIT_TN_SNG_HY_60X, hdata[1], TLIMIT_LO_SNG_HY_60X,
		  TLIMIT_HI_SNG_HY_60X, &pf_total);
	GMC306_TEST_DATA(TLIMIT_NO_SNG_HZ_60X, TLIMIT_TN_SNG_HZ_60X, hdata[2], TLIMIT_LO_SNG_HZ_60X,
		  TLIMIT_HI_SNG_HZ_60X, &pf_total);
	GMC306_TEST_DATA(TLIMIT_NO_SNG_ST2_60X, TLIMIT_TN_SNG_ST2_60X, (int)i2cData[8],
		  TLIMIT_LO_SNG_ST2_60X, TLIMIT_HI_SNG_ST2_60X, &pf_total);

	/* Set to Self-test mode (Set CNTL register) */
	if (GMECS_SetMode(GME_MODE_SELF_TEST) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* Wait for DRDY pin changes to HIGH. */
	/* usleep(AKM_MEASURE_TIME_US); */
	/* Get measurement data from AK09911 */
	/* ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TEMP + ST2 */
	/* = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 + 1 = 9byte */
	/* if (AKD_GetMagneticData(i2cData) != AKD_SUCCESS) { */
	if (GMECS_GetData(i2cData, SENSOR_DATA_SIZE) < 0) {
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
		return 0;
	}

	/* TEST */
	i2cData[0] &= 0x7F;
	GMC306_TEST_DATA(TLIMIT_NO_SLF_ST1_60X, TLIMIT_TN_SLF_ST1_60X, (int)i2cData[0],
		  TLIMIT_LO_SLF_ST1_60X, TLIMIT_HI_SLF_ST1_60X, &pf_total);

	/* hdata[0] = (int)((((uint)(i2cData[2]))<<8)+(uint)(i2cData[1])); */
	/* hdata[1] = (int)((((uint)(i2cData[4]))<<8)+(uint)(i2cData[3])); */
	/* hdata[2] = (int)((((uint)(i2cData[6]))<<8)+(uint)(i2cData[5])); */

	hdata[0] = (int16_t) (i2cData[2] | (i2cData[1] << 8));
	hdata[1] = (int16_t) (i2cData[4] | (i2cData[3] << 8));
	hdata[2] = (int16_t) (i2cData[6] | (i2cData[5] << 8));

	/* TEST */
	GMC306_TEST_DATA(TLIMIT_NO_SLF_RVHX_60X,
		  TLIMIT_TN_SLF_RVHX_60X,
		  (hdata[0]) * (asax / 128 + 1),
		  TLIMIT_LO_SLF_RVHX_60X, TLIMIT_HI_SLF_RVHX_60X, &pf_total);

	GMC306_TEST_DATA(TLIMIT_NO_SLF_RVHY_60X,
		  TLIMIT_TN_SLF_RVHY_60X,
		  (hdata[1]) * (asay / 128 + 1),
		  TLIMIT_LO_SLF_RVHY_60X, TLIMIT_HI_SLF_RVHY_60X, &pf_total);

	GMC306_TEST_DATA(TLIMIT_NO_SLF_RVHZ_60X,
		  TLIMIT_TN_SLF_RVHZ_60X,
		  (hdata[2]) * (asaz / 128 + 1),
		  TLIMIT_LO_SLF_RVHZ_60X, TLIMIT_HI_SLF_RVHZ_60X, &pf_total);

	GMC306_TEST_DATA(TLIMIT_NO_SLF_ST2_60X,
		  TLIMIT_TN_SLF_ST2_60X,
		  (int)i2cData[8], TLIMIT_LO_SLF_ST2_60X, TLIMIT_HI_SLF_ST2_60X, &pf_total);

	return pf_total;
}

/*!
 *Execute "Onboard Function Test" (includes "START" and "END" command).
 *@retval 1 The test is passed successfully.
 *@retval -1 The test is failed.
 *@retval 0 The test is aborted by kind of system error.
 */
int gmc306_FctShipmntTestProcess_Body(void)
{
	int pf_total = 1;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	GMC306_TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

	/* *********************************************** */
	/* Step 1 to 2 */
	/* *********************************************** */

	pf_total = FST_GMC306();

	/* *********************************************** */
	/* Judge Test Result */
	/* *********************************************** */
	GMC306_TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

	return pf_total;
}

static ssize_t store_shipment_test(struct device_driver *ddri, const char *buf, size_t count)
{
	/* struct i2c_client *client = this_client; */
	/* struct gmc306_i2c_data *data = i2c_get_clientdata(client); */
	/* int layout = 0; */


	return count;
}

static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;

	res = gmc306_FctShipmntTestProcess_Body();
	if (res == 1) {
		MAGN_LOG("shipment_test pass\n");
		strlcpy(result, "y", sizeof(result));
	} else if (res == -1) {
		MAGN_LOG("shipment_test fail\n");
		strlcpy(result, "n", sizeof(result));
	} else {
		MAGN_LOG("shipment_test NaN\n");
		strlcpy(result, "NaN", sizeof(result));
	}

	return sprintf(buf, "%s\n", result);
}

static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[GMC306_BUFSIZE];

	sprintf(strbuf, "gmadfs");
	return sprintf(buf, "%s", strbuf);
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[GMC306_BUFSIZE];

	gmc306_ReadChipInfo(strbuf, GMC306_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	char sensordata[SENSOR_DATA_SIZE];
	char strbuf[GMC306_BUFSIZE];

	GMECS_SetMode_SngMeasure();
	mdelay(10);
	GMECS_GetData(sensordata, SENSOR_DATA_SIZE);

	sprintf(strbuf, "%d %d %d %d %d %d %d %d %d\n", sensordata[0], sensordata[1], sensordata[2],
		sensordata[3], sensordata[4], sensordata[5], sensordata[6], sensordata[7],
		sensordata[8]);

	return sprintf(buf, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct gmc306_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		       data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0],
		       data->cvt.sign[1], data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1],
		       data->cvt.map[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct gmc306_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &layout);
	if (ret != 0) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt))
			MAGN_PR_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		else if (!hwmsen_get_convert(data->hw->direction, &data->cvt))
			MAGN_PR_ERR("invalid layout: %d, restore to %d\n", layout,
				 data->hw->direction);
		else {
			MAGN_PR_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			ret = hwmsen_get_convert(0, &data->cvt);
			if (!ret)
				MAGN_PR_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
	} else
		MAGN_PR_ERR("invalid format = '%s'\n", buf);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct gmc306_i2c_data *data = i2c_get_clientdata(client);
	ssize_t len = 0;

	if (data->hw)
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
				data->hw->i2c_num, data->hw->direction, data->hw->power_id,
				data->hw->power_vol);
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");

	len += snprintf(buf + len, PAGE_SIZE - len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct gmc306_i2c_data *obj = i2c_get_clientdata(this_client);

	if (obj == NULL) {
		MAGN_PR_ERR("gmc306_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct gmc306_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;

	if (obj == NULL) {
		MAGN_PR_ERR("gmc306_i2c_data is null!!\n");
		return 0;
	}

	if (sscanf(buf, "0x%x", &trace) == 1)
		atomic_set(&obj->trace, trace);
	else
		MAGN_PR_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *buf)
{
	ssize_t _tLength = 0;
	struct mag_hw *_ptAccelHw = hw;

	MAGN_LOG("[%s] default direction: %d\n", __func__, _ptAccelHw->direction);

	_tLength = snprintf(buf, PAGE_SIZE, "default direction = %d\n", _ptAccelHw->direction);

	return _tLength;
}

static ssize_t store_chip_orientation(struct device_driver *ddri, const char *buf, size_t tCount)
{
	int _nDirection = 0;
	int ret = 0;
	struct gmc306_i2c_data *_pt_i2c_obj = i2c_get_clientdata(this_client);

	if (_pt_i2c_obj == NULL)
		return 0;

	ret = kstrtoint(buf, 10, &_nDirection);
	if (ret != 0) {
		if (hwmsen_get_convert(_nDirection, &_pt_i2c_obj->cvt))
			MAGN_PR_ERR("ERR: fail to set direction\n");
	}

	MAGN_LOG("[%s] set direction: %d\n", __func__, _nDirection);

	return tCount;
}

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	ssize_t res = 0;
	u8 uData = GME_REG_MODE;
	struct gmc306_i2c_data *obj = i2c_get_clientdata(this_client);

	if (obj == NULL) {
		MAGN_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	ret = GMI2C_RxData(&uData, 1);
	if (ret < 0)
		MAGN_LOG("%s:%d Error.\n", __func__, __LINE__);
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", uData);
	return res;
}
static ssize_t show_regiter_map(struct device_driver *ddri, char *buf)
{
	u8  _bIndex		= 0;
	u8  _baRegMap[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x10, 0x11, 0x12, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55};
   /* u8  _baRegValue[20]; */
	ssize_t	_tLength	 = 0;
	char tmp[2] = {0};

	for (_bIndex = 0; _bIndex < sizeof(_baRegMap)/sizeof(_baRegMap[0]); _bIndex++) {
		tmp[0] = _baRegMap[_bIndex];
		GMI2C_RxData(tmp, 1);
		_tLength += snprintf((buf + _tLength), (PAGE_SIZE - _tLength), "Reg[0x%02X]: 0x%02X\n",
			_baRegMap[_bIndex], tmp[0]);
	}

	return _tLength;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon, S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(shipmenttest, S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace, S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DRIVER_ATTR(power, S_IRUGO, show_power_status, NULL);
static DRIVER_ATTR(regmap, S_IRUGO, show_regiter_map, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *gmc306_attr_list[] = {
	&driver_attr_daemon,
	&driver_attr_shipmenttest,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
	&driver_attr_orientation,
	&driver_attr_power,
	&driver_attr_regmap,
};

/*----------------------------------------------------------------------------*/
static int gmc306_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(gmc306_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, gmc306_attr_list[idx]);
		if (err) {
			MAGN_PR_ERR("driver_create_file (%s) = %d\n",
				 gmc306_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int gmc306_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(gmc306_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, gmc306_attr_list[idx]);

	return err;
}

/*----------------------------------------------------------------------------*/
static int gmc306_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gmc306_i2c_data *obj = i2c_get_clientdata(client);

	gmc306_power(obj->hw, 0);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int gmc306_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gmc306_i2c_data *obj = i2c_get_clientdata(client);

	gmc306_power(obj->hw, 1);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int gmc306_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, GMC306_DEV_NAME, sizeof(info->type));
	return 0;
}

static int gmc306_enable(int en)
{
	int value = 0;
	int err = 0;
	struct gmc306_i2c_data *f_obj = i2c_get_clientdata(this_client);

	if (f_obj == NULL)
		return -1;

	value = en;
	factory_mode = 1;
	if (value == 1) {
		f_obj->enable = true;
		err = GMECS_SetMode(GME_MODE_SNG_MEASURE);
		if (err < 0) {
			MAGN_PR_ERR("%s:GMECS_SetMode Error.\n", __func__);
			return err;
		}
	} else {
		f_obj->enable = false;
		err = GMECS_SetMode(GME_MODE_POWERDOWN);
		if (err < 0) {
			MAGN_PR_ERR("%s:GMECS_SetMode Error.\n", __func__);
			return err;
		}
	}
	if (f_obj->flush) {
		if (value == 1) {
			MAGN_LOG("will call gmc306_flush in gmc306_enable\n");
			gmc306_flush();
		} else
			f_obj->flush = false;
	}
	wake_up(&open_wq);
	return err;
}

static int gmc306_set_delay(u64 ns)
{
	int value = 0;

	value = (int)ns / 1000 / 1000;

	if (value <= 10)
		akmd_delay = 10;
	else
		akmd_delay = value;

	return 0;
}

static int gmc306_open_report_data(int open)
{
	return 0;
}

static int gmc306_coordinate_convert(int16_t *mag_data)
{
	struct i2c_client *client = this_client;
	struct gmc306_i2c_data *data = i2c_get_clientdata(client);
	int16_t temp_data[3];
	int i = 0;

	for (i = 0; i < 3; i++)
		temp_data[i] = mag_data[i];
	/* remap coordinate */
	mag_data[data->cvt.map[GME_AXIS_X]] =
		data->cvt.sign[GME_AXIS_X] * temp_data[GME_AXIS_X];
	mag_data[data->cvt.map[GME_AXIS_Y]] =
		data->cvt.sign[GME_AXIS_Y] * temp_data[GME_AXIS_Y];
	mag_data[data->cvt.map[GME_AXIS_Z]] =
		data->cvt.sign[GME_AXIS_Z] * temp_data[GME_AXIS_Z];
	return 0;
}
static int gmc306_get_data(int *x, int *y, int *z, int *status)
{
	char i2cData[SENSOR_DATA_SIZE];
	int16_t mag[3];

	GMECS_SetMode_SngMeasure();
	mdelay(10);

	GMECS_GetData(i2cData, SENSOR_DATA_SIZE);
#ifdef GMC306 // change the index 	,GME_DEVICE_GMC306 defined in Android.mk
	i2cData[1]^=i2cData[2];i2cData[2]^=i2cData[1];i2cData[1]^=i2cData[2];
	i2cData[3]^=i2cData[4];i2cData[4]^=i2cData[3];i2cData[3]^=i2cData[4];
	i2cData[5]^=i2cData[6];i2cData[6]^=i2cData[5];i2cData[5]^=i2cData[6];
	i2cData[8]<<=3;
#endif 
	memcpy(&mag,&i2cData[1],6);

	/*data[0] = (int16_t)(strbuf[1] | (strbuf[2] << 8));
	data[1] = (int16_t)(strbuf[3] | (strbuf[4] << 8));
	data[2] = (int16_t)(strbuf[5] | (strbuf[6] << 8));
*/
	gmc306_coordinate_convert(mag);

	//*x = mag[0] * CONVERT_M_DIV * GMECS_ASA_CACULATE(akm_fuse[0]);
	//*y = mag[1] * CONVERT_M_DIV * GMECS_ASA_CACULATE(akm_fuse[1]);
	//*z = mag[2] * CONVERT_M_DIV * GMECS_ASA_CACULATE(akm_fuse[2]);
	*x = ( mag[0] *  CONVERT_M_DIV * akm_fuse[0] /128 )+ ( mag[0] * CONVERT_M_DIV);
	*y = ( mag[1] *  CONVERT_M_DIV * akm_fuse[1] /128 )+ ( mag[1] * CONVERT_M_DIV);
	*z = ( mag[2] *  CONVERT_M_DIV * akm_fuse[2] /128 )+ ( mag[2] * CONVERT_M_DIV);

	*status = i2cData[0]|i2cData[8];
	return 0;
}

static int gmc306_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;

	if (value <= 10)
		akmd_delay = 10;
	else
		akmd_delay = value;

	MAGN_LOG("gmc306 mag set delay = (%d) ok.\n", value);
	return 0;
}

static int gmc306_flush(void)
{
	/*Only flush after sensor was enabled*/
	int err = 0;
	struct gmc306_i2c_data *f_obj = i2c_get_clientdata(this_client);

	if (f_obj == NULL)
		return -1;

	if (!f_obj->enable) {
		f_obj->flush = true;
		return 0;
	}
	err = mag_flush_report();
	if (err >= 0)
		f_obj->flush = false;
	return err;
}

static int gmc306_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = gmc306_enable(enabledisable == true ? 1 : 0);
	if (err) {
		MAGN_PR_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = gmc306_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MAGN_PR_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int gmc306_factory_get_data(int32_t data[3], int *status)
{
	/* get raw data */
	return  gmc306_get_data(&data[0], &data[1], &data[2], status);
}
static int gmc306_factory_get_raw_data(int32_t data[3])
{
	MAGN_LOG("do not support gmc306_factory_get_raw_data!\n");
	return 0;
}
static int gmc306_factory_enable_calibration(void)
{
	return 0;
}
static int gmc306_factory_clear_cali(void)
{
	return 0;
}
static int gmc306_factory_set_cali(int32_t data[3])
{
	return 0;
}
static int gmc306_factory_get_cali(int32_t data[3])
{
	return 0;
}
static int gmc306_factory_do_self_test(void)
{
	return 0;
}

static struct mag_factory_fops gmc306_factory_fops = {
	.enable_sensor = gmc306_factory_enable_sensor,
	.get_data = gmc306_factory_get_data,
	.get_raw_data = gmc306_factory_get_raw_data,
	.enable_calibration = gmc306_factory_enable_calibration,
	.clear_cali = gmc306_factory_clear_cali,
	.set_cali = gmc306_factory_set_cali,
	.get_cali = gmc306_factory_get_cali,
	.do_self_test = gmc306_factory_do_self_test,
};

static struct mag_factory_public gmc306_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &gmc306_factory_fops,
};

/*----------------------------------------------------------------------------*/
static int gmc306_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct i2c_client *new_client;
	struct gmc306_i2c_data *data;
	struct mag_control_path ctl = { 0 };
	struct mag_data_path mag_data = { 0 };

	MAGN_LOG("gmc306_i2c_probe\n");
	err = get_mag_dts_func(client->dev.of_node, hw);
	if (err) {
		MAGN_PR_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit;
	}

	data = kzalloc(sizeof(struct gmc306_i2c_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->hw = hw;
	/*data->hw->direction from dts is for AKMD, rang is 1-8*/
	/*now use gmc306_coordinate_convert api, so the rang is 0-7 */
	data->hw->direction--;
	err = hwmsen_get_convert(data->hw->direction, &data->cvt);
	if (err) {
		MAGN_PR_ERR("invalid direction: %d\n", data->hw->direction);
		goto exit_kfree;
	}
	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);
	/* init_waitqueue_head(&data_ready_wq); */
	init_waitqueue_head(&open_wq);
	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);
	this_client = new_client;

	/* Check connection */
	err = GMECS_CheckDevice();
	if (err < 0) {
		MAGN_PR_ERR("GMC306 gmc306_probe: check device connect error\n");
		goto exit_init_failed;
	}

	err = mag_factory_device_register(&gmc306_factory_device);
	if (err) {
		MAGN_PR_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}


	/* Register sysfs attribute */
	err = gmc306_create_attr(&(gmc306_init_info.platform_diver_addr->driver));
	if (err) {
		MAGN_PR_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.enable = gmc306_enable;
	ctl.set_delay = gmc306_set_delay;
	ctl.open_report_data = gmc306_open_report_data;
	ctl.batch = gmc306_batch;
	ctl.flush = gmc306_flush;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = data->hw->is_batch_supported;
	//strlcpy(ctl.libinfo.libname, "akl", sizeof(ctl.libinfo.libname));
	strlcpy(ctl.libinfo.libname, "gmc", sizeof(ctl.libinfo.libname));
	ctl.libinfo.layout = GMECS_SetCert();
	ctl.libinfo.deviceid = akm_device;

	err = mag_register_control_path(&ctl);
	if (err) {
		MAGN_PR_ERR("register mag control path err\n");
		goto exit_kfree;
	}

	mag_data.div = CONVERT_M_DIV;
	mag_data.get_data = gmc306_get_data;

	err = mag_register_data_path(&mag_data);
	if (err) {
		MAGN_PR_ERR("register data control path err\n");
		goto exit_kfree;
	}

	MAGN_LOG("%s: OK\n", __func__);
	gmc306_init_flag = 1;
	return 0;

exit_sysfs_create_group_failed:
exit_init_failed:
exit_misc_device_register_failed:
exit_kfree:
	kfree(data);
	data = NULL;
exit:
	MAGN_PR_ERR("%s: err = %d\n", __func__, err);
	gmc306_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int gmc306_i2c_remove(struct i2c_client *client)
{
	int err;

	err = gmc306_delete_attr(&(gmc306_init_info.platform_diver_addr->driver));
	if (err)
		MAGN_PR_ERR("gmc306_delete_attr fail: %d\n", err);

	this_client = NULL;
	i2c_unregister_device(client);
	mag_factory_device_deregister(&gmc306_factory_device);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int gmc306_remove(void)
{
	gmc306_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&gmc306_i2c_driver);
	return 0;
}

static int gmc306_local_init(void)
{
	gmc306_power(hw, 1);
	if (i2c_add_driver(&gmc306_i2c_driver)) {
		MAGN_PR_ERR("i2c_add_driver error\n");
		return -1;
	}
	if (-1 == gmc306_init_flag)
		return -1;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init gmc306_init(void)
{
	mag_driver_add(&gmc306_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit gmc306_exit(void)
{
#ifdef CONFIG_CUSTOM_KERNEL_MAGNETOMETER_MODULE
	mag_success_Flag = false;
#endif
}

/*----------------------------------------------------------------------------*/
module_init(gmc306_init);
module_exit(gmc306_exit);

MODULE_AUTHOR("MTK");
MODULE_DESCRIPTION("GMC306 compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

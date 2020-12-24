/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* device tree should be defined in flashlight-dt.h */
#ifndef TLV61310_DTNAME
#define TLV61310_DTNAME "mediatek,flashlights_tlv61310"
#endif
#ifndef TLV61310_DTNAME_I2C
#define TLV61310_DTNAME_I2C "mediatek,flashlights_tlv61310_i2c"
#endif

#define TLV61310_NAME "flashlights-tlv61310"

/* define registers */
#define TLV61310_REG_ENABLE (0x01)
#define TLV61310_MASK_ENABLE_LED1 (0x01)
#define TLV61310_MASK_ENABLE_LED2 (0x02)
#define TLV61310_DISABLE (0x00)
#define TLV61310_ENABLE_LED1 (0x01)
#define TLV61310_ENABLE_LED1_TORCH (0x09)
#define TLV61310_ENABLE_LED1_FLASH (0x0D)
#define TLV61310_ENABLE_LED2 (0x02)
#define TLV61310_ENABLE_LED2_TORCH (0x0A)
#define TLV61310_ENABLE_LED2_FLASH (0x0E)

#define TLV61310_REG_TORCH_LEVEL_LED1 (0x05)
#define TLV61310_REG_FLASH_LEVEL_LED1 (0x03)
#define TLV61310_REG_TORCH_LEVEL_LED2 (0x06)
#define TLV61310_REG_FLASH_LEVEL_LED2 (0x04)

#define TLV61310_REG_TIMING_CONF (0x08)
#define TLV61310_TORCH_RAMP_TIME (0x10)
#define TLV61310_FLASH_TIMEOUT   (0x0F)

/* define channel, level */
#define TLV61310_CHANNEL_NUM 2
#define TLV61310_CHANNEL_CH1 0
#define TLV61310_CHANNEL_CH2 1

#define TLV61310_LEVEL_NUM 26
#define TLV61310_LEVEL_TORCH 7

#define TLV61310_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(tlv61310_mutex);
static struct work_struct tlv61310_work_ch1;
static struct work_struct tlv61310_work_ch2;

/* define pinctrl */
#define TLV61310_PINCTRL_PIN_HWEN 0
#define TLV61310_PINCTRL_PINSTATE_LOW 0
#define TLV61310_PINCTRL_PINSTATE_HIGH 1
#define TLV61310_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define TLV61310_PINCTRL_STATE_HWEN_LOW  "hwen_low"
static struct pinctrl *tlv61310_pinctrl;
static struct pinctrl_state *tlv61310_hwen_high;
static struct pinctrl_state *tlv61310_hwen_low;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *tlv61310_i2c_client;

/* platform data */
struct tlv61310_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* tlv61310 chip data */
struct tlv61310_chip_data {
	struct i2c_client *client;
	struct tlv61310_platform_data *pdata;
	struct mutex lock;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int tlv61310_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	tlv61310_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(tlv61310_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(tlv61310_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	tlv61310_hwen_high = pinctrl_lookup_state(tlv61310_pinctrl, TLV61310_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(tlv61310_hwen_high)) {
		pr_err("Failed to init (%s)\n", TLV61310_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(tlv61310_hwen_high);
	}
	tlv61310_hwen_low = pinctrl_lookup_state(tlv61310_pinctrl, TLV61310_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(tlv61310_hwen_low)) {
		pr_err("Failed to init (%s)\n", TLV61310_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(tlv61310_hwen_low);
	}

	return ret;
}

static int tlv61310_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(tlv61310_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case TLV61310_PINCTRL_PIN_HWEN:
		if (state == TLV61310_PINCTRL_PINSTATE_LOW && !IS_ERR(tlv61310_hwen_low))
			pinctrl_select_state(tlv61310_pinctrl, tlv61310_hwen_low);
		else if (state == TLV61310_PINCTRL_PINSTATE_HIGH && !IS_ERR(tlv61310_hwen_high))
			pinctrl_select_state(tlv61310_pinctrl, tlv61310_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_debug("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * tlv61310 operations
 *****************************************************************************/
static const int tlv61310_current[TLV61310_LEVEL_NUM] = {
	 22,  46,  70,  93,  116, 140, 163, 198, 245, 304,
	351, 398, 445, 503,  550, 597, 656, 703, 750, 796,
	855, 902, 949, 996, 1054, 1101
};

static const unsigned char tlv61310_torch_level[TLV61310_LEVEL_NUM] = {
	0x11, 0x23, 0x35, 0x47, 0x59, 0x6A, 0x7C, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char tlv61310_flash_level[TLV61310_LEVEL_NUM] = {
	0x03, 0x08, 0x0C, 0x10, 0x14, 0x19, 0x1D, 0x21, 0x25, 0x2A,
	0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43, 0x48, 0x4C, 0x50, 0x54,
	0x59, 0x5D, 0x61, 0x65, 0x6A, 0x6E
};

static unsigned char tlv61310_reg_enable;
static int tlv61310_level_ch1 = -1;
static int tlv61310_level_ch2 = -1;

static int tlv61310_is_torch(int level)
{
	if (level >= TLV61310_LEVEL_TORCH)
		return -1;

	return 0;
}

static int tlv61310_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= TLV61310_LEVEL_NUM)
		level = TLV61310_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int tlv61310_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct tlv61310_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* flashlight enable function */
static int tlv61310_enable_ch1(void)
{
	unsigned char reg, val;

	reg = TLV61310_REG_ENABLE;
	if (!tlv61310_is_torch(tlv61310_level_ch1)) {
		/* torch mode */
		tlv61310_reg_enable |= TLV61310_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		tlv61310_reg_enable |= TLV61310_ENABLE_LED1_FLASH;
	}
	val = tlv61310_reg_enable;

	return tlv61310_write_reg(tlv61310_i2c_client, reg, val);
}

static int tlv61310_enable_ch2(void)
{
	unsigned char reg, val;

	reg = TLV61310_REG_ENABLE;
	if (!tlv61310_is_torch(tlv61310_level_ch2)) {
		/* torch mode */
		tlv61310_reg_enable |= TLV61310_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		tlv61310_reg_enable |= TLV61310_ENABLE_LED2_FLASH;
	}
	val = tlv61310_reg_enable;

	return tlv61310_write_reg(tlv61310_i2c_client, reg, val);
}

static int tlv61310_enable(int channel)
{
	if (channel == TLV61310_CHANNEL_CH1)
		tlv61310_enable_ch1();
	else if (channel == TLV61310_CHANNEL_CH2)
		tlv61310_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int tlv61310_disable_ch1(void)
{
	unsigned char reg, val;

	reg = TLV61310_REG_ENABLE;
	if (tlv61310_reg_enable & TLV61310_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		tlv61310_reg_enable &= (~TLV61310_ENABLE_LED1);
	} else {
		/* if LED 2 is disable, disable LED 1 and clear mode */
		tlv61310_reg_enable &= (~TLV61310_ENABLE_LED1_FLASH);
	}
	val = tlv61310_reg_enable;

	return tlv61310_write_reg(tlv61310_i2c_client, reg, val);
}

static int tlv61310_disable_ch2(void)
{
	unsigned char reg, val;

	reg = TLV61310_REG_ENABLE;
	if (tlv61310_reg_enable & TLV61310_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		tlv61310_reg_enable &= (~TLV61310_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		tlv61310_reg_enable &= (~TLV61310_ENABLE_LED2_FLASH);
	}
	val = tlv61310_reg_enable;

	return tlv61310_write_reg(tlv61310_i2c_client, reg, val);
}

static int tlv61310_disable(int channel)
{
	if (channel == TLV61310_CHANNEL_CH1)
		tlv61310_disable_ch1();
	else if (channel == TLV61310_CHANNEL_CH2)
		tlv61310_disable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int tlv61310_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = tlv61310_verify_level(level);

	/* set torch brightness level */
	reg = TLV61310_REG_TORCH_LEVEL_LED1;
	val = tlv61310_torch_level[level];
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	tlv61310_level_ch1 = level;

	/* set flash brightness level */
	reg = TLV61310_REG_FLASH_LEVEL_LED1;
	val = tlv61310_flash_level[level];
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	return ret;
}

static int tlv61310_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = tlv61310_verify_level(level);

	/* set torch brightness level */
	reg = TLV61310_REG_TORCH_LEVEL_LED2;
	val = tlv61310_torch_level[level];
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	tlv61310_level_ch2 = level;

	/* set flash brightness level */
	reg = TLV61310_REG_FLASH_LEVEL_LED2;
	val = tlv61310_flash_level[level];
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	return ret;
}

static int tlv61310_set_level(int channel, int level)
{
	if (channel == TLV61310_CHANNEL_CH1)
		tlv61310_set_level_ch1(level);
	else if (channel == TLV61310_CHANNEL_CH2)
		tlv61310_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int tlv61310_init(void)
{
	int ret;
	unsigned char reg, val;

	tlv61310_pinctrl_set(TLV61310_PINCTRL_PIN_HWEN, TLV61310_PINCTRL_PINSTATE_HIGH);
	msleep(20);

	/* clear enable register */
	reg = TLV61310_REG_ENABLE;
	val = TLV61310_DISABLE;
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	tlv61310_reg_enable = val;

	reg = TLV61310_REG_FLASH_LEVEL_LED1; /* LED1 Flash Brightness Register */
	val = 0x3F;
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	reg = TLV61310_REG_TORCH_LEVEL_LED1; /* LED1 Torch Brightness Register */
	val = 0x3F;
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	reg = TLV61310_REG_FLASH_LEVEL_LED2; /* LED2 Flash Brightness Register */
	val = 0x3F;
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	reg = TLV61310_REG_TORCH_LEVEL_LED2; /* LED2 Torch Brightness Register */
	val = 0x3F;
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	/* set torch current ramp time and flash timeout */
	reg = TLV61310_REG_TIMING_CONF;
	val = TLV61310_TORCH_RAMP_TIME | TLV61310_FLASH_TIMEOUT;
	ret = tlv61310_write_reg(tlv61310_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int tlv61310_uninit(void)
{
	tlv61310_disable(TLV61310_CHANNEL_CH1);
	tlv61310_disable(TLV61310_CHANNEL_CH2);
	tlv61310_pinctrl_set(TLV61310_PINCTRL_PIN_HWEN, TLV61310_PINCTRL_PINSTATE_LOW);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer tlv61310_timer_ch1;
static struct hrtimer tlv61310_timer_ch2;
static unsigned int tlv61310_timeout_ms[TLV61310_CHANNEL_NUM];

static void tlv61310_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	tlv61310_disable_ch1();
}

static void tlv61310_work_disable_ch2(struct work_struct *data)
{
	pr_debug("lt work queue callback\n");
	tlv61310_disable_ch2();
}

static enum hrtimer_restart tlv61310_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&tlv61310_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tlv61310_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&tlv61310_work_ch2);
	return HRTIMER_NORESTART;
}

static int tlv61310_timer_start(int channel, ktime_t ktime)
{
	if (channel == TLV61310_CHANNEL_CH1)
		hrtimer_start(&tlv61310_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == TLV61310_CHANNEL_CH2)
		hrtimer_start(&tlv61310_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int tlv61310_timer_cancel(int channel)
{
	if (channel == TLV61310_CHANNEL_CH1)
		hrtimer_cancel(&tlv61310_timer_ch1);
	else if (channel == TLV61310_CHANNEL_CH2)
		hrtimer_cancel(&tlv61310_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int tlv61310_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= TLV61310_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		tlv61310_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		tlv61310_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (tlv61310_timeout_ms[channel]) {
				ktime = ktime_set(tlv61310_timeout_ms[channel] / 1000,
						(tlv61310_timeout_ms[channel] % 1000) * 1000000);
				tlv61310_timer_start(channel, ktime);
			}
			tlv61310_enable(channel);
		} else {
			tlv61310_disable(channel);
			tlv61310_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = TLV61310_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = TLV61310_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = tlv61310_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = tlv61310_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = TLV61310_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int tlv61310_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int tlv61310_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int tlv61310_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&tlv61310_mutex);
	if (set) {
		if (!use_count)
			ret = tlv61310_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = tlv61310_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&tlv61310_mutex);

	return ret;
}

static ssize_t tlv61310_strobe_store(struct flashlight_arg arg)
{
	tlv61310_set_driver(1);
	tlv61310_set_level(arg.channel, arg.level);
	tlv61310_timeout_ms[arg.channel] = 0;
	tlv61310_enable(arg.channel);
	msleep(arg.dur);
	tlv61310_disable(arg.channel);
	tlv61310_set_driver(0);

	return 0;
}

static struct flashlight_operations tlv61310_ops = {
	tlv61310_open,
	tlv61310_release,
	tlv61310_ioctl,
	tlv61310_strobe_store,
	tlv61310_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int tlv61310_chip_init(struct tlv61310_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * tlv61310_init();
	 */

	return 0;
}

static int tlv61310_parse_dt(struct device *dev,
		struct tlv61310_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, TLV61310_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int tlv61310_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tlv61310_platform_data *pdata = dev_get_platdata(&client->dev);
	struct tlv61310_chip_data *chip;
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct tlv61310_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = tlv61310_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	tlv61310_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&tlv61310_work_ch1, tlv61310_work_disable_ch1);
	INIT_WORK(&tlv61310_work_ch2, tlv61310_work_disable_ch2);

	/* init timer */
	hrtimer_init(&tlv61310_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tlv61310_timer_ch1.function = tlv61310_timer_func_ch1;
	hrtimer_init(&tlv61310_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tlv61310_timer_ch2.function = tlv61310_timer_func_ch2;
	tlv61310_timeout_ms[TLV61310_CHANNEL_CH1] = 100;
	tlv61310_timeout_ms[TLV61310_CHANNEL_CH2] = 100;

	/* init chip hw */
	tlv61310_chip_init(chip);

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &tlv61310_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(TLV61310_NAME, &tlv61310_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}

	pr_debug("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int tlv61310_i2c_remove(struct i2c_client *client)
{
	struct tlv61310_platform_data *pdata = dev_get_platdata(&client->dev);
	struct tlv61310_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(TLV61310_NAME);

	/* flush work queue */
	flush_work(&tlv61310_work_ch1);
	flush_work(&tlv61310_work_ch2);

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id tlv61310_i2c_id[] = {
	{TLV61310_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id tlv61310_i2c_of_match[] = {
	{.compatible = TLV61310_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver tlv61310_i2c_driver = {
	.driver = {
		.name = TLV61310_NAME,
#ifdef CONFIG_OF
		.of_match_table = tlv61310_i2c_of_match,
#endif
	},
	.probe = tlv61310_i2c_probe,
	.remove = tlv61310_i2c_remove,
	.id_table = tlv61310_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int tlv61310_probe(struct platform_device *dev)
{
	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (tlv61310_pinctrl_init(dev)) {
		pr_debug("Failed to init pinctrl.\n");
		return -1;
	}

	if (i2c_add_driver(&tlv61310_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

	pr_debug("Probe done.\n");

	return 0;
}

static int tlv61310_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&tlv61310_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tlv61310_of_match[] = {
	{.compatible = TLV61310_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, tlv61310_of_match);
#else
static struct platform_device tlv61310_platform_device[] = {
	{
		.name = TLV61310_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, tlv61310_platform_device);
#endif

static struct platform_driver tlv61310_platform_driver = {
	.probe = tlv61310_probe,
	.remove = tlv61310_remove,
	.driver = {
		.name = TLV61310_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = tlv61310_of_match,
#endif
	},
};

static int __init flashlight_tlv61310_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&tlv61310_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&tlv61310_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_tlv61310_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&tlv61310_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_tlv61310_init);
module_exit(flashlight_tlv61310_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight TLV61310 Driver");


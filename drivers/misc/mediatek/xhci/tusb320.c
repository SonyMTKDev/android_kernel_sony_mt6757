/*
 * tusb320.c - Driver for TUSB320 USB Type-C connector chip
 *
 * Copyright (C) 2015 TI, Inc.
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Author: HUAWEI, Inc.
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
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <asm/irq.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/usb/typec.h>
#include <linux/usb/class-dual-role.h>
#include <mt-plat/battery_meter.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/power_supply.h>


#include "tusb320.h"
#define TRYSNK_TIMEOUT_MS 600
#define TUSB320_RESET_DURATION_MS 25
#define TUSB320_IRQ_CLEAN_DELAY_MS  1000
#define TUSB320_IRQ_CLEAN_MAX_RETRY  3
#define DUAL_ROLE_SET_MODE_WAIT_MS 1500
static enum typec_current_mode tusb320_current_mode_detect(void);
static enum typec_current_mode tusb320_current_advertise_get(void);
static int tusb320_current_advertise_set(enum typec_current_mode current_mode);
static enum typec_attached_state tusb320_attached_state_detect(void);
static enum typec_port_mode tusb320_port_mode_get(void);
static int tusb320_port_mode_set(enum typec_port_mode port_mode);
static ssize_t tusb320_dump_regs(char *buf);
static struct tusb320_device_info *g_tusb320_dev;
struct typec_device_ops tusb320_ops = {
	.current_detect = tusb320_current_mode_detect,
	.attached_state_detect = tusb320_attached_state_detect,
	.current_advertise_get = tusb320_current_advertise_get,
	.current_advertise_set = tusb320_current_advertise_set,
	.port_mode_get = tusb320_port_mode_get,
	.port_mode_set = tusb320_port_mode_set,
	.dump_regs = tusb320_dump_regs
};
static int tusb320_read_reg(u8 reg, u8 * val)
{
	struct tusb320_device_info *di = g_tusb320_dev;
	struct i2c_msg msgs[] = {
		{
		 .addr = di->client->addr,
		 .flags = 0,
		 .buf = &reg,
		 .len = sizeof(reg),
		 }
		,
		{
		 .addr = di->client->addr,
		 .flags = I2C_M_RD,
		 .buf = val,
		 .len = sizeof(u8),
		 }
		,
	};
	int ret;
	ret = i2c_transfer(di->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		pr_err("%s: i2c_transfer error %d\n", __func__, ret);
	}
	return ret;
}
static int tusb320_write_reg(u8 reg, u8 val)
{
	struct tusb320_device_info *di = g_tusb320_dev;
	u8 buf[] = { reg, val };
	struct i2c_msg msgs[] = {
		{
		 .addr = di->client->addr,
		 .flags = 0,
		 .buf = buf,
		 .len = sizeof(buf),
		 }
		,
	};
	int ret;
	ret = i2c_transfer(di->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		pr_err("%s: transfer error %d\n", __func__, ret);
	return ret;
}
static int tusb320_int_clear(void)
{
	u8 reg_val;
	int ret;
	ret = tusb320_read_reg(TUSB320_REG_ATTACH_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s: read error\n", __func__);
		return ret;
	}
	reg_val |= TUSB320_REG_STATUS_INT;
	ret = tusb320_write_reg(TUSB320_REG_ATTACH_STATUS, reg_val);
	if (ret < 0) {
		pr_err("%s: write error\n", __func__);
		return ret;
	}
	return ret;
}
static enum typec_current_mode tusb320_current_mode_detect(void)
{
	u8 reg_val, mask_val;
	int ret;
	enum typec_current_mode current_mode = TYPEC_CURRENT_MODE_DEFAULT;
	ret = tusb320_read_reg(TUSB320_REG_CURRENT_MODE, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_CURRENT_MODE error\n", __func__);
		return current_mode;
	}
	pr_debug("%s: REG_CURRENT_MODE 08H is 0x%x\n", __func__, reg_val);
	mask_val = reg_val & TUSB320_REG_CUR_MODE_DETECT_MASK;
	switch (mask_val) {
	case TUSB320_REG_CUR_MODE_DETECT_DEFAULT:
		current_mode = TYPEC_CURRENT_MODE_DEFAULT;
		break;
	case TUSB320_REG_CUR_MODE_DETECT_MID:
		current_mode = TYPEC_CURRENT_MODE_MID;
		break;
	case TUSB320_REG_CUR_MODE_DETECT_HIGH:
		current_mode = TYPEC_CURRENT_MODE_HIGH;
		break;
	default:
		current_mode = TYPEC_CURRENT_MODE_UNSPPORTED;
	}
	pr_debug("%s: current mode is %d\n", __func__, current_mode);
	return current_mode;
}
static enum typec_current_mode tusb320_current_advertise_get(void)
{
	u8 reg_val, mask_val;
	int ret;
	enum typec_current_mode current_mode = TYPEC_CURRENT_MODE_DEFAULT;
	ret = tusb320_read_reg(TUSB320_REG_CURRENT_MODE, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_CURRENT_MODE error\n", __func__);
		return current_mode;
	}
	pr_info("%s: REG_CURRENT_MODE 08H is 0x%x\n", __func__, reg_val);
	mask_val = reg_val & TUSB320_REG_CUR_MODE_ADVERTISE_MASK;
	switch (mask_val) {
	case TUSB320_REG_CUR_MODE_ADVERTISE_DEFAULT:
		current_mode = TYPEC_CURRENT_MODE_DEFAULT;
		break;
	case TUSB320_REG_CUR_MODE_ADVERTISE_MID:
		current_mode = TYPEC_CURRENT_MODE_MID;
		break;
	case TUSB320_REG_CUR_MODE_ADVERTISE_HIGH:
		current_mode = TYPEC_CURRENT_MODE_HIGH;
		break;
	default:
		current_mode = TYPEC_CURRENT_MODE_UNSPPORTED;
	}
	pr_info("%s: current advertise is %d\n", __func__, current_mode);
	return current_mode;
}
static int tusb320_current_advertise_set(enum typec_current_mode current_mode)
{
	u8 reg_val, mask_val;
	int ret;
	ret = tusb320_read_reg(TUSB320_REG_CURRENT_MODE, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_CURRENT_MODE error\n", __func__);
		return -1;
	}
	pr_debug("%s: REG_CURRENT_MODE 08H is 0x%x\n", __func__, reg_val);
	switch (current_mode) {
	case TYPEC_CURRENT_MODE_MID:
		mask_val = TUSB320_REG_CUR_MODE_ADVERTISE_MID;
		break;
	case TYPEC_CURRENT_MODE_HIGH:
		mask_val = TUSB320_REG_CUR_MODE_ADVERTISE_HIGH;
		break;
	default:
		mask_val = TUSB320_REG_CUR_MODE_ADVERTISE_DEFAULT;
	}
	if (mask_val == (reg_val & TUSB320_REG_CUR_MODE_ADVERTISE_MASK)) {
		pr_info("%s: current advertise is %d already\n", __func__,
			current_mode);
		return 0;
	}
	reg_val &= ~TUSB320_REG_CUR_MODE_ADVERTISE_MASK;
	reg_val |= mask_val;
	ret = tusb320_write_reg(TUSB320_REG_CURRENT_MODE, reg_val);
	if (ret < 0) {
		pr_err("%s: write REG_CURRENT_MODE error\n", __func__);
		return -1;
	}
	pr_info("%s: current advertise set to %d\n", __func__, current_mode);
	return 0;
}
static enum typec_attached_state tusb320_attached_state_detect(void)
{
	u8 reg_val, mask_val;
	int ret;
	enum typec_attached_state attached_state = TYPEC_NOT_ATTACHED;
	ret = tusb320_read_reg(TUSB320_REG_ATTACH_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_ATTACH_STATUS error\n", __func__);
		return attached_state;
	}
	pr_info("%s: REG_ATTACH_STATUS 09H is 0x%x\n", __func__, reg_val);
	mask_val = reg_val & TUSB320_REG_STATUS_MODE;
	switch (mask_val) {
	case TUSB320_REG_STATUS_AS_DFP:
		attached_state = TYPEC_ATTACHED_AS_DFP;
		break;
	case TUSB320_REG_STATUS_AS_UFP:
		attached_state = TYPEC_ATTACHED_AS_UFP;
		break;
	case TUSB320_REG_STATUS_TO_ACCESSORY:
		attached_state = TYPEC_ATTACHED_TO_ACCESSORY;
		break;
	default:
		attached_state = TYPEC_NOT_ATTACHED;
	}
	pr_info("%s: attached state is %d\n", __func__, attached_state);
	return attached_state;
}
static enum typec_port_mode tusb320_port_mode_get(void)
{
	u8 reg_val, mask_val;
	int ret;
	enum typec_port_mode port_mode = TYPEC_MODE_ACCORDING_TO_PROT;
	ret = tusb320_read_reg(TUSB320_REG_MODE_SET, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_MODE_SET error\n", __func__);
		return port_mode;
	}
	pr_info("%s: REG_MODE_SET is 0x%x\n", __func__, reg_val);
	mask_val = reg_val & TUSB320_REG_SET_MODE;
	switch (mask_val) {
	case TUSB320_REG_SET_UFP:
		port_mode = TYPEC_UFP_MODE;
		break;
	case TUSB320_REG_SET_DFP:
		port_mode = TYPEC_DFP_MODE;
		break;
	case TUSB320_REG_SET_DRP:
		port_mode = TYPEC_DRP_MODE;
		break;
	default:
		port_mode = TYPEC_MODE_ACCORDING_TO_PROT;
		break;
	}
	pr_info("%s: port mode is %d\n", __func__, port_mode);
	return port_mode;
}
static int tusb320_port_mode_set(enum typec_port_mode port_mode)
{
	u8 reg_val, mask_val;
	int ret;
	ret = tusb320_read_reg(TUSB320_REG_MODE_SET, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_MODE_SET error\n", __func__);
		return -1;
	}
	pr_info("%s: REG_MODE_SET 0aH is 0x%x\n", __func__, reg_val);
	switch (port_mode) {
	case TYPEC_UFP_MODE:
		mask_val = TUSB320_REG_SET_UFP;
		break;
	case TYPEC_DFP_MODE:
		mask_val = TUSB320_REG_SET_DFP;
		break;
	case TYPEC_DRP_MODE:
		mask_val = TUSB320_REG_SET_DRP;
		break;
	default:
		mask_val = TUSB320_REG_SET_BY_PORT;
		break;
	}
	reg_val &= ~TUSB320_REG_SET_MODE;
	reg_val |= mask_val;
	ret = tusb320_write_reg(TUSB320_REG_MODE_SET, reg_val);
	if (ret < 0) {
		pr_err("%s: write REG_MODE_SET error\n", __func__);
		return -1;
	}
	pr_info("%s: port mode set to %d\n", __func__, port_mode);
	return 0;
}
static ssize_t tusb320_dump_regs(char *buf)
{
	int i;
	char reg[REGISTER_NUM] = { 0 };
	for (i = 8; i < 11; i++) {
		tusb320_read_reg(i, &reg[i]);
	}
	return scnprintf(buf, PAGE_SIZE,
			 "0x%02X,0x%02X,0x%02X\n", reg[8], reg[9], reg[10]);
}
static int tusb320_dump_register_proc_show(struct seq_file *m, void *v)
{
	int i=0;
	u8 tusb320_reg[12] = {0};

	for (i=0;i<11;i++)
	{
		tusb320_read_reg(i+0x00, &tusb320_reg[i]);
		seq_printf(m,"[0x%x]=0x%x \n", i, tusb320_reg[i]);
	}
	tusb320_read_reg(0x45, &tusb320_reg[11]);
	seq_printf(m,"[0x45]=0x%x \n", tusb320_reg[11]);

	return 0;
}

static int tusb320_dump_register_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tusb320_dump_register_proc_show, NULL);
}

static const struct file_operations tusb320_dump_register_proc_fops = {
	.open	= tusb320_dump_register_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static int cc_status_proc_show(struct seq_file *m, void *v)
{
	u8 tusb320_reg = 0;

	tusb320_read_reg(0x09, &tusb320_reg);
	tusb320_reg = ((tusb320_reg & 0x20) >> 5);
	seq_printf(m,"%x \n", tusb320_reg +1);

	return 0;
}

static int cc_status_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cc_status_proc_show, NULL);
}

static const struct file_operations cc_status_proc_fops = {
	.open	= cc_status_proc_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};
static void tusb320_soft_reset(void)
{
	u8 reg_val;
	int ret;
	pr_info("%s\n", __func__);
	ret = tusb320_read_reg(TUSB320_REG_MODE_SET, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_MODE_SET error\n", __func__);
	}
	reg_val |= TUSB320_REG_SET_SOFT_RESET;
	ret = tusb320_write_reg(TUSB320_REG_MODE_SET, reg_val);
	if (ret < 0) {
		pr_err("%s: write REG_MODE_SET error\n", __func__);
	}
}
static int tusb320_drp_duty_cycle_set(int duty_cycle)
{
	u8 reg_val, mask_val;
	int ret;
	pr_info("%s: set to %d\n", __func__, duty_cycle);
	ret = tusb320_read_reg(TUSB320_REG_ATTACH_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_REG_ATTACH_STATUS error\n", __func__);
		return -1;
	}
	mask_val = duty_cycle & TUSB320_REG_STATUS_DRP_DUTY_CYCLE;
	if (mask_val == (reg_val & TUSB320_REG_STATUS_DRP_DUTY_CYCLE)) {
		pr_info("%s: DRP duty cycle is already %d\n", __func__,
			duty_cycle);
		return 0;
	}
	reg_val &= ~TUSB320_REG_STATUS_DRP_DUTY_CYCLE;
	reg_val |= mask_val;
	ret = tusb320_write_reg(TUSB320_REG_ATTACH_STATUS, reg_val);
	if (ret < 0) {
		pr_err("%s: write REG_REG_ATTACH_STATUS error\n", __func__);
		return -1;
	}
	return 0;
}
static void tusb320_rd_rp_disable(int set)
{
	u8 reg_val;
	int ret;
	ret = tusb320_read_reg(TUSB320_REG_DISABLE, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_DISABLE error\n", __func__);
	}
	if (set == DISABLE_SET) {
		pr_info("%s: set\n", __func__);
		reg_val |= TUSB320_REG_SET_DISABLE_RD_RP;
	} else {
		pr_info("%s: clear\n", __func__);
		reg_val &= ~((u8) TUSB320_REG_SET_DISABLE_RD_RP);
	}
	ret = tusb320_write_reg(TUSB320_REG_DISABLE, reg_val);
	if (ret < 0) {
		pr_err("%s: write REG_DISABLE error\n", __func__);
	}
}
static void tusb320_disabled_state_start(void)
{
	tusb320_port_mode_set(TYPEC_UFP_MODE);
	tusb320_rd_rp_disable(DISABLE_SET);
	tusb320_soft_reset();
	mdelay(TUSB320_RESET_DURATION_MS);
}
static void tusb320_disabled_state_exit(enum typec_port_mode port_mode)
{
	tusb320_port_mode_set(port_mode);
	tusb320_rd_rp_disable(DISABLE_CLEAR);
}
static void tusb320_wdog_work(struct work_struct *w)
{
	struct tusb320_device_info *di = g_tusb320_dev;
	int ret = 0;
	bool work = 0;
	pr_err("%s: start\n", __func__);
	mutex_lock(&di->mutex);
	if (di->clean_failded) {
		pr_err("%s: clean interrupt mask\n", __func__);
		ret = tusb320_int_clear();
		if (ret < 0
		    && di->clean_retry_count < TUSB320_IRQ_CLEAN_MAX_RETRY) {
			pr_err("%s: clean interrupt mask error\n", __func__);
			work = 1;
			di->clean_retry_count++;
		}
		di->clean_failded = 0;
	}
	di->trysnk_attempt = 0;
	mutex_unlock(&di->mutex);
	/* make sure set the DRP duty cycle back to default */
	tusb320_drp_duty_cycle_set(TUSB320_REG_STATUS_DRP_DUTY_CYCLE_30);
	if (work)
		schedule_delayed_work(&di->g_wdog_work,
				      msecs_to_jiffies
				      (TUSB320_IRQ_CLEAN_DELAY_MS));
	/* to notify userspace that the state might have changed */
	if (di->dual_role)
		dual_role_instance_changed(di->dual_role);
	pr_err("%s: end\n", __func__);
}

//CEI comment start: Fix for NONSTANDARD_CHARGER or Type-C//
int tusb320_init_done = 0;
enum typec_current_mode tusb320_current_mode_get(void)
{
	enum typec_current_mode current_mode = tusb320_current_mode_detect();
	return current_mode;
}
//CEI comment end//

static void tusb320_intb_work(struct work_struct *work)
{
	int ret;
	struct tusb320_device_info *di =
	    container_of(work, struct tusb320_device_info, g_intb_work);
	enum typec_current_mode current_mode;
	enum typec_attached_state attached_state;
	pr_info("%s: start\n", __func__);
	ret = tusb320_int_clear();
	/* cancel watch dog work */
	cancel_delayed_work(&di->g_wdog_work);
	if (ret < 0) {
		pr_err("%s: clean interrupt mask error\n", __func__);
		mutex_lock(&di->mutex);
		di->clean_failded = 1;
		di->clean_retry_count = 0;
		mutex_unlock(&di->mutex);
		schedule_delayed_work(&di->g_wdog_work,
				      msecs_to_jiffies
				      (TUSB320_IRQ_CLEAN_DELAY_MS));
	}
	current_mode = tusb320_current_mode_detect();
	attached_state = tusb320_attached_state_detect();
	mutex_lock(&di->mutex);
	if (TYPEC_ATTACHED_AS_DFP == attached_state) {
		if (REVERSE_ATTEMPT == di->reverse_state) {
			pr_info("%s: reversed success, Sink detected\n",
				__func__);
			di->reverse_state = REVERSE_COMPLETE;
			di->sink_attached = 1;
			/* turn on VBUS */
			//typec_sink_detected_handler(TYPEC_SINK_DETECTED);
	                set_chr_boost_current_limit(750); //ori 1500mA.
			complete(&di->reverse_completion);
		} else if (di->trysnk_attempt) {
			pr_info("%s: TrySNK fail, Sink detected again\n",
				__func__);
			/* TrySNK has been attempted, clear the flag */
			di->trysnk_attempt = 0;
			di->sink_attached = 1;
			/* turn on VBUS */
			//typec_sink_detected_handler(TYPEC_SINK_DETECTED);
	                set_chr_boost_current_limit(750); //ori 1500mA.
			tusb320_drp_duty_cycle_set(TUSB320_REG_STATUS_DRP_DUTY_CYCLE_30);
			/* notify the attached state */
			complete(&di->reverse_completion);
		} else {
			pr_info("%s: Sink detected, perform TrySNK\n", __func__);
			di->trysnk_attempt = 1;
			/* set current advertisement to default */
//			tusb320_current_advertise_set(TYPEC_CURRENT_MODE_DEFAULT);
			/* reset the chip and reset it again after 25ms */
			tusb320_soft_reset();
			mdelay(TUSB320_RESET_DURATION_MS);
			tusb320_soft_reset();
			pr_info("%s: schedule wdog work\n", __func__);
			schedule_delayed_work(&di->g_wdog_work,
					      msecs_to_jiffies
					      (TRYSNK_TIMEOUT_MS));
			/* Change DRP duty Cycle to 60% */
			tusb320_drp_duty_cycle_set(TUSB320_REG_STATUS_DRP_DUTY_CYCLE_60);
		}
	} else if (TYPEC_ATTACHED_AS_UFP == attached_state) {
		/*
		 * exception handling:
		 * if CC pin is not stable, the state transition may from
		 * AS DFP to AS UFP direct, VBUS should be turned off first
		 */
		if (di->sink_attached) {
			pr_info("%s: UFP state, remove Sink first\n", __func__);
			//typec_sink_detected_handler(TYPEC_SINK_REMOVED);
			di->sink_attached = 0;
		}
		if (REVERSE_ATTEMPT == di->reverse_state) {
			pr_info
			    ("%s: reversed success, Source and VBUS detected\n",
			     __func__);
			di->reverse_state = REVERSE_COMPLETE;
		}
		if (di->trysnk_attempt) {
			pr_info
			    ("%s: TrySNK success, Source and VBUS detected\n",
			     __func__);
			di->trysnk_attempt = 0;
			tusb320_drp_duty_cycle_set(TUSB320_REG_STATUS_DRP_DUTY_CYCLE_30);
		}
		/* notify the attached state */
		complete(&di->reverse_completion);
	} else if (TYPEC_NOT_ATTACHED == attached_state) {
		/* set current advertisement to high */
//		tusb320_current_advertise_set(TYPEC_CURRENT_MODE_HIGH);
		/* turn off VBUS when unattached */
                set_chr_enable_otg(0);
                set_chr_boost_current_limit(0);
		if (di->sink_attached) {
			pr_info("%s: Sink removed\n", __func__);
			//typec_sink_detected_handler(TYPEC_SINK_REMOVED);
			di->sink_attached = 0;
		}
		/* make sure set the port mode back to default when unattached */
		if (di->reverse_state == REVERSE_COMPLETE) {
			pr_info("%s: unattached, set mode back to DRP\n",
				__func__);
			tusb320_port_mode_set(TYPEC_MODE_ACCORDING_TO_PROT);
			di->reverse_state = 0;
		}
	}
	mutex_unlock(&di->mutex);
	/* to notify userspace that the state might have changed */
	if (di->dual_role)
		dual_role_instance_changed(di->dual_role);
	pr_info("%s: end\n", __func__);
}
static irqreturn_t tusb320_irq_handler(int irq, void *dev_id)
{
	int gpio_value_intb = 0;
	struct tusb320_device_info *di = dev_id;
	pr_info("%s: start\n", __func__);
	gpio_value_intb = gpio_get_value(di->gpio_intb);
	if (1 == gpio_value_intb) {
		pr_err("%s: intb high when interrupt occured!\n", __func__);
	}
	schedule_work(&di->g_intb_work);
	return IRQ_HANDLED;
}
static int tusb320_device_check(struct tusb320_device_info *di)
{
	u8 reg_val;
	int ret;
	ret = tusb320_read_reg(TUSB320_REG_DEVICE_ID, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_DEVICE_ID error\n", __func__);
		return ret;
	}
	return 0;
}
static enum dual_role_property fusb_drp_properties[] = {
	DUAL_ROLE_PROP_SUPPORTED_MODES,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};
 /* Callback for "cat /sys/class/dual_role_usb/otg_default/<property>" */
static int dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
				    enum dual_role_property prop,
				    unsigned int *val)
{
	struct tusb320_device_info *di = dual_role_get_drvdata(dual_role);
	enum typec_attached_state attached_state;
	if (!di) {
		pr_err("%s: device info NULL\n", __func__);
		return -EFAULT;
	}
	attached_state = tusb320_attached_state_detect();
	pr_info("%s: attached_state = %d, prop = %d\n", __func__,attached_state,prop);

	if (prop == DUAL_ROLE_PROP_SUPPORTED_MODES)
		*val = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
	else if (attached_state == TYPEC_ATTACHED_AS_DFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (attached_state == TYPEC_ATTACHED_AS_UFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}
/* Decides whether userspace can change a specific property */
static int dual_role_is_writeable(struct dual_role_phy_instance *drp,
				  enum dual_role_property prop)
{
	int ret = 0;

	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
	case DUAL_ROLE_PROP_DR:
	case DUAL_ROLE_PROP_PR:
		ret = 1;
		break;

	default:
		break;
	}

	return ret;
}
/* 1. Check to see if current attached_state is same as requested state
 * if yes, then, return.
 * 2. Disonect current session
 * 3. Set approrpriate mode (dfp or ufp)
 * 4. wait for 1.5 secs to see if we get into the corresponding target state
 * if yes, return
 * 5. if not, fallback to Try.SNK
 * 6. wait for 1.5 secs to see if we get into one of the attached states
 * 7. return -EIO
 * Also we have to fallback to Try.SNK state machine on cable disconnect
 */
static int dual_role_set_prop(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	struct tusb320_device_info *di = dual_role_get_drvdata(dual_role);
	enum typec_attached_state attached_state;
	enum typec_port_mode port_mode;

	int timeout = 0;
	int ret = 0;
	if (!di) {
		pr_err("%s: device info NULL\n", __func__);
		return -EFAULT;
	}
        attached_state = tusb320_attached_state_detect();

	pr_info("%s: start\n", __func__);

	if (prop == DUAL_ROLE_PROP_MODE) {
		if (*val == DUAL_ROLE_PROP_MODE_DFP) {
			pr_info("%s: Setting DFP mode\n", __func__);
			port_mode = TYPEC_DFP_MODE;
		} else if (*val == DUAL_ROLE_PROP_MODE_UFP) {
			pr_info("%s: Setting UFP mode\n", __func__);
			port_mode = TYPEC_UFP_MODE;
		} else {
			pr_err("%s: Trying to set invalid mode\n", __func__);
			return -EINVAL;
		}
	} else if (prop == DUAL_ROLE_PROP_PR) {
		if (*val == DUAL_ROLE_PROP_PR_SRC) {
			pr_info("%s: Setting SRC mode\n", __func__);
			port_mode = TYPEC_DFP_MODE;
		} else if (*val == DUAL_ROLE_PROP_PR_SNK) {
			pr_info("%s: Setting SNK mode\n", __func__);
			port_mode = TYPEC_UFP_MODE;
		} else {
			pr_err("%s: Trying to set invalid PR\n", __func__);
			return -EINVAL;
		}
	} else if (prop == DUAL_ROLE_PROP_DR) {
		if (*val == DUAL_ROLE_PROP_DR_HOST) {
			pr_info("%s: Setting HOST mode\n", __func__);
			port_mode = TYPEC_DFP_MODE;
		} else if (*val == DUAL_ROLE_PROP_DR_DEVICE) {
			pr_info("%s: Setting DEVICE mode\n", __func__);
			port_mode = TYPEC_UFP_MODE;
		} else {
			pr_err("%s: Trying to set invalid DR\n", __func__);
			return -EINVAL;
		}
	} else {
		pr_info("%s: Property cannot be set\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&di->mutex);

	if (((attached_state == TYPEC_ATTACHED_AS_DFP) &&
		(port_mode == TYPEC_UFP_MODE)) || 
		((attached_state == TYPEC_ATTACHED_AS_UFP) &&
		(port_mode == TYPEC_DFP_MODE))) {

		pr_info("%s: enter set mode = %d\n",__func__,port_mode);
		disable_irq(di->irq_intb);
		di->reverse_state = REVERSE_ATTEMPT;
		if (attached_state == TYPEC_ATTACHED_AS_DFP) {
			di->sink_attached = 0;
			/* turns off VBUS first */
			//typec_sink_detected_handler(TYPEC_SINK_REMOVED);
			set_chr_enable_otg(0);
			set_chr_boost_current_limit(0);
		}
		/* transition to Disable State */
		tusb320_disabled_state_start();
		/* exiting from Disable State and set mode to DFP */
		tusb320_disabled_state_exit(port_mode);
		enable_irq(di->irq_intb);

	} else {
		pr_info("%s: attached_state = %u, no change.\n",__func__,attached_state);
		mutex_unlock(&di->mutex);
		return 0;
	}

	mutex_unlock(&di->mutex);

	reinit_completion(&di->reverse_completion);
	timeout = wait_for_completion_timeout(&di->reverse_completion,
					msecs_to_jiffies
					(DUAL_ROLE_SET_MODE_WAIT_MS));

	if (!timeout) {
		pr_err("%s: reverse failed, set mode to DRP\n", __func__);
		disable_irq(di->irq_intb);
		/* transition to Disabled state */
		tusb320_disabled_state_start();
		/* exit from Disabled state and set mode to DRP */
		tusb320_disabled_state_exit(TYPEC_MODE_ACCORDING_TO_PROT);
		di->reverse_state = 0;
		enable_irq(di->irq_intb);
		pr_info("%s: wait for the attached state\n", __func__);
		reinit_completion(&di->reverse_completion);
		wait_for_completion_timeout(&di->reverse_completion,
					    msecs_to_jiffies
					    (DUAL_ROLE_SET_MODE_WAIT_MS));
		ret = -EIO;
	}
	pr_err("%s: end ret = %d\n", __func__, ret);
	return ret;
}

static int tusb320_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0;
	struct tusb320_device_info *di = NULL;
	struct device *cdev = &client->dev;
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;
	struct power_supply *usb_psy;
	struct device_node *node = cdev->of_node;

	di = devm_kzalloc(cdev, sizeof(*di), GFP_KERNEL);
	if (!di) {
		pr_err("tusb320_device_info is NULL!\n");
		return -ENOMEM;
	}
	pr_info("%s: start\n", __func__);

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_err(cdev, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	} else
		pr_info("Found USB supply\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c_check_functionality error!\n", __func__);
		ret = -ENODEV;
		goto err_i2c_check_functionality_0;
	}

	g_tusb320_dev = di;
	di->dev = cdev;
	di->client = client;
	di->usb_psy = usb_psy;
	node = di->dev->of_node;
	i2c_set_clientdata(client, di);
	ret = tusb320_device_check(di);
	if (ret < 0) {
		pr_err("%s: check device error, the chip is not TUSB320!\n",
		       __func__);
		goto err_i2c_check_functionality_0;
	}
#if 0
	di->gpio_enb = of_get_named_gpio(node, "tusb320_typec,gpio_enb", 0);
	if (!gpio_is_valid(di->gpio_enb)) {
		pr_err("%s: of get gpio_enb error\n", __func__);
		ret = -EINVAL;
		goto err_i2c_check_functionality_0;
	}
	di->gpio_intb = of_get_named_gpio(node, "tusb320_typec,gpio_intb", 0);
	if (!gpio_is_valid(di->gpio_intb)) {
		pr_err("%s: of get gpio_intb error\n", __func__);
		ret = -EINVAL;
		goto err_i2c_check_functionality_0;
	}
	ret = gpio_request(di->gpio_enb, "tusb320_en");
	if (ret < 0) {
		pr_err("%s: gpio_request error, ret=%d, gpio=%d\n", __func__,
		       ret, di->gpio_enb);
		ret = -ENOMEM;
		goto err_gpio_enb_request_1;
	}
	ret = gpio_direction_output(di->gpio_enb, 0);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input error, ret=%d, gpio=%d\n",
		       __func__, ret, di->gpio_enb);
		goto err_gpio_enb_request_1;
	}
	ret = gpio_request(di->gpio_intb, "tusb320_int");
	if (ret < 0) {
		pr_err("%s: gpio_request error, ret=%d, gpio_intb=%d\n",
		       __func__, ret, di->gpio_intb);
		ret = -ENOMEM;
		goto err_gpio_intb_request_2;
	}
	di->irq_intb = gpio_to_irq(di->gpio_intb);
	if (di->irq_intb < 0) {
		pr_err("%s: gpio_to_irq error, gpio_intb=%d\n", __func__,
		       di->gpio_intb);
		goto err_gpio_intb_request_2;
	}
	ret = gpio_direction_input(di->gpio_intb);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input error, ret=%d, gpio_intb=%d\n",
		       __func__, ret, di->gpio_intb);
		goto err_gpio_intb_request_2;
	}
#endif
	mutex_init(&di->mutex);
	INIT_WORK(&di->g_intb_work, tusb320_intb_work);
	INIT_DELAYED_WORK(&di->g_wdog_work, tusb320_wdog_work);
	init_completion(&di->reverse_completion);
	node = of_find_compatible_node(NULL, NULL, "mediatek,typec_control");
	if(!node)
		pr_info("[tusb320] %s cannot get the node\n",__func__);
	di->irq_intb = irq_of_parse_and_map(node, 0);
	pr_info("%s, di->irq_intb = %d\n", __func__, di->irq_intb);

	if (di->irq_intb > 0) {
		ret = devm_request_irq(di->dev, di->irq_intb,
				  tusb320_irq_handler,
				  IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
				  "tusb320_int", di);
		if (ret) {
			pr_err("%s: request_irq error, ret=%d\n", __func__, ret);
			di->irq_intb = -1;
			goto err_irq_request_3;
		}
	} else
		pr_err("%s: IRQ not configured\n", __func__);

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		desc =
		    //devm_kzalloc(&client->dev,
		    devm_kzalloc(cdev,
				 sizeof(struct dual_role_phy_desc), GFP_KERNEL);
		if (!desc) {
			pr_err("unable to allocate dual role descriptor\n");
			goto err_irq_request_3;
		}
		desc->name = "otg_default";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = dual_role_get_local_prop;
		desc->set_property = dual_role_set_prop;
		desc->properties = fusb_drp_properties;
		desc->num_properties = ARRAY_SIZE(fusb_drp_properties);
		desc->property_is_writeable = dual_role_is_writeable;
		dual_role = devm_dual_role_instance_register(cdev, desc);
		if (IS_ERR(dual_role)) {
			dev_err(cdev, "failed to register dual role instance\n");
			return -EINVAL;
		}
		dual_role->drv_data = di;
		di->dual_role = dual_role;
		di->desc = desc;
	}

	ret = tusb320_int_clear(); /* returns non-zero positive value if successful */
	if (ret < 0) {
		pr_err("%s: clean interrupt mask error %d\n", __func__, ret);
	}

	proc_create("tusb320", 0, NULL, &tusb320_dump_register_proc_fops);
	proc_create("cc_status", 0, NULL, &cc_status_proc_fops);
	enable_irq_wake(di->irq_intb);
	pr_info("%s: schedule_work \n", __func__);
	schedule_work(&di->g_intb_work);
	pr_info("%s: end\n", __func__);

//CEI comment start: Fix for NONSTANDARD_CHARGER or Type-C//
	tusb320_init_done = 1;
//CEI comment end//

	return 0;

err_irq_request_3:
	free_irq(di->gpio_intb, di);

err_i2c_check_functionality_0:
	devm_kfree(&client->dev, di);
	g_tusb320_dev = NULL;
	pr_err("%s: FAIL, ret = %d\n", __func__, ret);
	return ret;
}
static int tusb320_remove(struct i2c_client *client)
{
	struct tusb320_device_info *di = i2c_get_clientdata(client);
	free_irq(di->irq_intb, di);
	gpio_set_value(di->gpio_enb, 1);
	gpio_free(di->gpio_enb);
	gpio_free(di->gpio_intb);
	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		devm_dual_role_instance_unregister(di->dev, di->dual_role);
		devm_kfree(di->dev, di->desc);
	}
	return 0;
}
static const struct of_device_id typec_tusb320_ids[] = {
	{.compatible = "TI,tusb320"},
	{},
};
MODULE_DEVICE_TABLE(of, typec_tusb320_ids);
static const struct i2c_device_id tusb320_i2c_id[] = {
	{"tusb320", 0},
	{}
};
static struct i2c_driver tusb320_i2c_driver = {
	.driver = {
		   .name = "tusb320",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(typec_tusb320_ids),
		   },
	.probe = tusb320_probe,
	.remove = tusb320_remove,
	.id_table = tusb320_i2c_id,
};
static __init int tusb320_i2c_init(void)
{
	int ret = 0;
	pr_info("%s: entry\n", __func__);
	ret = i2c_add_driver(&tusb320_i2c_driver);
	if (ret) {
		pr_err("%s: i2c_add_driver error!!!\n", __func__);
	}
	pr_info("%s: end\n", __func__);
	return ret;
}
static __exit void tusb320_i2c_exit(void)
{
	i2c_del_driver(&tusb320_i2c_driver);
}
module_init(tusb320_i2c_init);
module_exit(tusb320_i2c_exit);
MODULE_AUTHOR("TI");
MODULE_DESCRIPTION("I2C bus driver for tusb320 Type-C connector");
MODULE_LICENSE("GPL v2");

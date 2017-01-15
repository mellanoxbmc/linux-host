/*
 * Copyright (c) 2016 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2016 Vadim Pasternak <vadimp@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_data/mlxcpld-hotplug.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/* Offset of event and mask registers from status register. */
#define MLXCPLD_CTRL_REG_AGGR_ADRR	0x3a
#define MLXCPLD_CTRL_REG_PSU_ADRR	0x58
#define MLXCPLD_CTRL_REG_PWR_ADRR	0x64
#define MLXCPLD_CTRL_REG_FAN_ADRR	0x88
#define MLXCPLD_CTRL_EVENT_OFF		1
#define MLXCPLD_CTRL_MASK_OFF		2
#define MLXCPLD_CTRL_AGGR_MASK_OFF	1

#define MLXCPLD_CTRL_ATTRS_NUM		48
#define MLXCPLD_CTRL_LABEL_MAX_SIZE	24

/**
 * enum mlxcpld_ctrl_attr_type - sysfs attributes for hotplug events:
 * @MLXCPLD_CTRL_ATTR_TYPE_PSU: power supply unit attribute;
 * @MLXCPLD_CTRL_ATTR_TYPE_PWR: power cable attribute;
 * @MLXCPLD_CTRL_ATTR_TYPE_FAN: FAN drawer attribute;
 * @MLXCPLD_CTRL_ATTR_TYPE_RST: reset attribute;
 * @MLXCPLD_CTRL_ATTR_TYPE_RST_CAUSE: reset cause attribute;
 * @MLXCPLD_CTRL_ATTR_TYPE_MUX: mux attribute;
 * @MLXCPLD_CTRL_ATTR_TYPE_GPRW: general purpose read/write attribute;
 * @MLXCPLD_CTRL_ATTR_TYPE_GPRO: general purpose read only attribute;
 */
enum mlxcpld_ctrl_attr_type {
	MLXCPLD_CTRL_ATTR_TYPE_PSU,
	MLXCPLD_CTRL_ATTR_TYPE_PWR,
	MLXCPLD_CTRL_ATTR_TYPE_FAN,
	MLXCPLD_CTRL_ATTR_TYPE_RST,
	MLXCPLD_CTRL_ATTR_TYPE_RST_CAUSE,
	MLXCPLD_CTRL_ATTR_TYPE_MUX,
	MLXCPLD_CTRL_ATTR_TYPE_GPRW,
	MLXCPLD_CTRL_ATTR_TYPE_GPRO,
};

/**
 * struct mlxcpld_ctrl_data - attributes control data:
 * @label: attribute label;
 * @label: attribute register offset;
 * @mask: attribute mask;
 * @bit: attribute effective bit;
 */
struct mlxcpld_ctrl_data {
	char label[MLXCPLD_CTRL_LABEL_MAX_SIZE];
	u8 reg;
	u8 mask;
	u8 bit;
};

/**
 * struct mlxcpld_ctrl_priv_data - platform private data:
 * @irq: platform interrupt number;
 * @client: i2c slave device;
 * @plat: platform data;
 * @rst: reset signals;
 * @cause: reset cause info;
 * @mux: mux control;
 * @gprw: general purpose read/write registers;
 * @gpro: general purpose read only registers;
 * @hwmon: hwmon device;
 * @mlxcpld_ctrl_attr: sysfs attributes array;
 * @mlxcpld_ctrl_dev_attr: sysfs sensor device attribute array;
 * @group: sysfs attribute group;
 * @groups: list of sysfs attribute group for hwmon registration;
 * @dwork: delayed work template;
 * @lock: spin lock;
 * @aggr_cache: last value of aggregation register status;
 * @psu_cache: last value of PSU register status;
 * @pwr_cache: last value of power register status;
 * @fan_cache: last value of FAN register status;
 * @rst_count: number of available reset attributes;
 * @cause_count: number of available reset cuase attributes;
 * @gprw_count: number of available general purpose read-write attributes;
 * @gpro_count: number of available general purpose read only attributes;
 * @set_handler: if set - interrupt handler should be connected;
 */
struct mlxcpld_ctrl_priv_data {
	int irq;
	struct i2c_client *client;
	struct mlxcpld_hotplug_platform_data *plat;
	struct mlxcpld_ctrl_data *rst;
	struct mlxcpld_ctrl_data *cause;
	struct mlxcpld_ctrl_data *mux;
	struct mlxcpld_ctrl_data *gprw;
	struct mlxcpld_ctrl_data *gpro;
	struct device *hwmon;
	struct attribute *mlxcpld_ctrl_attr[MLXCPLD_CTRL_ATTRS_NUM + 1];
	struct sensor_device_attribute_2
			mlxcpld_ctrl_dev_attr[MLXCPLD_CTRL_ATTRS_NUM];
	struct attribute_group group;
	const struct attribute_group *groups[2];
	struct delayed_work dwork;
	spinlock_t lock;
	u8 aggr_cache;
	u8 psu_cache;
	u8 pwr_cache;
	u8 fan_cache;
	u8 rst_count;
	u8 cause_count;
	u8 mux_count;
	u8 gprw_count;
	u8 gpro_count;
	bool set_handler;
};

static ssize_t mlxcpld_ctrl_attr_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mlxcpld_ctrl_priv_data *priv = i2c_get_clientdata(client);
	int index = to_sensor_dev_attr_2(attr)->index;
	int nr = to_sensor_dev_attr_2(attr)->nr;
	struct mlxcpld_ctrl_data *data;
	u8 reg_val = 0;

	switch (nr) {
	case MLXCPLD_CTRL_ATTR_TYPE_PSU:
		/* Bit = 0 : PSU is present. */
		reg_val = !!!(i2c_smbus_read_byte_data(priv->client,
				priv->plat->psu_reg_offset) & BIT(index));
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_PWR:
		/* Bit = 1 : power cable is attached. */
		reg_val = !!(i2c_smbus_read_byte_data(priv->client,
				priv->plat->pwr_reg_offset) & BIT(index %
				priv->plat->pwr_count));
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_FAN:
		/* Bit = 0 : FAN is present. */
		reg_val = !!!(i2c_smbus_read_byte_data(priv->client,
				priv->plat->fan_reg_offset) & BIT(index %
				priv->plat->fan_count));
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_RST:
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_RST_CAUSE:
		data = priv->cause + BIT(index % priv->cause_count);
		reg_val = !!(i2c_smbus_read_byte_data(client, data->reg) |
			     data->mask);
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_MUX:
		data = priv->mux + BIT(index % priv->mux_count);
		reg_val = i2c_smbus_read_byte_data(client, data->reg) &
			  data->mask;
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_GPRW:
		data = priv->gprw + BIT(index % priv->gprw_count);
		reg_val = i2c_smbus_read_byte_data(client, data->reg);
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_GPRO:
		data = priv->gpro + BIT(index % priv->gpro_count);
		reg_val = i2c_smbus_read_byte_data(client, data->reg);
		break;
	}

	return sprintf(buf, "%u\n", reg_val);
}

static ssize_t mlxcpld_ctrl_attr_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mlxcpld_ctrl_priv_data *priv = i2c_get_clientdata(client);
	int index = to_sensor_dev_attr_2(attr)->index;
	int nr = to_sensor_dev_attr_2(attr)->nr;
	struct mlxcpld_ctrl_data *data;
	u8 mask = 0, reg_val = 0, val;
	int err;

	switch (nr) {
	case MLXCPLD_CTRL_ATTR_TYPE_PSU:
	case MLXCPLD_CTRL_ATTR_TYPE_PWR:
	case MLXCPLD_CTRL_ATTR_TYPE_FAN:
	case MLXCPLD_CTRL_ATTR_TYPE_RST_CAUSE:
	case MLXCPLD_CTRL_ATTR_TYPE_GPRO:
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_RST:
		err = kstrtou8(buf, 10, &val);
		if (err)
			return err;

		data = priv->rst + BIT(index % priv->rst_count);
		val = !!val;
		if (val)
			mask = ~data->mask;
		reg_val = i2c_smbus_read_byte_data(client, data->reg) |
			  data->mask;

		err = i2c_smbus_write_byte_data(client, data->reg,
				  reg_val | mask);
		if (err) {
			dev_err(&client->dev, "Failed to reset %s\n",
				data->label);
			return err;
		}
		break;

	case MLXCPLD_CTRL_ATTR_TYPE_MUX:
		err = kstrtou8(buf, 10, &val);
		if (err)
			return err;

		data = priv->mux + BIT(index % priv->mux_count);
		reg_val = i2c_smbus_read_byte_data(client, data->reg) &
			  data->mask;

		val = !!val;
		if (val)
			reg_val |= BIT(data->bit);
		else
			reg_val &= ~BIT(data->bit);

		err = i2c_smbus_write_byte_data(client, data->reg, reg_val &
						data->mask);
		if (err) {
			dev_err(&client->dev, "Failed to set %s\n",
				data->label);
			return err;
		}

		break;

	case MLXCPLD_CTRL_ATTR_TYPE_GPRW:
		err = kstrtou8(buf, 10, &val);
		if (err)
			return err;

		data = priv->gprw + BIT(index % priv->gprw_count);

		err = i2c_smbus_write_byte_data(client, data->reg, val);
		if (err) {
			dev_err(&client->dev, "Failed to set %s\n",
				data->label);
			return err;
		}
		break;
	}

	return len;

}

#define PRIV_ATTR(i) priv->mlxcpld_ctrl_attr[i]
#define PRIV_DEV_ATTR(i) priv->mlxcpld_ctrl_dev_attr[i]
static int mlxcpld_ctrl_attr_init(struct mlxcpld_ctrl_priv_data *priv)
{
	int num_attrs = priv->plat->psu_count + priv->plat->pwr_count +
			priv->plat->fan_count + priv->rst_count +
			priv->cause_count + priv->mux_count +
			priv->gprw_count + priv->gpro_count;
	struct mlxcpld_ctrl_data *rst = priv->rst;
	struct mlxcpld_ctrl_data *cause = priv->cause;
	struct mlxcpld_ctrl_data *mux = priv->mux;
	struct mlxcpld_ctrl_data *gprw = priv->gprw;
	struct mlxcpld_ctrl_data *gpro = priv->gpro;
	int i;

	priv->group.attrs = devm_kzalloc(&priv->client->dev, num_attrs *
					 sizeof(struct attribute *),
					 GFP_KERNEL);
	if (!priv->group.attrs)
		return -ENOMEM;

	for (i = 0; i < num_attrs; i++) {
		PRIV_ATTR(i) = &PRIV_DEV_ATTR(i).dev_attr.attr;

		if (i < priv->plat->psu_count) {
			PRIV_ATTR(i)->name = devm_kasprintf(&priv->client->dev,
						GFP_KERNEL, "psu%u", i + 1);
			PRIV_DEV_ATTR(i).nr = MLXCPLD_CTRL_ATTR_TYPE_PSU;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = 0444;
			PRIV_DEV_ATTR(i).dev_attr.show =
							mlxcpld_ctrl_attr_show;
		} else if (i < priv->plat->psu_count + priv->plat->pwr_count) {
			PRIV_ATTR(i)->name = devm_kasprintf(&priv->client->dev,
						GFP_KERNEL, "pwr%u", i %
						priv->plat->pwr_count + 1);
			PRIV_DEV_ATTR(i).nr = MLXCPLD_CTRL_ATTR_TYPE_PWR;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = 0444;
			PRIV_DEV_ATTR(i).dev_attr.show =
							mlxcpld_ctrl_attr_show;
		} else if (i < priv->plat->psu_count + priv->plat->pwr_count +
			   priv->plat->fan_count){
			PRIV_ATTR(i)->name = devm_kasprintf(&priv->client->dev,
						GFP_KERNEL, "fan%u", i %
						priv->plat->fan_count + 1);
			PRIV_DEV_ATTR(i).nr = MLXCPLD_CTRL_ATTR_TYPE_FAN;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = 0444;
			PRIV_DEV_ATTR(i).dev_attr.show =
							mlxcpld_ctrl_attr_show;
		} else if (i < priv->plat->psu_count + priv->plat->pwr_count +
			   priv->plat->fan_count + priv->rst_count) {
			PRIV_ATTR(i)->name = devm_kasprintf(&priv->client->dev,
						GFP_KERNEL, rst->label);
			PRIV_DEV_ATTR(i).nr = MLXCPLD_CTRL_ATTR_TYPE_RST;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = 0200;
			PRIV_DEV_ATTR(i).dev_attr.store =
						mlxcpld_ctrl_attr_store;
			rst++;
		} else if (i < priv->plat->psu_count + priv->plat->pwr_count +
			   priv->plat->fan_count + priv->rst_count +
			   priv->cause_count) {
			PRIV_ATTR(i)->name = devm_kasprintf(&priv->client->dev,
						GFP_KERNEL, cause->label);
			PRIV_DEV_ATTR(i).nr = MLXCPLD_CTRL_ATTR_TYPE_RST_CAUSE;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = 0444;
			PRIV_DEV_ATTR(i).dev_attr.show = mlxcpld_ctrl_attr_show;
			cause++;
		} else if (i < priv->plat->psu_count + priv->plat->pwr_count +
			   priv->plat->fan_count + priv->rst_count +
			   priv->cause_count + priv->mux_count) {
			PRIV_ATTR(i)->name = devm_kasprintf(&priv->client->dev,
						GFP_KERNEL, mux->label);
			PRIV_DEV_ATTR(i).nr = MLXCPLD_CTRL_ATTR_TYPE_MUX;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = 0644;
			PRIV_DEV_ATTR(i).dev_attr.show = mlxcpld_ctrl_attr_show;
			PRIV_DEV_ATTR(i).dev_attr.store =
						mlxcpld_ctrl_attr_store;
			mux++;
		} else if (i < priv->plat->psu_count + priv->plat->pwr_count +
			   priv->plat->fan_count + priv->rst_count +
			   priv->cause_count + priv->mux_count +
			   priv->gprw_count) {
			PRIV_ATTR(i)->name = devm_kasprintf(&priv->client->dev,
						GFP_KERNEL, gprw->label);
			PRIV_DEV_ATTR(i).nr = MLXCPLD_CTRL_ATTR_TYPE_GPRW;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = 0644;
			PRIV_DEV_ATTR(i).dev_attr.show = mlxcpld_ctrl_attr_show;
			PRIV_DEV_ATTR(i).dev_attr.store =
						mlxcpld_ctrl_attr_store;
			gprw++;
		} else {
			PRIV_ATTR(i)->name = devm_kasprintf(&priv->client->dev,
						GFP_KERNEL, gpro->label);
			PRIV_DEV_ATTR(i).nr = MLXCPLD_CTRL_ATTR_TYPE_GPRO;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = 0444;
			PRIV_DEV_ATTR(i).dev_attr.show = mlxcpld_ctrl_attr_show;
			gpro++;
		}

		if (!PRIV_ATTR(i)->name) {
			dev_err(&priv->client->dev, "Memory allocation failed for sysfs attribute %d.\n",
				i + 1);
			return -ENOMEM;
		}

		PRIV_DEV_ATTR(i).dev_attr.attr.name = PRIV_ATTR(i)->name;
		PRIV_DEV_ATTR(i).index = i;
		sysfs_attr_init(&PRIV_DEV_ATTR(i).dev_attr.attr);
	}

	priv->group.attrs = priv->mlxcpld_ctrl_attr;
	priv->groups[0] = &priv->group;
	priv->groups[1] = NULL;

	return 0;
}

static int mlxcpld_ctrl_device_create(struct device *dev,
					 struct mlxcpld_hotplug_device *item)
{
	item->adapter = i2c_get_adapter(item->bus);
	if (!item->adapter) {
		dev_err(dev, "Failed to get adapter for bus %d\n",
			item->bus);
		return -EFAULT;
	}

	item->client = i2c_new_device(item->adapter, &item->brdinfo);
	if (!item->client) {
		dev_err(dev, "Failed to create client %s at bus %d at addr 0x%02x\n",
			item->brdinfo.type, item->bus, item->brdinfo.addr);
		i2c_put_adapter(item->adapter);
		item->adapter = NULL;
		return -EFAULT;
	}

	return 0;
}

static void mlxcpld_ctrl_device_destroy(struct mlxcpld_hotplug_device *item)
{
	if (item->client) {
		i2c_unregister_device(item->client);
		item->client = NULL;
	}

	if (item->adapter) {
		i2c_put_adapter(item->adapter);
		item->adapter = NULL;
	}
}

static inline void
mlxcpld_ctrl_work_helper(struct device *dev, struct i2c_client *client,
			 struct mlxcpld_hotplug_device *item, u8 is_inverse,
			 u16 offset, u8 mask, u8 *cache)
{
	u8 val, asserted, clear = 0;
	int bit;

	/* Mask event. */
	i2c_smbus_write_byte_data(client, offset + MLXCPLD_CTRL_MASK_OFF,
				  clear);
	/* Read status. */
	val = i2c_smbus_read_byte_data(client, offset) & mask;
	asserted = *cache ^ val;
	*cache = val;

	/*
	 * Validate if item related to received signal type is valid.
	 * It should never happen, excepted the situation when some
	 * piece of hardware is broken. In such situation just produce
	 * error message and return. Caller must continue to handle the
	 * signals from other devices if any.
	 */
	if (unlikely(!item)) {
		dev_err(dev, "False signal is received: register at offset 0x%02x, mask 0x%02x.\n",
			offset, mask);
		return;
	}

	for_each_set_bit(bit, (unsigned long *)&asserted, 8) {
		if (val & BIT(bit)) {
			if (is_inverse)
				mlxcpld_ctrl_device_destroy(item + bit);
			else
				mlxcpld_ctrl_device_create(dev, item + bit);
		} else {
			if (is_inverse)
				mlxcpld_ctrl_device_create(dev, item + bit);
			else
				mlxcpld_ctrl_device_destroy(item + bit);
		}
	}

	/* Acknowledge event. */
	i2c_smbus_write_byte_data(client, offset + MLXCPLD_CTRL_EVENT_OFF,
				  clear);
	/* Unmask event. */
	i2c_smbus_write_byte_data(client, offset + MLXCPLD_CTRL_MASK_OFF,
				  mask);
}

/*
 * mlxcpld_ctrl_work_handler - performs traversing of CPLD interrupt
 * registers according to the below hierarchy schema:
 *
 *                   Aggregation registers (status/mask)
 * PSU registers:           *---*
 * *-----------------*      |   |
 * |status/event/mask|----->| * |
 * *-----------------*      |   |
 * Power registers:         |   |
 * *-----------------*      |   |
 * |status/event/mask|----->| * |---> CPU
 * *-----------------*      |   |
 * FAN registers:
 * *-----------------*      |   |
 * |status/event/mask|----->| * |
 * *-----------------*      |   |
 *                          *---*
 * In case some system changed are detected: FAN in/out, PSU in/out, power
 * cable attached/detached, relevant device is created or destroyed.
 */
static void mlxcpld_ctrl_work_handler(struct work_struct *work)
{
	struct mlxcpld_ctrl_priv_data *priv = container_of(work,
				struct mlxcpld_ctrl_priv_data, dwork.work);
	u8 val, aggr_asserted, clear = 0;
	unsigned long flags;

	/* Mask aggregation event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->top_aggr_offset +
				  MLXCPLD_CTRL_AGGR_MASK_OFF, clear);
	/* Read aggregation status. */
	val = i2c_smbus_read_byte_data(priv->client,
		priv->plat->top_aggr_offset) & priv->plat->top_aggr_mask;
	aggr_asserted = priv->aggr_cache ^ val;
	priv->aggr_cache = val;

	/* Handle PSU configuration changes. */
	if (aggr_asserted & priv->plat->top_aggr_psu_mask)
		mlxcpld_ctrl_work_helper(&priv->client->dev, priv->client,
					 priv->plat->psu, 1,
					 priv->plat->psu_reg_offset,
					 priv->plat->psu_mask,
					 &priv->psu_cache);

	/* Handle power cable configuration changes. */
	if (aggr_asserted & priv->plat->top_aggr_pwr_mask)
		mlxcpld_ctrl_work_helper(&priv->client->dev, priv->client,
					 priv->plat->pwr, 0,
					 priv->plat->pwr_reg_offset,
					 priv->plat->pwr_mask,
					 &priv->pwr_cache);

	/* Handle FAN configuration changes. */
	if (aggr_asserted & priv->plat->top_aggr_fan_mask)
		mlxcpld_ctrl_work_helper(&priv->client->dev, priv->client,
					 priv->plat->fan, 1,
					 priv->plat->fan_reg_offset,
					 priv->plat->fan_mask,
					 &priv->fan_cache);

	if (aggr_asserted) {
		spin_lock_irqsave(&priv->lock, flags);

		/*
		 * It is possible, that some signals have been inserted, while
		 * interrupt has been masked by mlxcpld_ctrl_work_handler.
		 * In this case such signals will be missed. In order to handle
		 * these signals delayed work is canceled and work task
		 * re-scheduled for immediate execution. It allows to handle
		 * missed signals, if any. In other case work handler just
		 * validates that no new signals have been received during
		 * masking.
		 */
		cancel_delayed_work(&priv->dwork);
		schedule_delayed_work(&priv->dwork, 0);

		spin_unlock_irqrestore(&priv->lock, flags);

		return;
	}

	/* Unmask aggregation event (no need acknowledge). */
	i2c_smbus_write_byte_data(priv->client, priv->plat->top_aggr_offset +
				  MLXCPLD_CTRL_AGGR_MASK_OFF,
				  priv->plat->top_aggr_mask);
}

static void mlxcpld_ctrl_set_irq(struct mlxcpld_ctrl_priv_data *priv)
{
	u8 clear = 0;

	/* Clear psu presense event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->psu_reg_offset +
				  MLXCPLD_CTRL_EVENT_OFF, clear);
	/* Set psu initial status as mask and unmask psu event. */
	priv->psu_cache = priv->plat->psu_mask;
	i2c_smbus_write_byte_data(priv->client, priv->plat->psu_reg_offset +
				  MLXCPLD_CTRL_MASK_OFF,
				  priv->plat->psu_mask);

	/* Clear power cable event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->pwr_reg_offset +
				  MLXCPLD_CTRL_EVENT_OFF, clear);
	/* Keep power initial status as zero and unmask power event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->pwr_reg_offset +
				  MLXCPLD_CTRL_MASK_OFF,
				  priv->plat->pwr_mask);

	/* Clear fan presense event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->fan_reg_offset +
				  MLXCPLD_CTRL_EVENT_OFF, clear);
	/* Set fan initial status as mask and unmask fan event. */
	priv->fan_cache = priv->plat->fan_mask;
	i2c_smbus_write_byte_data(priv->client, priv->plat->fan_reg_offset +
				  MLXCPLD_CTRL_MASK_OFF,
				  priv->plat->fan_mask);

	/* Keep aggregation initial status as zero and unmask events. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->top_aggr_offset +
				  MLXCPLD_CTRL_AGGR_MASK_OFF,
				  priv->plat->top_aggr_mask);

	/* Invoke work handler for initializing hot plug devices setting. */
	mlxcpld_ctrl_work_handler(&priv->dwork.work);

	enable_irq(priv->irq);
}

static void mlxcpld_ctrl_unset_irq(struct mlxcpld_ctrl_priv_data *priv)
{
	int i;
	u8 clear = 0;

	disable_irq(priv->irq);
	cancel_delayed_work_sync(&priv->dwork);

	/* Mask aggregation event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->top_aggr_offset +
				  MLXCPLD_CTRL_AGGR_MASK_OFF, clear);

	/* Mask psu presense event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->psu_reg_offset +
				  MLXCPLD_CTRL_MASK_OFF, clear);
	/* Clear psu presense event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->psu_reg_offset +
				  MLXCPLD_CTRL_EVENT_OFF, clear);

	/* Mask power cable event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->pwr_reg_offset +
				  MLXCPLD_CTRL_MASK_OFF, clear);
	/* Clear power cable event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->pwr_reg_offset +
				  MLXCPLD_CTRL_EVENT_OFF, clear);

	/* Mask fan presense event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->fan_reg_offset +
				  MLXCPLD_CTRL_MASK_OFF, clear);
	/* Clear fan presense event. */
	i2c_smbus_write_byte_data(priv->client, priv->plat->fan_reg_offset +
				  MLXCPLD_CTRL_EVENT_OFF, clear);

	/* Remove all the attached devices. */
	for (i = 0; i < priv->plat->psu_count; i++)
		mlxcpld_ctrl_device_destroy(priv->plat->psu + i);

	for (i = 0; i < priv->plat->pwr_count; i++)
		mlxcpld_ctrl_device_destroy(priv->plat->pwr + i);

	for (i = 0; i < priv->plat->fan_count; i++)
		mlxcpld_ctrl_device_destroy(priv->plat->fan + i);
}

static irqreturn_t mlxcpld_ctrl_irq_handler(int irq, void *dev)
{
	struct mlxcpld_ctrl_priv_data *priv =
				(struct mlxcpld_ctrl_priv_data *)dev;

	/* Schedule work task for immediate execution.*/
	schedule_delayed_work(&priv->dwork, 0);

	return IRQ_HANDLED;
}

static int
mlxcpld_ctrl_of_child_parser(struct device_node *np,
			     struct mlxcpld_hotplug_device *hpdev,
			      u8 *aggr_mask, u16 *reg, u8 *mask)
{
	struct device_node *child;
	const char *drvname;

	if (of_property_read_u8(np, "aggr_mask", aggr_mask))
		return -ENODEV;

	if (of_property_read_u16(np, "reg", reg))
		return -ENODEV;

	if (of_property_read_u8(np, "mask", mask))
		return -ENODEV;

	for_each_child_of_node(np, child) {
		if (of_property_read_string(child, "type", &drvname))
			return -ENODEV;
		strlcpy(hpdev->brdinfo.type, drvname, I2C_NAME_SIZE);

		if (of_property_read_u16(child, "bus", &hpdev->bus))
			return -ENODEV;

		if (of_property_read_u16(child, "addr", &hpdev->brdinfo.addr))
			return -ENODEV;
		hpdev++;
	}

	return 0;
}

static int
mlxcpld_ctrl_of_child_data_parser(struct device_node *np,
				  struct mlxcpld_ctrl_data *data)
{
	struct device_node *child;
	const char *label;

	for_each_child_of_node(np, child) {
		if (of_property_read_string(child, "label", &label))
			return -ENODEV;
		strlcpy(data->label, label, MLXCPLD_CTRL_LABEL_MAX_SIZE);

		if (of_property_read_u8(child, "reg", &data->reg))
			return -ENODEV;

		of_property_read_u8(child, "mask", &data->mask);
		of_property_read_u8(child, "bit", &data->bit);
		data++;
	}

	return 0;
}

static int
mlxcpld_ctrl_probe(struct i2c_client *client,
		   const struct i2c_device_id *id)
{
	struct device_node *child, *np = client->dev.of_node;
	struct mlxcpld_hotplug_platform_data *pdata;
	struct mlxcpld_ctrl_priv_data *priv;
	struct gpio_chip *gpiochip;
	int gpio;
	int err;

	if (!np)
		return -ENODEV;

	if (!of_device_is_compatible(np, "mellanox,mlxcpld-ctrl"))
		return -ENODEV;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->plat = pdata;

	of_property_read_u16(np, "top_aggr_offset",
				 &pdata->top_aggr_offset);

	of_property_read_u8(np, "top_aggr_mask", &pdata->top_aggr_mask);

	if (!of_property_read_u32(np, "top_aggr_irq", &priv->irq)) {

		err = devm_request_irq(&client->dev, priv->irq,
			       mlxcpld_ctrl_irq_handler, 0, client->name,
			       priv);
		if (err) {
			dev_err(&client->dev, "Failed to request irq: %d\n",
				err);
			return err;
		}
		disable_irq(priv->irq);
		priv->set_handler = true;
	}

	for_each_child_of_node(np, child) {
		/* Add Power Supply units. */
		if (!of_node_cmp(child->name, "psu")) {
			pdata->psu_count = of_get_child_count(child);
			pdata->psu = devm_kzalloc(&client->dev,
						sizeof(*pdata->psu) *
						pdata->psu_count, GFP_KERNEL);
			if (!pdata->psu)
				return -ENOMEM;

			err = mlxcpld_ctrl_of_child_parser(child, pdata->psu,
						&pdata->top_aggr_psu_mask,
						&pdata->psu_reg_offset,
						&pdata->psu_mask);

			if (err)
				return err;
		}

		/* Add Power Cables. */
		if (!of_node_cmp(child->name, "pwr")) {
			pdata->pwr_count = of_get_child_count(child);
			pdata->pwr = devm_kzalloc(&client->dev,
						sizeof(*pdata->pwr) *
						pdata->pwr_count, GFP_KERNEL);
			if (!pdata->pwr)
				return -ENOMEM;

			err = mlxcpld_ctrl_of_child_parser(child, pdata->pwr,
						&pdata->top_aggr_pwr_mask,
						&pdata->pwr_reg_offset,
						&pdata->pwr_mask);

			if (err)
				return err;
		}


		/* Add FAN units. */
		if (!of_node_cmp(child->name, "fan")) {
			pdata->fan_count = of_get_child_count(child);
			pdata->fan = devm_kzalloc(&client->dev,
						sizeof(*pdata->fan) *
						pdata->fan_count, GFP_KERNEL);
			if (!pdata->fan)
				return -ENOMEM;

			err = mlxcpld_ctrl_of_child_parser(child, pdata->fan,
						&pdata->top_aggr_fan_mask,
						&pdata->fan_reg_offset,
						&pdata->fan_mask);

			if (err)
				return err;
		}

		/* Add reset triggers. */
		if (!of_node_cmp(child->name, "reset")) {
			priv->rst_count = of_get_child_count(child);
			priv->rst = devm_kzalloc(&client->dev,
						sizeof(*priv->rst) *
						priv->rst_count, GFP_KERNEL);
			if (!priv->rst)
				return -ENOMEM;

			err = mlxcpld_ctrl_of_child_data_parser(child,
								priv->rst);

			if (err)
				return err;
		}

		/* Add reset cause attributes. */
		if (!of_node_cmp(child->name, "cause")) {
			priv->cause_count = of_get_child_count(child);
			priv->cause = devm_kzalloc(&client->dev,
						sizeof(*priv->cause) *
						priv->cause_count, GFP_KERNEL);
			if (!priv->cause)
				return -ENOMEM;

			err = mlxcpld_ctrl_of_child_data_parser(child,
								priv->cause);
			if (err)
				return err;
		}

		/* Add mux attributes. */
		if (!of_node_cmp(child->name, "mux")) {
			priv->mux_count = of_get_child_count(child);
			priv->mux = devm_kzalloc(&client->dev,
						sizeof(*priv->mux) *
						priv->mux_count, GFP_KERNEL);
			if (!priv->mux)
				return -ENOMEM;

			err = mlxcpld_ctrl_of_child_data_parser(child,
								priv->mux);
			if (err)
				return err;
		}

		/* Add general purpose read-write attributes. */
		if (!of_node_cmp(child->name, "gprw")) {
			priv->gprw_count = of_get_child_count(child);
			priv->gprw = devm_kzalloc(&client->dev,
						sizeof(*priv->gprw) *
						priv->gprw_count, GFP_KERNEL);
			if (!priv->gprw)
				return -ENOMEM;

			err = mlxcpld_ctrl_of_child_data_parser(child,
								priv->gprw);
			if (err)
				return err;
		}

		/* Add general purpose read only attributes. */
		if (!of_node_cmp(child->name, "gpro")) {
			priv->gpro_count = of_get_child_count(child);
			priv->gpro = devm_kzalloc(&client->dev,
						sizeof(*priv->gprw) *
						priv->gpro_count, GFP_KERNEL);
			if (!priv->gpro)
				return -ENOMEM;

			err = mlxcpld_ctrl_of_child_data_parser(child,
								priv->gpro);
			if (err)
				return err;
		}
	}

	priv->client = client;
	priv->plat = pdata;

	spin_lock_init(&priv->lock);

	err = mlxcpld_ctrl_attr_init(priv);
	if (err) {
		dev_err(&client->dev, "Failed to allocate attributes: %d\n",
			err);
		return err;
	}

	priv->hwmon = devm_hwmon_device_register_with_groups(&client->dev,
					"mlxcpld_ctrl", priv, priv->groups);
	if (IS_ERR(priv->hwmon)) {
		dev_err(&client->dev, "Failed to register hwmon device %ld\n",
			PTR_ERR(priv->hwmon));
		return PTR_ERR(priv->hwmon);
	}

	i2c_set_clientdata(client, priv);

	if (priv->set_handler) {
		INIT_DELAYED_WORK(&priv->dwork, mlxcpld_ctrl_work_handler);
		/* Perform initial interrupts setup. */
		mlxcpld_ctrl_set_irq(priv);
	}

	return 0;
}

static int mlxcpld_ctrl_remove(struct i2c_client *client)
{
	struct mlxcpld_ctrl_priv_data *priv = i2c_get_clientdata(client);

	/* Clean interrupts setup. */
	if (priv->set_handler)
		mlxcpld_ctrl_unset_irq(priv);

	return 0;
}

static const struct i2c_device_id mlxcpld_ctrl_id[] = {
	{ "mlxcpld_ctrl", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mlxcpld_ctrl_id);

static const struct of_device_id mlxcpld_ctrl_dt_match[] = {
	{ .compatible = "mellanox,mlxcpld-ctrl" },
	{ },
};
MODULE_DEVICE_TABLE(of, mlxcpld_ctrl_dt_match);

static struct i2c_driver mlxcpld_ctrl_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver		= {
	.name		= "mlxcpld-ctrl",
		.of_match_table = of_match_ptr(mlxcpld_ctrl_dt_match),
	},
	.probe		= mlxcpld_ctrl_probe,
	.remove		= mlxcpld_ctrl_remove,
	.id_table	= mlxcpld_ctrl_id,
};

module_i2c_driver(mlxcpld_ctrl_driver);

MODULE_AUTHOR("Vadim Pasternak <vadimp@mellanox.com>");
MODULE_DESCRIPTION("Mellanox CPLD control platform driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:mlxcpld-ctrl");

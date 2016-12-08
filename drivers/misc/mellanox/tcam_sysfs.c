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

#include "tcam.h"

/*
 * i2c sysfs
 */

#define show(value, reg)						\
static ssize_t								\
show_##value(struct device *dev, struct device_attribute *attr,		\
	     char *buf) {						\
	struct mlxnp_tcam_data *data = dev_get_drvdata(dev);		\
	u8 val;								\
	int err;							\
	err = data->read_value(data, reg, &val);			\
	if (err)							\
		return err;						\
	return sprintf(buf, "0x%x\n", val);				\
}

#define set(value, reg)							\
static ssize_t								\
set_##value(struct device *dev, struct device_attribute *attr,		\
	    const char *buf, size_t count) {				\
	struct mlxnp_tcam_data *data = dev_get_drvdata(dev);		\
	int val = simple_strtoul(buf, NULL, 16);			\
	int err;							\
	if (val >>8 )							\
		return -EINVAL;						\
	err = data->write_value(data, reg, val);			\
	if (err)							\
		return err;						\
	return count;							\
}

struct nps_reset_register {
	union {
		struct {
			u8 por_reset	: 1,
			hw_reset		: 1,
			__reserved		: 6;
		};
		u8 value;
	};
};

struct qsfp_sfp_registers {
	union {
		struct{
			union {
				struct {
					u8 qsfp0	: 8;
				};
				u8 reg0;
			};
			union {
				struct {
					u8 qsfp1	: 4,
					sfp0		: 2,
					__reserved0	: 2;
				};
				u8 reg1;
			};
		};
		union {
			struct {
				u16 qsfp		: 12,
				sfp				: 2,
				__reserved1		: 2;
			};
			u16 value;
		};
	};
};


/* cpld_reset */
show(cpld_reset, MLXNP_TCAM_CPLD_RESET);
set(cpld_reset, MLXNP_TCAM_CPLD_RESET);
static DEVICE_ATTR(cpld_reset, S_IWUSR | S_IRUGO,
		show_cpld_reset, set_cpld_reset);

/* TCAM exist */
static ssize_t show_tcam_exist(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mlxnp_tcam_data *data = dev_get_drvdata(dev);
	int res;

	mutex_lock(&data->sync_mutex);

	res = mlxnp_tcam_exist(data);
	if ((res != 1) && (res != 0))
		goto access_error;

	mutex_unlock(&data->sync_mutex);
	return sprintf(buf, "0x%x\n", res);
access_error:
	mutex_unlock(&data->sync_mutex);
	return res;
}

static DEVICE_ATTR(tcam_exist, S_IRUGO,
		show_tcam_exist, NULL);

int mlxnp_tcam_create_sysfs(struct mlxnp_tcam_data *data)
{
	int err;

	err = device_create_file(data->access_dev, &dev_attr_cpld_reset);
	if (err)
		return err;

	err = device_create_file(data->access_dev, &dev_attr_tcam_exist);
	if (err)
		return err;

	switch (data->type) {
	case tcam1_nl:
		break;
	case tcam2_nl:
		break;
	}

	return 0;
}

void mlxnp_tcam_remove_sysfs(struct mlxnp_tcam_data *data)
{
	device_remove_file(data->access_dev, &dev_attr_cpld_reset);

	device_remove_file(data->access_dev, &dev_attr_tcam_exist);

	switch (data->type) {
	case tcam1_nl:
		break;
	case tcam2_nl:
		break;
	}
}

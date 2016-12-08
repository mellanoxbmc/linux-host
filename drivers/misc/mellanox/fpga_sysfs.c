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

#include "fpga.h"

#define show(value, reg)						\
static ssize_t								\
mlxnp_fpga_show_##value(struct device *dev,				\
			struct device_attribute *attr, char *buf) {	\
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);		\
	u8 val;								\
	int err;							\
	err = data->read_value(data, reg, &val);			\
	if (err)							\
		return err;						\
	return sprintf(buf, "0x%x\n", val);				\
}

#define set(value, reg)							\
static ssize_t								\
mlxnp_fpga_set_##value(struct device *dev,				\
		       struct device_attribute *attr,			\
		       const char *buf, size_t count) {			\
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);		\
	int val = simple_strtoul(buf, NULL, 16);			\
	int err;							\
	if (val >> 8)							\
        	return -EINVAL;						\
	err = data->write_value(data, reg, val);			\
	if (err)							\
		return err;						\
	return count;							\
}

struct mlxnp_fpga_reset_register {
	union {
		struct {
			u8 por_reset	: 1,
			hw_reset	: 1,
			__reserve	: 6;
		};
		u8 value;
	};
};

struct mlxnp_fpga_qsfp_sfp_registers {
	union {
		struct {
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


/* board_type */
show(board_type, MLXNP_FPGA_BOARD_TYPE);
static DEVICE_ATTR(board_type, S_IRUGO, mlxnp_fpga_show_board_type, NULL);

/* board_subtype */
show(board_subtype, MLXNP_FPGA_BOARD_SUBTYPE0);
static DEVICE_ATTR(board_subtype, S_IRUGO, mlxnp_fpga_show_board_subtype,
		   NULL);

/* fpga_version */
show(fpga_version, MLXNP_FPGA_VERSION);
static DEVICE_ATTR(fpga_version, S_IRUGO, mlxnp_fpga_show_fpga_version, NULL);

/* bus_diag2 */
show(bus_diag2, MLXNP_FPGA_BUS_DIAG2);
set(bus_diag2, MLXNP_FPGA_BUS_DIAG2);
static DEVICE_ATTR(bus_diag2, S_IWUSR | S_IRUGO, mlxnp_fpga_show_bus_diag2,
		   mlxnp_fpga_set_bus_diag2);

/* um0_reset */
show(um0_reset, MLXNP_FPGA2_UM0_RESET);
set(um0_reset, MLXNP_FPGA2_UM0_RESET);
static DEVICE_ATTR(um0_reset, S_IWUSR | S_IRUGO, mlxnp_fpga_show_um0_reset,
		   mlxnp_fpga_set_um0_reset);

/* um1_reset */
show(um1_reset, MLXNP_FPGA1_UM1_RESET);
set(um1_reset, MLXNP_FPGA1_UM1_RESET);
static DEVICE_ATTR(um1_reset, S_IWUSR | S_IRUGO, mlxnp_fpga_show_um1_reset,
		   mlxnp_fpga_set_um1_reset);

/* power up request */
show(power_request, MLXNP_FPGA1_POWER_UP_REQ);
set(power_request, MLXNP_FPGA1_POWER_UP_REQ);
static DEVICE_ATTR(power_request, S_IWUSR | S_IRUGO,
		   mlxnp_fpga_show_power_request,
		   mlxnp_fpga_set_power_request);

/* QSFP exist */
static ssize_t
mlxnp_fpga_show_qsfp_exist(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_qsfp_sfp_registers exist_reg;
	int err;

	mutex_lock(&data->sync_mutex);

	exist_reg.value = 0;

	err = data->read_value(data, MLXNP_FPGA_QSFP0_EXIST, &exist_reg.reg0);
	if (err)
		goto access_error;
	err = data->read_value(data, MLXNP_FPGA_QSFP1_SFP_EXIST,
			       &exist_reg.reg1);
	if (err)
		goto access_error;

	mutex_unlock(&data->sync_mutex);

	return sprintf(buf, "0x%x\n", exist_reg.qsfp);

access_error:
	mutex_unlock(&data->sync_mutex);
	return err;
}

static DEVICE_ATTR(qsfp_exist, S_IRUGO,
		mlxnp_fpga_show_qsfp_exist, NULL);

/* SFP reset */
static ssize_t
mlxnp_fpga_show_sfp_exist(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_qsfp_sfp_registers exist_reg;
	int err;

	exist_reg.value = 0;

	err = data->read_value(data, MLXNP_FPGA_QSFP1_SFP_EXIST,
			       &exist_reg.reg1);
	if (err)
		goto access_error;

	return sprintf(buf, "0x%x\n", exist_reg.sfp);

access_error:
	return err;
}

static DEVICE_ATTR(sfp_exist, S_IRUGO, mlxnp_fpga_show_sfp_exist, NULL);


/* QSFP reset */
static ssize_t
mlxnp_fpga_show_qsfp_reset(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_qsfp_sfp_registers reset_reg;
	int err;

	mutex_lock(&data->sync_mutex);

	reset_reg.value = 0;

	err = data->read_value(data, MLXNP_FPGA_QSFP0_RESET, &reset_reg.reg0);
	if (err)
		goto access_error;

	err = data->read_value(data, MLXNP_FPGA_QSFP1_SFP_RESET,
			       &reset_reg.reg1);
	if (err)
		goto access_error;

	mutex_unlock(&data->sync_mutex);

	return sprintf(buf, "0x%x\n", reset_reg.qsfp);

access_error:
	mutex_unlock(&data->sync_mutex);
	return err;
}

static ssize_t
mlxnp_fpga_set_qsfp_reset(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_qsfp_sfp_registers reset_reg;
	int val = simple_strtoul(buf, NULL, 16);
	int err;

	if (val>>12)
		return -EINVAL;

	mutex_lock(&data->sync_mutex);
	reset_reg.value = 0;

	err = data->read_value(data, MLXNP_FPGA_QSFP0_RESET, &reset_reg.reg0);
	if (err)
		goto access_error;
	err = data->read_value(data, MLXNP_FPGA_QSFP1_SFP_RESET,
			       &reset_reg.reg1);
	if (err)
		goto access_error;

	reset_reg.qsfp = val;

	err = data->write_value(data, MLXNP_FPGA_QSFP0_RESET, reset_reg.reg0);
	if (err)
		goto access_error;

	err = data->write_value(data, MLXNP_FPGA_QSFP1_SFP_RESET,
				reset_reg.reg1);
	if (err)
		goto access_error;

	mutex_unlock(&data->sync_mutex);

	return count;

access_error:
	mutex_unlock(&data->sync_mutex);
	return err;
}

static DEVICE_ATTR(qsfp_reset, S_IWUSR | S_IRUGO, mlxnp_fpga_show_qsfp_reset,
		   mlxnp_fpga_set_qsfp_reset);

/* SFP reset */
static ssize_t
mlxnp_fpga_show_sfp_reset(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_qsfp_sfp_registers reset_reg;
	int err;

	reset_reg.value = 0;

	err = data->read_value(data, MLXNP_FPGA_QSFP1_SFP_RESET,
			       &reset_reg.reg1);
	if (err)
		goto access_error;

	return sprintf(buf, "0x%x\n", reset_reg.sfp);

access_error:
	return err;
}

static ssize_t
mlxnp_fpga_set_sfp_reset(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_qsfp_sfp_registers reset_reg;
	int val = simple_strtoul(buf, NULL, 16);
	int err;

	if (val>>2)
		return -EINVAL;

	mutex_lock(&data->sync_mutex);
	reset_reg.value = 0;

	err = data->read_value(data, MLXNP_FPGA_QSFP1_SFP_RESET,
			       &reset_reg.reg1);
	if (err)
		goto access_error;

	reset_reg.sfp = val;

	err = data->write_value(data, MLXNP_FPGA_QSFP1_SFP_RESET,
				reset_reg.reg1);
	if (err)
		goto access_error;

	mutex_unlock(&data->sync_mutex);

	return count;

access_error:
	mutex_unlock(&data->sync_mutex);
	return err;
}

static DEVICE_ATTR(sfp_reset, S_IWUSR | S_IRUGO, mlxnp_fpga_show_sfp_reset,
		   mlxnp_fpga_set_sfp_reset);

/* POR reset */
static ssize_t
mlxnp_fpga_show_por_reset(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_reset_register reset_reg;
	int err;

	err = data->read_value(data, MLXNP_FPGA2_NPS_RESET, &reset_reg.value);
	if (err)
		goto access_error;

	return sprintf(buf, "0x%x\n", reset_reg.por_reset);

access_error:
	return err;
}

static ssize_t
mlxnp_fpga_set_por_reset(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_reset_register reset_reg;
	int val = simple_strtoul(buf, NULL, 16);
	int err;

	if (val >> 1)
		return -EINVAL;

	mutex_lock(&data->sync_mutex);
	err = data->read_value(data, MLXNP_FPGA2_NPS_RESET, &reset_reg.value);
	if (err)
		goto access_error;
	reset_reg.por_reset = val;
	err = data->write_value(data, MLXNP_FPGA2_NPS_RESET, reset_reg.value);
	if (err)
		goto access_error;

	mutex_unlock(&data->sync_mutex);

	return count;

access_error:
	mutex_unlock(&data->sync_mutex);
	return err;
}

static DEVICE_ATTR(por_reset, S_IWUSR | S_IRUGO, mlxnp_fpga_show_por_reset, mlxnp_fpga_set_por_reset);

/* HW reset */
static ssize_t
mlxnp_fpga_show_hw_reset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_reset_register reset_reg;
	int err;

	err = data->read_value(data, MLXNP_FPGA2_NPS_RESET, &reset_reg.value);
	if (err)
		goto access_error;

	return sprintf(buf, "0x%x\n", reset_reg.hw_reset);

access_error:
	return err;
}

static ssize_t
mlxnp_fpga_set_hw_reset(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_reset_register reset_reg;
	int val = simple_strtoul(buf, NULL, 16);
	int err;

	if (val>>1)
		return -EINVAL;

	mutex_lock(&data->sync_mutex);
	err = data->read_value(data, MLXNP_FPGA2_NPS_RESET, &reset_reg.value);
	if (err)
		goto access_error;

	reset_reg.hw_reset = val;
	err = data->write_value(data, MLXNP_FPGA2_NPS_RESET, reset_reg.value);
	if (err)
		goto access_error;

	mutex_unlock(&data->sync_mutex);

	return count;

access_error:
	mutex_unlock(&data->sync_mutex);
	return err;
}

static DEVICE_ATTR(hw_reset, S_IWUSR | S_IRUGO, mlxnp_fpga_show_hw_reset,
		   mlxnp_fpga_set_hw_reset);

/* SPI enable */
static ssize_t
mlxnp_fpga_show_spi_enable(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_spi_ctrl_register spi_ctrl_reg;
	int err;

	err = data->read_value(data, MLXNP_FPGA2_SPI_FLASH_CTRL,
				    &spi_ctrl_reg.data);
	if (err)
		goto access_error;

	return sprintf(buf, "0x%x\n", spi_ctrl_reg.oe);

access_error:
	return err;
}

static ssize_t
mlxnp_fpga_set_spi_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mlxnp_fpga_data *data = dev_get_drvdata(dev);
	struct mlxnp_fpga_spi_ctrl_register spi_ctrl_reg;
	int val = simple_strtoul(buf, NULL, 16);
	int err;

	if (val >> 1)
		return -EINVAL;

	mutex_lock(&data->sync_mutex);

	err = data->read_value(data, MLXNP_FPGA2_SPI_FLASH_CTRL,
			       &spi_ctrl_reg.data);
	if (err)
		goto access_error;

	spi_ctrl_reg.oe = val;
	err = data->write_value(data, MLXNP_FPGA2_SPI_FLASH_CTRL,
				spi_ctrl_reg.data);
	if (err)
		goto access_error;

	mutex_unlock(&data->sync_mutex);

	return count;
access_error:
	mutex_unlock(&data->sync_mutex);
	return err;
}

static DEVICE_ATTR(spi_enable, S_IWUSR | S_IRUGO, mlxnp_fpga_show_spi_enable,
		   mlxnp_fpga_set_spi_enable);

int mlxnp_fpga_create_sysfs(struct mlxnp_fpga_data *data)
{
	int err;

	err = device_create_file(data->access_dev, &dev_attr_board_type);
	if (err)
		goto error;

	err = device_create_file(data->access_dev, &dev_attr_board_subtype);
	if (err)
		goto error;

	err = device_create_file(data->access_dev, &dev_attr_fpga_version);
	if (err)
		goto error;

	err = device_create_file(data->access_dev, &dev_attr_bus_diag2);
	if (err)
		goto error;

	err = device_create_file(data->access_dev, &dev_attr_qsfp_reset);
	if (err)
		goto error;

	err = device_create_file(data->access_dev, &dev_attr_sfp_reset);
	if (err)
		goto error;

	err = device_create_file(data->access_dev, &dev_attr_qsfp_exist);
	if (err)
		goto error;

	err = device_create_file(data->access_dev, &dev_attr_sfp_exist);
	if (err)
		goto error;


	switch (data->type) {
	case fpga1_nps400:
		err = device_create_file(data->access_dev,
					 &dev_attr_um1_reset);
		if (err)
			goto error;
		err = device_create_file(data->access_dev,
					 &dev_attr_power_request);
		if (err)
			goto error;
		break;

	case fpga2_nps400:
		err = device_create_file(data->access_dev,
					 &dev_attr_um0_reset);
		if (err)
			goto error;
		err = device_create_file(data->access_dev,
					 &dev_attr_por_reset);
		if (err)
			goto error;
		err = device_create_file(data->access_dev, &dev_attr_hw_reset);
		if (err)
			goto error;
		err = device_create_file(data->access_dev,
					 &dev_attr_spi_enable);
		if (err)
			goto error;
		break;
	}

	return 0;
error:
	mlxnp_fpga_remove_sysfs(data);
	return err;
}

void mlxnp_fpga_remove_sysfs(struct mlxnp_fpga_data *data)
{
	device_remove_file(data->access_dev, &dev_attr_board_type);
	device_remove_file(data->access_dev, &dev_attr_board_subtype);
	device_remove_file(data->access_dev, &dev_attr_fpga_version);
	device_remove_file(data->access_dev, &dev_attr_bus_diag2);
	device_remove_file(data->access_dev, &dev_attr_qsfp_reset);
	device_remove_file(data->access_dev, &dev_attr_sfp_reset);
	device_remove_file(data->access_dev, &dev_attr_qsfp_exist);
	device_remove_file(data->access_dev, &dev_attr_sfp_exist);

	switch (data->type) {
	case fpga1_nps400:
		device_remove_file(data->access_dev, &dev_attr_um1_reset);
		device_remove_file(data->access_dev, &dev_attr_power_request);
		break;
	case fpga2_nps400:
		device_remove_file(data->access_dev, &dev_attr_um0_reset);
		device_remove_file(data->access_dev, &dev_attr_por_reset);
		device_remove_file(data->access_dev, &dev_attr_hw_reset);
		device_remove_file(data->access_dev, &dev_attr_spi_enable);
		break;
	}
}

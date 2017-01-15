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

#include <linux/delay.h>
#include "fpga.h"
#include "tcam.h"
#include "mlxnp_ioctl.h"

/*
 * Char device
 */

#define MLXNP_TCAM_RESET_DELAY	10

struct mlxnp_tcam_serdes_reg {
	u16 addr;
	u16 value;
};

struct mlxnp_fpga_data_avail mlxnp_fpga_avail = {NULL, NULL};

static inline int
get_relevant_fpga_data(struct mlxnp_tcam_data *tcam_data,
		       struct mlxnp_fpga_data *fpga_data)
{
	int res = 0;
	enum mlxnp_tcam_type kind = tcam_data->type;

	switch (kind) {
	case tcam1_nl:
		if (!mlxnp_fpga_avail.fpga1) {
			dev_err(tcam_data->access_dev, "Can't find fpga1 data\n");

			return -ENODEV;
		}
		*fpga_data = *mlxnp_fpga_avail.fpga1;
		break;

	case tcam2_nl:
		if (!mlxnp_fpga_avail.fpga2) {
			dev_err(tcam_data->access_dev, "Can't find fpga2 data\n");

			return -ENODEV;
		}
		*fpga_data = *mlxnp_fpga_avail.fpga2;
		break;

	default:
		res = -EINVAL;
	}

	return res;
}

static long _mlxnp_tcam_get_reg(struct mlxnp_tcam_data *data,
		unsigned int dev, unsigned int addr, unsigned int *value)
{
	struct mlxnp_fpga_data fpga_data;
	int res = 0;

	res = get_relevant_fpga_data(data, &fpga_data);
	if (res)
		goto cleanup;

	dev_dbg(data->io_dev, "get device=0x%x, addr=0x%x\n", dev, addr);
	res = mlxnp_fpga_get_tcam_reg(&fpga_data, dev, addr, value);

cleanup:
	return res;
}

static long
_mlxnp_tcam_set_reg(struct mlxnp_tcam_data *data, unsigned int dev,
		    unsigned int addr, unsigned int value)
{
	struct mlxnp_fpga_data fpga_data;
	int res = 0;

	res = get_relevant_fpga_data(data, &fpga_data);
	if (res)
		goto cleanup;

	dev_dbg(data->io_dev, "set 0x%x to device=0x%x, addr=0x%x\n", value, dev,
		addr);
	res = mlxnp_fpga_set_tcam_reg(&fpga_data, dev, addr, value);

cleanup:
	return res;
}

static long
mlxnp_tcam_get_reg(struct mlxnp_tcam_data *data,
		   struct mlxnp_tcam_reg *user_reg)
{
	struct mlxnp_tcam_reg kernel_reg;
	int res = 0;

	mutex_lock(&data->sync_mutex);

	if (copy_from_user(&kernel_reg, user_reg, sizeof(kernel_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

	res = _mlxnp_tcam_get_reg(data, kernel_reg.device, kernel_reg.address,
				&kernel_reg.data);

	if (copy_to_user(user_reg, &kernel_reg, sizeof(kernel_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

cleanup:
	mutex_unlock(&data->sync_mutex);
	return res;
}

static long mlxnp_tcam_set_reg(struct mlxnp_tcam_data *data,
			       struct mlxnp_tcam_reg *user_reg)
{
	struct mlxnp_tcam_reg kernel_reg;
	int res = 0;

	mutex_lock(&data->sync_mutex);

	if (copy_from_user(&kernel_reg, user_reg, sizeof(kernel_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

	res = _mlxnp_tcam_set_reg(data, kernel_reg.device, kernel_reg.address,
				  kernel_reg.data);

cleanup:
	mutex_unlock(&data->sync_mutex);
	return res;
}

static long
mlxnp_tcam_set_reset(struct mlxnp_tcam_data *data, int s_reset, int c_reset)
{
	unsigned int reset_value;
	int res = 0;

	reset_value = MLXNP_TCAM_CPLD_SRST_DEASSERT |
		      MLXNP_TCAM_CPLD_CRST_DEASSERT;

	/* SRST */
	if (s_reset)
		reset_value &= ~MLXNP_TCAM_CPLD_SRST_DEASSERT;

	/* CRST */
	if (c_reset)
		reset_value &= ~MLXNP_TCAM_CPLD_CRST_DEASSERT;

	dev_dbg(data->io_dev, "set reset srst=%d, crst=%d\n", s_reset, c_reset);
	res = data->write_value(data, MLXNP_TCAM_CPLD_RESET, reset_value);
	if (res)
		return res;

	usleep_range(MLXNP_TCAM_RESET_DELAY, MLXNP_TCAM_RESET_DELAY * 2);

	return res;
}

static long
mlxnp_tcam_reset(struct mlxnp_tcam_data *data,
		 struct mlxnp_tcam_reset_info *user_tcam_info)
{
	int res = 0;
	struct mlxnp_tcam_reset_info kernel_tcam_info;

	mutex_lock(&data->sync_mutex);

	if (copy_from_user(&kernel_tcam_info, user_tcam_info,
			   sizeof(struct mlxnp_tcam_reset_info))) {
		res = -EFAULT;
		goto error;
	}

	res = mlxnp_tcam_set_reset(data, kernel_tcam_info.s_reset,
				 kernel_tcam_info.c_reset);
	if (res)
		goto error;

	mutex_unlock(&data->sync_mutex);

	return 0;
error:
	dev_err(data->io_dev, "Failed to reset\n");
	mutex_unlock(&data->sync_mutex);
	return res;
}

int mlxnp_tcam_exist(struct mlxnp_tcam_data *data)
{
	struct mlxnp_fpga_data fpga_data;
	u8 board_subtype;
	int res = 0;

	res = get_relevant_fpga_data(data, &fpga_data);
	if (res)
		return res;

	res = fpga_data.read_value(&fpga_data, MLXNP_FPGA_BOARD_SUBTYPE0,
				   &board_subtype);
	if (res)
		return res;

	switch (data->type) {
	case tcam1_nl:
	case tcam2_nl:
		if (board_subtype == 1)
			res = 1;
		else
			res = 0;
		break;
	default:
		res = -EINVAL;
	}

	return res;
}

static int mlxnp_tcam_dev_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct mlxnp_tcam_data *data = dev_get_drvdata(miscdev->this_device);

	if (mlxnp_tcam_exist(data) != 1)
		return -ENXIO;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	return 0;
}

static long mlxnp_tcam_dev_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct miscdevice *miscdev = file->private_data;
	struct mlxnp_tcam_data *data = dev_get_drvdata(miscdev->this_device);
	long ret = 0;

	switch (cmd) {
	case MLXNP_TCAM_SET_REG:
		ret = mlxnp_tcam_set_reg(data, (struct mlxnp_tcam_reg *)arg);
		break;

	case MLXNP_TCAM_GET_REG:
		ret = mlxnp_tcam_get_reg(data, (struct mlxnp_tcam_reg *)arg);
		break;

	case MLXNP_TCAM_RESET:
		ret = mlxnp_tcam_reset(data,
				      (struct mlxnp_tcam_reset_info *)arg);
		break;

	default:
		return -ENOTTY;
	}

	return ret;
}

static const struct file_operations mlxnp_tcam_fops = {
	.open		= mlxnp_tcam_dev_open,
	.unlocked_ioctl	= mlxnp_tcam_dev_ioctl,
};

int mlxnp_tcam_dev_init(struct mlxnp_tcam_data *data)
{
	int err;

	data->miscdev.minor	= MISC_DYNAMIC_MINOR;
	data->miscdev.name	= data->name;
	data->miscdev.fops	= &mlxnp_tcam_fops;

	err = misc_register(&data->miscdev);
	if (err)
		return err;

	dev_set_drvdata(data->miscdev.this_device, data);

	return 0;
}

int mlxnp_tcam_dev_deinit(struct mlxnp_tcam_data *data)
{
	misc_deregister(&data->miscdev);

	return 0;
}

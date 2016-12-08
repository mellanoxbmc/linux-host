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
#include "mlxnp_ioctl.h"

/*
 * Char device
 */

static long
mlxnp_fpga_get_reg(struct mlxnp_fpga_data *data,
		   struct mlxnp_fpga_reg *user_reg)
{
	struct mlxnp_fpga_reg kernel_reg;
	u8 value;
	int res = 0;

	mutex_lock(&data->sync_mutex);

	if (copy_from_user(&kernel_reg, user_reg, sizeof(*user_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

	res = data->read_value(data, kernel_reg.address, &value);
	if (res)
		goto cleanup;

	kernel_reg.data = value;

	if (copy_to_user(user_reg, &kernel_reg,
			 sizeof(struct mlxnp_fpga_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

cleanup:
	mutex_unlock(&data->sync_mutex);
	return res;
}

static long
mlxnp_fpga_set_reg(struct mlxnp_fpga_data *data,
		   struct mlxnp_fpga_reg *user_reg)
{
	struct mlxnp_fpga_reg kernel_reg;
	int res = 0;

	mutex_lock(&data->sync_mutex);

	if (copy_from_user(&kernel_reg, user_reg, sizeof(*user_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

	res = data->write_value(data, kernel_reg.address, kernel_reg.data);

cleanup:
	mutex_unlock(&data->sync_mutex);
	return res;
}

static int mlxnp_fpga_dev_open(struct inode *inode, struct file *file)
{
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	return 0;
}

static long
mlxnp_fpga_dev_ioctl(struct file *file, unsigned int cmd,
		     unsigned long arg)
{
	struct miscdevice *miscdev = file->private_data;
	struct mlxnp_fpga_data *data = dev_get_drvdata(miscdev->this_device);
	long ret = 0;

	switch (cmd) {
	case MLXNP_FPGA_SET_REG:
		ret = mlxnp_fpga_set_reg(data, (struct mlxnp_fpga_reg*)arg);
		break;

	case MLXNP_FPGA_GET_REG:
		ret = mlxnp_fpga_get_reg(data, (struct mlxnp_fpga_reg*)arg);
		break;

	case MLXNP_FPGA_SET_NP_REG:
		ret = mlxnp_fpga_set_np_reg(data, (struct mlxnp_fpga_reg*)arg);
		break;

	case MLXNP_FPGA_GET_NP_REG:
		ret = mlxnp_fpga_get_np_reg(data, (struct mlxnp_fpga_reg*)arg);
		break;

	default:
		return -ENOTTY;
	}

	return ret;
}

static const struct file_operations mlxnp_fpga_fops = {
	.open		= mlxnp_fpga_dev_open,
	.unlocked_ioctl	= mlxnp_fpga_dev_ioctl,
};

int mlxnp_fpga_dev_init(struct mlxnp_fpga_data *data)
{
	int err;

	data->miscdev.minor	= MISC_DYNAMIC_MINOR;
	data->miscdev.name	= data->name;
	data->miscdev.fops	= &mlxnp_fpga_fops;

	err = misc_register(&data->miscdev);
	if (err)
		goto error;

	dev_set_drvdata(data->miscdev.this_device, data);

	return 0;
error:
	mlxnp_fpga_dev_deinit(data);
	return err;
}

int mlxnp_fpga_dev_deinit(struct mlxnp_fpga_data *data)
{
	misc_deregister(&data->miscdev);

	return 0;
}

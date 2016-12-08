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
#include <linux/platform_device.h>
#include "tcam.h"
#include "lpc.h"

static const struct platform_device_id mlxnp_tcam_idtable[] = {
        { .name = "tcam1_nl" },
        { .name = "tcam2_nl" },
        { }
};

static int
mlxnp_tcam_lpc_read_value(struct mlxnp_tcam_data *data, u8 reg, u8 *value)
{
	mlxnp_lpc_read_comm(reg, value, 1);

	return 0;
}

static int
mlxnp_tcam_lpc_read_values(struct mlxnp_tcam_data *data, u8 reg, u8 length,
			   u8 *values)
{
	mlxnp_lpc_read_comm(reg, values, length);

	return 0;
}

static int
mlxnp_tcam_lpc_write_value(struct mlxnp_tcam_data *data, u8 reg, u8 value)
{
	mlxnp_lpc_write_comm(reg, &value, 1);

	return 0;
}

static int
mlxnp_tcam_lpc_write_values(struct mlxnp_tcam_data *data, u8 reg, u8 length,
			    const u8 *values)
{
	mlxnp_lpc_write_comm(reg, values, length);

	return 0;
}

static int mlxnp_tcam_probe(struct platform_device *pdev)
{
	struct mlxnp_tcam_data *data;
	int err;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->kind = "lpc";
	data->access_dev = &pdev->dev;
	data->u.lpc.pdev = pdev;
	data->name = pdev->name;
	data->type = pdev->id;
	mutex_init(&data->sync_mutex);

	err = mlxnp_tcam_create_sysfs(data);
	if (err)
		goto sysfs_create_error;

	err = mlxnp_tcam_dev_init(data);
	if (err)
		goto dev_init_error;

	data->read_value = mlxnp_tcam_lpc_read_value;
	data->read_values = mlxnp_tcam_lpc_read_values;
	data->write_value = mlxnp_tcam_lpc_write_value;
	data->write_values = mlxnp_tcam_lpc_write_values;

	platform_set_drvdata(pdev, data);

	return 0;

dev_init_error:
	mlxnp_tcam_dev_deinit(data);
sysfs_create_error:
	mlxnp_tcam_remove_sysfs(data);

	return err;
}

static int mlxnp_tcam_remove(struct platform_device *pdev)
{
	struct mlxnp_tcam_data *data = platform_get_drvdata(pdev);

	mlxnp_tcam_dev_deinit(data);
	mlxnp_tcam_remove_sysfs(data);

	return 0;
}

static struct platform_driver mlxnp_tcam_lpc_driver = {
	.driver = {
		.name = "mlxnp-tcam",
	},
	.probe = mlxnp_tcam_probe,
	.remove = mlxnp_tcam_remove,
	.id_table = mlxnp_tcam_idtable,
};

module_platform_driver(mlxnp_tcam_lpc_driver);

MODULE_AUTHOR("Mellanox Technologies");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Mellanox NPS400 TCAM LPC driver");
MODULE_DEVICE_TABLE(platfrom, mlxnp_tcam_idtable);

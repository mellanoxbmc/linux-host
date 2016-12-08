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

static const struct i2c_device_id mlxnp_fpga_idtable[] = {
	{ "fpga1_nps400", fpga1_nps400 },
	{ "fpga2_nps400", fpga2_nps400 },
	{ }
};

struct mlxnp_fpga_data_avail mlxnp_fpga_avail = {NULL, NULL};
EXPORT_SYMBOL(mlxnp_fpga_avail);

static int
mlxnp_fpga_i2c_read_value(struct mlxnp_fpga_data *fpga_data, u8 reg, u8 *value)
{
	struct i2c_client *client = fpga_data->u.i2c.client;
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return ret;
	*value = ret;

	return 0;
}

static int
mlxnp_fpga_i2c_read_values(struct mlxnp_fpga_data *fpga_data, u8 reg,
			   u8 length, u8 *values)
{
	struct i2c_client *client = fpga_data->u.i2c.client;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);

	if (ret < 0)
		return ret;

	return 0;
}

static int
mlxnp_fpga_i2c_write_value(struct mlxnp_fpga_data *fpga_data, u8 reg, u8 value)
{
	struct i2c_client *client = fpga_data->u.i2c.client;

	return i2c_smbus_write_byte_data(client, reg, value);
}

static int
mlxnp_fpga_i2c_write_values(struct mlxnp_fpga_data *fpga_data, u8 reg,
			    u8 length, const u8 *values)
{
	struct i2c_client *client = fpga_data->u.i2c.client;

	return i2c_smbus_write_i2c_block_data(client, reg, length, values);
}

static int
mlxnp_fpga_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mlxnp_fpga_data *data;
	int err;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->kind = "i2c";
	data->access_dev = &client->dev;
	data->u.i2c.client = client;
	data->name = client->name;
	data->type = id->driver_data;
	mutex_init(&data->sync_mutex);

	i2c_set_clientdata(client, data);
	
	err = mlxnp_fpga_create_sysfs(data);
	if (err)
		goto init_error0;

	err = mlxnp_fpga_dev_init(data);
	if (err)
		goto init_error1;

	/* add fpga data to available fpgas */
	switch (data->type) {
	case fpga1_nps400:
		if (mlxnp_fpga_avail.fpga1) {
			err = -EADDRINUSE;
			goto init_error3;
		}
		mlxnp_fpga_avail.fpga1 = data;
		break;

	case fpga2_nps400:
		err = mlxnp_fpga_spi_init(data);
		if (err)
			goto init_error2;

		if (mlxnp_fpga_avail.fpga2) {
			err = -EADDRINUSE;
			goto init_error3;
		}
		mlxnp_fpga_avail.fpga2 = data;
		break;

	default:
		err = -EINVAL;
		goto init_error3;
	}

	data->read_value = mlxnp_fpga_i2c_read_value;
	data->read_values = mlxnp_fpga_i2c_read_values;
	data->write_value = mlxnp_fpga_i2c_write_value;
	data->write_values = mlxnp_fpga_i2c_write_values;

	return 0;

init_error3:
	mlxnp_fpga_spi_deinit(data);
init_error2:
	mlxnp_fpga_dev_deinit(data);
init_error1:
	mlxnp_fpga_remove_sysfs(data);
init_error0:
	return err;
}

static int mlxnp_fpga_remove(struct i2c_client *client)
{
	struct mlxnp_fpga_data *data = i2c_get_clientdata(client);

	mutex_destroy(&data->sync_mutex);

	/* specific deinit */
	switch (data->type) {
	case fpga1_nps400:
		break;
	case fpga2_nps400:
		mlxnp_fpga_spi_deinit(data);
		break;
	}

	mlxnp_fpga_dev_deinit(data);
	mlxnp_fpga_remove_sysfs(data);

	/* remove fpga data from available fpgas */
	switch (data->type) {
	case fpga1_nps400:
		mlxnp_fpga_avail.fpga1 = NULL;
		break;
	case fpga2_nps400:
		mlxnp_fpga_avail.fpga2 = NULL;
		break;
	}

	return 0;
}

static struct i2c_driver mlxnp_fpga_driver = {
	.driver = {
		.name	= "mlxnp_fpga",
	},

	.id_table	= mlxnp_fpga_idtable,
	.probe		= mlxnp_fpga_probe,
	.remove		= mlxnp_fpga_remove,
};

module_i2c_driver(mlxnp_fpga_driver);

MODULE_AUTHOR("Mellanox Technologies");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Mellanox NPS400 FPGA I2C driver");
MODULE_DEVICE_TABLE(i2c, mlxnp_fpga_idtable);

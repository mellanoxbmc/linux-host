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

#ifndef __MLXNP_GBOX_H_
#define __MLXNP_GBOX_H_

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include "fpga.h"
#include "mlxnp_ioctl.h"

#define MLXNP_GBOX_NUM_OF_LANES			16

/* TCAM CPLD registers */
#define MLXNP_GBOX_CPLD_RESET			0x2
#define MLXNP_GBOX_CPLD_SRST_DEASSERT		0x1
#define MLXNP_GBOX_CPLD_CRST_DEASSERT		0x2

/* TCAM device registers */
/* Device 1 */
#define MLXNP_GBOX_MDIO_DEVICE_1		0x1
#define MLXNP_GBOX_GLOBAL_DEV_CONFIG_16_31	0xE

enum mlxnp_gbox_type {
	gbox1_nl,
	gbox2_nl,
};

/* TCAM private data */
struct mlxnp_gbox_data {
	const char *kind;
	const char *name;
	enum mlxnp_gbox_type type;
	/* character device */
	struct miscdevice miscdev;
	struct device *io_dev;
	struct device *access_dev;
        union {
                struct {
			struct i2c_client *client;
                } i2c;
                struct {
                        struct platform_device *pdev;
                } lpc;
        } u;
	/* sync */
	struct mutex sync_mutex;
	/* access */
	int (*read_value)(struct mlxnp_gbox_data *tcam_data, u8 reg,
			  u8 *value);
	int (*read_values)(struct mlxnp_gbox_data *tcam_data, u8 reg,
			   u8 length, u8 *values);
	int (*write_value)(struct mlxnp_gbox_data *tcam_data, u8 reg,
			   u8 value);
	int (*write_values)(struct mlxnp_gbox_data *tcam_data, u8 reg,
			    u8 length, const u8 *values);
};

/* sysfs */
int mlxnp_gbox_create_sysfs(struct mlxnp_gbox_data *data);
void mlxnp_gbox_remove_sysfs(struct mlxnp_gbox_data *data);

/* dev */
int mlxnp_gbox_exist(struct mlxnp_gbox_data *data);
int mlxnp_gbox_dev_init(struct mlxnp_gbox_data *data);
int mlxnp_gbox_dev_deinit(struct mlxnp_gbox_data *data);

/* fpga */
extern long mlxnp_fpga_get_gbox_reg(struct mlxnp_fpga_data *data,
				    unsigned int dev, unsigned int addr,
				    unsigned int *value);
extern long mlxnp_fpga_set_gbox_reg(struct mlxnp_fpga_data *data,
				    unsigned int dev, unsigned int addr,
				    unsigned int value);

#endif /* __MLXNP_GBOX_H_ */

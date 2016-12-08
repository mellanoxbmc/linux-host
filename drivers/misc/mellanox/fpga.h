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

#ifndef __MLXNP_FPGA_H_
#define __MLXNP_FPGA_H_

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/flash.h>
#include "mlxnp_ioctl.h"

/* Common FPGA Registers */
#define MLXNP_FPGA_BOARD_TYPE		0x00
#define MLXNP_FPGA_BOARD_SUBTYPE0	0x01
#define MLXNP_FPGA_VERSION		0x03
#define MLXNP_FPGA_BUS_DIAG2		0x06
#define MLXNP_FPGA2_NPS_RESET		0x10
#define MLXNP_FPGA2_UM0_RESET		0x15
#define MLXNP_FPGA_QSFP0_RESET		0x18
#define MLXNP_FPGA_QSFP1_SFP_RESET	0x19
#define MLXNP_FPGA_QSFP0_EXIST		0x1a
#define MLXNP_FPGA_QSFP1_SFP_EXIST	0x1b

/* FPGA1 Registers */
#define MLXNP_FPGA1_POWER_UP_REQ	0x09
#define MLXNP_FPGA1_UM1_RESET		0x15

/* FPGA2 Registers */
#define MLXNP_FPGA2_SPI_FLASH_PROG	0x0f
#define MLXNP_FPGA2_SPI_FLASH_CTRL	0x37

struct mlxnp_fpga_spi_ctrl_register {
	union {
		struct {
			u8 oe		: 1,
			__reserved	: 6,
			reset		: 1;
		};
		u8 data;
	};
};

enum mlxnp_fpga_type {
	fpga1_nps400,
	fpga2_nps400,
};

/* FPGA private data */
struct mlxnp_fpga_data {
	const char *kind;
	const char *name;
	enum mlxnp_fpga_type type;
	/* for now, this must be first */
	 struct spi_bitbang bitbang;
	 struct spi_device *dataflash;
	 struct spi_board_info	board_info;
	/* character device */
	struct miscdevice miscdev;
	//struct device *io_dev;
	struct device *access_dev;
	const struct i2c_device_id *id;
        union {
                struct {
			struct i2c_client *client;
                } i2c;
                struct {
                        struct platform_device *pdev;
                } lpc;
        } u;
	/* spi controller */
	struct spi_master *master;
	/* sync */
	struct mutex sync_mutex;
	/* access */
	int (*read_value)(struct mlxnp_fpga_data *tcam_data, u8 reg,
			  u8 *value);
	int (*read_values)(struct mlxnp_fpga_data *tcam_data, u8 reg,
			   u8 length, u8 *values);
	int (*write_value)(struct mlxnp_fpga_data *tcam_data, u8 reg,
			   u8 value);
	int (*write_values)(struct mlxnp_fpga_data *tcam_data, u8 reg,
			    u8 length, const u8 *values);
};

struct mlxnp_fpga_data_avail {
	struct mlxnp_fpga_data *fpga1;
	struct mlxnp_fpga_data *fpga2;
};

/* sysfs */
int mlxnp_fpga_create_sysfs(struct mlxnp_fpga_data *data);
void mlxnp_fpga_remove_sysfs(struct mlxnp_fpga_data *data);
/* dev */
int mlxnp_fpga_dev_init(struct mlxnp_fpga_data *data);
int mlxnp_fpga_dev_deinit(struct mlxnp_fpga_data *data);
long mlxnp_fpga_get_np_reg(struct mlxnp_fpga_data *data,
			   struct mlxnp_fpga_reg *user_reg);
long mlxnp_fpga_set_np_reg(struct mlxnp_fpga_data *data,
			   struct mlxnp_fpga_reg *user_reg);
extern long mlxnp_fpga_get_tcam_reg(struct mlxnp_fpga_data *data,
			     unsigned int dev, unsigned int addr,
			     unsigned int *value);
extern long mlxnp_fpga_set_tcam_reg(struct mlxnp_fpga_data *data,
			     unsigned int dev, unsigned int addr,
			     unsigned int value);
extern long mlxnp_fpga_get_gbox_reg(struct mlxnp_fpga_data *data,
				    unsigned int dev, unsigned int addr,
				    unsigned int *value);
extern long mlxnp_fpga_set_gbox_reg(struct mlxnp_fpga_data *data,
				    unsigned int dev, unsigned int addr,
				    unsigned int value);
/* spi */
int mlxnp_fpga_spi_init(struct mlxnp_fpga_data *data);
void mlxnp_fpga_spi_deinit(struct mlxnp_fpga_data *data);

#endif /* __MLXNP_FPGA_H_ */

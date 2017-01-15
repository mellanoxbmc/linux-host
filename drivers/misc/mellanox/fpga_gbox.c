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

/* MDC/MDIO GBOX */
#define MLNXP_GBOX_WAIT_RETRIES		25
#define MLNXP_GBOX_WAIT_INTERVAL	200
#define MLNXP_GBOX_PORT_ADDRESS		0x10

/* FPGA GBOX registers */
#define MLXNP_FPGA_FPGA_GBOX_PORT_ADDR	0x50
#define MLXNP_FPGA_FPGA_GBOX_DATA_LSB	0x54
#define MLXNP_FPGA_FPGA_GBOX_STATUS	0x57

struct mlxnp_fpga_gbox_registers {
	union {
		struct {
			u8 port_addr	: 5,
			__port_reserved	: 3;
		};
		u8 port;
	};
	union {
		struct {
			u8 device_addr	: 5,
			__dev_reserved	: 3;
		};
		u8 device;
	};
	union {
		struct {
			u16 address_lsb	: 8,
			address_msb	: 8;
		};
		u16 address;
	};
	union {
		struct {
			u16 data_lsb	: 8,
			data_msb	: 8;
		};
		u16 data;
	};
	union {
		struct {
			u8 wr_rd_n	: 1,
			__cntl_reserved	: 7;
		};
		u8 control;
	};
	union {
		struct {
			u8 ready		: 1,
			valid			: 1,
			__sts_reserved	: 6;
		};
		u8 status;
	};
} __attribute__((packed));

static long
mlxnp_fpga_wait_gbox_status(struct mlxnp_fpga_data *data,
			    struct mlxnp_fpga_gbox_registers *gbox_regs,
			    int wait_for_ready, int wait_for_valid)
{
	int retries = MLNXP_GBOX_WAIT_RETRIES;
	int err;

	/* wait for gbox mdc/mdio status */
	while (retries--) {
		err = data->read_value(data, MLXNP_FPGA_FPGA_GBOX_STATUS,
				&gbox_regs->status);
		if (err)
			return err;
		if ((!wait_for_ready || gbox_regs->ready) &&
				(!wait_for_valid || gbox_regs->valid))
			break;
		usleep_range(MLNXP_GBOX_WAIT_INTERVAL,
			     MLNXP_GBOX_WAIT_INTERVAL * 2);
	}

	if (wait_for_ready) {
		if (!gbox_regs->ready) {
			dev_err(data->access_dev, "%s: MDC/MDIO is not ready\n",
				data->id->name);
			return -EBUSY;
		}
	}

	if (wait_for_valid) {
		if (!gbox_regs->valid) {
			dev_err(data->access_dev, "%s: MDC/MDIO is not valid\n",
				data->id->name);
			return -EBUSY;
		}
	}

	return 0;
}

static long
mlxnp_fpga_read_gbox_data(struct mlxnp_fpga_data *data,
			  struct mlxnp_fpga_gbox_registers *gbox_regs)
{
	int err;

	/* read gbox mdc/mdio data */
	err = data->read_values(data, MLXNP_FPGA_FPGA_GBOX_DATA_LSB, 2,
				(u8 *)&gbox_regs->data);
	if (err)
		return err;

	return 0;
}

static long
_mlxnp_fpga_get_gbox_reg(struct mlxnp_fpga_data *data, unsigned int dev,
			 unsigned int addr, unsigned int *value)
{
	struct mlxnp_fpga_gbox_registers gbox_regs;
	int res = 0;

	/* initialize GBOX registers */
	gbox_regs.port = 0;
	gbox_regs.device = 0;
	gbox_regs.address = 0;
	gbox_regs.data = 0;
	gbox_regs.control = 0;
	gbox_regs.status = 0;

	/* wait for GBOX readiness */
	res = mlxnp_fpga_wait_gbox_status(data, &gbox_regs, 1, 0);
	if (res)
		goto cleanup;

	/* update GBOX registers with transaction details */
	gbox_regs.port = MLNXP_GBOX_PORT_ADDRESS;
	gbox_regs.device = dev;
	gbox_regs.address = addr;

	/* set transaction to READ */
	gbox_regs.wr_rd_n = 0;

	/* write GBOX registers values */
	res = data->write_values(data, MLXNP_FPGA_FPGA_GBOX_PORT_ADDR, 7,
				 (u8 *)&gbox_regs);
	if (res)
		goto cleanup;

	/* wait for GBOX transaction finish (ready, valid) */
	res = mlxnp_fpga_wait_gbox_status(data, &gbox_regs, 1, 1);
	if (res)
		goto cleanup;

	/* read GBOX data */
	res = mlxnp_fpga_read_gbox_data(data, &gbox_regs);
	if (res)
		goto cleanup;
	*value = gbox_regs.data;

	dev_dbg(data->access_dev, "%s MDC/MDIO: read 0x%x from 0x%x,0x%x\n",
		data->id->name, *value, gbox_regs.device, gbox_regs.address);

cleanup:
	return res;
}

static long
_mlxnp_fpga_set_gbox_reg(struct mlxnp_fpga_data *data, unsigned int dev,
			  unsigned int addr, unsigned int value)
{
	struct mlxnp_fpga_gbox_registers gbox_regs;
	int res = 0;

	/* initialize GBOX registers */
	gbox_regs.port = 0;
	gbox_regs.device = 0;
	gbox_regs.address = 0;
	gbox_regs.data = 0;
	gbox_regs.control = 0;
	gbox_regs.status = 0;

	/* wait for GBOX readiness */
	res = mlxnp_fpga_wait_gbox_status(data, &gbox_regs, 1, 0);
	if (res)
		goto cleanup;

	/* update GBOX registers with transaction details */
	gbox_regs.port = MLNXP_GBOX_PORT_ADDRESS;
	gbox_regs.device = dev;
	gbox_regs.address = addr;
	gbox_regs.data = value;

	/* set transaction to WRITE */
	gbox_regs.wr_rd_n = 1;

	/* write GBOX registers values */
	res = data->write_values(data, MLXNP_FPGA_FPGA_GBOX_PORT_ADDR, 7,
				 (u8 *)&gbox_regs);
	if (res)
		goto cleanup;

	/* wait for GBOX transaction finish (ready) */
	res = mlxnp_fpga_wait_gbox_status(data, &gbox_regs, 1, 0);
	if (res)
		goto cleanup;

	dev_dbg(data->access_dev, "%s MDC/MDIO: written 0x%x to 0x%x,0x%x\n",
		data->id->name, value, gbox_regs.device, gbox_regs.address);

cleanup:
	return res;
}


long mlxnp_fpga_get_gbox_reg(struct mlxnp_fpga_data *data, unsigned int dev,
			     unsigned int addr, unsigned int *value)
{
	int res = 0;

	mutex_lock(&data->sync_mutex);

	res = _mlxnp_fpga_get_gbox_reg(data, dev, addr, value);

	mutex_unlock(&data->sync_mutex);
	return res;
}

long mlxnp_fpga_set_gbox_reg(struct mlxnp_fpga_data *data, unsigned int dev,
			     unsigned int addr, unsigned int value)
{
	int res = 0;

	mutex_lock(&data->sync_mutex);

	res = _mlxnp_fpga_set_gbox_reg(data, dev, addr, value);

	mutex_unlock(&data->sync_mutex);

	return res;
}

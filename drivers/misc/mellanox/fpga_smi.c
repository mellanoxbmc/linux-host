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

#define MLXNP_FPGA_CLUSTER_REG_ADDR_MASK		0xFF000000
#define MLXNP_FPGA_CLUSTER_REG_MASK			0xF6000000
#define MLXNP_FPGA_IS_CLUSTER_REGISTER(address) ((address & \
				       MLXNP_FPGA_CLUSTER_REG_ADDR_MASK) == \
				       MLXNP_FPGA_CLUSTER_REG_MASK)
#define MLXNP_FPGA_NON_CLUSTER(BLOCK,REG) (MLXNP_FPGA_CLUSTER_REG_MASK + (((1 \
				<< 14) *  BLOCK))  + (((1 <<2 ) * REG)))

/* SMI */
#define MLXNP_FPGA_SMI_WAIT_RETRIES		25
#define MLXNP_FPGA_SMI_WAIT_INTERVAL		200

/* FPGA SMI registers */
#define MLXNP_FPGA_FPGA2_SMI_ADDR0		0x90
#define MLXNP_FPGA_FPGA2_SMI_DATA0		0x94
#define MLXNP_FPGA_FPGA2_SMI_STATUS		0x99

/* NPS indirect access registers and values */
#define MLXNP_FPGA_EZCB_CRQI_BLOCK		0x41f
#define MLXNP_FPGA_REG_CFGB_GLB_MSID_CFG \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_EZCB_CRQI_BLOCK, \
					       0x471)

#define MLXNP_FPGA_CFGB_BLOCK			0x41a
#define MLXNP_FPGA_REG_CFGB_IND_LITTLE_END \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x229)
#define MLXNP_FPGA_REG_IND_REQUEST_DATA0 \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x200)
#define MLXNP_FPGA_REG_IND_REQUEST_DATA1 \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x201)
#define MLXNP_FPGA_REG_IND_REQUEST_DATA2 \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x202)
#define MLXNP_FPGA_REG_IND_REQUEST_DATA3 \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x203)
#define MLXNP_FPGA_REG_IND_CMD \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x226)
#define MLXNP_FPGA_REG_IND_CMD_STS \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x227)
#define MLXNP_FPGA_REG_IND_RESPONSE_DATA0 \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x300)
#define MLXNP_FPGA_REG_IND_RESPONSE_DATA3 \
			MLXNP_FPGA_NON_CLUSTER(MLXNP_FPGA_CFGB_BLOCK, 0x303)

#define MLXNP_FPGA_VAL_CFGB_GLB_MSID_CFG	0xffe00000
#define MLXNP_FPGA_VAL_IND_REQUEST_DATA0_GET	0x860000
#define MLXNP_FPGA_VAL_IND_REQUEST_DATA0_SET	0x060000
#define MLXNP_FPGA_VAL_IND_REQUEST_DATA2	0xa000000
#define MLXNP_FPGA_VAL_IND_CMD			0x2

#define MLXNP_FPGA_IND_CMD_RETRIES		4
#define MLXNP_FPGA_IND_CMD_INTERVAL		500

struct mlxnp_fpga_smi_registers {
	union {
		struct {
			u32 addr0	: 8,
			addr1		: 8,
			addr2		: 8,
			addr3		: 8;
		};
		u32 addr;
	};
	union {
		struct {
			u32 data0	: 8,
			data1		: 8,
			data2		: 8,
			data3		: 8;
		};
		u32 data;
	};
	union {
		struct {
			u8 block_sel	: 1,
			__unused	: 1,
			wr_rd_n		: 1,
			__reserved1	: 5;
		};
		u8 control;
	};
	union {
		struct {
			u8 ready	: 1,
			valid		: 1,
			err		: 1,
			__reserved2	: 5;
		};
		u8 status;
	};
} __attribute__((packed));

static long
mlxnp_fpga_wait_smi_status(struct mlxnp_fpga_data *data,
			   struct mlxnp_fpga_smi_registers *smi_regs,
			   int wait_for_ready, int wait_for_valid,
			   int check_err)
{
	int retries = MLXNP_FPGA_SMI_WAIT_RETRIES;
	int err;

	/* wait for SMI status */
	while (retries--) {
		err = data->read_value(data, MLXNP_FPGA_FPGA2_SMI_STATUS,
				       &smi_regs->status);
		if (err)
			return err;
		if ((!wait_for_ready || smi_regs->ready) &&
				(!wait_for_valid || smi_regs->valid))
			break;
		usleep_range(MLXNP_FPGA_SMI_WAIT_INTERVAL,
			     MLXNP_FPGA_SMI_WAIT_INTERVAL * 2);
	}

	if (wait_for_ready) {
		if (!smi_regs->ready) {
			dev_err(data->access_dev, "%s: SMI is not ready\n",
				data->id->name);
			return -EBUSY;
		}
	}
	if (wait_for_valid) {
		if (!smi_regs->valid) {
			dev_err(data->access_dev, "%s: SMI is not valid\n",
				data->id->name);
			return -EBUSY;
		}
	}

	if (check_err && smi_regs->err) {
		dev_err(data->access_dev, "%s: SMI status indicates a request error\n",
			data->id->name);
		return -EFAULT;
	}
	return 0;
}

static long
mlxnp_fpga_read_smi_data(struct mlxnp_fpga_data *data,
			 struct mlxnp_fpga_smi_registers *smi_regs)
{
	int err;

	/* read SMI data */
	err = data->read_values(data, MLXNP_FPGA_FPGA2_SMI_DATA0, 4,
				(u8 *)&smi_regs->data);
	if (err)
		return err;

	return 0;
}

static long
_mlxnp_fpga_get_np_reg(struct mlxnp_fpga_data *data, unsigned int addr,
		       unsigned int *value)
{
	struct mlxnp_fpga_smi_registers smi_regs;
	int res = 0;

	/* initialize SMI registers */
	smi_regs.addr = 0;
	smi_regs.data = 0;
	smi_regs.control = 0;
	smi_regs.status = 0;

	/* wait for SMI readiness */
	res = mlxnp_fpga_wait_smi_status(data, &smi_regs, 1, 0, 0);
	if (res)
		goto cleanup;

	/* update SMI registers with transaction details */
	smi_regs.addr = addr;

	/* set transaction to READ */
	smi_regs.wr_rd_n = 0;

	/* write SMI registers values */
	res = data->write_values(data, MLXNP_FPGA_FPGA2_SMI_ADDR0, 9,
				 (u8 *)&smi_regs);
	if (res)
		goto cleanup;

	/* wait for SMI transaction finish (ready, valid and no errors) */
	res = mlxnp_fpga_wait_smi_status(data, &smi_regs, 1, 1, 1);
	if (res)
		goto cleanup;

	/* read SMI data */
	res = mlxnp_fpga_read_smi_data(data, &smi_regs);
	if (res)
		goto cleanup;
	*value = smi_regs.data;

	pr_debug("%s SMI: read 0x%x from 0x%x\n", data->id->name, *value, addr);

cleanup:
	return res;
}

static long
_mlxnp_fpga_set_np_reg(struct mlxnp_fpga_data *data, unsigned int addr,
		       unsigned int value)
{
	struct mlxnp_fpga_smi_registers smi_regs;
	int res = 0;

	/* initialize SMI registers */
	smi_regs.addr = 0;
	smi_regs.data = 0;
	smi_regs.control = 0;
	smi_regs.status = 0;

	/* wait for SMI readiness */
	res = mlxnp_fpga_wait_smi_status(data, &smi_regs, 1, 0, 0);
	if (res)
		goto cleanup;

	/* update SMI registers with transaction details */
	smi_regs.addr = addr;
	smi_regs.data = value;

	/* set transaction to WRITE */
	smi_regs.wr_rd_n = 1;

	/* write SMI registers values */
	res = data->write_values(data, MLXNP_FPGA_FPGA2_SMI_ADDR0, 9,
				 (u8 *)&smi_regs);
	if (res)
		goto cleanup;

	/* wait for SMI transaction finish (ready, no errors) */
	res = mlxnp_fpga_wait_smi_status(data, &smi_regs, 1, 0, 1);
	if (res)
		goto cleanup;

	pr_debug("%s SMI: written 0x%x to 0x%x\n", data->id->name, value, addr);

cleanup:
	return res;
}

static long mlxnp_fpga_wait_ind_cmd(struct mlxnp_fpga_data *data)
{
	int retries = MLXNP_FPGA_IND_CMD_RETRIES;
	unsigned int value = 0;
	int res = 0;

	while (retries--) {
		res = _mlxnp_fpga_get_np_reg(data, MLXNP_FPGA_REG_IND_CMD_STS, &value);
		if (res) {
			dev_err(data->access_dev, "%s: failed to read indirect access status from NPS\n",
				data->id->name);
			return res;
		}
		if (value == 1)
			break;
		usleep_range(MLXNP_FPGA_IND_CMD_INTERVAL, MLXNP_FPGA_IND_CMD_INTERVAL * 2);
	}

	if (value != 1) {
		dev_err(data->access_dev, "%s: indirect access to NPS timed out\n",
			data->id->name);
		return -EBUSY;
	}
	return 0;
}

long mlxnp_fpga_get_np_reg(struct mlxnp_fpga_data *data,
		struct mlxnp_fpga_reg *user_reg)
{
	struct mlxnp_fpga_reg kernel_reg;
	int res = 0;

	mutex_lock(&data->sync_mutex);

	if (copy_from_user(&kernel_reg, user_reg,
			sizeof(*user_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

	if (MLXNP_FPGA_IS_CLUSTER_REGISTER(kernel_reg.address)) {
		res = _mlxnp_fpga_set_np_reg(data,
				MLXNP_FPGA_REG_CFGB_GLB_MSID_CFG,
				MLXNP_FPGA_VAL_CFGB_GLB_MSID_CFG);
		if (res)
			goto cleanup;

		res = _mlxnp_fpga_get_np_reg(data,
					     MLXNP_FPGA_REG_CFGB_IND_LITTLE_END,
					     &kernel_reg.data);
		if (res)
			goto cleanup;

		if (kernel_reg.data) {
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA3,
					MLXNP_FPGA_VAL_IND_REQUEST_DATA0_GET);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA2,
					kernel_reg.address);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA1,
					MLXNP_FPGA_VAL_IND_REQUEST_DATA2);
			if (res)
				goto cleanup;
		} else {
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA0,
					MLXNP_FPGA_VAL_IND_REQUEST_DATA0_GET);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data, MLXNP_FPGA_REG_IND_REQUEST_DATA1,
					kernel_reg.address);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data, MLXNP_FPGA_REG_IND_REQUEST_DATA2,
					MLXNP_FPGA_VAL_IND_REQUEST_DATA2);
			if (res)
				goto cleanup;
		}

		res = _mlxnp_fpga_set_np_reg(data, MLXNP_FPGA_REG_IND_CMD,
					MLXNP_FPGA_VAL_IND_CMD);
		if (res)
			goto cleanup;

		res = mlxnp_fpga_wait_ind_cmd(data);
		if (res)
			goto cleanup;

		if(kernel_reg.data){
			res = _mlxnp_fpga_get_np_reg(data,
					MLXNP_FPGA_REG_IND_RESPONSE_DATA0,
					&kernel_reg.data);
			if (res)
				goto cleanup;
		} else {
			res = _mlxnp_fpga_get_np_reg(data,
					MLXNP_FPGA_REG_IND_RESPONSE_DATA3,
					&kernel_reg.data);
			if (res)
				goto cleanup;
		}
	} else
		res = _mlxnp_fpga_get_np_reg(data, kernel_reg.address,
					     &kernel_reg.data);

	if (copy_to_user(user_reg, &kernel_reg, sizeof(*user_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

cleanup:
	mutex_unlock(&data->sync_mutex);
	return res;
}

long mlxnp_fpga_set_np_reg(struct mlxnp_fpga_data *data,
			   struct mlxnp_fpga_reg *user_reg)
{
	struct mlxnp_fpga_reg kernel_reg;
	int res = 0;
	unsigned int ldata;

	mutex_lock(&data->sync_mutex);

	if (copy_from_user(&kernel_reg, user_reg,
			sizeof(*user_reg))) {
		res = -EFAULT;
		goto cleanup;
	}

	if ( MLXNP_FPGA_IS_CLUSTER_REGISTER(kernel_reg.address) ) {
		res = _mlxnp_fpga_set_np_reg(data,
					     MLXNP_FPGA_REG_CFGB_GLB_MSID_CFG,
					     MLXNP_FPGA_VAL_CFGB_GLB_MSID_CFG);
		if (res)
			goto cleanup;

		res = _mlxnp_fpga_get_np_reg(data,
					     MLXNP_FPGA_REG_CFGB_IND_LITTLE_END,
					     &ldata);
		if (res)
			goto cleanup;

		if (ldata) { /* little endian */
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA3,
					MLXNP_FPGA_VAL_IND_REQUEST_DATA0_SET);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA2,
					kernel_reg.address);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA1,
					MLXNP_FPGA_VAL_IND_REQUEST_DATA2);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA0,
					kernel_reg.data);
			if (res)
				goto cleanup;
		} else {
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA0,
					MLXNP_FPGA_VAL_IND_REQUEST_DATA0_SET);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA1,
					kernel_reg.address);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA2,
					MLXNP_FPGA_VAL_IND_REQUEST_DATA2);
			if (res)
				goto cleanup;
			res = _mlxnp_fpga_set_np_reg(data,
					MLXNP_FPGA_REG_IND_REQUEST_DATA3,
					kernel_reg.data);
			if (res)
				goto cleanup;
		}

		res = _mlxnp_fpga_set_np_reg(data, MLXNP_FPGA_REG_IND_CMD,
					     MLXNP_FPGA_VAL_IND_CMD);
		if (res)
			goto cleanup;
		res = mlxnp_fpga_wait_ind_cmd(data);
		if (res)
			goto cleanup;

	} else
		res = _mlxnp_fpga_set_np_reg(data, kernel_reg.address,
					     kernel_reg.data);

cleanup:
	mutex_unlock(&data->sync_mutex);
	return res;
}

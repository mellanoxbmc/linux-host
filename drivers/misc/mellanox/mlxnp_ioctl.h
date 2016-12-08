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

#ifndef __MLXNP_IOCTL_H_
#define __MLXNP_IOCTL_H_

#include <linux/ioctl.h>

struct mlxnp_tcam_reg {
	unsigned int device;
	unsigned int address;
	unsigned int data;
};

enum mlxnp_tcam_freq {
	freq_10_3,
	freq_12_5,
};

struct mlxnp_tcam_reset_info {
	int s_reset;
	int c_reset;
};

struct mlxnp_fpga_reg {
	unsigned int address;
	unsigned int data;
};

struct mlxnp_gbox_reg {
	unsigned int device;
	unsigned int address;
	unsigned int data;
};

struct mlxnp_gbox_reset_info {
	int s_reset;
	int c_reset;
};

#define MLXNP_TCAM_BASE		0xe3
#define MLXNP_TCAM_SET_REG	_IOW(MLXNP_TCAM_BASE, 1, struct mlxnp_tcam_reg)
#define MLXNP_TCAM_GET_REG	_IOR(MLXNP_TCAM_BASE, 2, struct mlxnp_tcam_reg)
#define MLXNP_TCAM_RESET	_IOW(MLXNP_TCAM_BASE, 3, struct mlxnp_tcam_reset_info)

#define MLXNP_FPGA_BASE		0xe4
#define MLXNP_FPGA_SET_REG	_IOW(MLXNP_FPGA_BASE, 1, struct mlxnp_fpga_reg)
#define MLXNP_FPGA_GET_REG	_IOR(MLXNP_FPGA_BASE, 2, struct mlxnp_fpga_reg)
#define MLXNP_FPGA_SET_NP_REG	_IOW(MLXNP_FPGA_BASE, 3, struct mlxnp_fpga_reg)
#define MLXNP_FPGA_GET_NP_REG	_IOR(MLXNP_FPGA_BASE, 4, struct mlxnp_fpga_reg)

#define MLXNP_GBOX_BASE		0xe6
#define MLXNP_GBOX_SET_REG	_IOW(MLXNP_GBOX_BASE, 1, struct mlxnp_gbox_reg)
#define MLXNP_GBOX_GET_REG	_IOR(MLXNP_GBOX_BASE, 2, struct mlxnp_gbox_reg)
#define MLXNP_GBOX_RESET	_IOW(MLXNP_GBOX_BASE, 3, \
				struct mlxnp_gbox_reset_info)

#endif /* __MLXNP_IOCTL_H_ */

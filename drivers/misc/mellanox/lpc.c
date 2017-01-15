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

#include <linux/io.h>
#include "lpc.h"

#define MLXPNP_LPC_REG_BASE_ADRR     0x2000 /* LPC bus access */

static void mlxnp_lpc_write_buf(const u8 *data, u8 len, u32 addr)
{
	int i;

	for (i = 0; i < len - len % 4; i += 4)
		outl(*(u32 *)(data + i), addr + i);
	for (; i < len; ++i)
		outb(*(data + i), addr + i);
}

static void mlxnp_lpc_read_buf(u8 *data, u8 len, u32 addr)
{
	int i;

	for (i = 0; i < len - len % 4; i += 4)
		*(u32 *)(data + i) = inl(addr + i);
	for (; i < len; ++i)
		*(data + i) = inb(addr + i);
}

void mlxnp_lpc_read_comm(u32 addr, u8 *data, u8 datalen)
{
	addr += MLXPNP_LPC_REG_BASE_ADRR;

	switch (datalen) {
	case 1:
		*(data) = inb(addr);
		break;
	case 2:
		*((u16 *)data) = inw(addr);
		break;
	case 3:
		*((u16 *)data) = inw(addr);
		*(data + 2) = inb(addr + 2);
		break;
	case 4:
		*((u32 *)data) = inl(addr);
		break;
	default:
		mlxnp_lpc_read_buf(data, datalen, addr);
		break;
	}
}

void mlxnp_lpc_write_comm(u32 addr, const u8 *data, u8 datalen)
{
	addr += MLXPNP_LPC_REG_BASE_ADRR;

	switch (datalen) {
	case 1:
		outb(*(data), addr);
		break;
	case 2:
		outw(*((u16 *)data), addr);
		break;
	case 3:
		outw(*((u16 *)data), addr);
		outb(*(data + 2), addr + 2);
		break;
	case 4:
		outl(*((u32 *)data), addr);
		break;
	default:
		mlxnp_lpc_write_buf(data, datalen, addr);
		break;
	}
}


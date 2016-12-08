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

#ifndef __LINUX_PLATFORM_DATA_PWM_MLXCPLD_H
#define __LINUX_PLATFORM_DATA_PWM_MLXCPLD_H

/**
 * struct pwm_mlxcpld_params - single item data:
 * @min: minimum value;
 * @max: maximum value;
 * @data_regs: device board information;
 *
 * Structure represents FAN tacho or pwm device data.
 */
struct pwm_mlxcpld_params {
	u16 min;
	u16 max;
	u16 round;
	u16 scale;
	u8 data_reg;
};

/**
 * struct pwm_mlxcpld_platform_data_item - single item data:
 * @count: counter of same type objects;
 * @param: item setting info;
 *
 * Structure represents FAN tacho or pwm device data.
 */
struct pwm_mlxcpld_platform_data_item {
	u8 count;
	struct pwm_mlxcpld_params *param;
};

/**
 * struct mlxcpld_hotplug_platform_data - device platform data:
 * @tacho - tacho data;
 * @pwm - pwm data;
 * @fan_max_state - number of colling levels;
 * @fan_cooling_levels - pointer to cooling level array;
 *
 * Structure represents FAN platform data.
 */
struct pwm_mlxcpld_platform_data {
	struct pwm_mlxcpld_platform_data_item tacho;
	struct pwm_mlxcpld_platform_data_item pwm;
	unsigned int fan_max_state;
	unsigned int *fan_cooling_levels;
};

#endif /* __LINUX_PLATFORM_DATA_PWM_MLXCPLD_H */

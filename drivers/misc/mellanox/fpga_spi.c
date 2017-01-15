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

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/mtd/partitions.h>
#include "fpga.h"

/* FPGA SPI */
#define MLXNP_FPGA_SPI_CS_DEACTIVATE	1
#define MLXNP_FPGA_SPI_CS_ACTIVATE	0
#define MLXNP_FPGA_SPI_MAX_SPEED_HZ	(15 * 1000 * 1000)

struct mlxnp_fpga_spi_register {
	union {
		struct {
			u8 clk		: 1,
			mosi		: 1,
			cs		: 1,
			miso		: 1,
			__reserved	: 4;
		};
		u8 data;
	};
};

struct mlxnp_fpga_spi_register current_spi_register;

static bool mlxnp_fpga_spi_enabled(struct mlxnp_fpga_data *data)
{
	struct mlxnp_fpga_spi_ctrl_register reg;

	if (data->read_value(data, MLXNP_FPGA2_SPI_FLASH_CTRL, &reg.data))
		return false;

	return reg.oe;
}

static int
mlxnp_fpga_set_spi_register(struct spi_device *spidev,
			    struct mlxnp_fpga_spi_register reg)
{
	struct mlxnp_fpga_data *data = spidev->controller_data;

	return data->write_value(data, MLXNP_FPGA2_SPI_FLASH_PROG, reg.data);
}

static int
mlxnp_fpga_get_spi_register(struct spi_device *spidev,
			    struct mlxnp_fpga_spi_register *reg)
{
	struct mlxnp_fpga_data *data = spidev->controller_data;

	return data->read_value(data, MLXNP_FPGA2_SPI_FLASH_PROG, &reg->data);
}

static inline void mlxnp_fpga_setsck(struct spi_device *spidev, int is_on)
{
	current_spi_register.clk = !!is_on;
	mlxnp_fpga_set_spi_register(spidev, current_spi_register);
}

static inline void mlxnp_fpga_setmosi(struct spi_device *spidev, int is_on)
{
	current_spi_register.mosi = !!is_on;
	mlxnp_fpga_set_spi_register(spidev, current_spi_register);
}

static inline int mlxnp_fpga_getmiso(struct spi_device *spidev)
{
	mlxnp_fpga_get_spi_register(spidev, &current_spi_register);

	return current_spi_register.miso;
}

#define mlxnp_fpga_spidelay(X)		do { } while (0)

/*
 * This function was taken from spi-bitbang-txrx.h that is not part of our linux headers.
 *
 * The code that knows what GPIO pins do what should have declared four
 * functions, ideally as inlines, before including this header:
 *
 *  void mlxnp_fpga_setsck(struct spi_device *, int is_on);
 *  void mlxnp_fpga_setmosi(struct spi_device *, int is_on);
 *  int mlxnp_fpga_getmiso(struct spi_device *);
 *  void mlxnp_fpga_spidelay(unsigned);
 *
 * mlxnp_fpga_setsck()'s is_on parameter is a zero/nonzero boolean.
 *
 * mlxnp_fpga_setmosi()'s is_on parameter is a zero/nonzero boolean.
 *
 * mlxnp_fpga_getmiso() is required to return 0 or 1 only. Any other value is invalid
 * and will result in improper operation.
 *
 * A non-inlined routine would call bitbang_txrx_*() routines.  The
 * main loop could easily compile down to a handful of instructions,
 * especially if the delay is a NOP (to run at peak speed).
 *
 * Since this is software, the timings may not be exactly what your board's
 * chips need ... there may be several reasons you'd need to tweak timings
 * in these routines, not just to make it faster or slower to match a
 * particular CPU clock rate.
 */
static inline u32
mlxnp_fpga_bitbang_txrx_be_cpha1(struct spi_device *spi, unsigned nsecs,
				 unsigned cpol, unsigned flags, u32 word,
				 u8 bits)
{
        /* if (cpol == 0) this is SPI_MODE_1; else this is SPI_MODE_3 */

        bool oldbit = !(word & (1 << 31));
        /* clock starts at inactive polarity */
        for (word <<= (32 - bits); likely(bits); bits--) {
                /* setup MSB (to slave) on leading edge */
                mlxnp_fpga_setsck(spi, !cpol);
                if ((flags & SPI_MASTER_NO_TX) == 0) {
                        if ((word & (1 << 31)) != oldbit) {
                                mlxnp_fpga_setmosi(spi, word & (1 << 31));
                                oldbit = word & (1 << 31);
                        }
                }
                mlxnp_fpga_spidelay(nsecs); /* T(setup) */

                mlxnp_fpga_setsck(spi, cpol);
                mlxnp_fpga_spidelay(nsecs);

                /* sample MSB (from slave) on trailing edge */
                word <<= 1;
                if ((flags & SPI_MASTER_NO_RX) == 0)
                        word |= mlxnp_fpga_getmiso(spi);
        }

        return word;
}

static inline u32 
mlxnp_fpga_txrx_word_mode3(struct spi_device *spi, unsigned nsecs, u32 word, u8 bits)
{
	return mlxnp_fpga_bitbang_txrx_be_cpha1(spi, nsecs, 1, 0, word, bits);
}

static void fpga_nps_chipselect(struct spi_device *spidev, int value)
{
	current_spi_register.cs = MLXNP_FPGA_SPI_CS_DEACTIVATE;
	if (value)
		current_spi_register.cs = MLXNP_FPGA_SPI_CS_ACTIVATE;

	mlxnp_fpga_set_spi_register(spidev, current_spi_register);
}

static struct mtd_partition partitions[] = {
	{
        	.name           = "uImage",
        	.offset         = 0,
        	.size           = 0xe70000,
	}, {
        	.name           = "u-boot",
        	.offset         = MTDPART_OFS_APPEND,
        	.size           = 0x80000,
	}, {
        	.name           = "dtb",
        	.offset         = MTDPART_OFS_APPEND,
        	.size           = 0x10000,
	}, {
        	.name           = "btl",
        	.offset         = MTDPART_OFS_APPEND,
        	.size           = 0x80000,
	}, {
        	.name           = "btl-redundancy",
        	.offset         = MTDPART_OFS_APPEND,
        	.size           = 0x70000,
	}, {
        	.name           = "u-boot-env",
        	.offset         = MTDPART_OFS_APPEND,
        	.size           = 0x10000,
	}
};

static struct flash_platform_data mlxnp_fpga_flash = {
        .name           = "npsflash",
        .parts          = partitions,
        .nr_parts       = ARRAY_SIZE(partitions),
};

int mlxnp_fpga_spi_init(struct mlxnp_fpga_data *data)
{
	struct spi_master *master;
	int err;

	data->read_value(data, MLXNP_FPGA2_SPI_FLASH_PROG,
			 &current_spi_register.data);

	master = spi_alloc_master(data->access_dev, sizeof(*data));
	if (!master)
		return -ENOMEM;

	data->master = master;
	spi_master_set_devdata(master, data);

	/* setup the master state. */
	//////////////////master->bus_num = data->client->adapter->nr;
	master->num_chipselect = 1;
	master->mode_bits = SPI_MODE_3;
	/////////////////master->dev.of_node = data->client->dev.of_node;

	/* setup bitbang */
	data->bitbang.master = master;
	data->bitbang.chipselect = fpga_nps_chipselect;
	data->bitbang.txrx_word[SPI_MODE_3] = mlxnp_fpga_txrx_word_mode3;

	err = spi_bitbang_start(&data->bitbang);
	if (err < 0)
		return err;

	data->board_info.max_speed_hz = MLXNP_FPGA_SPI_MAX_SPEED_HZ;
	strcpy(data->board_info.modalias, "s25fl256s1");
	data->board_info.platform_data = &mlxnp_fpga_flash;
	data->board_info.chip_select = 0;
	data->board_info.mode = SPI_MODE_3;
	data->board_info.controller_data = data;

	if (!mlxnp_fpga_spi_enabled(data))
		return 0;

	data->dataflash = spi_new_device(master, &data->board_info);
	if (data->dataflash) {
		dev_err(data->access_dev, "NPS flash at %s\n",
			dev_name(&data->dataflash->dev));

		return -ENODEV;
	}

	return 0;
}

void mlxnp_fpga_spi_deinit(struct mlxnp_fpga_data *data)
{
	spi_bitbang_stop(&data->bitbang);
}

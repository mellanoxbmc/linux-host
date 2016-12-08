obj-$(CONFIG_MLXNP_TCAM_I2C)        	+= mlxnp_tcam_i2c.o
mlxnp_tcam_i2c-$(CONFIG_MLXNP_TCAM_I2C)	+= tcam_sysfs.o \
					   tcam_dev.o \
					   fpga_tcam.o \
					   tcam_i2c.o

obj-$(CONFIG_MLXNP_TCAM_LPC)        	+= mlxnp_tcam_lpc.o
mlxnp_tcam_lpc-$(CONFIG_MLXNP_TCAM_LPC)	+= tcam_sysfs.o \
					   tcam_dev.o \
					   fpga_tcam.o \
					   lpc.o \
					   tcam_lpc.o

obj-$(CONFIG_MLXNP_FPGA_I2C)        	+= mlxnp_fpga_i2c.o
mlxnp_fpga_i2c-$(CONFIG_MLXNP_FPGA_I2C)	+= fpga_sysfs.o \
					   fpga_dev.o \
					   fpga_smi.o \
					   fpga_spi.o \
					   fpga_tcam.o \
					   fpga_i2c.o

obj-$(CONFIG_MLXNP_FPGA_LPC)        	+= mlxnp_fpga_lpc.o
mlxnp_fpga_lpc-$(CONFIG_MLXNP_FPGA_LPC)	+= fpga_sysfs.o \
					   fpga_dev.o \
					   fpga_smi.o \
					   fpga_spi.o \
					   fpga_tcam.o \
					   fpga_gbox.o \
					   lpc.o \
					   fpga_lpc.o

obj-$(CONFIG_MLXNP_GBOX_LPC)        	+= mlxnp_gbox_lpc.o
mlxnp_gbox_lpc-$(CONFIG_MLXNP_FPGA_LPC)	+= gbox_sysfs.o \
					   gbox_dev.o \
					   fpga_gbox.o \
					   lpc.o \
					   gbox_lpc.o

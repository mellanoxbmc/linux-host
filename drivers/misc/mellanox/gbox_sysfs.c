#include "gbox.h"

/*
 * i2c sysfs
 */

#define show(value, reg)						\
static ssize_t								\
show_##value(struct device *dev, struct device_attribute *attr,		\
	     char *buf) {						\
	struct mlxnp_gbox_data *data = dev_get_drvdata(dev);		\
	u8 val;								\
	int err;							\
	err = data->read_value(data, reg, &val);			\
	if (err)							\
		return err;						\
	return sprintf(buf, "0x%x\n", val);				\
}

#define set(value, reg)							\
static ssize_t								\
set_##value(struct device *dev, struct device_attribute *attr,		\
	    const char *buf, size_t count) {				\
	struct mlxnp_gbox_data *data = dev_get_drvdata(dev);		\
	int val = simple_strtoul(buf, NULL, 16);			\
	int err;							\
	if (val >>8 )							\
		return -EINVAL;						\
	err = data->write_value(data, reg, val);			\
	if (err)							\
		return err;						\
	return count;							\
}

struct nps_reset_register {
	union {
		struct {
			u8 por_reset	: 1,
			hw_reset		: 1,
			__reserved		: 6;
		};
		u8 value;
	};
};

struct qsfp_sfp_registers {
	union {
		struct{
			union {
				struct {
					u8 qsfp0	: 8;
				};
				u8 reg0;
			};
			union {
				struct {
					u8 qsfp1	: 4,
					sfp0		: 2,
					__reserved0	: 2;
				};
				u8 reg1;
			};
		};
		union {
			struct {
				u16 qsfp		: 12,
				sfp				: 2,
				__reserved1		: 2;
			};
			u16 value;
		};
	};
};


/* cpld_reset */
show(cpld_reset, MLXNP_GBOX_CPLD_RESET);
set(cpld_reset, MLXNP_GBOX_CPLD_RESET);
static DEVICE_ATTR(cpld_reset, S_IWUSR | S_IRUGO,
		show_cpld_reset, set_cpld_reset);

/* GBOX exist */
static ssize_t show_gbox_exist(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mlxnp_gbox_data *data = dev_get_drvdata(dev);
	int res;

	mutex_lock(&data->sync_mutex);

	res = mlxnp_gbox_exist(data);
	if ((res != 1) && (res != 0))
		goto access_error;

	mutex_unlock(&data->sync_mutex);
	return sprintf(buf, "0x%x\n", res);
access_error:
	mutex_unlock(&data->sync_mutex);
	return res;
}

static DEVICE_ATTR(gbox_exist, S_IRUGO,
		show_gbox_exist, NULL);

int mlxnp_gbox_create_sysfs(struct mlxnp_gbox_data *data)
{
	int err;

	err = device_create_file(data->access_dev, &dev_attr_cpld_reset);
	if (err)
		return err;

	err = device_create_file(data->access_dev, &dev_attr_gbox_exist);
	if (err)
		return err;

	switch (data->type) {
	case gbox1_nl:
		break;
	case gbox2_nl:
		break;
	}

	return 0;
}

void mlxnp_gbox_remove_sysfs(struct mlxnp_gbox_data *data)
{
	device_remove_file(data->access_dev, &dev_attr_cpld_reset);

	device_remove_file(data->access_dev, &dev_attr_gbox_exist);

	switch (data->type) {
	case gbox1_nl:
		break;
	case gbox2_nl:
		break;
	}
}

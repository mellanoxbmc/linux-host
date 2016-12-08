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

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_data/pwm-mlxcpld.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>

#define PWM_MLXCPLD_PWM_MAX_NUM		1
#define PWM_MLXCPLD_TACHO_MAX_NUM	8
#define PWM_MLXCPLD_TACHO_ATTR_NUM	3
#define PWM_MLXCPLD_PWM_ATTR_NUM	3
#define PWM_MLXCPLD_ATTRS_NUM		(PWM_MLXCPLD_PWM_MAX_NUM * \
					 PWM_MLXCPLD_PWM_ATTR_NUM + \
					 PWM_MLXCPLD_TACHO_MAX_NUM * \
					 PWM_MLXCPLD_TACHO_ATTR_NUM)
#define PWM_MLXCPLD_MAX_PWM		255
#define PWM_MLXCPLD_DEFAULT_PWM		153

/**
 * enum pwm_mlxcpld_attr_type - sysfs attributes for:
 * @PWM_MLXCPLD_ATTR_TYPE_TACHO: FAN tacho attributes;
 * @PWM_MLXCPLD_ATTR_TYPE_PWM: FAN pwm attributes;
 */
enum pwm_mlxcpld_attr_type {
	PWM_MLXCPLD_ATTR_TYPE_TACHO,
	PWM_MLXCPLD_ATTR_TYPE_TACHO_MIN,
	PWM_MLXCPLD_ATTR_TYPE_TACHO_MAX,
	PWM_MLXCPLD_ATTR_TYPE_PWM,
	PWM_MLXCPLD_ATTR_TYPE_PWM_MIN,
	PWM_MLXCPLD_ATTR_TYPE_PWM_MAX,
};

struct pwm_mlxcpld {
	struct mutex lock;
	struct pwm_device *pwm;
	unsigned int pwm_value;
	unsigned int fan_state;
	unsigned int fan_max_state;
	unsigned int *fan_cooling_levels;
	struct thermal_cooling_device *cdev;
	struct platform_device *pdev;
	struct pwm_mlxcpld_platform_data *pdata;
	struct device *hwmon;
	struct attribute *pwm_mlxcpld_attr[PWM_MLXCPLD_ATTRS_NUM + 1];
	struct sensor_device_attribute_2
			pwm_mlxcpld_dev_attr[PWM_MLXCPLD_ATTRS_NUM];
	struct attribute_group group;
	const struct attribute_group *groups[2];
};

static int
__pwm_mlxcpld_store_pwm(struct pwm_mlxcpld *pwm_mlxcpld, unsigned long pwm)
{
	struct pwm_args pargs;
	unsigned long duty;
	int ret = 0;

	pwm_get_args(pwm_mlxcpld->pwm, &pargs);

	mutex_lock(&pwm_mlxcpld->lock);
	if (pwm_mlxcpld->pwm_value == pwm)
		goto exit_set_pwm_err;

	duty = DIV_ROUND_UP(pwm * (pargs.period - 1), PWM_MLXCPLD_MAX_PWM);
	ret = pwm_config(pwm_mlxcpld->pwm, duty, pargs.period);
	if (ret)
		goto exit_set_pwm_err;

	if (pwm == 0)
		pwm_disable(pwm_mlxcpld->pwm);

	if (pwm_mlxcpld->pwm_value == 0) {
		ret = pwm_enable(pwm_mlxcpld->pwm);
		if (ret)
			goto exit_set_pwm_err;
	}

	pwm_mlxcpld->pwm_value = pwm;
exit_set_pwm_err:
	mutex_unlock(&pwm_mlxcpld->lock);
	return ret;
}

static void
pwm_mlxcpld_update_state(struct pwm_mlxcpld *pwm_mlxcpld, unsigned long pwm)
{
	int i;

	for (i = 0; i < pwm_mlxcpld->fan_max_state; ++i)
		if (pwm < pwm_mlxcpld->fan_cooling_levels[i + 1])
			break;

	pwm_mlxcpld->fan_state = i;
}

static ssize_t
pwm_mlxcpld_store_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct pwm_mlxcpld *pwm_mlxcpld = dev_get_drvdata(dev);
	unsigned long pwm;
	int ret;

	if (kstrtoul(buf, 10, &pwm) || pwm > PWM_MLXCPLD_MAX_PWM)
		return -EINVAL;

	ret = __pwm_mlxcpld_store_pwm(pwm_mlxcpld, pwm);
	if (ret)
		return ret;

	pwm_mlxcpld_update_state(pwm_mlxcpld, pwm);

	return count;
}

static ssize_t pwm_mlxcpld_attr_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_mlxcpld *pwm_mlxcpld = platform_get_drvdata(pdev);
	int index = to_sensor_dev_attr_2(attr)->index;
	int nr = to_sensor_dev_attr_2(attr)->nr;
	u8 val = 0;
	unsigned long lval;

	switch (nr) {
	case PWM_MLXCPLD_ATTR_TYPE_TACHO:
		val = inb(pwm_mlxcpld->pdata->tacho.param[index].data_reg);
		lval = 1500000 / (val *
		       pwm_mlxcpld->pdata->tacho.param[index].scale +
		       pwm_mlxcpld->pdata->tacho.param[index].round);

		return sprintf(buf, "%lu\n", lval);

	case PWM_MLXCPLD_ATTR_TYPE_TACHO_MIN:
		val = pwm_mlxcpld->pdata->tacho.param[index].min;
		break;

	case PWM_MLXCPLD_ATTR_TYPE_TACHO_MAX:
		val = pwm_mlxcpld->pdata->tacho.param[index].max;
		break;

	case PWM_MLXCPLD_ATTR_TYPE_PWM:
		val = inb(pwm_mlxcpld->pdata->pwm.param[index].data_reg);
		break;

	case PWM_MLXCPLD_ATTR_TYPE_PWM_MIN:
		val = pwm_mlxcpld->pdata->pwm.param[index].min;
		break;

	case PWM_MLXCPLD_ATTR_TYPE_PWM_MAX:
		val = pwm_mlxcpld->pdata->pwm.param[index].max;
		break;

	}

	return sprintf(buf, "%u\n", val);
}

#define PRIV_ATTR(i) pwm_mlxcpld->pwm_mlxcpld_attr[i]
#define PRIV_DEV_ATTR(i) pwm_mlxcpld->pwm_mlxcpld_dev_attr[i]
static int pwm_mlxcpld_attr_init(struct pwm_mlxcpld *pwm_mlxcpld)
{
	int num_attrs = pwm_mlxcpld->pdata->tacho.count *
			PWM_MLXCPLD_TACHO_ATTR_NUM +
			pwm_mlxcpld->pdata->pwm.count *
			PWM_MLXCPLD_PWM_ATTR_NUM;
	int i, j, k = 0;

	pwm_mlxcpld->group.attrs = devm_kzalloc(&pwm_mlxcpld->pdev->dev,
					num_attrs * sizeof(struct attribute *),
					GFP_KERNEL);
	if (!pwm_mlxcpld->group.attrs)
		return -ENOMEM;

	for (i = 0; i < pwm_mlxcpld->pdata->tacho.count; i++) {
		for (j = 0; j < PWM_MLXCPLD_TACHO_ATTR_NUM; j++, k++) {
			//k = i * PWM_MLXCPLD_TACHO_ATTR_NUM + j;
			PRIV_ATTR(k) = &PRIV_DEV_ATTR(k).dev_attr.attr;
			PRIV_DEV_ATTR(k).dev_attr.attr.mode = S_IRUGO;
			PRIV_DEV_ATTR(k).dev_attr.show = pwm_mlxcpld_attr_show;

			switch (j) {
			case 0:
				PRIV_DEV_ATTR(k).nr =
					PWM_MLXCPLD_ATTR_TYPE_TACHO;
				PRIV_ATTR(i)->name = devm_kasprintf(
					&pwm_mlxcpld->pdev->dev, GFP_KERNEL,
					"fan%u_input", i + 1);
				break;

			case 1:
				PRIV_DEV_ATTR(k).nr =
					PWM_MLXCPLD_ATTR_TYPE_TACHO_MIN;
				PRIV_ATTR(k)->name = devm_kasprintf(
					&pwm_mlxcpld->pdev->dev, GFP_KERNEL,
					"fan%u_min", i + 1);
				break;

			case 2:
				PRIV_DEV_ATTR(k).nr =
					PWM_MLXCPLD_ATTR_TYPE_TACHO_MAX;
				PRIV_ATTR(k)->name = devm_kasprintf(
					&pwm_mlxcpld->pdev->dev, GFP_KERNEL,
					"fan%u_max", i + 1);
				break;
			}

			if (!PRIV_ATTR(k)->name) {
				dev_err(&pwm_mlxcpld->pdev->dev, "Memory allocation failed for sysfs attribute %d.\n",
					k + 1);
				return -ENOMEM;
			}

			PRIV_DEV_ATTR(k).dev_attr.attr.name = PRIV_ATTR(k)->name;
			PRIV_DEV_ATTR(k).index = i;
			sysfs_attr_init(&PRIV_DEV_ATTR(k).dev_attr.attr);
		}
	}

	for (i = 0; i < pwm_mlxcpld->pdata->pwm.count; i++) {
		for (j = 0; j < PWM_MLXCPLD_PWM_ATTR_NUM; j++, k++) {
			//k = i * PWM_MLXCPLD_PWM_ATTR_NUM + j;
			PRIV_ATTR(k) = &PRIV_DEV_ATTR(k).dev_attr.attr;
			PRIV_DEV_ATTR(k).dev_attr.attr.mode = S_IRUGO;
			PRIV_DEV_ATTR(k).dev_attr.show = pwm_mlxcpld_attr_show;

			switch (j) {
			case 0:
				PRIV_DEV_ATTR(k).nr =
					PWM_MLXCPLD_ATTR_TYPE_PWM;
				PRIV_DEV_ATTR(k).dev_attr.attr.mode = S_IWUSR |
							S_IRUGO;
				PRIV_DEV_ATTR(k).dev_attr.show =
							pwm_mlxcpld_attr_show;
				PRIV_DEV_ATTR(k).dev_attr.store =
							pwm_mlxcpld_store_pwm;
				PRIV_ATTR(k)->name = devm_kasprintf(
					&pwm_mlxcpld->pdev->dev, GFP_KERNEL,
					"pwm%u", i + 1);
				break;

			case 1:
				PRIV_DEV_ATTR(k).nr =
					PWM_MLXCPLD_ATTR_TYPE_PWM_MIN;
				PRIV_DEV_ATTR(k).dev_attr.attr.mode = S_IRUGO;
				PRIV_DEV_ATTR(k).dev_attr.show =
							pwm_mlxcpld_attr_show;
				PRIV_ATTR(k)->name = devm_kasprintf(
					&pwm_mlxcpld->pdev->dev, GFP_KERNEL,
					"pwm%u_min", i + 1);
				break;

			case 2:
				PRIV_DEV_ATTR(k).nr =
					PWM_MLXCPLD_ATTR_TYPE_PWM_MAX;
				PRIV_DEV_ATTR(k).dev_attr.attr.mode = S_IRUGO;
				PRIV_DEV_ATTR(k).dev_attr.show =
							pwm_mlxcpld_attr_show;
				PRIV_ATTR(k)->name = devm_kasprintf(
					&pwm_mlxcpld->pdev->dev, GFP_KERNEL,
					"pwm%u_max", i + 1);
				break;
			}

			if (!PRIV_ATTR(k)->name) {
				dev_err(&pwm_mlxcpld->pdev->dev, "Memory allocation failed for sysfs attribute %d.\n",
					k + 1);
				return -ENOMEM;
			}

			PRIV_DEV_ATTR(k).dev_attr.attr.name = PRIV_ATTR(k)->name;
			PRIV_DEV_ATTR(k).index = i;
			sysfs_attr_init(&PRIV_DEV_ATTR(k).dev_attr.attr);
		}
	}

#if 0
	for (i = 0; i < num_attrs; i++) {
		PRIV_ATTR(i) = &PRIV_DEV_ATTR(i).dev_attr.attr;

		if (i < pwm_mlxcpld->pdata->tacho.count) {
			PRIV_ATTR(i)->name = devm_kasprintf(
				&pwm_mlxcpld->pdev->dev, GFP_KERNEL, "fan%u_input", i
				+ 1);
			PRIV_DEV_ATTR(i).nr = PWM_MLXCPLD_ATTR_TYPE_TACHO;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = S_IRUGO;
			PRIV_DEV_ATTR(i).dev_attr.show = pwm_mlxcpld_attr_show;
		} else  {
			PRIV_ATTR(i)->name = devm_kasprintf(
				&pwm_mlxcpld->pdev->dev, GFP_KERNEL, "pwm%u",
				i % pwm_mlxcpld->pdata->pwm.count + 1);
			PRIV_DEV_ATTR(i).nr = PWM_MLXCPLD_ATTR_TYPE_PWM;
			PRIV_DEV_ATTR(i).dev_attr.attr.mode = S_IWUSR |
							      S_IRUGO;
			PRIV_DEV_ATTR(i).dev_attr.show = pwm_mlxcpld_attr_show;
			PRIV_DEV_ATTR(i).dev_attr.store =
							pwm_mlxcpld_store_pwm;
		}

		if (!PRIV_ATTR(i)->name) {
			dev_err(&pwm_mlxcpld->pdev->dev, "Memory allocation failed for sysfs attribute %d.\n",
				i + 1);
			return -ENOMEM;
		}

		PRIV_DEV_ATTR(i).dev_attr.attr.name = PRIV_ATTR(i)->name;
		PRIV_DEV_ATTR(i).index = i;
		sysfs_attr_init(&PRIV_DEV_ATTR(i).dev_attr.attr);
	}
#endif

	pwm_mlxcpld->group.attrs = pwm_mlxcpld->pwm_mlxcpld_attr;
	pwm_mlxcpld->groups[0] = &pwm_mlxcpld->group;
	pwm_mlxcpld->groups[1] = NULL;

	return 0;
}

/* thermal cooling device callbacks */
static int
pwm_mlxcpld_get_max_state(struct thermal_cooling_device *cdev,
			  unsigned long *state)
{
	struct pwm_mlxcpld *pwm_mlxcpld = cdev->devdata;

	if (!pwm_mlxcpld)
		return -EINVAL;

	*state = pwm_mlxcpld->fan_max_state;

	return 0;
}

static int
pwm_mlxcpld_get_cur_state(struct thermal_cooling_device *cdev,
			  unsigned long *state)
{
	struct pwm_mlxcpld *pwm_mlxcpld = cdev->devdata;

	if (!pwm_mlxcpld)
		return -EINVAL;

	*state = pwm_mlxcpld->fan_state;

	return 0;
}

static int
pwm_mlxcpld_set_cur_state(struct thermal_cooling_device *cdev,
			  unsigned long state)
{
	struct pwm_mlxcpld *pwm_mlxcpld = cdev->devdata;
	int ret;

	if (!pwm_mlxcpld || (state > pwm_mlxcpld->fan_max_state))
		return -EINVAL;

	if (state == pwm_mlxcpld->fan_state)
		return 0;

	ret = __pwm_mlxcpld_store_pwm(pwm_mlxcpld,
				    pwm_mlxcpld->fan_cooling_levels[state]);
	if (ret) {
		dev_err(&cdev->device, "Cannot set pwm!\n");
		return ret;
	}

	pwm_mlxcpld->fan_state = state;

	return ret;
}

static const struct thermal_cooling_device_ops pwm_mlxcpld_cooling_ops = {
	.get_max_state = pwm_mlxcpld_get_max_state,
	.get_cur_state = pwm_mlxcpld_get_cur_state,
	.set_cur_state = pwm_mlxcpld_set_cur_state,
};

#ifdef CONFIG_OF
static int
pwm_mlxcpld_of_get_cooling_data(struct device *dev,
				struct pwm_mlxcpld *pwm_mlxcpld)
{
	struct device_node *np = dev->of_node;
	int num, i, ret;

	if (!of_find_property(np, "cooling-levels", NULL))
		return 0;

	ret = of_property_count_u32_elems(np, "cooling-levels");
	if (ret <= 0) {
		dev_err(dev, "Wrong data!\n");
		return ret ? : -EINVAL;
	}

	num = ret;
	pwm_mlxcpld->fan_cooling_levels = devm_kzalloc(dev, num * sizeof(u32),
						   GFP_KERNEL);
	if (!pwm_mlxcpld->fan_cooling_levels)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "cooling-levels",
					 pwm_mlxcpld->fan_cooling_levels, num);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read!\n");
		return ret;
	}

	for (i = 0; i < num; i++) {
		if (pwm_mlxcpld->fan_cooling_levels[i] > PWM_MLXCPLD_MAX_PWM) {
			dev_err(dev, "PWM fan state[%d]:%d > %d\n", i,
				pwm_mlxcpld->fan_cooling_levels[i],
				PWM_MLXCPLD_MAX_PWM);
			return -EINVAL;
		}
	}

	pwm_mlxcpld->fan_max_state = num - 1;

	return 0;
}
#else
pwm_mlxcpld_of_get_cooling_data(struct device *dev,
				struct pwm_mlxcpld *pwm_mlxcpld)
{
	return -EINVAL;
}
#endif

static int
pwm_mlxcpld_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		       int duty_ns, int period_ns)
{
	struct pwm_mlxcpld *pwm_mlxcpld = dev_get_drvdata(pwm->chip->dev);
	int i;

	for (i = 0; i < pwm_mlxcpld->pdata->pwm.count; i++)
		outb(PWM_MLXCPLD_DEFAULT_PWM,
		     pwm_mlxcpld->pdata->pwm.param[i].data_reg);

	return 0;
}

static int
pwm_mlxcpld_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pwm_mlxcpld *pwm_mlxcpld = dev_get_drvdata(pwm->chip->dev);
	int i;

	for (i = 0; i < pwm_mlxcpld->pdata->pwm.count; i++)
		outb(pwm->args.period,
		     pwm_mlxcpld->pdata->pwm.param[i].data_reg);

	return 0;
}

static void
pwm_mlxcpld_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pwm_mlxcpld *pwm_mlxcpld = dev_get_drvdata(pwm->chip->dev);
	int i;

	for (i = 0; i < pwm_mlxcpld->pdata->pwm.count; i++)
		outb(0, pwm_mlxcpld->pdata->pwm.param[i].data_reg);
}

static struct pwm_ops pwm_mlxcpld_pwm_ops = {
	.config = pwm_mlxcpld_pwm_config,
	.enable = pwm_mlxcpld_pwm_enable,
	.disable = pwm_mlxcpld_pwm_disable,
};

static struct pwm_chip pwm_mlxcpld_pwm_chip = {
	.ops = &pwm_mlxcpld_pwm_ops,
	.base = -1,
	.npwm = 1,
	.of_pwm_n_cells = 1,
};

static struct pwm_device pwm_mlxcpld_pwm = {
	.chip = &pwm_mlxcpld_pwm_chip,
	.args.period = PWM_MLXCPLD_DEFAULT_PWM,
	.args.polarity = PWM_POLARITY_NORMAL,
	.state.enabled = false,
};

static int pwm_mlxcpld_probe(struct platform_device *pdev)
{
	struct pwm_mlxcpld_platform_data *pwm_mlxcpld_pdata;
	struct thermal_cooling_device *cdev;
	struct pwm_mlxcpld *pwm_mlxcpld;
	struct pwm_args pargs;
	int duty_cycle;
	int err;

        pwm_mlxcpld = devm_kzalloc(&pdev->dev, sizeof(*pwm_mlxcpld),
                                   GFP_KERNEL);
        if (!pwm_mlxcpld)
                return -ENOMEM;

	pwm_mlxcpld_pdata = dev_get_platdata(&pdev->dev);
	if (!pwm_mlxcpld_pdata) {
		err = pwm_mlxcpld_of_get_cooling_data(&pdev->dev, pwm_mlxcpld);
		if (err)
			dev_err(&pdev->dev, "Failed to get platform data.\n");
		return err;
	}

	mutex_init(&pwm_mlxcpld->lock);

	pwm_mlxcpld->pwm = &pwm_mlxcpld_pwm;
	pwm_mlxcpld->pwm->chip->dev = &pdev->dev;
	pwm_mlxcpld->pdev = pdev;
	pwm_mlxcpld->pdata = pwm_mlxcpld_pdata;

	platform_set_drvdata(pdev, pwm_mlxcpld);

	pwm_get_args(pwm_mlxcpld->pwm, &pargs);

	duty_cycle = pargs.period - 1;
	pwm_mlxcpld->pwm_value = PWM_MLXCPLD_MAX_PWM;

	err = pwm_config(pwm_mlxcpld->pwm, duty_cycle, pargs.period);
	if (err) {
		dev_err(&pdev->dev, "Failed to configure PWM\n");
		return err;
	}

	/* Enbale PWM output */
	err = pwm_enable(pwm_mlxcpld->pwm);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable PWM\n");
		return err;
	}

	err = pwm_mlxcpld_attr_init(pwm_mlxcpld);
	if (err) {
		dev_err(&pdev->dev, "Failed to allocate attributes: %d\n", err);
		return err;
	}

	pwm_mlxcpld->hwmon = devm_hwmon_device_register_with_groups(&pdev->dev,
					"pwm_mlxcpld", pwm_mlxcpld,
					pwm_mlxcpld->groups);
	if (IS_ERR(pwm_mlxcpld->hwmon)) {
		dev_err(&pdev->dev, "Failed to register hwmon device %ld\n",
			PTR_ERR(pwm_mlxcpld->hwmon));
		pwm_disable(pwm_mlxcpld->pwm);
		return PTR_ERR(pwm_mlxcpld->hwmon);
	}

	pwm_mlxcpld->fan_state = pwm_mlxcpld->fan_max_state;
	if (IS_ENABLED(CONFIG_THERMAL)) {
		cdev = thermal_of_cooling_device_register(pdev->dev.of_node,
			"pwm-mlxcpld", pwm_mlxcpld, &pwm_mlxcpld_cooling_ops);
		if (IS_ERR(cdev)) {
			dev_err(&pdev->dev,
				"Failed to register pwm-mlxcpld as cooling device");
			pwm_disable(pwm_mlxcpld->pwm);
			return PTR_ERR(cdev);
		}
		pwm_mlxcpld->cdev = cdev;
		thermal_cdev_update(cdev);
	}

	return 0;
}

static int pwm_mlxcpld_remove(struct platform_device *pdev)
{
	struct pwm_mlxcpld *pwm_mlxcpld = platform_get_drvdata(pdev);

	thermal_cooling_device_unregister(pwm_mlxcpld->cdev);
	if (pwm_mlxcpld->pwm_value)
		pwm_disable(pwm_mlxcpld->pwm);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_mlxcpld_suspend(struct device *dev)
{
	struct pwm_mlxcpld *pwm_mlxcpld = dev_get_drvdata(dev);

	if (pwm_mlxcpld->pwm_value)
		pwm_disable(pwm_mlxcpld->pwm);
	return 0;
}

static int pwm_mlxcpld_resume(struct device *dev)
{
	struct pwm_mlxcpld *pwm_mlxcpld = dev_get_drvdata(dev);
	struct pwm_args pargs;
	unsigned long duty;
	int ret;

	if (pwm_mlxcpld->pwm_value == 0)
		return 0;

	pwm_get_args(pwm_mlxcpld->pwm, &pargs);
	duty = DIV_ROUND_UP(pwm_mlxcpld->pwm_value * (pargs.period - 1),
			    PWM_MLXCPLD_MAX_PWM);
	ret = pwm_config(pwm_mlxcpld->pwm, duty, pargs.period);
	if (ret)
		return ret;
	return pwm_enable(pwm_mlxcpld->pwm);
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_mlxcpld_pm, pwm_mlxcpld_suspend, \
			 pwm_mlxcpld_resume);

static const struct of_device_id of_pwm_mlxcpld_match[] = {
	{ .compatible = "pwm-mlxcpld", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_mlxcpld_match);

static struct platform_driver pwm_mlxcpld_driver = {
	.probe		= pwm_mlxcpld_probe,
	.remove		= pwm_mlxcpld_remove,
	.driver	= {
		.name		= "pwm-mlxcpld",
		.pm		= &pwm_mlxcpld_pm,
		.of_match_table	= of_pwm_mlxcpld_match,
	},
};

module_platform_driver(pwm_mlxcpld_driver);

MODULE_AUTHOR("Vadim Pasternak <vadimp@mellanox.com>");
MODULE_ALIAS("platform:pwm-mlxcpld");
MODULE_DESCRIPTION("MLXCPLD FAN driver");
MODULE_LICENSE("Dual BSD/GPL");

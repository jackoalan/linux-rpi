// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bitbanging I2C bus driver using the GPIO API
 *
 * Copyright (C) 2007 Atmel Corporation
 */
#include <linux/completion.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_data/i2c-gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct i2c_gpio_private_data {
	struct citrus_core *citrus_core;
	struct i2c_adapter adap;
	struct i2c_algo_bit_data bit_data;
	struct i2c_gpio_platform_data pdata;
};

/*
 * Toggle SDA by changing the output value of the pin. This is only
 * valid for pins configured as open drain (i.e. setting the value
 * high effectively turns off the output driver.)
 */
static void i2c_gpio_setsda_val(void *data, int state)
{
	struct i2c_gpio_private_data *priv = data;

	citrus_core_set_i2c_sda(priv->citrus_core, state);
}

/*
 * Toggle SCL by changing the output value of the pin. This is used
 * for pins that are configured as open drain and for output-only
 * pins. The latter case will break the i2c protocol, but it will
 * often work in practice.
 */
static void i2c_gpio_setscl_val(void *data, int state)
{
	struct i2c_gpio_private_data *priv = data;

	citrus_core_set_i2c_scl(priv->citrus_core, state);
}

static void i2c_gpio_setscl2_val(void *data, int state)
{
	struct i2c_gpio_private_data *priv = data;

	citrus_core_set_i2c_scl2(priv->citrus_core, state);
}

static int i2c_gpio_getsda(void *data)
{
	struct i2c_gpio_private_data *priv = data;

	return citrus_core_get_i2c_sda(priv->citrus_core);
}

static int i2c_gpio_getscl(void *data)
{
	struct i2c_gpio_private_data *priv = data;

	return citrus_core_get_i2c_scl(priv->citrus_core);
}

static int i2c_gpio_getscl2(void *data)
{
	struct i2c_gpio_private_data *priv = data;

	return citrus_core_get_i2c_scl2(priv->citrus_core);
}

static int i2c_gpio_pre_xfer(struct i2c_adapter *i2c_adap)
{
	struct i2c_algo_bit_data *algo_data = i2c_adap->algo_data;
	struct i2c_gpio_private_data *priv = algo_data->data;

	citrus_core_lock_i2c(priv->citrus_core);

	return 0;
}

static void i2c_gpio_post_xfer(struct i2c_adapter *i2c_adap)
{
	struct i2c_algo_bit_data *algo_data = i2c_adap->algo_data;
	struct i2c_gpio_private_data *priv = algo_data->data;

	citrus_core_unlock_i2c(priv->citrus_core);
}

static void of_i2c_gpio_get_props(struct device_node *np,
				  struct i2c_gpio_platform_data *pdata)
{
	u32 reg;

	of_property_read_u32(np, "i2c-gpio,delay-us", &pdata->udelay);

	if (!of_property_read_u32(np, "i2c-gpio,timeout-ms", &reg))
		pdata->timeout = msecs_to_jiffies(reg);

	pdata->sda_is_open_drain =
		of_property_read_bool(np, "i2c-gpio,sda-open-drain");
	pdata->scl_is_open_drain =
		of_property_read_bool(np, "i2c-gpio,scl-open-drain");
	pdata->scl_is_output_only =
		of_property_read_bool(np, "i2c-gpio,scl-output-only");
}

#ifdef CONFIG_OF
static const struct of_device_id i2c_citrus_dt_ids[] = {
	{ .compatible = "i2c-citrus" },
	{}
};
MODULE_DEVICE_TABLE(of, i2c_citrus_dt_ids);

static const struct of_device_id i2c_citrus_dt_ids2[] = {
	{ .compatible = "i2c-citrus2" },
	{}
};
MODULE_DEVICE_TABLE(of, i2c_citrus_dt_ids2);

static struct device_node *i2c_citrus_probe_dt(struct device *dev, bool scl2)
{
	struct device_node *i2c_node;

	i2c_node = of_find_matching_node(dev->of_node,
					 scl2 ? i2c_citrus_dt_ids2
					      : i2c_citrus_dt_ids);
	if (!i2c_node || !of_device_is_available(i2c_node)) {
		if (!scl2) /* scl2 is optional */
			dev_err_probe(dev, -ENODEV,
				      "i2c-citrus device node not found\n");
		of_node_put(i2c_node);
		return ERR_PTR(-ENODEV);
	}
	dev_dbg(dev, scl2 ? "Found i2c-citrus2 node\n"
			  : "Found i2c-citrus node\n");

	return i2c_node;
}
#else
#error Must be compiled with CONFIG_OF
#endif

static void i2c_citrus_remove(void *data)
{
	struct i2c_adapter *adap = data;
	of_node_put(adap->dev.of_node);
	i2c_del_adapter(adap);
}

static int i2c_citrus_probe(struct device *dev, struct citrus_core *citrus,
			    bool scl2)
{
	struct i2c_gpio_private_data *priv;
	struct i2c_gpio_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	struct device_node *np;
	int ret;

	np = i2c_citrus_probe_dt(dev, scl2);
	if (IS_ERR(np))
		return PTR_ERR(np);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		of_node_put(np);
		return -ENOMEM;
	}

	priv->citrus_core = citrus;
	adap = &priv->adap;
	bit_data = &priv->bit_data;
	pdata = &priv->pdata;

	of_i2c_gpio_get_props(np, pdata);

	bit_data->can_do_atomic = true;

	bit_data->setsda = i2c_gpio_setsda_val;
	bit_data->setscl = scl2 ? i2c_gpio_setscl2_val : i2c_gpio_setscl_val;

	bit_data->pre_xfer = i2c_gpio_pre_xfer;
	bit_data->post_xfer = i2c_gpio_post_xfer;

	if (!pdata->scl_is_output_only)
		bit_data->getscl = scl2 ? i2c_gpio_getscl2 : i2c_gpio_getscl;
	bit_data->getsda = i2c_gpio_getsda;

	if (pdata->udelay)
		bit_data->udelay = pdata->udelay;
	else if (pdata->scl_is_output_only)
		bit_data->udelay = 50;			/* 10 kHz */
	else
		bit_data->udelay = 5;			/* 100 kHz */

	if (pdata->timeout)
		bit_data->timeout = pdata->timeout;
	else
		bit_data->timeout = HZ / 10;		/* 100 ms */

	bit_data->data = priv;

	adap->owner = THIS_MODULE;

	strscpy(adap->name, dev_name(dev), sizeof(adap->name));

	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = dev;
	adap->dev.of_node = np;

	adap->nr = -1;
	ret = i2c_bit_add_numbered_bus(adap);
	if (ret) {
		of_node_put(np);
		return ret;
	}

	return devm_add_action_or_reset(dev, i2c_citrus_remove, adap);
}

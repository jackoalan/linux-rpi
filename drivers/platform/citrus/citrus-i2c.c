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
#ifdef CONFIG_I2C_GPIO_FAULT_INJECTOR
	struct dentry *debug_dir;
	/* these must be protected by bus lock */
	struct completion scl_irq_completion;
	u64 scl_irq_data;
#endif
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

#ifdef CONFIG_I2C_GPIO_FAULT_INJECTOR
static struct dentry *i2c_gpio_debug_dir;

#define setsda(bd, val)	((bd)->setsda((bd)->data, val))
#define setscl(bd, val)	((bd)->setscl((bd)->data, val))
#define getsda(bd)	((bd)->getsda((bd)->data))
#define getscl(bd)	((bd)->getscl((bd)->data))

#define WIRE_ATTRIBUTE(wire) \
static int fops_##wire##_get(void *data, u64 *val)		\
{								\
	struct i2c_gpio_private_data *priv = data;		\
								\
	i2c_lock_bus(&priv->adap, I2C_LOCK_ROOT_ADAPTER);	\
	*val = get##wire(&priv->bit_data);			\
	i2c_unlock_bus(&priv->adap, I2C_LOCK_ROOT_ADAPTER);	\
	return 0;						\
}								\
static int fops_##wire##_set(void *data, u64 val)		\
{								\
	struct i2c_gpio_private_data *priv = data;		\
								\
	i2c_lock_bus(&priv->adap, I2C_LOCK_ROOT_ADAPTER);	\
	set##wire(&priv->bit_data, val);			\
	i2c_unlock_bus(&priv->adap, I2C_LOCK_ROOT_ADAPTER);	\
	return 0;						\
}								\
DEFINE_DEBUGFS_ATTRIBUTE(fops_##wire, fops_##wire##_get, fops_##wire##_set, "%llu\n")

WIRE_ATTRIBUTE(scl);
WIRE_ATTRIBUTE(sda);

static void i2c_gpio_incomplete_transfer(struct i2c_gpio_private_data *priv,
					u32 pattern, u8 pattern_size)
{
	struct i2c_algo_bit_data *bit_data = &priv->bit_data;
	int i;

	i2c_lock_bus(&priv->adap, I2C_LOCK_ROOT_ADAPTER);

	/* START condition */
	setsda(bit_data, 0);
	udelay(bit_data->udelay);

	/* Send pattern, request ACK, don't send STOP */
	for (i = pattern_size - 1; i >= 0; i--) {
		setscl(bit_data, 0);
		udelay(bit_data->udelay / 2);
		setsda(bit_data, (pattern >> i) & 1);
		udelay((bit_data->udelay + 1) / 2);
		setscl(bit_data, 1);
		udelay(bit_data->udelay);
	}

	i2c_unlock_bus(&priv->adap, I2C_LOCK_ROOT_ADAPTER);
}

static int fops_incomplete_addr_phase_set(void *data, u64 addr)
{
	struct i2c_gpio_private_data *priv = data;
	u32 pattern;

	if (addr > 0x7f)
		return -EINVAL;

	/* ADDR (7 bit) + RD (1 bit) + Client ACK, keep SDA hi (1 bit) */
	pattern = (addr << 2) | 3;

	i2c_gpio_incomplete_transfer(priv, pattern, 9);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_incomplete_addr_phase, NULL, fops_incomplete_addr_phase_set, "%llu\n");

static int fops_incomplete_write_byte_set(void *data, u64 addr)
{
	struct i2c_gpio_private_data *priv = data;
	u32 pattern;

	if (addr > 0x7f)
		return -EINVAL;

	/* ADDR (7 bit) + WR (1 bit) + Client ACK (1 bit) */
	pattern = (addr << 2) | 1;
	/* 0x00 (8 bit) + Client ACK, keep SDA hi (1 bit) */
	pattern = (pattern << 9) | 1;

	i2c_gpio_incomplete_transfer(priv, pattern, 18);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_incomplete_write_byte, NULL, fops_incomplete_write_byte_set, "%llu\n");

static int i2c_gpio_fi_act_on_scl_irq(struct i2c_gpio_private_data *priv,
				       irqreturn_t handler(int, void*))
{
	int ret, irq = gpiod_to_irq(priv->scl);

	if (irq < 0)
		return irq;

	i2c_lock_bus(&priv->adap, I2C_LOCK_ROOT_ADAPTER);

	ret = gpiod_direction_input(priv->scl);
	if (ret)
		goto unlock;

	reinit_completion(&priv->scl_irq_completion);

	ret = request_irq(irq, handler, IRQF_TRIGGER_FALLING,
			  "i2c_gpio_fault_injector_scl_irq", priv);
	if (ret)
		goto output;

	wait_for_completion_interruptible(&priv->scl_irq_completion);

	free_irq(irq, priv);
 output:
	ret = gpiod_direction_output(priv->scl, 1) ?: ret;
 unlock:
	i2c_unlock_bus(&priv->adap, I2C_LOCK_ROOT_ADAPTER);

	return ret;
}

static irqreturn_t lose_arbitration_irq(int irq, void *dev_id)
{
	struct i2c_gpio_private_data *priv = dev_id;

	setsda(&priv->bit_data, 0);
	udelay(priv->scl_irq_data);
	setsda(&priv->bit_data, 1);

	complete(&priv->scl_irq_completion);

	return IRQ_HANDLED;
}

static int fops_lose_arbitration_set(void *data, u64 duration)
{
	struct i2c_gpio_private_data *priv = data;

	if (duration > 100 * 1000)
		return -EINVAL;

	priv->scl_irq_data = duration;
	/*
	 * Interrupt on falling SCL. This ensures that the master under test has
	 * really started the transfer. Interrupt on falling SDA did only
	 * exercise 'bus busy' detection on some HW but not 'arbitration lost'.
	 * Note that the interrupt latency may cause the first bits to be
	 * transmitted correctly.
	 */
	return i2c_gpio_fi_act_on_scl_irq(priv, lose_arbitration_irq);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_lose_arbitration, NULL, fops_lose_arbitration_set, "%llu\n");

static irqreturn_t inject_panic_irq(int irq, void *dev_id)
{
	struct i2c_gpio_private_data *priv = dev_id;

	udelay(priv->scl_irq_data);
	panic("I2C fault injector induced panic");

	return IRQ_HANDLED;
}

static int fops_inject_panic_set(void *data, u64 duration)
{
	struct i2c_gpio_private_data *priv = data;

	if (duration > 100 * 1000)
		return -EINVAL;

	priv->scl_irq_data = duration;
	/*
	 * Interrupt on falling SCL. This ensures that the master under test has
	 * really started the transfer.
	 */
	return i2c_gpio_fi_act_on_scl_irq(priv, inject_panic_irq);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_inject_panic, NULL, fops_inject_panic_set, "%llu\n");

static void i2c_gpio_fault_injector_init(struct platform_device *pdev)
{
	struct i2c_gpio_private_data *priv = platform_get_drvdata(pdev);

	/*
	 * If there will be a debugfs-dir per i2c adapter somewhen, put the
	 * 'fault-injector' dir there. Until then, we have a global dir with
	 * all adapters as subdirs.
	 */
	if (!i2c_gpio_debug_dir) {
		i2c_gpio_debug_dir = debugfs_create_dir("i2c-fault-injector", NULL);
		if (!i2c_gpio_debug_dir)
			return;
	}

	priv->debug_dir = debugfs_create_dir(pdev->name, i2c_gpio_debug_dir);
	if (!priv->debug_dir)
		return;

	init_completion(&priv->scl_irq_completion);

	debugfs_create_file_unsafe("incomplete_address_phase", 0200, priv->debug_dir,
				   priv, &fops_incomplete_addr_phase);
	debugfs_create_file_unsafe("incomplete_write_byte", 0200, priv->debug_dir,
				   priv, &fops_incomplete_write_byte);
	if (priv->bit_data.getscl) {
		debugfs_create_file_unsafe("inject_panic", 0200, priv->debug_dir,
					   priv, &fops_inject_panic);
		debugfs_create_file_unsafe("lose_arbitration", 0200, priv->debug_dir,
					   priv, &fops_lose_arbitration);
	}
	debugfs_create_file_unsafe("scl", 0600, priv->debug_dir, priv, &fops_scl);
	debugfs_create_file_unsafe("sda", 0600, priv->debug_dir, priv, &fops_sda);
}

static void i2c_gpio_fault_injector_exit(struct platform_device *pdev)
{
	struct i2c_gpio_private_data *priv = platform_get_drvdata(pdev);

	debugfs_remove_recursive(priv->debug_dir);
}
#else
static inline void i2c_gpio_fault_injector_init(struct platform_device *pdev) {}
static inline void i2c_gpio_fault_injector_exit(struct platform_device *pdev) {}
#endif /* CONFIG_I2C_GPIO_FAULT_INJECTOR*/

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

static struct device_node *i2c_citrus_probe_dt(struct platform_device *pdev)
{
	struct device_node *i2c_node;

	i2c_node = of_find_matching_node(pdev->dev.of_node, i2c_citrus_dt_ids);
	if (!i2c_node || !of_device_is_available(i2c_node)) {
		dev_err_probe(&pdev->dev, -ENODEV, "i2c-citrus device node not found\n");
		return ERR_PTR(-ENODEV);
	}
	dev_dbg(&pdev->dev, "Found i2c-citrus node\n");

	return i2c_node;
}
#else
#error Must be compiled with CONFIG_OF
#endif

static int i2c_citrus_probe(struct platform_device *pdev, struct citrus_core *citrus)
{
	struct i2c_gpio_private_data *priv;
	struct i2c_gpio_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	int ret;

	np = i2c_citrus_probe_dt(pdev);
	if (IS_ERR(np))
		return PTR_ERR(np);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->citrus_core = citrus;
	adap = &priv->adap;
	bit_data = &priv->bit_data;
	pdata = &priv->pdata;

	of_i2c_gpio_get_props(np, pdata);

	bit_data->can_do_atomic = true;

	bit_data->setsda = i2c_gpio_setsda_val;
	bit_data->setscl = i2c_gpio_setscl_val;

	bit_data->pre_xfer = i2c_gpio_pre_xfer;
	bit_data->post_xfer = i2c_gpio_post_xfer;

	if (!pdata->scl_is_output_only)
		bit_data->getscl = i2c_gpio_getscl;
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

	if (pdev->id != PLATFORM_DEVID_NONE || !pdev->dev.of_node ||
	    of_property_read_u32(pdev->dev.of_node, "reg", &adap->nr))
		adap->nr = pdev->id;
	ret = i2c_bit_add_numbered_bus(adap);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);

	i2c_gpio_fault_injector_init(pdev);

	return 0;
}

static int i2c_citrus_remove(struct platform_device *pdev)
{
	struct i2c_gpio_private_data *priv;
	struct i2c_adapter *adap;

	i2c_gpio_fault_injector_exit(pdev);

	priv = platform_get_drvdata(pdev);
	adap = &priv->adap;

	i2c_del_adapter(adap);

	return 0;
}

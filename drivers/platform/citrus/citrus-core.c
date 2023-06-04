#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/gpio/consumer.h>

struct citrus_core {
	struct device		*pdev_dev;
	struct gpio_desc	*sck;
	struct gpio_desc	*mosi;
	struct mutex		bus_mutex;
};

/*
 * STARTUP
 *
 * Core device will wait
 *
 * MASTER MODE
 *
 * While in master mode, core device
 *
 * SPI SUB-MODE
 *
 *
 *
 * SLAVE MODE
 *
 *
 */

#define WITH_POINTER_LOGGING 0

#if WITH_POINTER_LOGGING
#define citrus_info(citrus, fmt, ...) \
	dev_info(citrus->pdev_dev, "c:%p t:%p " fmt, citrus, current, ## __VA_ARGS__)
#define citrus_dbg(citrus, fmt, ...) \
	dev_dbg(citrus->pdev_dev, "c:%p t:%p " fmt, citrus, current, ## __VA_ARGS__)
#else
#define citrus_info(citrus, fmt, ...) \
	dev_info(citrus->pdev_dev, fmt, ## __VA_ARGS__)
#define citrus_dbg(citrus, fmt, ...) \
	dev_dbg(citrus->pdev_dev, fmt, ## __VA_ARGS__)
#endif

static inline void citrus_core_lock_spi(struct citrus_core *citrus)
{
	citrus_dbg(citrus, "citrus_core_lock_spi: locking\n");
	mutex_lock(&citrus->bus_mutex);
	citrus_dbg(citrus, "citrus_core_lock_spi: locked\n");
}

static inline void citrus_core_unlock_spi(struct citrus_core *citrus)
{
	citrus_dbg(citrus, "citrus_core_lock_spi: unlocking\n");
	mutex_unlock(&citrus->bus_mutex);
	citrus_dbg(citrus, "citrus_core_lock_spi: unlocked\n");
}

static inline void citrus_core_set_spi_sck(struct citrus_core *citrus, int value)
{
	citrus_dbg(citrus, "citrus_core_set_spi_sck: %d\n", value);
	gpiod_set_value_cansleep(citrus->sck, value);
}

static inline void citrus_core_set_spi_mosi(struct citrus_core *citrus, int value)
{
	citrus_dbg(citrus, "citrus_core_set_spi_mosi: %d\n", value);
	gpiod_set_value_cansleep(citrus->mosi, value);
}

static inline int citrus_core_set_spi_mosi_direction_input(struct citrus_core *citrus)
{
	citrus_dbg(citrus, "citrus_core_set_spi_mosi_direction_input\n");
	return gpiod_direction_input(citrus->mosi);
}

static inline int citrus_core_set_spi_mosi_direction_output(struct citrus_core *citrus, int value)
{
	citrus_dbg(citrus, "citrus_core_set_spi_mosi_direction_output: %d\n", value);
	return gpiod_direction_output(citrus->mosi, value);
}

#include "citrus-spi.c"

static inline void citrus_core_lock_i2c(struct citrus_core *citrus)
{
	citrus_dbg(citrus, "citrus_core_lock_i2c: locking\n");
	mutex_lock(&citrus->bus_mutex);
	citrus_dbg(citrus, "citrus_core_lock_i2c: locked\n");
}

static inline void citrus_core_unlock_i2c(struct citrus_core *citrus)
{
	citrus_dbg(citrus, "citrus_core_lock_i2c: unlocking\n");
	mutex_unlock(&citrus->bus_mutex);
	citrus_dbg(citrus, "citrus_core_lock_i2c: unlocked\n");
}

static inline void citrus_core_set_i2c_sda(struct citrus_core *citrus, int value)
{
	citrus_dbg(citrus, "citrus_core_set_i2c_sda: %d\n", value);
	gpiod_set_value_cansleep(citrus->mosi, value);
}

static inline void citrus_core_set_i2c_scl(struct citrus_core *citrus, int value)
{
	citrus_dbg(citrus, "citrus_core_set_i2c_scl: %d\n", value);
	gpiod_set_value_cansleep(citrus->sck, value);
}

static inline int citrus_core_get_i2c_sda(struct citrus_core *citrus)
{
	int value;
	value = gpiod_get_value_cansleep(citrus->mosi);
	citrus_dbg(citrus, "citrus_core_get_i2c_sda: %d\n", value);
	return value;
}

static inline int citrus_core_get_i2c_scl(struct citrus_core *citrus)
{
	int value;
	value = gpiod_get_value_cansleep(citrus->sck);
	citrus_dbg(citrus, "citrus_core_get_i2c_scl: %d\n", value);
	return value;
}

#include "citrus-i2c.c"

static int citrus_core_request(struct device *dev, struct citrus_core *citrus)
{
	citrus->mosi = devm_gpiod_get(dev, "mosi", GPIOD_OUT_LOW_OPEN_DRAIN);
	if (IS_ERR(citrus->mosi))
		return PTR_ERR(citrus->mosi);

	citrus->sck = devm_gpiod_get(dev, "sck", GPIOD_OUT_LOW_OPEN_DRAIN);
	return PTR_ERR_OR_ZERO(citrus->sck);
}

static int citrus_core_device_init(struct platform_device *pdev,
				   struct citrus_core *citrus)
{
	int status;

	status = citrus_core_request(&pdev->dev, citrus);
	if (status)
		return status;

	platform_set_drvdata(pdev, citrus);
	return 0;
}

static int citrus_core_probe(struct platform_device *pdev)
{
	struct citrus_core *citrus;
	int result;

	citrus = devm_kzalloc(&pdev->dev, sizeof(*citrus), GFP_KERNEL);
	if (!citrus)
		return -ENOMEM;

	citrus->pdev_dev = &pdev->dev;
	mutex_init(&citrus->bus_mutex);

	citrus_dbg(citrus, "Probing citrus-core\n");

	result = citrus_core_device_init(pdev, citrus);
	citrus_dbg(citrus, "citrus_core_device_init: %d\n", result);
	if (result)
		return result;

	result = spi_citrus_probe(pdev, citrus);
	citrus_dbg(citrus, "spi_citrus_probe: %d\n", result);
	if (result)
		return result;

	result = i2c_citrus_probe(pdev, citrus);
	citrus_dbg(citrus, "i2c_citrus_probe: %d\n", result);
	if (result)
		return result;

	citrus_info(citrus, "Citrus successfully probed\n");
	return 0;
}

static int citrus_core_remove(struct platform_device *pdev)
{
	return i2c_citrus_remove(pdev);
}

static const struct of_device_id citrus_core_of_match[] = {
	{ .compatible = "citrus-core", },
	{},
};
MODULE_DEVICE_TABLE(of, citrus_core_of_match);

static struct platform_driver citrus_core_driver = {
	.probe = citrus_core_probe,
	.remove = citrus_core_remove,
	.driver = {
		.name = "citrus-core",
		.of_match_table = citrus_core_of_match,
	}
};

module_platform_driver(citrus_core_driver);

MODULE_AUTHOR("Jack Andersen <jackoalan@gmail.com>");
MODULE_DESCRIPTION("I2C Multiplexing for use with 2 hyperpixel displays");
MODULE_LICENSE("GPL v2");

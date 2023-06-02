#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/gpio/consumer.h>

struct citrus_core {
	struct device		*pdev_dev;
	struct gpio_desc	*cs;
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

static inline void citrus_core_lock_spi(struct citrus_core *citrus)
{
	mutex_lock(&citrus->bus_mutex);
	dev_dbg(citrus->pdev_dev, "citrus_core_lock_spi: locked");
}

static inline void citrus_core_unlock_spi(struct citrus_core *citrus)
{
	mutex_unlock(&citrus->bus_mutex);
	dev_dbg(citrus->pdev_dev, "citrus_core_lock_spi: unlocked");
}

static inline void citrus_core_set_spi_sck(struct citrus_core *citrus, int value)
{
	dev_dbg(citrus->pdev_dev, "citrus_core_set_spi_sck: %d", value);
	gpiod_set_value_cansleep(citrus->sck, value);
}

static inline void citrus_core_set_spi_cs(struct citrus_core *citrus, int value)
{
	dev_dbg(citrus->pdev_dev, "citrus_core_set_spi_cs: %d", value);
	gpiod_set_value_cansleep(citrus->cs, value);
}

static inline int citrus_core_set_spi_cs_direction_output(struct citrus_core *citrus, int value)
{
	dev_dbg(citrus->pdev_dev, "citrus_core_set_spi_cs_direction_output: %d", value);
	return gpiod_direction_output(citrus->cs, value);
}

static inline void citrus_core_set_spi_mosi(struct citrus_core *citrus, int value)
{
	dev_dbg(citrus->pdev_dev, "citrus_core_set_spi_mosi: %d", value);
	gpiod_set_value_cansleep(citrus->mosi, value);
}

static inline int citrus_core_set_spi_mosi_direction_input(struct citrus_core *citrus)
{
	dev_dbg(citrus->pdev_dev, "citrus_core_set_spi_mosi_direction_input");
	return gpiod_direction_input(citrus->mosi);
}

static inline int citrus_core_set_spi_mosi_direction_output(struct citrus_core *citrus, int value)
{
	dev_dbg(citrus->pdev_dev, "citrus_core_set_spi_mosi_direction_output: %d", value);
	return gpiod_direction_output(citrus->mosi, value);
}

#include "citrus-spi.c"

static int citrus_core_request(struct device *dev, struct citrus_core *citrus)
{
	citrus->mosi = devm_gpiod_get(dev, "mosi", GPIOD_OUT_LOW);
	if (IS_ERR(citrus->mosi))
		return PTR_ERR(citrus->mosi);

	citrus->sck = devm_gpiod_get(dev, "sck", GPIOD_OUT_LOW);
	if (IS_ERR(citrus->sck))
		return PTR_ERR(citrus->sck);

	citrus->cs = devm_gpiod_get(dev, "cs", GPIOD_OUT_HIGH);
	return PTR_ERR_OR_ZERO(citrus->cs);
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

static void citrus_core_device_deinit(struct platform_device *pdev,
				      struct citrus_core *citrus)
{

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

	dev_dbg(&pdev->dev, "Probing citrus-core");

	result = citrus_core_device_init(pdev, citrus);
	dev_dbg(&pdev->dev, "citrus_core_device_init: %d", result);
	if (result)
		return result;

	result = spi_citrus_probe(pdev, citrus);
	dev_dbg(&pdev->dev, "spi_citrus_probe: %d", result);

	return result;
}

static int citrus_core_remove(struct platform_device *pdev)
{
	struct citrus_core *citrus = platform_get_drvdata(pdev);

	citrus_core_device_deinit(pdev, citrus);
	return 0;
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

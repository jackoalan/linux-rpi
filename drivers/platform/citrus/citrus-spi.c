// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Modified GPIO driver for multiplexing SPI on Citrus platform
 *
 *  Copyright (C) 2023 Jack Andersen
 *
 * SPI master driver using generic bitbanged GPIO
 *
 * Copyright (C) 2006,2008 David Brownell
 * Copyright (C) 2017 Linus Walleij
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <linux/delay.h>

/*
 * This bitbanging SPI master driver should help make systems usable
 * when a native hardware SPI engine is not available, perhaps because
 * its driver isn't yet working or because the I/O pins it requires
 * are used for other purposes.
 *
 * platform_device->driver_data ... points to spi_gpio
 *
 * spi->controller_state ... reserved for bitbang framework code
 *
 * spi->master->dev.driver_data ... points to spi_gpio->bitbang
 */

struct spi_citrus {
	struct spi_bitbang		bitbang;
	struct citrus_core		*citrus_core;
};

/*----------------------------------------------------------------------*/

/*
 * Because the overhead of going through four GPIO procedure calls
 * per transferred bit can make performance a problem, this code
 * is set up so that you can use it in either of two ways:
 *
 *   - The slow generic way:  set up platform_data to hold the GPIO
 *     numbers used for MISO/MOSI/SCK, and issue procedure calls for
 *     each of them.  This driver can handle several such busses.
 *
 *   - The quicker inlined way:  only helps with platform GPIO code
 *     that inlines operations for constant GPIOs.  This can give
 *     you tight (fast!) inner loops, but each such bus needs a
 *     new driver.  You'll define a new C file, with Makefile and
 *     Kconfig support; the C code can be a total of six lines:
 *
 *		#define DRIVER_NAME	"myboard_spi2"
 *		#define	SPI_MISO_GPIO	119
 *		#define	SPI_MOSI_GPIO	120
 *		#define	SPI_SCK_GPIO	121
 *		#define	SPI_N_CHIPSEL	4
 *		#include "spi-gpio.c"
 */

#define GENERIC_BITBANG	/* vs tight inlines */

/*----------------------------------------------------------------------*/

static inline struct spi_citrus *__pure
spi_to_spi_citrus(const struct spi_device *spi)
{
	const struct spi_bitbang	*bang;
	struct spi_citrus		*spi_gpio;

	bang = spi_master_get_devdata(spi->master);
	spi_gpio = container_of(bang, struct spi_citrus, bitbang);
	return spi_gpio;
}

/* These helpers are in turn called by the bitbang inlines */
static inline void setsck(const struct spi_device *spi, int is_on)
{
	struct spi_citrus *spi_gpio = spi_to_spi_citrus(spi);
	citrus_core_set_spi_sck(spi_gpio->citrus_core, is_on);
}

static inline void setmosi(const struct spi_device *spi, int is_on)
{
	struct spi_citrus *spi_gpio = spi_to_spi_citrus(spi);
	citrus_core_set_spi_mosi(spi_gpio->citrus_core, is_on);
}

static inline int getmiso(const struct spi_device *spi)
{
	return 0;
}

/*
 * NOTE: Because we coexist with timing-sensitive I2C devices, we cannot clock
 * "as fast as we can".
 */
#define spidelay(x) ndelay(x)

#include "../../spi/spi-bitbang-txrx.h"

/*
 * These functions can leverage inline expansion of GPIO calls to shrink
 * costs for a txrx bit, often by factors of around ten (by instruction
 * count).  That is particularly visible for larger word sizes, but helps
 * even with default 8-bit words.
 *
 * REVISIT overheads calling these functions for each word also have
 * significant performance costs.  Having txrx_bufs() calls that inline
 * the txrx_word() logic would help performance, e.g. on larger blocks
 * used with flash storage or MMC/SD.  There should also be ways to make
 * GCC be less stupid about reloading registers inside the I/O loops,
 * even without inlined GPIO calls; __attribute__((hot)) on GCC 4.3?
 */

static u32 spi_citrus_txrx_word_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits, unsigned flags)
{
	if (unlikely(spi->mode & SPI_LSB_FIRST))
		return bitbang_txrx_le_cpha0(spi, nsecs, 0, flags, word, bits);
	else
		return bitbang_txrx_be_cpha0(spi, nsecs, 0, flags, word, bits);
}

static u32 spi_citrus_txrx_word_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits, unsigned flags)
{
	if (unlikely(spi->mode & SPI_LSB_FIRST))
		return bitbang_txrx_le_cpha1(spi, nsecs, 0, flags, word, bits);
	else
		return bitbang_txrx_be_cpha1(spi, nsecs, 0, flags, word, bits);
}

static u32 spi_citrus_txrx_word_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits, unsigned flags)
{
	if (unlikely(spi->mode & SPI_LSB_FIRST))
		return bitbang_txrx_le_cpha0(spi, nsecs, 1, flags, word, bits);
	else
		return bitbang_txrx_be_cpha0(spi, nsecs, 1, flags, word, bits);
}

static u32 spi_citrus_txrx_word_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits, unsigned flags)
{
	if (unlikely(spi->mode & SPI_LSB_FIRST))
		return bitbang_txrx_le_cpha1(spi, nsecs, 1, flags, word, bits);
	else
		return bitbang_txrx_be_cpha1(spi, nsecs, 1, flags, word, bits);
}

/*
 * These functions do not call setmosi or getmiso if respective flag
 * (SPI_MASTER_NO_RX or SPI_MASTER_NO_TX) is set, so they are safe to
 * call when such pin is not present or defined in the controller.
 * A separate set of callbacks is defined to get highest possible
 * speed in the generic case (when both MISO and MOSI lines are
 * available), as optimiser will remove the checks when argument is
 * constant.
 */

static u32 spi_citrus_spec_txrx_word_mode0(struct spi_device *spi,
					   unsigned nsecs, u32 word, u8 bits, unsigned flags)
{
	flags = spi->master->flags;
	if (unlikely(spi->mode & SPI_LSB_FIRST))
		return bitbang_txrx_le_cpha0(spi, nsecs, 0, flags, word, bits);
	else
		return bitbang_txrx_be_cpha0(spi, nsecs, 0, flags, word, bits);
}

static u32 spi_citrus_spec_txrx_word_mode1(struct spi_device *spi,
					   unsigned nsecs, u32 word, u8 bits, unsigned flags)
{
	flags = spi->master->flags;
	if (unlikely(spi->mode & SPI_LSB_FIRST))
		return bitbang_txrx_le_cpha1(spi, nsecs, 0, flags, word, bits);
	else
		return bitbang_txrx_be_cpha1(spi, nsecs, 0, flags, word, bits);
}

static u32 spi_citrus_spec_txrx_word_mode2(struct spi_device *spi,
					   unsigned nsecs, u32 word, u8 bits, unsigned flags)
{
	flags = spi->master->flags;
	if (unlikely(spi->mode & SPI_LSB_FIRST))
		return bitbang_txrx_le_cpha0(spi, nsecs, 1, flags, word, bits);
	else
		return bitbang_txrx_be_cpha0(spi, nsecs, 1, flags, word, bits);
}

static u32 spi_citrus_spec_txrx_word_mode3(struct spi_device *spi,
					   unsigned nsecs, u32 word, u8 bits, unsigned flags)
{
	flags = spi->master->flags;
	if (unlikely(spi->mode & SPI_LSB_FIRST))
		return bitbang_txrx_le_cpha1(spi, nsecs, 1, flags, word, bits);
	else
		return bitbang_txrx_be_cpha1(spi, nsecs, 1, flags, word, bits);
}

/*----------------------------------------------------------------------*/

static void spi_citrus_chipselect(struct spi_device *spi, int is_active)
{
	struct spi_citrus *spi_gpio = spi_to_spi_citrus(spi);

	/* set initial clock line level */
	if (is_active) {
		citrus_core_set_spi_sck(spi_gpio->citrus_core, spi->mode & SPI_CPOL);
	}
}

static int spi_citrus_setup(struct spi_device *spi)
{
	return spi_bitbang_setup(spi);
}

static int spi_citrus_set_direction(struct spi_device *spi, bool output)
{
	struct spi_citrus *spi_gpio = spi_to_spi_citrus(spi);
	int ret;

	if (output)
		return citrus_core_set_spi_mosi_direction_output(spi_gpio->citrus_core, 1);

	/*
	 * Only change MOSI to an input if using 3WIRE mode.
	 * Otherwise, MOSI could be left floating if there is
	 * no pull resistor connected to the I/O pin, or could
	 * be left logic high if there is a pull-up. Transmitting
	 * logic high when only clocking MISO data in can put some
	 * SPI devices in to a bad state.
	 */
	if (spi->mode & SPI_3WIRE) {
		ret = citrus_core_set_spi_mosi_direction_input(spi_gpio->citrus_core);
		if (ret)
			return ret;
	}
	/*
	 * Send a turnaround high impedance cycle when switching
	 * from output to input. Theoretically there should be
	 * a clock delay here, but as has been noted above, the
	 * nsec delay function for bit-banged GPIO is simply
	 * {} because bit-banging just doesn't get fast enough
	 * anyway.
	 */
	if (spi->mode & SPI_3WIRE_HIZ) {
		citrus_core_set_spi_sck(spi_gpio->citrus_core, !(spi->mode & SPI_CPOL));
		citrus_core_set_spi_sck(spi_gpio->citrus_core, !!(spi->mode & SPI_CPOL));
	}
	return 0;
}

static void spi_citrus_cleanup(struct spi_device *spi)
{
	spi_bitbang_cleanup(spi);
}

static int spi_citrus_prepare_hardware(struct spi_master *spi)
{
	struct spi_citrus	*spi_gpio;
	struct spi_bitbang	*bb;

	spi_gpio = spi_master_get_devdata(spi);
	bb = &spi_gpio->bitbang;

	citrus_core_lock_spi(spi_gpio->citrus_core);

	mutex_lock(&bb->lock);
	bb->busy = 1;
	mutex_unlock(&bb->lock);

	return 0;
}

static int spi_citrus_unprepare_hardware(struct spi_master *spi)
{
	struct spi_citrus	*spi_gpio;
	struct spi_bitbang	*bb;

	spi_gpio = spi_master_get_devdata(spi);
	bb = &spi_gpio->bitbang;

	mutex_lock(&bb->lock);
	bb->busy = 0;
	mutex_unlock(&bb->lock);

	citrus_core_unlock_spi(spi_gpio->citrus_core);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id spi_citrus_dt_ids[] = {
	{ .compatible = "spi-citrus" },
	{}
};
MODULE_DEVICE_TABLE(of, spi_citrus_dt_ids);

static int spi_citrus_probe_dt(struct platform_device *pdev,
			       struct spi_master *master)
{
	struct device_node *spi_node;

	spi_node = of_find_matching_node(pdev->dev.of_node, spi_citrus_dt_ids);
	if (!spi_node || !of_device_is_available(spi_node)) {
		dev_err_probe(&pdev->dev, -ENODEV, "spi-citrus device node not found\n");
		return -ENODEV;
	}
	dev_dbg(&pdev->dev, "Found spi-citrus node\n");

	master->dev.of_node = spi_node;
	master->use_gpio_descriptors = true;

	return 0;
}
#else
#error Must be compiled with CONFIG_OF
#endif

int spi_citrus_probe(struct platform_device *pdev, struct citrus_core *citrus)
{
	int				status;
	struct spi_master		*master;
	struct spi_citrus		*spi_gpio;
	struct device			*dev = &pdev->dev;
	struct spi_bitbang		*bb;

	dev_dbg(dev, "Probing spi-citrus\n");

	master = devm_spi_alloc_master(dev, sizeof(*spi_gpio));
	if (!master)
		return -ENOMEM;

	if (!pdev->dev.of_node) {
		dev_err_probe(dev, -ENODEV,
			      "spi-citrus must be probed with of_node\n");
		return -ENODEV;
	}

	status = spi_citrus_probe_dt(pdev, master);
	if (status)
		return status;

	spi_gpio = spi_master_get_devdata(master);
	spi_gpio->citrus_core = citrus;

	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	master->mode_bits = SPI_3WIRE | SPI_3WIRE_HIZ | SPI_CPHA | SPI_CPOL |
			    SPI_CS_HIGH | SPI_LSB_FIRST;

	master->bus_num = (s16)pdev->id;
	master->setup = spi_citrus_setup;
	master->cleanup = spi_citrus_cleanup;

	bb = &spi_gpio->bitbang;
	bb->master = master;
	/*
	 * There is some additional business, apart from driving the CS GPIO
	 * line, that we need to do on selection. This makes the local
	 * callback for chipselect always get called.
	 */
	master->flags |= SPI_MASTER_GPIO_SS;
	bb->chipselect = spi_citrus_chipselect;
	bb->set_line_direction = spi_citrus_set_direction;

	if (master->flags & SPI_MASTER_NO_TX) {
		bb->txrx_word[SPI_MODE_0] = spi_citrus_spec_txrx_word_mode0;
		bb->txrx_word[SPI_MODE_1] = spi_citrus_spec_txrx_word_mode1;
		bb->txrx_word[SPI_MODE_2] = spi_citrus_spec_txrx_word_mode2;
		bb->txrx_word[SPI_MODE_3] = spi_citrus_spec_txrx_word_mode3;
	} else {
		bb->txrx_word[SPI_MODE_0] = spi_citrus_txrx_word_mode0;
		bb->txrx_word[SPI_MODE_1] = spi_citrus_txrx_word_mode1;
		bb->txrx_word[SPI_MODE_2] = spi_citrus_txrx_word_mode2;
		bb->txrx_word[SPI_MODE_3] = spi_citrus_txrx_word_mode3;
	}
	bb->setup_transfer = spi_bitbang_setup_transfer;

	status = spi_bitbang_init(&spi_gpio->bitbang);
	if (status)
		return status;

	master->prepare_transfer_hardware = spi_citrus_prepare_hardware;
	master->unprepare_transfer_hardware = spi_citrus_unprepare_hardware;

	return devm_spi_register_master(&pdev->dev, master);
}

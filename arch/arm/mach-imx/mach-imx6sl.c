/*
 * Copyright 2013-2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/regmap.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <linux/gpio.h>
#include <linux/memblock.h>

#include "regs-anadig.h"
#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

/*EANAB EPD PMIC enable pins*/
#define MX6SL_PIN_PM_EPD_EN     IMX_GPIO_NR(3, 25)      /*KEY_ROW0 - PM_EPD_EN*/
#define MX6SL_PIN_PM_EPD_ENOP   IMX_GPIO_NR(2, 11)      /*EPD_PWR_COM - PM_EPD_ENOP*/
static DEFINE_MUTEX(force_bsession_lock);

static void __init imx6sl_fec_clk_init(void)
{
	struct regmap *gpr;

	/* set FEC clock from internal PLL clock source */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sl-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6SL_GPR1_FEC_CLOCK_MUX2_SEL_MASK, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6SL_GPR1_FEC_CLOCK_MUX1_SEL_MASK, 0);
	} else
		pr_err("failed to find fsl,imx6sl-iomux-gpr regmap\n");
}

static inline void imx6sl_fec_init(void)
{
	imx6sl_fec_clk_init();
	imx6_enet_mac_init("fsl,imx6sl-fec", "fsl,imx6sl-ocotp");
}

static void __init imx6sl_init_late(void)
{
	/* imx6sl reuses imx6q cpufreq driver */
	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ))
		platform_device_register_simple("imx6q-cpufreq", -1, NULL, 0);

	/* cpuidle will be enabled later for i.MX6SLL */
	if (cpu_is_imx6sll())
		imx6sll_cpuidle_init();
	else
		imx6sl_cpuidle_init();
}

#ifdef CONFIG_USB_REX_WAN
static void imx_anatop_override_usb2__vbus_detect(void)
{
	u32 contents;

	contents = readl_relaxed(MX6Q_IO_ADDRESS(MX6Q_ANATOP_BASE_ADDR) + HW_ANADIG_USB2_VBUS_DETECT);
	contents |= BM_ANADIG_USB2_VBUS_DETECT_VBUSVALID_TO_B;
	writel_relaxed(contents, MX6Q_IO_ADDRESS(MX6Q_ANATOP_BASE_ADDR) + HW_ANADIG_USB2_VBUS_DETECT);
}
#endif

//Fred USB bringup -- need to override VBUS detection based on the USB status from PMIC
static bool usbotg_bsession_on = false;

bool usbotg_bsession_forced(void)
{
	return usbotg_bsession_on;
}
EXPORT_SYMBOL(usbotg_bsession_forced);

void usbotg_force_bsession(bool connected)
{
        u32 contents;

	mutex_lock(&force_bsession_lock);
        contents = readl_relaxed(MX6Q_IO_ADDRESS(MX6Q_ANATOP_BASE_ADDR) + HW_ANADIG_USB1_VBUS_DETECT);

        contents |= BM_ANADIG_USB1_VBUS_DETECT_VBUS_OVERRIDE_EN;

        if (connected) {
                contents |= BM_ANADIG_USB1_VBUS_DETECT_BVALID_OVERRIDE;
                contents |= BM_ANADIG_USB1_VBUS_DETECT_VBUSVALID_OVERRIDE;
                contents &= ~BM_ANADIG_USB1_VBUS_DETECT_SESSEND_OVERRIDE;
        } else {
                contents &= ~BM_ANADIG_USB1_VBUS_DETECT_BVALID_OVERRIDE;
                contents &= ~BM_ANADIG_USB1_VBUS_DETECT_VBUSVALID_OVERRIDE;
                contents |= BM_ANADIG_USB1_VBUS_DETECT_SESSEND_OVERRIDE;
        }
        writel_relaxed(contents, MX6Q_IO_ADDRESS(MX6Q_ANATOP_BASE_ADDR) + HW_ANADIG_USB1_VBUS_DETECT);

	usbotg_bsession_on = connected;
	mutex_unlock(&force_bsession_lock);
}
EXPORT_SYMBOL(usbotg_force_bsession);

static void __init imx6sl_init_machine(void)
{
	struct device *parent;

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table, NULL, parent);

	if (!cpu_is_imx6sll())
		imx6sl_fec_init();
	imx_anatop_init();
#ifdef CONFIG_USB_REX_WAN
	imx_anatop_override_usb2__vbus_detect();
#endif
	imx6sl_pm_init();
}

static void __init imx6sl_init_irq(void)
{
	imx_gpc_check_dt();
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	irqchip_init();
}

static void __init imx6sl_map_io(void)
{
	debug_ll_io_init();
	imx6_pm_map_io();
#ifdef CONFIG_CPU_FREQ
	imx_busfreq_map_io();
#endif
}

int gpio_epd_init_pins(void)
{
        int ret = 0;

        ret = gpio_request(MX6SL_PIN_PM_EPD_EN, "epd_pm_en");
        if(unlikely(ret))
        {
                printk(KERN_ERR "Fred: Failed to request PM_EPD_EN\n");
                return ret;
        }
        ret = gpio_request(MX6SL_PIN_PM_EPD_ENOP, "epd_pm_enop");
        if(unlikely(ret))
        {
                printk(KERN_ERR "Fred: Failed to request PM_EPD_ENOP\n");
                goto free_epd_en;
        }


        /* set PM_EPD_EN and PM_EPD_ENOP to 0 */
        gpio_direction_output(MX6SL_PIN_PM_EPD_EN, 0);
        gpio_direction_output(MX6SL_PIN_PM_EPD_ENOP, 0);

        return ret;

free_epd_en:
        gpio_free(MX6SL_PIN_PM_EPD_EN);
        return ret;
}
EXPORT_SYMBOL(gpio_epd_init_pins);

void gpio_epd_enable_hv(int enable)
{
        gpio_direction_output(MX6SL_PIN_PM_EPD_EN, enable);
}
EXPORT_SYMBOL(gpio_epd_enable_hv);

void gpio_epd_enable_vcom(int enable)
{
        gpio_direction_output(MX6SL_PIN_PM_EPD_ENOP, enable);
}
EXPORT_SYMBOL(gpio_epd_enable_vcom);

void gpio_epd_free_pins(void)
{
        gpio_free(MX6SL_PIN_PM_EPD_EN);
        gpio_free(MX6SL_PIN_PM_EPD_ENOP);
}
EXPORT_SYMBOL(gpio_epd_free_pins);

extern unsigned long int ramoops_phys_addr;
extern unsigned long int ramoops_mem_size;
static void imx6sl_reserve(void)
{
	phys_addr_t phys;
	phys_addr_t max_phys;

#ifdef CONFIG_PSTORE_RAM
	max_phys = memblock_end_of_DRAM();
	/* reserve 64M for uboot avoid ram console data is cleaned by uboot */
	phys = memblock_alloc_base(SZ_1M, SZ_4K, max_phys - SZ_64M);
	if (phys) {
		memblock_remove(phys, SZ_1M);
		memblock_reserve(phys, SZ_1M);
		ramoops_phys_addr = phys;
		ramoops_mem_size = SZ_1M;
	} else {
		ramoops_phys_addr = 0;
		ramoops_mem_size = 0;
		pr_err("no memory reserve for ramoops.\n");
	}
#endif
	return;
}

static const char * const imx6sl_dt_compat[] __initconst = {
	"fsl,imx6sl",
	"fsl,imx6sll",
	NULL,
};

DT_MACHINE_START(IMX6SL, "Freescale i.MX6 SoloLite (Device Tree)")
	.map_io		= imx6sl_map_io,
	.init_irq	= imx6sl_init_irq,
	.init_machine	= imx6sl_init_machine,
	.init_late      = imx6sl_init_late,
	.dt_compat	= imx6sl_dt_compat,
	.reserve        = imx6sl_reserve,
MACHINE_END

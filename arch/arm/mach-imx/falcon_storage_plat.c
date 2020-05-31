/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/falcon_storage.h>

#include <linux/clk.h>

#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/mmc/host.h>
#include <linux/busfreq-imx.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/cpufreq.h>

#include "hardware.h"

#define SECTOR_SIZE	512

#define DEV_NODE	"/falcon"
#define USDHC2_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/usdhc@02194000"
#define uSDHC2_VEND_SPEC	0xC0
#define FRC_SDCLK_ON 		0x100
#define VSELECT 			0x02
#define EXT_DMA_EN 			0x01
static void __iomem *usdhc2 = NULL;

struct clk *qb_clk;
static int probe_done;

struct pltfm_falcon_data {
	struct pinctrl *pinctrl;
	struct clk *clks[4];
	bool clk_enabled;
};

static void __iomem * map_registers(const char *path){
	struct device_node *np;
	void __iomem *mem = NULL;
     	np = of_find_node_by_path(path);
	mem = of_iomap(np,0);
	of_node_put(np);
	return mem;
}

/*
 *  Block device Host controller parameters
 */
static struct falcon_blk_host_param mx7_falcon_blk_param = {
	.name = "mx7-falcon-mmc",
#if 1	 /*use Multisector I/O */
	.max_seg_size = 0xfe00,
	.max_hw_segs = 128,
	.max_phys_segs = 128,
	.max_req_size = 0xfe00 * 128,
	.max_blk_size = SECTOR_SIZE,
	.max_blk_count = 0xfe00 * 128 / SECTOR_SIZE,
#else	/* use Bounce Buffer */
	.max_seg_size = 0xfe00 * 128,
	.max_hw_segs = 1,
	.max_phys_segs = 128,
	.max_req_size = 0xfe00 * 128,
	.max_blk_size = SECTOR_SIZE,
	.max_blk_count = 0xfe00 * 128 / SECTOR_SIZE,
#endif
	.irq = -1,
	.dma_mask	= 0xffffffff,

	.heads	= 4,			/* same as the Linux SD driver */
	.sectors = 16,			/* same as the Linux SD driver */
};

static struct falcon_blk_host_param mx6_falcon_blk_param = {};

/**
 * Get block dev host controller parameter
 *
 * @return    addr of host param structure
 */
struct falcon_blk_host_param *falcon_blk_get_hostinfo(void)
{
	if (cpu_is_imx7d())
		return &mx7_falcon_blk_param;
	else
		return &mx6_falcon_blk_param;
}

/**
 * Do platform depending operations
 * This is called before real HW access is done.
 */
void falcon_blk_platform_pre(void)
{
	request_bus_freq(BUS_FREQ_HIGH);
}

/**
 * Do platform depending operations
 * This is called after real HW access is done.
 */
void falcon_blk_platform_post(void)
{
	release_bus_freq(BUS_FREQ_HIGH);
}

/**
 * Initialization for platform depending operations
 * This is called once when falcon block wrapper driver is initalized.
 */
void falcon_blk_platform_mx7_init(void)
{
	int i;
	struct device_node *np;
	const __be32 *hdl;
	struct clk *clk;
	struct clk_table_t {
		char *name;
	} clk_table[] = { {"per"}, {"ipg"}, {""} };

	np = of_find_node_by_path(DEV_NODE);

	if (!np) {
		pr_err("of_find_node_by_path error\n");
		return;
	}

	hdl = of_get_property(np, "timer", NULL);

	if (!hdl) {
		pr_err("of_get_property errror\n");
		return;
	}

	np = of_find_node_by_phandle(be32_to_cpup(hdl));

	if (!np) {
		pr_err("of_find_node_by_phandle error\n");
		return;
	}

	if (of_get_property(np, "clock-names", NULL)) {
		for (i = 0; strcmp(clk_table[i].name, ""); i++) {
			char *clk_name = clk_table[i].name;

			clk = of_clk_get_by_name(np, clk_name);

			if (IS_ERR(clk))
				pr_err("of_clk_get_by_name error: %s\n",
				       clk_name);
			else {
				clk_prepare_enable(clk);
				pr_info("qbblk(timer): Enabled %s clock\n",
					clk_name);
			}
		}
	} else {
		pr_err("of_get_property errror\n");
		return;
	}

	if (!probe_done) {
		pr_err("Don't setting storage...\n");
		return;
	}
}

void falcon_blk_platform_mx6_init(void)
{
	int i, irq;
	struct device_node *np;
	struct device_node *storage_np;

	np = of_find_node_by_path(DEV_NODE);

	if (!np) {
		pr_err("of_find_node_by_path error\n");
		return;
	}
	// params
	{
		int ret = 0;
		const char *str;

		ret = of_property_read_string(np, "storage-name", &str);
		if (0 != ret) {
			pr_err("of_get_property(storage-name) error\n");
		} else {
			mx6_falcon_blk_param.name = str;
		}
	}

	GET_FALCON_PROP_U32(mx6_falcon_blk_param, max_seg_size);
	GET_FALCON_PROP_U16(mx6_falcon_blk_param, max_hw_segs);
	GET_FALCON_PROP_U16(mx6_falcon_blk_param, max_phys_segs);
	GET_FALCON_PROP_U32(mx6_falcon_blk_param, max_req_size);
	GET_FALCON_PROP_U32(mx6_falcon_blk_param, max_blk_size);
	GET_FALCON_PROP_U32(mx6_falcon_blk_param, max_blk_count);

	GET_FALCON_PROP_S32(mx6_falcon_blk_param, irq);
	GET_FALCON_PROP_U64(mx6_falcon_blk_param, dma_mask);

	GET_FALCON_PROP_U8(mx6_falcon_blk_param, heads);
	GET_FALCON_PROP_U8(mx6_falcon_blk_param, sectors);

}

void falcon_blk_platform_init(void)
{
	if (cpu_is_imx7d())
		falcon_blk_platform_mx7_init();
	else
		falcon_blk_platform_mx6_init();
}

/**
 * Do platform depended operations
 * This function is called in suspend
 */
void falcon_blk_platform_suspend(void)
{
}

/**
 * Do platform depended operations
 * This function is called in resume.
 */
void falcon_blk_platform_resume(void)
{
}

#ifdef CONFIG_PM
static int falcon_platform_suspend(struct device *dev)
{
	return 0;
}

static int falcon_platform_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops falcon_sdhci_pmop = {
	SET_SYSTEM_SLEEP_PM_OPS(falcon_platform_suspend, falcon_platform_resume)
};

#endif


static int falcon_blk_platform_probe(struct platform_device *pdev)
{
	int ret;
	int i, irq;
	struct clk *clk;
	struct pinctrl *pinctrl;
	struct regulator *vmmc;
	struct clk_table_t {
		char *name;
	} clk_table[] = { {"per"}, {"ipg"}, {"ahb"}, {"epit"}, {""} };
        int err;
	struct pltfm_falcon_data *fln_data;

	fln_data = devm_kzalloc(&pdev->dev, sizeof(*fln_data), GFP_KERNEL);
	if (!fln_data) {
		err = -ENOMEM;
		goto end_falcon_probe;
	}

	/* Set clk */
	for (i = 0; strcmp(clk_table[i].name, ""); i++) {
		char *clk_name = clk_table[i].name;

		clk = devm_clk_get(&pdev->dev, clk_name);
		fln_data->clks[i] = clk;

		if (IS_ERR(clk))
			pr_err("of_clk_get_by_name error: %s\n", clk_name);
		else {
			clk_prepare_enable(clk);
			pr_info("qbblk(storage): Enabled %s clock\n", clk_name);
		}
	}

	/* Set pinctrl */
	pinctrl = devm_pinctrl_get(&pdev->dev);
	fln_data->pinctrl = pinctrl;
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		pr_err("failed to get pinctrl\n");
		return ret;
	}
	ret = pinctrl_select_state(pinctrl,
				   pinctrl_lookup_state(pinctrl,
							"default"));
	if (ret) {
		pr_err("failed to activate default pinctrl state\n");
		return ret;
	}
	pr_info("qbblk(storage): Set pinctrl\n");


	/* Get irq */
	irq = platform_get_irq(pdev, 0);
	if (irq != 0)
		mx6_falcon_blk_param.irq = irq;

	fln_data->clk_enabled = true;
	platform_set_drvdata(pdev, fln_data);

	return 0;
end_falcon_probe:
	return err;
}

static const struct of_device_id falcon_blk_dt_ids[] = {
	{ .compatible = "falcon_blk" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, falcon_blk_dt_ids);

static struct platform_driver falcon_blk_driver = {
	.driver		= {
		.name	= "falcon_blk",
		.of_match_table = falcon_blk_dt_ids,
#ifdef CONFIG_PM
		.pm = &falcon_sdhci_pmop,
#endif
	},
	.probe		= falcon_blk_platform_probe,
};
module_platform_driver(falcon_blk_driver);


/*
 * Local Variables:
 * mode: c
 * c-file-style: "K&R"
 * tab-width: 8
 * indent-tabs-mode: t
 * c-basic-offset: 8
 * End:
 */

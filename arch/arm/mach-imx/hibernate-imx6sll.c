/*
* Copyright (c) 2018 Amazon.com, Inc. or its affiliates. All rights reserved.
*
* PROPRIETARY/CONFIDENTIAL
*
*/

#include <linux/mm.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/dcache.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/workqueue.h>

#include "common.h"
#include "hardware.h"
#include "cpuidle.h"

/*
* is_hibernation = 1 for in-house hibernation
* is_hibernation = 0 for vendor hibernation
*/
#if defined(CONFIG_TOI)
extern u8 is_hibernation;
#endif

void usbotg_force_bsession(bool connected);

#define CCM_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/ccm@020c4000"
#define CCM_ANALOG_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/anatop@020c8000"
#define OCOTP_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/ocotp-ctrl@021bc000"
#define IOMUX_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/iomuxc@020e0000"
#define IOMUX_GPR_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/iomuxc-gpr@020e4000"
#define IOMUXC_SNVS_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/iomuxc-snvs@021c8000"
#define GPT_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/gpt@02098000"
#define GPC_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/gpc@020dc000"
#define GIC_DIST_BASE_DEVICE_TREE_PATH  "/interrupt-controller@00a01000", 0
#define GIC_CPU_BASE_DEVICE_TREE_PATH   "/interrupt-controller@00a01000", 1
#define USBC1_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/usb@02184000"
#define USBC2_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/usb@02184200"
#define USBMISC_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/usbmisc@02184800"
#define USBPHY1_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/usbphy@020c9000"
#define USBPHY2_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/usbphy@020ca000"
#define GPIO1_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/gpio@0209c000"
#define GPIO2_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/gpio@020a0000"
#define GPIO3_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/gpio@020a4000"
#define GPIO4_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/gpio@020a8000"
#define GPIO5_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/gpio@020ac000"
#define GPIO6_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/gpio@020b0000"
#define SNVS_GPR_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/snvs-gpr@0x021c4000"
#define SNVS_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02000000/snvs@020cc000"
#define I2C1_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/i2c@021a0000"
#define I2C2_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/i2c@021a4000"
#define I2C3_BASE_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/i2c@021a8000"
#define USDHC1_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/usdhc@02190000"
#define USDHC2_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/usdhc@02194000"
#define USDHC3_DEVICE_TREE_PATH  "/soc/aips-bus@02100000/usdhc@02198000"
#define UART4_DEVICE_TREE_PATH "/soc/aips-bus@02000000/spba-bus@02000000/serial@02018000"
#define UART1_DEVICE_TREE_PATH "/soc/aips-bus@02000000/spba-bus@02000000/serial@02020000"
#define UART2_DEVICE_TREE_PATH "/soc/aips-bus@02000000/spba-bus@02000000/serial@02024000"


#define	IOMUXC_GPR0_OFFS			0x0000
#define	IOMUXC_GPR1_OFFS			0x0004
#define	IOMUXC_GPR2_OFFS			0x0008
#define	IOMUXC_GPR3_OFFS			0x000C
#define	IOMUXC_GPR4_OFFS			0x0010
#define	IOMUXC_GPR5_OFFS			0x0014
#define	IOMUXC_GPR6_OFFS			0x0018
#define	IOMUXC_GPR7_OFFS			0x001C
#define	IOMUXC_GPR8_OFFS			0x0020
#define	IOMUXC_GPR9_OFFS			0x0024
#define	IOMUXC_GPR10_OFFS			0x0028
#define	IOMUXC_GPR11_OFFS			0x002C
#define	IOMUXC_GPR12_OFFS			0x0030
#define	IOMUXC_GPR13_OFFS			0x0034


#define GIC_CPU_CTRL                    0x00
#define GIC_CPU_PRIMASK                 0x04
#define GIC_CPU_BINPOINT                0x08
#define GIC_CPU_INTACK                  0x0c
#define GIC_CPU_EOI                     0x10
#define GIC_CPU_RUNNINGPRI              0x14
#define GIC_CPU_HIGHPRI                 0x18

#define GIC_DIST_CTRL                   0x000
#define GIC_DIST_CTR                    0x004
#define GIC_DIST_ENABLE_SET             0x100
#define GIC_DIST_ENABLE_CLEAR           0x180
#define GIC_DIST_PENDING_SET            0x200
#define GIC_DIST_PENDING_CLEAR          0x280
#define GIC_DIST_ACTIVE_BIT             0x300
#define GIC_DIST_PRI                    0x400
#define GIC_DIST_TARGET                 0x800
#define GIC_DIST_CONFIG                 0xc00
#define GIC_DIST_SOFTINT                0xf00

#define	GPC_CNTR_OFFS		0x0
#define	GPC_PGR_OFFS		0x4
#define	GPC_IMR1_OFFS		0x8
#define	GPC_IMR2_OFFS		0xC
#define	GPC_IMR3_OFFS		0x10
#define	GPC_IMR4_OFFS		0x14
#define	GPC_ISR1_OFFS		0x18
#define	GPC_ISR2_OFFS		0x1C
#define	GPC_ISR3_OFFS		0x20
#define	GPC_ISR4_OFFS		0x24
#define	PGC_DISPLAY_CTRL_OFFS	0x240
#define	PGC_DISPLAY_PUPSCR_OFFS	0x244
#define	PGC_DISPLAY_PDNSCR_OFFS	0x248
#define	PGC_DISPLAY_SR_OFFS	0x24C
#define	PGC_GPU_CTRL_OFFS	0x260
#define	PGC_GPU_PUPSCR_OFFS	0x264
#define	PGC_GPU_PDNSCR_OFFS	0x268
#define	PGC_GPU_SR_OFFS		0x26C
#define	PGC_CPU_CTRL_OFFS	0x2A0
#define	PGC_CPU_PUPSCR_OFFS	0x2A4
#define	PGC_CPU_PDNSCR_OFFS	0x2A8
#define	PGC_CPU_SR_OFFS		0x2AC
#define	DVFSC_THRS_OFFS		0x180
#define	DVFSC_COUN_OFFS		0x184
#define	DVFSC_SIG1_OFFS		0x188
#define	DVFSC_DVFSSIG0_OFFS	0x18C
#define	DVFSC_DVFSGPC0_OFFS	0x190
#define	DVFSC_DVFSGPC1_OFFS	0x194
#define	DVFSC_DVFSGPBT_OFFS	0x198
#define	DVFSC_DVFSEMAC_OFFS	0x19C
#define	DVFSC_CNTR_OFFS		0x1A0
#define	DVFSC_DVFSLTR0_0_OFFS	0x1A4
#define	DVFSC_DVFSLTR0_1_OFFS	0x1A8
#define	DVFSC_DVFSLTR1_0_OFFS	0x1AC
#define	DVFSC_DVFSLTR1_1_OFFS	0x1B0
#define	DVFSC_DVFSPT0_OFFS	0x1B4
#define	DVFSC_DVFSPT1_OFFS	0x1B8
#define	DVFSC_DVFSPT2_OFFS	0x1BC
#define	DVFSC_DVFSPT3_OFFS	0x1C0


#define BM_ANADIG_ANA_MISC2_CONTROL0 0x00000080
#define BM_ANADIG_ANA_MISC2_REG0_OK 0x00000040
#define BM_ANADIG_ANA_MISC2_REG0_ENABLE_BO 0x00000020
#define BM_ANADIG_ANA_MISC2_RSVD0 0x00000010
#define BM_ANADIG_ANA_MISC2_REG0_BO_STATUS 0x00000008
#define BP_ANADIG_ANA_MISC2_REG0_BO_OFFSET      0
#define BM_ANADIG_ANA_MISC2_REG0_BO_OFFSET 0x00000007
#define BF_ANADIG_ANA_MISC2_REG0_BO_OFFSET(v)  \
	(((v) << 0) & BM_ANADIG_ANA_MISC2_REG0_BO_OFFSET)



#define UART_UCR1_OFFS  0x80
#define UART_UCR2_OFFS  0x84
#define UART_UCR3_OFFS  0x88
#define UART_UCR4_OFFS  0x8C
#define UART_UFCR_OFFS  0x90
#define UART_USR1_OFFS  0x94
#define UART_USR2_OFFS  0x98
#define UART_UESC_OFFS  0x9C
#define UART_UTIM_OFFS  0xA0
#define UART_UBIR_OFFS  0xA4
#define UART_UBMR_OFFS  0xA8
#define UART_UBRC_OFFS  0xAC
#define UART_ONEMS_OFFS 0xB0
#define UART_UTS_OFFS   0xB4
#define UART_UMCR_OFFS  0xB8

#define GPIO_DR         0x0
#define GPIO_GDIR       0x4
#define GPIO_PSR        0x8
#define GPIO_ICR1       0xc
#define GPIO_ICR2       0x10
#define GPIO_IMR        0x14
#define GPIO_ISR        0x18
#define GPIO_EDGE_SEL   0x1c

#define	GPT_CR_OFFS	0x000
#define	GPT_PR_OFFS	0x004
#define	GPT_SR_OFFS	0x008
#define	GPT_IR_OFFS	0x00C
#define	GPT_OCR1_OFFS	0x010
#define	GPT_OCR2_OFFS	0x014
#define	GPT_OCR3_OFFS	0x018
#define	GPT_ICR1_OFFS	0x01C
#define	GPT_ICR2_OFFS	0x020
#define	GPT_CNT_OFFS	0x024

#define	CCM_CCR_OFFS	0x00
#define	CCM_CCDR_OFFS	0x04
#define	CCM_CSR_OFFS	0x08
#define	CCM_CCSR_OFFS	0x0C
#define	CCM_CACRR_OFFS	0x10
#define	CCM_CBCDR_OFFS	0x14
#define	CCM_CBCMR_OFFS	0x18
#define	CCM_CSCMR1_OFFS	0x1C
#define	CCM_CSCMR2_OFFS	0x20
#define	CCM_CSCDR1_OFFS	0x24
#define	CCM_CS1CDR_OFFS	0x28
#define	CCM_CS2CDR_OFFS	0x2C
#define	CCM_CDCDR_OFFS	0x30
#define	CCM_CHSCCDR_OFFS	0x34
#define	CCM_CSCDR2_OFFS	0x38
#define	CCM_CSCDR3_OFFS	0x3C
#define	CCM_CDHIPR_OFFS	0x48
#define	CCM_CTOR_OFFS	0x50
#define	CCM_CLPCR_OFFS	0x54
#define	CCM_CISR_OFFS	0x58
#define	CCM_CIMR_OFFS	0x5C
#define	CCM_CCOSR_OFFS	0x60
#define	CCM_CGPR_OFFS	0x64
#define	CCM_CCGR0_OFFS	0x68
#define	CCM_CCGR1_OFFS	0x6C
#define	CCM_CCGR2_OFFS	0x70
#define	CCM_CCGR3_OFFS	0x74
#define	CCM_CCGR4_OFFS	0x78
#define	CCM_CCGR5_OFFS	0x7C
#define	CCM_CCGR6_OFFS	0x80
#define	CCM_CMEOR_OFFS	0x88

#define	CCM_ANALOG_PLL_ARM_OFFS		0x000
#define	CCM_ANALOG_PLL_ARM_SET_OFFS	0x004
#define	CCM_ANALOG_PLL_ARM_CLR_OFFS	0x008
#define	CCM_ANALOG_PLL_ARM_TOG_OFFS	0x00C
#define	CCM_ANALOG_PLL_USB1_OFFS	0x010
#define	CCM_ANALOG_PLL_USB1_SET_OFFS	0x014
#define	CCM_ANALOG_PLL_USB1_CLR_OFFS	0x018
#define	CCM_ANALOG_PLL_USB1_TOG_OFFS	0x01C
#define	CCM_ANALOG_PLL_USB2_OFFS	0x020
#define	CCM_ANALOG_PLL_USB2_SET_OFFS	0x024
#define	CCM_ANALOG_PLL_USB2_CLR_OFFS	0x028
#define	CCM_ANALOG_PLL_USB2_TOG_OFFS	0x02C
#define	CCM_ANALOG_PLL_SYS_OFFS		0x030
#define	CCM_ANALOG_PLL_SYS_SET_OFFS	0x034
#define	CCM_ANALOG_PLL_SYS_CLR_OFFS	0x038
#define	CCM_ANALOG_PLL_SYS_TOG_OFFS	0x03C
#define	CCM_ANALOG_PLL_SYS_SS_OFFS	0x040
#define	CCM_ANALOG_PLL_SYS_NUM_OFFS	0x050
#define	CCM_ANALOG_PLL_SYS_DENOM_OFFS	0x060
#define	CCM_ANALOG_PLL_AUDIO_OFFS	0x070
#define	CCM_ANALOG_PLL_AUDIO_SET_OFFS	0x074
#define	CCM_ANALOG_PLL_AUDIO_CLR_OFFS	0x078
#define	CCM_ANALOG_PLL_AUDIO_TOG_OFFS	0x07C
#define	CCM_ANALOG_PLL_AUDIO_NUM_OFFS	0x080
#define	CCM_ANALOG_PLL_AUDIO_DENOM_OFFS	0x090
#define	CCM_ANALOG_PLL_VIDEO_OFFS	0x0A0
#define	CCM_ANALOG_PLL_VIDEO_SET_OFFS	0x0A4
#define	CCM_ANALOG_PLL_VIDEO_CLR_OFFS	0x0A8
#define	CCM_ANALOG_PLL_VIDEO_TOG_OFFS	0x0AC
#define	CCM_ANALOG_PLL_VIDEO_NUM_OFFS	0x0B0
#define	CCM_ANALOG_PLL_VIDEO_DENOM_OFFS	0x0C0
#define	CCM_ANALOG_PLL_MLB_OFFS		0x0D0
#define	CCM_ANALOG_PLL_MLB_SET_OFFS	0x0D4
#define	CCM_ANALOG_PLL_MLB_CLR_OFFS	0x0D8
#define	CCM_ANALOG_PLL_MLB_TOG_OFFS	0x0DC
#define	CCM_ANALOG_PLL_ENET_OFFS	0x0E0
#define	CCM_ANALOG_PLL_ENET_SET_OFFS	0x0E4
#define	CCM_ANALOG_PLL_ENET_CLR_OFFS	0x0E8
#define	CCM_ANALOG_PLL_ENET_TOG_OFFS	0x0EC
#define	CCM_ANALOG_PFD_480_OFFS		0x0F0
#define	CCM_ANALOG_PFD_480_SET_OFFS	0x0F4
#define	CCM_ANALOG_PFD_480_CLR_OFFS	0x0F8
#define	CCM_ANALOG_PFD_480_TOG_OFFS	0x0FC
#define	CCM_ANALOG_PFD_528_OFFS		0x100
#define	CCM_ANALOG_PFD_528_SET_OFFS	0x104
#define	CCM_ANALOG_PFD_528_CLR_OFFS	0x108
#define	CCM_ANALOG_PFD_528_TOG_OFFS	0x10C
#define	CCM_ANALOG_REG_1P1_OFFS		0x110
#define	CCM_ANALOG_REG_3P0_OFFS		0x120
#define	CCM_ANALOG_REG_2P5_OFFS		0x130
#define	CCM_ANALOG_REG_CORE_OFFS	0x140
#define	CCM_ANALOG_MISC0_OFFS		0x150
#define	CCM_ANALOG_MISC0_SET_OFFS	0x154
#define	CCM_ANALOG_MISC0_CLR_OFFS	0x158
#define	CCM_ANALOG_MISC0_TOG_OFFS	0x15C
#define	CCM_ANALOG_MISC1_OFFS		0x160
#define	CCM_ANALOG_MISC2_OFFS		0x170
#define	CCM_ANALOG_MISC2_SET_OFFS	0x174
#define	CCM_ANALOG_MISC2_CLR_OFFS	0x178
#define	CCM_ANALOG_MISC2_TOG_OFFS	0x17C
#define	CCM_ANALOG_DIGPROG_OFFS		0x260

/* Define the bits in register CCR */
#define MXC_CCM_CCR_RBC_EN			(1 << 27)
#define MXC_CCM_CCR_REG_BYPASS_CNT_MASK		(0x3F << 21)
#define MXC_CCM_CCR_REG_BYPASS_CNT_OFFSET	(21)
#define MXC_CCM_CCR_WB_COUNT_MASK		(0x7 << 16)
#define MXC_CCM_CCR_WB_COUNT_OFFSET		(16)
#define MXC_CCM_CCR_COSC_EN			(1 << 12)
#define MXC_CCM_CCR_OSCNT_MASK			(0xFF)
#define MXC_CCM_CCR_OSCNT_OFFSET		(0)

/* Define the bits in register CBCDR */
#define MXC_CCM_CBCDR_PERIPH_CLK2_PODF_MASK	(0x7 << 27)
#define MXC_CCM_CBCDR_PERIPH_CLK2_PODF_OFFSET	(27)
#define MXC_CCM_CBCDR_PERIPH2_CLK_SEL		(1 << 26)
#define MXC_CCM_CBCDR_PERIPH_CLK_SEL		(1 << 25)
#define MXC_CCM_CBCDR_MMDC_CH0_PODF_MASK	(0x7 << 19)
#define MXC_CCM_CBCDR_MMDC_CH0_PODF_OFFSET	(19)
#define MXC_CCM_CBCDR_AXI_PODF_MASK		(0x7 << 16)
#define MXC_CCM_CBCDR_AXI_PODF_OFFSET		(16)
#define MXC_CCM_CBCDR_AHB_PODF_MASK		(0x7 << 10)
#define MXC_CCM_CBCDR_AHB_PODF_OFFSET		(10)
#define MXC_CCM_CBCDR_IPG_PODF_MASK		(0x3 << 8)
#define MXC_CCM_CBCDR_IPG_PODF_OFFSET		(8)
#define MXC_CCM_CBCDR_AXI_ALT_SEL_MASK		(1 << 7)
#define MXC_CCM_CBCDR_AXI_ALT_SEL_OFFSET	(7)
#define MXC_CCM_CBCDR_AXI_SEL			(1 << 6)
#define MXC_CCM_CBCDR_MMDC_CH1_PODF_MASK	(0x7 << 3)
#define MXC_CCM_CBCDR_MMDC_CH1_PODF_OFFSET	(3)
#define MXC_CCM_CBCDR_PERIPH2_CLK2_PODF_MASK	(0x7 << 0)
#define MXC_CCM_CBCDR_PERIPH2_CLK2_PODF_OFFSET	(0)


/* Define the bits in register CBCMR */
#define MXC_CCM_CBCMR_GPU3D_SHADER_PODF_MASK		(0x7 << 29)
#define MXC_CCM_CBCMR_GPU3D_SHADER_PODF_OFFSET		(29)
#define MXC_CCM_CBCMR_GPU3D_CORE_PODF_MASK		(0x7 << 26)
#define MXC_CCM_CBCMR_GPU3D_CORE_PODF_OFFSET		(26)
#define MXC_CCM_CBCMR_GPU2D_CORE_PODF_MASK		(0x7 << 23)
#define MXC_CCM_CBCMR_GPU2D_CORE_PODF_OFFSET		(23)
#define MXC_CCM_CBCMR_PRE_PERIPH2_CLK_SEL_MASK		(0x3 << 21)
#define MXC_CCM_CBCMR_PRE_PERIPH2_CLK_SEL_OFFSET	(21)
#define MXC_CCM_CBCMR_PERIPH2_CLK2_SEL			(1 << 20)
#define MXC_CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK		(0x3 << 18)
#define MXC_CCM_CBCMR_PRE_PERIPH_CLK_SEL_OFFSET		(18)
#define MXC_CCM_CBCMR_GPU2D_CLK_SEL_MASK		(0x3 << 16)
#define MXC_CCM_CBCMR_GPU2D_CLK_SEL_OFFSET		(16)
#define MXC_CCM_CBCMR_MLB_CLK_SEL_MASK			(0x3 << 16)
#define MXC_CCM_CBCMR_MLB_CLK_SEL_OFFSET		(16)
#define MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_MASK		(0x3 << 14)
#define MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_OFFSET		(14)
#define MXC_CCM_CBCMR_PERIPH_CLK2_SEL_MASK		(0x3 << 12)
#define MXC_CCM_CBCMR_PERIPH_CLK2_SEL_OFFSET		(12)
#define MXC_CCM_CBCMR_VDOAXI_CLK_SEL			(1 << 11)
#define MXC_CCM_CBCMR_PCIE_AXI_CLK_SEL			(1 << 10)
#define MXC_CCM_CBCMR_GPU3D_SHADER_CLK_SEL_MASK		(0x3 << 8)
#define MXC_CCM_CBCMR_GPU3D_SHADER_CLK_SEL_OFFSET	(8)
#define MXC_CCM_CBCMR_GPU3D_CORE_CLK_SEL_MASK		(0x3 << 4)
#define MXC_CCM_CBCMR_GPU3D_CORE_CLK_SEL_OFFSET		(4)
#define MXC_CCM_CBCMR_GPU3D_AXI_CLK_SEL			(1 << 1)
#define MXC_CCM_CBCMR_GPU2D_AXI_CLK_SEL			(1 << 0)

#define USBNC_BASE                 0x02184800
#define USBNC_END              0x0218481C
#define USBC_BASE                  0x02184000
#define USBC_END               0x021845A8
#define USBPHY_BASE                0x020C9000
#define USBPHY_END             0x020CA080
#define USB_ANALOG_BASE            0x020C81A0
#define USB_ANALOG_END         0x020C8280

#define IMX6SLL_IRAM_SAVE_SIZE SZ_128K
// #define MX6Q_IRAM_BASE_ADDR 0x00904000
#define MX6Q_IRAM_BASE_ADDR 0x00900000
#define L2_CACHE_SIZE 		SZ_4K
#define L2_CACHE_BASE_ADDR 	0x00A02000

#define IMX6SLL_WDT_BASE		0x20BC000
#define IMX6SLL_WDT_WSR_OFF	0x02
#define IMX6SLL_WDT_SEQ1		0x5555
#define IMX6SLL_WDT_SEQ2		0xAAAA

#define WAIT_CDHIPR		readl_relaxed(ccm + CCM_CDHIPR_OFFS)
#define OFFS_MASK		0xfff

#define ANALOG_PLL_LOCK			(1 << 31)
#define ANALOG_PLL_BYPASS		(1 << 16)
#define ANALOG_PLL_ENABLE		(1 << 13)
#define ANALOG_PLL_POWER_DOWN	(1 << 12)
#define FRAC_MASK				0x3F
#define PFD_CLKGATE			0x80
#define ANALOG_PFD_FRAC_MASK	((FRAC_MASK << 24) | (FRAC_MASK << 16) | (FRAC_MASK << 8) | (FRAC_MASK << 0))
#define ANALOG_PFD_CLKGATE	((PFD_CLKGATE << 24) | (PFD_CLKGATE << 16) | (PFD_CLKGATE << 8) | (PFD_CLKGATE << 0))

/* SRC */
#define SRC_BASE_ADDR			0x020d8000
#define SRC_SCR_OFFSET			0x000

#define LOCAL_TIMER_LOAD		0x00
#define LOCAL_TIMER_CNTR		0x04
#define LOCAL_TIMER_CTRL		0x08
#define LOCAL_TIMER_INTSTAT		0x0C
#define LOCAL_TWD_ADDR			(SCU_BASE_ADDR + 0x600)

#define SCU_BASE_ADDR	0x00A00000
#define SCU_CTRL 		0x00

static u32 gic_storage[1000] = {0};
static u32 *gic_storage_ptr = NULL;
static u32 *storageBuffer, *currentPtr= NULL;
static void __iomem *uart1 = NULL;
static void __iomem *uart2 = NULL;
static void __iomem *uart4 = NULL;
static void __iomem *gpio1 = NULL; 
static void __iomem *gpio2 = NULL; 
static void __iomem *gpio3 = NULL; 
static void __iomem *gpio4 = NULL; 
static void __iomem *gpio5 = NULL; 
static void __iomem *gpio6 = NULL; 
static void __iomem *iomux = NULL;
static void __iomem *iomux_gpr = NULL;
static void __iomem *iomuxc_snvs = NULL;
static void __iomem *gpt = NULL;
static void __iomem *gpc = NULL;
static void __iomem *ccm = NULL;
static void __iomem *ccm_analog = NULL;
static void __iomem *usdhc1 = NULL;
static void __iomem *usdhc2 = NULL;
static void __iomem *usdhc3 = NULL;
static void __iomem *snvs_gpr = NULL;
static void __iomem *snvs_base = NULL;
static void __iomem *gic_dist_base = NULL;
static void __iomem *gic_cpu_base = NULL;
static void __iomem *i2c1_base = NULL; 
static void __iomem *i2c2_base = NULL; 
static void __iomem *i2c3_base = NULL; 
static u32 *save_sram = NULL; 
static u32 *sram_base = NULL; 
static u32 scu_base;
static u32 wdt_base;
static u32 src_base;
static u32 local_twd_base;
static u32 *save_l2_cache = NULL; 
static u32 *l2_cache_base = NULL; 
static void __iomem *usbc1_base = NULL;
static void __iomem *usbc2_base = NULL;
static void __iomem *usbmisc_base = NULL;
static void __iomem *usbphy1_base = NULL;
static void __iomem *usbphy2_base = NULL;
static void __iomem *usb_analog_base = NULL;
static void __iomem *ocotp_base = NULL;
static u32 ccm_ccr, ccm_cacrr, ccm_cbcdr, ccm_cbcmr;
static u32 global_counter_save = 0;
static u32 global_counter_restore = 0;
static u32 gicd_ctrl, gicc_ctrl, gicc_primask, gicc_binpoint;
static u32 pll1_sys, pll2_528, pll3_480usb1, pll4_audio, pll5_video, pll6_enet, pll7_480usb2, pll2_pfd_528, pll3_pfd_480;

u32 ocotp_regs_offset [] = {
	// 21B_C4E0
	// 0x4E0,
};

// usbotg1: usb@02184000 { --> usbc
// usbotg2: usb@02184200 { --> usbc
u32 usbc_regs_offset [] = {
	// 0x0,
	// 0x4,
	// 0x8,
	// 0xC,
	// 0x10,
	// 0x14,
	// 0x18,
	// 0x1C,
	// 0x20,
	// 0x24,
	// 0x28,
	// 0x2C,
	// 0x30,
	// 0x34,
	// 0x38,
	// 0x3C,
	// 0x40,
	// 0x44,
	// 0x48,
	// 0x4C,
	// 0x50,
	// 0x54,
	// 0x58,
	// 0x5C,
	// 0x60,
	// 0x64,
	// 0x68,
	// 0x6C,
	// 0x70,
	// 0x74,
	// 0x78,
	// 0x7C,
	0x80,
	0x84,
	0x88,
	0x8C,
	0x90,
	// 0x94,
	// 0x98,
	// 0x9C,
	// 0xA0,
	// 0xA4,
	// 0xA8,
	// 0xAC,
	// 0xB0,
	// 0xB4,
	// 0xB8,
	// 0xBC,
	// 0xC0,
	// 0xC4,
	// 0xC8,
	// 0xCC,
	// 0xD0,
	// 0xD4,
	// 0xD8,
	// 0xDC,
	// 0xE0,
	// 0xE4,
	// 0xE8,
	// 0xEC,
	// 0xF0,
	// 0xF4,
	// 0xF8,
	// 0xFC,
	// 0x100,
	// 0x104,
	// 0x108,
	// 0x10C,
	// 0x110,
	// 0x114,
	// 0x118,
	// 0x11C,
	// 0x120,
	// 0x124,
	// 0x128,
	// 0x12C,
	// 0x130,
	// 0x134,
	// 0x138,
	// 0x13C,
	0x140,
	0x144,
	0x148,
	0x14C,
	// 0x150,
	0x154,
	0x158,
	0x15C,
	0x160,
	0x164,
	// 0x168,
	// 0x16C,
	// 0x170,
	// 0x174,
	0x178,
	0x17C,
	0x180,
	0x184,
	// 0x188,
	// 0x18C,
	// 0x190,
	// 0x194,
	// 0x198,
	// 0x19C,
	// 0x1A0,
	0x1A4,
	0x1A8,
	0x1AC,
	0x1B0,
	0x1B4,
	// 0x1B8,
	0x1BC,
	0x1C0,
	0x1C4,
	0x1C8,
	0x1CC,
	0x1D0,
	0x1D4,
	0x1D8,
	0x1DC,
};

// usbmisc: usbmisc@02184800 { --> usbnc
u32 usbmisc_regs_offset [] = {
      0x0,
      0x4,
      0x8,
      0xC,
};


u32 usbphy_regs_offset [] = {
	0x0,
	0x4,
	0x8,
	0xC,
	0x10,
	0x14,
	0x18,
	0x1C,
	0x20,
	0x24,
	0x28,
	0x2C,
	0x30,
	0x34,
	0x38,
	0x3C,
	0x40,
	// 0x44,
	// 0x48,
	// 0x4C,
	0x50,
	0x54,
	0x58,
	0x5C,
	// 0x60,
	// 0x64,
	// 0x68,
	// 0x6C,
	0x70,
	0x74,
	0x78,
	0x7C,
	// 0x80,
};

u32 usb_analog_regs_offset [] = {
	0xA0,
	0xA4,
	0xA8,
	0xAC,
	0xB0,
	0xB4,
	0xB8,
	0xBC,
	// 0xC0,
	// 0xD0,
	0xF0,
	0xF4,
	0xF8,
	0xFC,
};


u32 usdhci_regs_offset [] = {
	// 0x0,
	// 0x4,
	// 0x8,
	// 0xC,
	// 0x20,
	0x28,
	// 0x2c,
	// 0x34,
	// 0x38,
	// 0x44,
	// 0x48,
	// 0x58,
	// 0x60,
	// 0x68,
	// 0x70,
	// 0xc0,
	// 0xc4,
	// 0xc8,
	// 0xcc
};

u32 snvs_gpr_regs_offset [] = {
	0x0,
	0x4,
	0x8,
	0xC,
};


u32 snvs_regs_offset [] = {
	0x0,
	0x4,
	0x8,
	0x14,
	0x24,
	0x28,
	0x2C,
	0x30,
	0x34,
	0x38,
	0x4C,
	0x5C,
	0x60,
	0x68,
	0x34
};


u32 iomuxc_snvs_regs_offset [] = {
	0x0,
	0x4,
	0x8,
	0xC,
	0x10,
	0x14,
	0x18,
	0x1C,
	0x20,
	0x24,
};

u32 i2c_regs_offset [] = {
	0x0,
	0x4,
	0x8,
	0xC,
	0x10,
	0x14,
};




u32 uart_regs_offset[] = {
	UART_UTIM_OFFS,
	UART_ONEMS_OFFS,
	UART_UCR4_OFFS,
	UART_UCR3_OFFS,
	UART_UCR2_OFFS,
	UART_UBMR_OFFS,
	UART_UBIR_OFFS,
	UART_UFCR_OFFS,
	UART_UCR1_OFFS,
};

static u32 gpio_regs_offset[] = {
	GPIO_DR,
	GPIO_GDIR,
	//      GPIO_PSR, //R
	GPIO_ICR1,
	GPIO_ICR2,
	GPIO_IMR,
	//      GPIO_ISR, //w1c
	GPIO_EDGE_SEL,
};

static u32 iomux_gpr_regs_offset[] = {
	IOMUXC_GPR0_OFFS,
	IOMUXC_GPR1_OFFS,
	IOMUXC_GPR2_OFFS,
	IOMUXC_GPR3_OFFS,
	IOMUXC_GPR4_OFFS,
	IOMUXC_GPR5_OFFS,
	IOMUXC_GPR6_OFFS,
	IOMUXC_GPR7_OFFS,
	IOMUXC_GPR8_OFFS,
	IOMUXC_GPR9_OFFS,
	IOMUXC_GPR10_OFFS,
	IOMUXC_GPR11_OFFS,
	IOMUXC_GPR12_OFFS,
	IOMUXC_GPR13_OFFS,
};



static u32 iomux_regs_offset[] = {
	0x14,
	0x18,
	0x1C,
	0x20,
	0x24,
	0x28,
	0x2C,
	0x30,
	0x34,
	0x38,
	0x3C,
	0x40,
	0x44,
	0x48,
	0x4C,
	0x50,
	0x54,
	0x58,
	0x5C,
	0x60,
	0x64,
	0x68,
	0x6C,
	0x70,
	0x74,
	0x78,
	0x7C,
	0x80,
	0x84,
	0x88,
	0x8C,
	0x90,
	0x94,
	0x98,
	0x9C,
	0xA0,
	0xA4,
	0xA8,
	0xAC,
	0xB0,
	0xB4,
	0xB8,
	0xBC,
	0xC0,
	0xC4,
	0xC8,
	0xCC,
	0xD0,
	0xD4,
	0xD8,
	0xDC,
	0xE0,
	0xE4,
	0xE8,
	0xEC,
	0xF0,
	0xF4,
	0xF8,
	0xFC,
	0x100,
	0x104,
	0x108,
	0x10C,
	0x110,
	0x114,
	0x118,
	0x11C,
	0x120,
	0x124,
	0x128,
	0x12C,
	0x130,
	0x134,
	0x138,
	0x13C,
	0x140,
	0x144,
	0x148,
	0x14C,
	0x150,
	0x154,
	0x158,
	0x15C,
	0x160,
	0x164,
	0x168,
	0x16C,
	0x170,
	0x174,
	0x178,
	0x17C,
	0x180,
	0x184,
	0x188,
	0x18C,
	0x190,
	0x194,
	0x198,
	0x19C,
	0x1A0,
	0x1A4,
	0x1A8,
	0x1AC,
	0x1B0,
	0x1B4,
	0x1B8,
	0x1BC,
	0x1C0,
	0x1C4,
	0x1C8,
	0x1CC,
	0x1D0,
	0x1D4,
	0x1D8,
	0x1DC,
	0x1E0,
	0x1E4,
	0x1E8,
	0x1EC,
	0x1F0,
	0x1F4,
	0x1F8,
	0x1FC,
	0x200,
	0x204,
	0x208,
	0x20C,
	0x210,
	0x214,
	0x218,
	0x21C,
	0x220,
	0x224,
	0x228,
	0x22C,
	0x230,
	0x234,
	0x238,
	0x23C,
	0x240,
	0x244,
	0x248,
	0x24C,
	0x250,
	0x254,
	0x258,
	0x25C,
	0x260,
	0x264,
	0x268,
	0x26C,
	0x270,
	0x274,
	0x278,
	0x27C,
	0x280,
	0x284,
	0x288,
	0x28C,
	0x290,
	0x294,
	0x298,
	0x29C,
	0x2A0,
	0x2A4,
	0x2A8,
	0x2AC,
	0x2B0,
	0x2B4,
	0x2B8,
	0x2BC,
	0x2C0,
	0x2C4,
	0x2C8,
	0x2CC,
	0x2D0,
	0x2D4,
	0x2D8,
	0x2DC,
	0x2E0,
	0x2E4,
	0x2E8,
	0x2EC,
	0x2F0,
	0x2F4,
	0x2F8,
	0x2FC,
	0x300,
	0x304,
	0x308,
	0x30C,
	0x310,
	0x314,
	0x318,
	0x31C,
	0x320,
	0x324,
	0x328,
	0x32C,
	0x330,
	0x334,
	0x338,
	0x33C,
	0x340,
	0x344,
	0x348,
	0x34C,
	0x350,
	0x354,
	0x358,
	0x35C,
	0x360,
	0x364,
	0x368,
	0x36C,
	0x370,
	0x374,
	0x378,
	0x37C,
	0x380,
	0x384,
	0x388,
	0x38C,
	0x390,
	0x394,
	0x398,
	0x39C,
	0x3A0,
	0x3A4,
	0x3A8,
	0x3AC,
	0x3B0,
	0x3B4,
	0x3B8,
	0x3BC,
	0x3C0,
	0x3C4,
	0x3C8,
	0x3CC,
	0x3D0,
	0x3D4,
	0x3D8,
	0x3DC,
	0x3E0,
	0x3E4,
	0x3E8,
	0x3EC,
	0x3F0,
	0x3F4,
	0x3F8,
	0x3FC,
	0x400,
	0x404,
	0x408,
	0x40C,
	0x410,
	0x414,
	0x418,
	0x41C,
	0x420,
	0x424,
	0x428,
	0x42C,
	0x430,
	0x434,
	0x438,
	0x43C,
	0x440,
	0x444,
	0x448,
	0x44C,
	0x450,
	0x454,
	0x458,
	0x45C,
	0x460,
	0x464,
	0x468,
	0x46C,
	0x470,
	0x474,
	0x478,
	0x47C,
	0x480,
	0x484,
	0x488,
	0x48C,
	0x490,
	0x494,
	0x498,
	0x49C,
	0x4A0,
	0x4A4,
	0x4A8,
	0x4AC,
	0x4B0,
	0x4B4,
	0x4B8,
	0x4BC,
	0x4C0,
	0x4C4,
	0x4C8,
	0x4CC,
	0x4D0,
	0x4D4,
	0x4D8,
	0x4DC,
	0x4E0,
	0x4E4,
	0x4E8,
	0x4EC,
	0x4F0,
	0x4F4,
	0x4F8,
	0x4FC,
	0x500,
	0x504,
	0x508,
	0x50C,
	0x510,
	0x514,
	0x518,
	0x51C,
	0x520,
	0x524,
	0x528,
	0x52C,
	0x530,
	0x534,
	0x538,
	0x53C,
	0x540,
	0x544,
	0x548,
	0x54C,
	0x550,
	0x554,
	0x558,
	0x55C,
	0x560,
	0x564,
	0x568,
	0x56C,
	0x570,
	0x574,
	0x578,
	0x57C,
	0x580,
	0x584,
	0x588,
	0x58C,
	0x590,
	0x594,
	0x598,
	0x59C,
	0x5A0,
	0x5A4,
	0x5A8,
	0x5AC,
	0x5B0,
	0x5B4,
	0x5B8,
	0x5BC,
	0x5C0,
	0x5C4,
	0x5C8,
	0x5CC,
	0x5D0,
	0x5D4,
	0x5D8,
	0x5DC,
	0x5E0,
	0x5E4,
	0x5E8,
	0x5EC,
	0x5F0,
	0x5F4,
	0x5F8,
	0x5FC,
	0x600,
	0x604,
	0x608,
	0x60C,
	0x610,
	0x614,
	0x618,
	0x61C,
	0x620,
	0x624,
	0x628,
	0x62C,
	0x630,
	0x634,
	0x638,
	0x63C,
	0x640,
	0x644,
	0x648,
	0x64C,
	0x650,
	0x654,
	0x658,
	0x65C,
	0x660,
	0x664,
	0x668,
	0x66C,
	0x670,
	0x674,
	0x678,
	0x67C,
	0x680,
	0x684,
	0x688,
	0x68C,
	0x690,
	0x694,
	0x698,
	0x69C,
	0x6A0,
	0x6A4,
	0x6A8,
	0x6AC,
	0x6B0,
	0x6B4,
	0x6B8,
	0x6BC,
	0x6C0,
	0x6C4,
	0x6C8,
	0x6CC,
	0x6D0,
	0x6D4,
	0x6D8,
	0x6DC,
	0x6E0,
	0x6E4,
	0x6E8,
	0x6EC,
	0x6F0,
	0x6F4,
	0x6F8,
	0x6FC,
	0x700,
	0x704,
	0x708,
	0x70C,
	0x710,
	0x714,
	0x718,
	0x71C,
	0x720,
	0x724,
	0x728,
	0x72C,
	0x730,
	0x734,
	0x738,
	0x73C,
	0x740,
	0x744,
	0x748,
	0x74C,
	0x750,
	0x754,
	0x758,
	0x75C,
	0x760,
	0x764,
	0x768,
	0x76C,
	0x770,
	0x774,
	0x778,
	0x77C,
	0x780,
	0x784,
	0x788,
	0x78C,
	0x790,
	0x794,
};


static u32 gpt_regs_offset[] = {
	GPT_CR_OFFS,
	GPT_PR_OFFS,
	GPT_SR_OFFS,
	GPT_IR_OFFS,
	GPT_OCR1_OFFS,
	GPT_OCR2_OFFS,
	GPT_OCR3_OFFS,
//      GPT_ICR1_OFFS, //R
//      GPT_ICR2_OFFS, //R
//      GPT_CNT_OFFS, //R
};


static u32 gpc_regs_offset[] = {
	GPC_CNTR_OFFS,
	GPC_PGR_OFFS,
	GPC_IMR1_OFFS,
	GPC_IMR2_OFFS,
	GPC_IMR3_OFFS,
	GPC_IMR4_OFFS,
	0x224,
	0x228,
	0x22C,
	// 0x230,
	// 0x234,
	// 0x238,
	// 0x23C,
	0x240,
	0x244,
	0x248,
	0x24C,
	// 0x250,
	// 0x254,
	// 0x258,
	// 0x25C,
	0x260,
	0x264,
	0x268,
	0x26C,
	// 0x270,
	// 0x274,
	// 0x278,
	// 0x27C,
	// 0x280,
	// 0x284,
	// 0x288,
	// 0x28C,
	// 0x290,
	// 0x294,
	// 0x298,
	// 0x29C,
	0x2A0,
	0x2A4,
	0x2A8,
	0x2AC,
};

static u32 ccm_regs_offset[] = {
	0x0,
	0x4,
	0x8,
	0xC,
	0x10,
	0x14,
	0x18,
	0x1C,
	0x20,
	0x24,
	0x28,
	0x2C,
	0x30,
	0x34,
	0x38,
	// 0x3C,
	// 0x40,
	// 0x44,
	0x48,
	// 0x4C,
	// 0x50,
	0x54,
	0x58,
	0x5C,
	0x60,
	0x64,
	0x68,
	0x6C,
	0x70,
	0x74,
	0x78,
	0x7C,
	0x80,
	// 0x84,
	0x88,
	// 0x8C,
};
static u32 ccm_analog_regs_offset[] = {
	0x0,
	// 0x4,
	// 0x8,
	// 0xC,
	0x10,
	// 0x14,
	// 0x18,
	// 0x1C,
	0x20,
	// 0x24,
	// 0x28,
	// 0x2C,
	0x30,
	// 0x34,
	// 0x38,
	// 0x3C,
	0x40,
	// 0x44,
	// 0x48,
	// 0x4C,
	0x50,
	// 0x54,
	// 0x58,
	// 0x5C,
	0x60,
	// 0x64,
	// 0x68,
	// 0x6C,
	0x70,
	// 0x74,
	// 0x78,
	// 0x7C,
	0x80,
	// 0x84,
	// 0x88,
	// 0x8C,
	0x90,
	// 0x94,
	// 0x98,
	// 0x9C,
	0xA0,
	// 0xA4,
	// 0xA8,
	// 0xAC,
	0xB0,
	// 0xB4,
	// 0xB8,
	// 0xBC,
	0xC0,
	// 0xC4,
	// 0xC8,
	// 0xCC,
	// 0xD0,
	// 0xD4,
	// 0xD8,
	// 0xDC,
	0xE0,
	// 0xE4,
	// 0xE8,
	// 0xEC,
	0xF0,
	// 0xF4,
	// 0xF8,
	// 0xFC,
	0x100,
	// 0x104,
	// 0x108,
	// 0x10C,
	// 0x110,
	// 0x114,
	// 0x118,
	// 0x11C,
	// 0x120,
	// 0x124,
	// 0x128,
	// 0x12C,
	// 0x130,
	// 0x134,
	// 0x138,
	// 0x13C,
	// 0x140,
	// 0x144,
	// 0x148,
	// 0x14C,

	0x150,
	// 0x154,
	// 0x158,
	// 0x15C,
	0x160,
	// 0x164,
	// 0x168,
	// 0x16C,
	0x170,
	// 0x174,
	// 0x178,
	// 0x17C,

	// This is for tempon
	0x180,
	0x190,

	// This is for XTALOSC24M
	0x150,
	0x270,
	0x2A0,
	0x2B0,
	0x2C0,
};

static u32* save_registers(void __iomem *base, u32* offset, int size,  u32* dest){
    int i=0;
    if(!base){
    	//pr_info("base is NULL\n");
		return dest;
    }
    for(i =0 ; i < size ; i++){
	// //pr_info("%d->[0x%X][0x%X] = 0x%X \n",global_counter_save, base +  offset[i],offset[i],  readl(base +  offset[i]));
	*dest++ = readl(base +  offset[i]);
	global_counter_save++;

    }
   
    return dest;
}

static u32* restore_registers(void __iomem *base, u32 *offset, int size, u32 *src)
{

   int i=0;
    if(!base)
	return src;
    for(i =0 ; i < size ; i++){
     	writel(*src++, base + offset[i]);
		// //pr_info("%d->[0x%X][0x%X] = 0x%X \n",global_counter_restore, base +  offset[i],offset[i],  readl(base +  offset[i]));
     	global_counter_restore++;
    }
   
    return src;
}

static u32* save_registers_16(void __iomem *base, u32* offset, int size,  u32* dest){
    int i=0;
    if(!base){
    	//pr_info("base is NULL\n");
		return dest;
    }
    for(i =0 ; i < size ; i++){
	// //pr_info("%d->[0x%X][0x%X] = 0x%X \n",global_counter_save, base +  offset[i],offset[i],  readw(base +  offset[i]));
	*dest++ = readw(base +  offset[i]);
	global_counter_save++;

    }
   
    return dest;
}

static u32* restore_registers_16(void __iomem *base, u32 *offset, int size, u32 *src)
{

   int i=0;
    if(!base)
	return src;
    for(i =0 ; i < size ; i++){
     	writew(*src++, base + offset[i]);
		// //pr_info("%d->[0x%X][0x%X] = 0x%X \n",global_counter_restore, base +  offset[i],offset[i],  readw(base +  offset[i]));
     	global_counter_restore++;
    }
   
    return src;
}

static void __iomem * map_registers(const char *path){
	struct device_node *np;
	void __iomem *mem = NULL;
     	np = of_find_node_by_path(path);
	mem = of_iomap(np,0);
	of_node_put(np);
	return mem;
}

static void __iomem * map_registers1(const char *path, int offset){
	struct device_node *np;
	void __iomem *mem = NULL;
     	np = of_find_node_by_path(path);
	mem = of_iomap(np,offset);
	of_node_put(np);
	return mem;
}

static void ccm_pll_load_reg(u32 reg, u32 val)
{
	int flag = 0;
	u32 reg_offs = reg & OFFS_MASK;

	if(val == __raw_readl(reg))
		return;


	if ((reg_offs == CCM_ANALOG_PLL_ARM_OFFS) ||
	    (reg_offs == CCM_ANALOG_PLL_SYS_OFFS)) {
		if ((val & ANALOG_PLL_ENABLE) && !(__raw_readl(reg) & ANALOG_PLL_ENABLE)) {
			flag = 1;
		}
	}
	else {
		if (val & ANALOG_PLL_ENABLE) {
			flag = 1;
		}
	}

	if (flag) {
		if (reg_offs != CCM_ANALOG_PLL_USB1_OFFS) {
			val |= ANALOG_PLL_BYPASS;
			val &= ~ANALOG_PLL_ENABLE;
		}

		__raw_writel(val, reg);

		val = __raw_readl(reg);
		if (!((reg_offs == CCM_ANALOG_PLL_USB1_OFFS) ||
		      (reg_offs == CCM_ANALOG_PLL_USB2_OFFS))) {
			val &= ~ANALOG_PLL_POWER_DOWN;
		}

		__raw_writel(val, reg);

		if (reg_offs == CCM_ANALOG_PLL_USB1_OFFS) {
			u32 anatop_base = ccm_analog;
			__raw_writel(BM_ANADIG_ANA_MISC2_CONTROL0,
				     anatop_base + CCM_ANALOG_MISC2_CLR_OFFS);
		}

		/* Wait for PLL to lock */
		while (!(__raw_readl(reg) & ANALOG_PLL_LOCK))
			;

		val = __raw_readl(reg);
		val &= ~ANALOG_PLL_BYPASS;
		val |= ANALOG_PLL_ENABLE;

		__raw_writel(val, reg);
	} else {
		if (reg_offs == CCM_ANALOG_PLL_USB1_OFFS) {
			u32 anatop_base = ccm_analog;
			__raw_writel(BM_ANADIG_ANA_MISC2_CONTROL0,
				     anatop_base + CCM_ANALOG_MISC2_SET_OFFS);
		}
		__raw_writel(val, reg);

		if ((reg_offs == CCM_ANALOG_PLL_ARM_OFFS) &&
		    (val & ANALOG_PLL_ENABLE)) {
			/* Wait for PLL to lock */
			while (!(__raw_readl(reg) & ANALOG_PLL_LOCK))
				;
		}
	}
}

void usdhc2_clock_disable(void) 
{
	pr_info("Before %s: CCM_CCGR6_OFFS ... 0x%X\n", __func__, __raw_readl(ccm + CCM_CCGR6_OFFS));
	__raw_writel ( __raw_readl(ccm + CCM_CCGR6_OFFS) & 0xFFFFFFCF , ccm + CCM_CCGR6_OFFS);
	pr_info("After %s: CCM_CCGR6_OFFS ... 0x%X\n", __func__, __raw_readl(ccm + CCM_CCGR6_OFFS));
}
EXPORT_SYMBOL(usdhc2_clock_disable);

void usdhc2_clock_enable(void) 
{
	pr_info("Before %s: CCM_CCGR6_OFFS ... 0x%X\n", __func__, __raw_readl(ccm + CCM_CCGR6_OFFS));
	__raw_writel ( __raw_readl(ccm + CCM_CCGR6_OFFS) | 0x30 , ccm + CCM_CCGR6_OFFS);
	pr_info("After %s: CCM_CCGR6_OFFS ... 0x%X\n", __func__, __raw_readl(ccm + CCM_CCGR6_OFFS));
}
EXPORT_SYMBOL(usdhc2_clock_enable);

static void ccm_save_regs(void)
{
	u32 base = ccm;
	u32 anatop_base = ccm_analog;

	currentPtr = save_registers(ccm_analog, ccm_analog_regs_offset, ARRAY_SIZE(ccm_analog_regs_offset), currentPtr);

	ccm_cbcmr = __raw_readl(base + CCM_CBCMR_OFFS);
	ccm_cbcdr = __raw_readl(base + CCM_CBCDR_OFFS);
	ccm_cacrr = __raw_readl(base + CCM_CACRR_OFFS);
	ccm_ccr = __raw_readl(base + CCM_CCR_OFFS);

	pll3_pfd_480 = __raw_readl(anatop_base + CCM_ANALOG_PFD_480_OFFS);
	pll2_pfd_528 = __raw_readl(anatop_base + CCM_ANALOG_PFD_528_OFFS);

	pll7_480usb2 = __raw_readl(anatop_base + CCM_ANALOG_PLL_USB2_OFFS);
	pll6_enet    = __raw_readl(anatop_base + CCM_ANALOG_PLL_ENET_OFFS);
	pll5_video   = __raw_readl(anatop_base + CCM_ANALOG_PLL_VIDEO_OFFS);
	pll4_audio   = __raw_readl(anatop_base + CCM_ANALOG_PLL_AUDIO_OFFS);
	pll3_480usb1 = __raw_readl(anatop_base + CCM_ANALOG_PLL_USB1_OFFS);
	pll2_528     = __raw_readl(anatop_base + CCM_ANALOG_PLL_SYS_OFFS);
	pll1_sys     = __raw_readl(anatop_base + CCM_ANALOG_PLL_ARM_OFFS);

	currentPtr = save_registers(ccm, ccm_regs_offset, ARRAY_SIZE(ccm_regs_offset), currentPtr);

	__raw_writel(0xffffffff, base + CCM_CCGR0_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR1_OFFS);
	__raw_writel(0xfc3fffff, base + CCM_CCGR2_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR3_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR4_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR5_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR6_OFFS);
	__raw_writel(0xffffffff, base + CCM_CMEOR_OFFS);

}

static void ccm_load_regs(void)
{
	u32 base = ccm;
	u32 anatop_base = ccm_analog;
	u32 reg;

	__raw_writel(0xffffffff, base + CCM_CCGR0_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR1_OFFS);
	__raw_writel(0xfc3fffff, base + CCM_CCGR2_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR3_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR4_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR5_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR6_OFFS);
	__raw_writel(0xffffffff, base + CCM_CMEOR_OFFS);


	currentPtr = restore_registers(ccm_analog, ccm_analog_regs_offset, ARRAY_SIZE(ccm_analog_regs_offset), currentPtr);
	writel(0x939B908C, ccm_analog  + 0xF0);

	ccm_pll_load_reg(anatop_base + CCM_ANALOG_PLL_ARM_OFFS, pll1_sys);
	ccm_pll_load_reg(anatop_base + CCM_ANALOG_PLL_SYS_OFFS, pll2_528);
	ccm_pll_load_reg(anatop_base + CCM_ANALOG_PLL_USB1_OFFS, pll3_480usb1);
	ccm_pll_load_reg(anatop_base + CCM_ANALOG_PLL_AUDIO_OFFS, pll4_audio);
	ccm_pll_load_reg(anatop_base + CCM_ANALOG_PLL_VIDEO_OFFS, pll5_video);
	ccm_pll_load_reg(anatop_base + CCM_ANALOG_PLL_ENET_OFFS, pll6_enet);
	ccm_pll_load_reg(anatop_base + CCM_ANALOG_PLL_USB2_OFFS, pll7_480usb2);

	/* PLL2 PDL2 Setting uses a setup of U-Boot(the same as Linux)
	   for the stable supply of periph_clk */
	__raw_writel(ANALOG_PFD_CLKGATE & 0xff00ffff,   anatop_base + CCM_ANALOG_PFD_528_SET_OFFS);
	__raw_writel(ANALOG_PFD_FRAC_MASK & 0xff00ffff, anatop_base + CCM_ANALOG_PFD_528_CLR_OFFS);
	__raw_writel((pll2_pfd_528 & ANALOG_PFD_CLKGATE & 0xff00ffff) ^ ANALOG_PFD_CLKGATE,
		                                        anatop_base + CCM_ANALOG_PFD_528_CLR_OFFS);
	__raw_writel(pll2_pfd_528 & 0xff00ffff,         anatop_base + CCM_ANALOG_PFD_528_SET_OFFS);
	udelay(100);	/* mx6sl only */

	__raw_writel(ANALOG_PFD_CLKGATE,   anatop_base + CCM_ANALOG_PFD_480_SET_OFFS);
	__raw_writel(ANALOG_PFD_FRAC_MASK, anatop_base + CCM_ANALOG_PFD_480_CLR_OFFS);
	__raw_writel((pll3_pfd_480 & ANALOG_PFD_CLKGATE) ^ ANALOG_PFD_CLKGATE,
		                           anatop_base + CCM_ANALOG_PFD_480_CLR_OFFS);
	__raw_writel(pll3_pfd_480,         anatop_base + CCM_ANALOG_PFD_480_SET_OFFS);
	//pr_info("%s: (CCM_ANALOG_PFD_480) = 0x%X\n", __func__, readl( ccm_analog  + 0xF0));

	udelay(100);	/* mx6sl only */

	__raw_writel(ccm_ccr, base + CCM_CCR_OFFS);
	__raw_writel(ccm_cacrr, base + CCM_CACRR_OFFS);

	while (WAIT_CDHIPR)
		;

	/* periph_clk */
	if (ccm_cbcdr & MXC_CCM_CBCDR_PERIPH_CLK_SEL) {
		reg = __raw_readl(base + CCM_CBCDR_OFFS) & ~MXC_CCM_CBCDR_PERIPH_CLK2_PODF_MASK;
		__raw_writel(reg | (ccm_cbcdr & MXC_CCM_CBCDR_PERIPH_CLK2_PODF_MASK),
			     base + CCM_CBCDR_OFFS);

		reg = __raw_readl(base + CCM_CBCMR_OFFS) & ~MXC_CCM_CBCMR_PERIPH_CLK2_SEL_MASK;
		__raw_writel(reg | (ccm_cbcmr & MXC_CCM_CBCMR_PERIPH_CLK2_SEL_MASK),
			     base + CCM_CBCMR_OFFS);

		while (WAIT_CDHIPR)
			;

		reg = __raw_readl(base + CCM_CBCDR_OFFS);
		__raw_writel(reg | (ccm_cbcdr & MXC_CCM_CBCDR_PERIPH_CLK_SEL),
			     base + CCM_CBCDR_OFFS);
	}
	else {
		__raw_writel(ccm_cbcmr, base + CCM_CBCMR_OFFS);

		reg = __raw_readl(base + CCM_CBCMR_OFFS) & ~MXC_CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK;
		__raw_writel(reg | (ccm_cbcmr & MXC_CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK),
			     base + CCM_CBCMR_OFFS);

		reg = __raw_readl(base + CCM_CBCDR_OFFS) & ~MXC_CCM_CBCDR_PERIPH_CLK_SEL;
		__raw_writel(reg | (ccm_cbcdr & MXC_CCM_CBCDR_PERIPH_CLK_SEL),
			     base + CCM_CBCDR_OFFS);
	}
	while (WAIT_CDHIPR)
		;

	/* mmdc_ch1 */
	if (ccm_cbcdr & MXC_CCM_CBCDR_PERIPH2_CLK_SEL) {
		reg = __raw_readl(base + CCM_CBCDR_OFFS) & ~MXC_CCM_CBCDR_PERIPH2_CLK2_PODF_MASK;
		__raw_writel(reg | (ccm_cbcdr & MXC_CCM_CBCDR_PERIPH2_CLK2_PODF_MASK),
			     base + CCM_CBCDR_OFFS);

		__raw_writel(ccm_cbcmr, base + CCM_CBCMR_OFFS);

		while (WAIT_CDHIPR)
			;

		reg = __raw_readl(base + CCM_CBCDR_OFFS);
		__raw_writel(reg | (ccm_cbcdr & MXC_CCM_CBCDR_PERIPH2_CLK_SEL),
			     base + CCM_CBCDR_OFFS);
	}
	else {
		__raw_writel(ccm_cbcmr, base + CCM_CBCMR_OFFS);

		reg = __raw_readl(base + CCM_CBCDR_OFFS) & ~MXC_CCM_CBCDR_PERIPH2_CLK_SEL;
		__raw_writel(reg | (ccm_cbcdr & MXC_CCM_CBCDR_PERIPH2_CLK_SEL),
			     base + CCM_CBCDR_OFFS);
	}
	while (WAIT_CDHIPR)
		;

	/* axi */
	if (ccm_cbcdr & MXC_CCM_CBCDR_AXI_SEL) {
		reg = __raw_readl(base + CCM_CBCDR_OFFS) & ~MXC_CCM_CBCDR_AXI_ALT_SEL_MASK;
		__raw_writel(reg | (ccm_cbcdr & MXC_CCM_CBCDR_AXI_ALT_SEL_MASK),
			     base + CCM_CBCDR_OFFS);

		__raw_writel(ccm_cbcdr, base + CCM_CBCDR_OFFS);
	}
	else {
		__raw_writel(ccm_cbcdr, base + CCM_CBCDR_OFFS);
	}
	while (WAIT_CDHIPR)
		;

	currentPtr = restore_registers(ccm, ccm_regs_offset, ARRAY_SIZE(ccm_regs_offset), currentPtr);

	__raw_writel(0xffffffff, base + CCM_CCGR0_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR1_OFFS);
	__raw_writel(0xfc3fffff, base + CCM_CCGR2_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR3_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR4_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR5_OFFS);
	__raw_writel(0xffffffff, base + CCM_CCGR6_OFFS);
	__raw_writel(0xffffffff, base + CCM_CMEOR_OFFS);

}

static int get_it_lines_number(void __iomem *base)
{
	int irqs;

	irqs = readl_relaxed(base + GIC_DIST_CTR) & 0x1f;
	irqs = 32 * (irqs + 1);

	if (irqs > 1020)
	        irqs = 1020;

	return irqs;
}

static u32 *storeContentRegs(u32* stack, void __iomem *base, int count)
{
    while (count-- > 0) {
		// //pr_info("GIC %d->[0x%X] = 0x%X \n",global_counter_save, base,  readl_relaxed(base));
		// global_counter_save++;
        *stack++ = readl_relaxed(base);
        udelay(10);
        base += 4;
    }

    return stack;
}

static u32 *restoreContentRegs(u32* stack, void __iomem *base, int count)
{
    while (count-- > 0) {
        writel_relaxed(*stack, base);
		// //pr_info("GIC %d->[0x%X] = 0x%X \n",global_counter_restore, base,  readl_relaxed(base));
		// global_counter_restore++;
		stack++;
        udelay(10);
		base +=4;
    }

    return stack;
}


static void gic_suspend(void)
{
	int gic_irqs = get_it_lines_number(gic_dist_base);

	/* 
	 * save Distributer regs
	 *   - save Distributor Control         (0x000)
	 *
	 *   - save Interrupt Set-Enable        (0x100 - 0x11C)
	 *   - save Interrupt Set-Pending       (0x200 - 0x27C)
	 *   - save Interrupt Processor Targets (0x800 - 0xBFB)
	 *   - save Interrupt Priority          (0x400 - 0x7F8)
	 *   - save Interrupt Configuration     (0xC00 - 0xCFC)
	 */
	gicd_ctrl = readl_relaxed(gic_dist_base + GIC_DIST_CTRL);

	//pr_info("%s: Saving gicd_ctrl = %d ... %d\n", __func__, gicd_ctrl, global_counter_save);

	//pr_info("%s: Saving gic_dist_base + GIC_DIST_ENABLE_SET ... %d\n", __func__, global_counter_save);
	gic_storage_ptr = storeContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_ENABLE_SET,
	                gic_irqs / 32); 
	//pr_info("%s: Saving gic_dist_base + GIC_DIST_PENDING_SET ... %d\n", __func__, global_counter_save);
	gic_storage_ptr = storeContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_PENDING_SET,
	                gic_irqs / 32);
	//pr_info("%s: Saving gic_dist_base + GIC_DIST_TARGET ... %d\n", __func__, global_counter_save);
	gic_storage_ptr = storeContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_TARGET,
	                gic_irqs / 4);
	//pr_info("%s: Saving gic_dist_base + GIC_DIST_PRI ... %d\n", __func__, global_counter_save);
	gic_storage_ptr = storeContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_PRI,
	                gic_irqs / 4);
	//pr_info("%s: Saving gic_dist_base + GIC_DIST_CONFIG ... %d\n", __func__, global_counter_save);
	gic_storage_ptr = storeContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_CONFIG,
	                gic_irqs / 16);

	/*
	 *  save CPU interface regs
	 */
	gicc_ctrl     = readl_relaxed(gic_cpu_base + GIC_CPU_CTRL);
	//pr_info("%s: gicc_ctrl = %d ... %d\n", __func__, gicc_ctrl, global_counter_save);
	gicc_primask  = readl_relaxed(gic_cpu_base + GIC_CPU_PRIMASK);
	//pr_info("%s: gicc_primask = %d ... %d\n", __func__, gicc_primask, global_counter_save);
	gicc_binpoint = readl_relaxed(gic_cpu_base + GIC_CPU_BINPOINT);
	//pr_info("%s: gicc_binpoint = %d ... %d\n", __func__, gicc_binpoint, global_counter_save);

}



static void gic_resume(void)
{
    int i;
    int gic_irqs = get_it_lines_number(gic_dist_base);

    /* disable Distributor */
    writel_relaxed(0, gic_dist_base + GIC_DIST_CTRL);

    /*
     * restore Distributer regs
     *
     *   - restore Interrupt Configuration     (0xC00 - 0xCFC)
     *   - restore Interrupt Priority          (0x400 - 0x7F8)
     *   - restore Interrupt Processor Targets (0x800 - 0xBFB)
     *   - restore Interrupt Set-Pending       (0x200 - 0x27C)
     *   - restore Interrupt Set-Enable        (0x100 - 0x11C)
     *
     *   - restore Distributor Control         (0x000)
     */
    /* clear interrupt */
    writel_relaxed(0xffff0000, gic_dist_base + GIC_DIST_ENABLE_CLEAR);
    for (i = 1; i < (gic_irqs / 32); i++)
            writel_relaxed(0xffffffff, gic_dist_base + GIC_DIST_ENABLE_CLEAR
                    + i * 4);
    /* clear pending */
    for (i = 0; i < (gic_irqs / 32); i++)
            writel_relaxed(0xffffffff, gic_dist_base + GIC_DIST_PENDING_CLEAR
                     + i * 4);


	//pr_info("%s: Restoring gic_dist_base + GIC_DIST_ENABLE_SET ... %d\n", __func__, global_counter_restore);
	gic_storage_ptr = restoreContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_ENABLE_SET,
		gic_irqs / 32);

	//pr_info("%s: Restoring gic_dist_base + GIC_DIST_PENDING_SET ... %d\n", __func__, global_counter_restore);
	gic_storage_ptr = restoreContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_PENDING_SET,
		gic_irqs / 32);

	//pr_info("%s: Restoring gic_dist_base + GIC_DIST_TARGET ... %d\n", __func__, global_counter_restore);
	gic_storage_ptr = restoreContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_TARGET,
		gic_irqs / 4);

	//pr_info("%s: Restoring gic_dist_base + GIC_DIST_PRI ... %d\n", __func__, global_counter_restore);
	gic_storage_ptr = restoreContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_PRI,
		gic_irqs / 4);

	//pr_info("%s: Restoring gic_dist_base + GIC_DIST_CONFIG ... %d\n", __func__, global_counter_restore);
    gic_storage_ptr = restoreContentRegs(gic_storage_ptr, gic_dist_base + GIC_DIST_CONFIG,
                    gic_irqs / 16);

	writel_relaxed(gicd_ctrl, gic_dist_base + GIC_DIST_CTRL);
	//pr_info("%s: Restoring gicd_ctrl = %d ... %d\n", __func__, readl_relaxed(gic_dist_base + GIC_DIST_CTRL), global_counter_restore);

    /*
     *  load CPU interface regs
     */
    writel_relaxed(gicc_primask,  gic_cpu_base + GIC_CPU_PRIMASK);
	//pr_info("%s: Restoring gicc_primask = %d ... %d\n", __func__, readl_relaxed(gic_cpu_base + GIC_CPU_PRIMASK), global_counter_restore);
		
    writel_relaxed(gicc_binpoint, gic_cpu_base + GIC_CPU_BINPOINT);
	//pr_info("%s: Restoring gicc_binpoint = %d ... %d\n", __func__, readl_relaxed(gic_cpu_base + GIC_CPU_BINPOINT), global_counter_restore);

    writel_relaxed(gicc_ctrl,     gic_cpu_base + GIC_CPU_CTRL);
	//pr_info("%s: Restoring gicc_ctrl = %d ... %d\n", __func__, readl_relaxed(gic_cpu_base + GIC_CPU_CTRL), global_counter_restore);

}

static void scu_enable(void)
{
 	u32 scu_ctrl;

#ifdef CONFIG_ARM_ERRATA_764369
	/* Cortex-A9 only */
	if ((read_cpuid(CPUID_ID) & 0xff0ffff0) == 0x410fc090) {
		scu_ctrl = __raw_readl(scu_base + 0x30);
		if (!(scu_ctrl & 1))
			__raw_writel(scu_ctrl | 0x1, scu_base + 0x30);
	}
#endif

	scu_ctrl = __raw_readl(scu_base + SCU_CTRL);
	if (scu_ctrl & 1)
		return;

	scu_ctrl |= 1;
	 // Allow SCU_CLK to be disabled when all cores are in WFI
	scu_ctrl |= 0x20;
	__raw_writel(scu_ctrl, scu_base + SCU_CTRL);

	flush_cache_all();

}

static void src_init(void)
{
	u32 src_scr;

	/* Disable SRC warm reset to work aound system reboot issue */
	src_scr = __raw_readl(src_base + SRC_SCR_OFFSET);
	src_scr &= ~0x1;
	__raw_writel(src_scr, src_base + SRC_SCR_OFFSET);
}


u32 twd_ctrl, twd_cntr, twd_load, twd_intstat;
void local_timer_suspend(void)
{
	twd_ctrl = __raw_readl(local_twd_base + LOCAL_TIMER_CTRL);
	twd_cntr = __raw_readl(local_twd_base + LOCAL_TIMER_CNTR);
	twd_load = __raw_readl(local_twd_base + LOCAL_TIMER_LOAD);
	twd_intstat = __raw_readl(local_twd_base + LOCAL_TIMER_INTSTAT);

	writel(0, local_twd_base + LOCAL_TIMER_LOAD);         // disable local timer

	//pr_info ("twd_ctrl = 0x%X \n", twd_ctrl);
	//pr_info ("twd_cntr = 0x%X \n", twd_cntr);
	//pr_info ("twd_load = 0x%X \n", twd_load);
	//pr_info ("twd_intstat = 0x%X \n", twd_intstat);

}

void local_timer_resume(void)
{
	/* clear interrupt if a local timer interrupt has occurred */
	writel(twd_intstat, local_twd_base + LOCAL_TIMER_INTSTAT);

	writel(twd_load, local_twd_base + LOCAL_TIMER_LOAD);
	writel(twd_cntr, local_twd_base + LOCAL_TIMER_CNTR);
	writel(twd_ctrl, local_twd_base + LOCAL_TIMER_CTRL);

	twd_ctrl = __raw_readl(local_twd_base + LOCAL_TIMER_CTRL);
	twd_cntr = __raw_readl(local_twd_base + LOCAL_TIMER_CNTR);
	twd_load = __raw_readl(local_twd_base + LOCAL_TIMER_LOAD);
	twd_intstat = __raw_readl(local_twd_base + LOCAL_TIMER_INTSTAT);
	//pr_info ("twd_ctrl = 0x%X \n", twd_ctrl);
	//pr_info ("twd_cntr = 0x%X \n", twd_cntr);
	//pr_info ("twd_load = 0x%X \n", twd_load);
	//pr_info ("twd_intstat = 0x%X \n", twd_intstat);

}

static void imx6sll_save_state(void)
{
    int i;

	//pr_info("%s: Saving IRAM... %d\n", __func__, global_counter_save);
    memset(save_sram, 0x0, IMX6SLL_IRAM_SAVE_SIZE);
	memcpy(save_sram, (u32 *)sram_base, IMX6SLL_IRAM_SAVE_SIZE);

	// //pr_info("%s: Saving L2 CACHE... %d\n", __func__, global_counter_save);
	// memset(save_l2_cache, 0x0, L2_CACHE_SIZE);
	// memcpy(save_l2_cache, (u32 *)l2_cache_base, L2_CACHE_SIZE);


	//service watchdog at beginning of snapshot capture
    __raw_writew(IMX6SLL_WDT_SEQ1, wdt_base + IMX6SLL_WDT_WSR_OFF);
    __raw_writew(IMX6SLL_WDT_SEQ2, wdt_base + IMX6SLL_WDT_WSR_OFF);


	/** GIC **/
	//pr_info("%s: Saving GIC... %d\n", __func__, global_counter_save);
	gic_storage_ptr = gic_storage;
    memset(gic_storage_ptr, 0x0, sizeof(u32)*1000);
	gic_suspend();
	local_timer_suspend();


    memset(storageBuffer, 0x0, 32*1024);
    currentPtr = storageBuffer;

    /* CCM and CCM_ANALOG */
	//pr_info("%s: Saving CCM and CCM_ANALOG... %d\n", __func__, global_counter_save);
	ccm_save_regs();

    /* OCOTP */
	//pr_info("%s: OCOTP... %d\n", __func__, global_counter_save);
	currentPtr = save_registers(ocotp_base, ocotp_regs_offset, ARRAY_SIZE(ocotp_regs_offset), currentPtr);


	/** IOMUX **/	
	//pr_info("%s: Saving IOMUX... %d\n", __func__, global_counter_save);
	currentPtr = save_registers(iomux, iomux_regs_offset, ARRAY_SIZE(iomux_regs_offset), currentPtr);

	/** IOMUX GPR **/	
	//pr_info("%s: Saving IOMUX_GPR... %d\n", __func__, global_counter_save);
	currentPtr = save_registers(iomux_gpr, iomux_gpr_regs_offset, ARRAY_SIZE(iomux_gpr_regs_offset), currentPtr);


	// /* IOMUXC SNVS */
	// //pr_info("%s: Saving SNVS GPR... %d\n", __func__, global_counter_save);
	// iomuxc_snvs = map_registers("/soc/aips-bus@02100000/iomuxc-snvs@021c8000");
	// currentPtr = save_registers(iomuxc_snvs, iomuxc_snvs_regs_offset, ARRAY_SIZE(iomuxc_snvs_regs_offset), currentPtr);


	/** GPT **/
	//pr_info("%s: Saving GPT... %d\n", __func__, global_counter_save);
	currentPtr = save_registers(gpt,gpt_regs_offset, ARRAY_SIZE(gpt_regs_offset), currentPtr);

	/** GPC **/
	//pr_info("%s: Saving GPC... %d\n", __func__, global_counter_save);
	currentPtr = save_registers(gpc,gpc_regs_offset, ARRAY_SIZE(gpc_regs_offset), currentPtr);

	/** USB **/
	// //pr_info("%s: Saving USBC1... %d\n", __func__, global_counter_save);
	// usbc1_base = map_registers("/soc/aips-bus@02100000/usb@02184000");
	// // while (!tmp);
	// currentPtr = save_registers(usbc1_base, usbc_regs_offset, ARRAY_SIZE(usbc_regs_offset), currentPtr);

	// //pr_info("%s: Saving USBC2... %d\n", __func__, global_counter_save);
	// usbc2_base = map_registers("/soc/aips-bus@02100000/usb@02184200");
	// currentPtr = save_registers(usbc2_base, usbc_regs_offset, ARRAY_SIZE(usbc_regs_offset), currentPtr);

	// //pr_info("%s: Saving USBMISC... %d\n", __func__, global_counter_save);
	// usbmisc_base = map_registers("/soc/aips-bus@02100000/usbmisc@02184800");
	// currentPtr = save_registers(usbmisc_base, usbmisc_regs_offset, ARRAY_SIZE(usbmisc_regs_offset), currentPtr);

	// //pr_info("%s: Saving USBPHY1... %d\n", __func__, global_counter_save);
	// usbphy1_base = map_registers("/soc/aips-bus@02000000/usbphy@020c9000");
	// currentPtr = save_registers(usbphy1_base, usbphy_regs_offset, ARRAY_SIZE(usbphy_regs_offset), currentPtr);

	// //pr_info("%s: Saving USBPHY2... %d\n", __func__, global_counter_save);
	// usbphy2_base = map_registers("/soc/aips-bus@02000000/usbphy@020ca000");
	// currentPtr = save_registers(usbphy2_base, usbphy_regs_offset, ARRAY_SIZE(usbphy_regs_offset), currentPtr);

	// //pr_info("%s: Saving USBANALOG ... %d\n", __func__, global_counter_save);
	// usb_analog_base = (u32)ioremap(USB_ANALOG_BASE, 0x00002000);
	//    if(usb_analog_base == NULL) {
	//            //pr_info(KERN_ERR "Can't get USBANALOG base addr\n");
	//            return ;
	//    }
	// currentPtr = save_registers(usb_analog_base, usb_analog_regs_offset, ARRAY_SIZE(usb_analog_regs_offset), currentPtr);



	/**GPIO **/
	//pr_info("%s: Saving GPIO... %d\n", __func__, global_counter_save);
	currentPtr = save_registers(gpio1, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = save_registers(gpio2, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = save_registers(gpio3, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = save_registers(gpio4, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = save_registers(gpio5, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = save_registers(gpio6, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);

	/** UART ***/
	//pr_info("%s: Saving UART... %d\n", __func__, global_counter_save);
	// currentPtr = save_registers(uart4, uart_regs_offset, ARRAY_SIZE(uart_regs_offset), currentPtr);
	// currentPtr = save_registers(uart1, uart_regs_offset, ARRAY_SIZE(uart_regs_offset), currentPtr);
	// currentPtr = save_registers(uart2, uart_regs_offset, ARRAY_SIZE(uart_regs_offset), currentPtr);

	/* SNVS GPR */
	//pr_info("%s: Saving SNVS GPR... %d\n", __func__, global_counter_save);
	currentPtr = save_registers(snvs_gpr, snvs_gpr_regs_offset, ARRAY_SIZE(snvs_gpr_regs_offset), currentPtr);


	/* SNVS*/
	//pr_info("%s: Saving SNVS ... %d\n", __func__, global_counter_save);
	currentPtr = save_registers(snvs_base, snvs_regs_offset, ARRAY_SIZE(snvs_regs_offset), currentPtr);


	/* I2C*/
	//pr_info("%s: Saving I2C1 ... %d\n", __func__, global_counter_save);
	currentPtr = save_registers_16(i2c1_base, i2c_regs_offset, ARRAY_SIZE(i2c_regs_offset), currentPtr);

	//pr_info("%s: Saving I2C2 ... %d\n", __func__, global_counter_save);
	currentPtr = save_registers_16(i2c2_base, i2c_regs_offset, ARRAY_SIZE(i2c_regs_offset), currentPtr);

	//pr_info("%s: Saving I2C1 ... %d\n", __func__, global_counter_save);
	currentPtr = save_registers_16(i2c3_base, i2c_regs_offset, ARRAY_SIZE(i2c_regs_offset), currentPtr);


	/** USDHC **/
	//pr_info("%s: Saving USDHC1... %d\n", __func__, global_counter_save);
	// currentPtr = save_registers(usdhc1, usdhci_regs_offset, ARRAY_SIZE(usdhci_regs_offset), currentPtr);

	//pr_info("%s: Saving USDHC2... %d\n", __func__, global_counter_save);
#if defined(CONFIG_TOI)
	if (is_hibernation)
		currentPtr = save_registers(usdhc2, usdhci_regs_offset, ARRAY_SIZE(usdhci_regs_offset), currentPtr);
#endif

	//pr_info("%s: Saving USDHC3... %d\n", __func__, global_counter_save);
	// currentPtr = save_registers(usdhc3, usdhci_regs_offset, ARRAY_SIZE(usdhci_regs_offset), currentPtr);

	/** GIC **/
	//pr_info("%s: Saving GIC... %d\n", __func__, global_counter_save);
	gic_dist_base = map_registers1("/interrupt-controller@00a01000", 0);
	gic_cpu_base =  map_registers1("/interrupt-controller@00a01000", 1);
	gic_suspend();	
	flush_cache_all();
	// __save_processor_state(&saved_context_regs);
}

static void imx6sll_restore_state(void)
{
	int i;

	// __restore_processor_state(&saved_context_regs);
	scu_enable();
	src_init();
	gic_storage_ptr = gic_storage;
	currentPtr = storageBuffer;

	//pr_info("%s: Restoring CCM and CCM_ANALOG registers... %d \n", __func__, global_counter_restore);
	ccm_load_regs();


    /* OCOTP */
	//pr_info("%s: Restoring OCOTP registers... %d\n", __func__, global_counter_save);
	currentPtr = restore_registers(ocotp_base, ocotp_regs_offset, ARRAY_SIZE(ocotp_regs_offset), currentPtr);


	//pr_info("%s: Restoring IOMUX registers... %d \n", __func__, global_counter_restore);
	/** IOMUX **/
	currentPtr= restore_registers(iomux, iomux_regs_offset, ARRAY_SIZE(iomux_regs_offset), currentPtr);

	/** IOMUX GPR **/
	//pr_info("%s: Restoring IOMUX_GPR... %d \n", __func__, global_counter_restore);
	currentPtr = restore_registers(iomux_gpr,iomux_gpr_regs_offset, ARRAY_SIZE(iomux_gpr_regs_offset), currentPtr);

	// /* IOMUXC SNVS */
	// //pr_info("%s: Restoring IOMUXC SNVS... %d\n", __func__, global_counter_restore);
	// currentPtr= restore_registers(iomuxc_snvs,iomuxc_snvs_regs_offset, ARRAY_SIZE(iomuxc_snvs_regs_offset), currentPtr);

	/** GPT **/
	//pr_info("%s: Restoring GPT registers... %d \n", __func__, global_counter_restore);
	currentPtr= restore_registers(gpt,gpt_regs_offset, ARRAY_SIZE(gpt_regs_offset), currentPtr);

	/** GPC **/
	//pr_info("%s: Restoring GPC registers... %d \n", __func__, global_counter_restore);
	currentPtr= restore_registers(gpc,gpc_regs_offset, ARRAY_SIZE(gpc_regs_offset), currentPtr);

	/** USB **/
	// //pr_info("%s: Restoring USBC1... %d\n", __func__, global_counter_restore);
	// currentPtr= restore_registers(usbc1_base,usbc_regs_offset, ARRAY_SIZE(usbc_regs_offset), currentPtr);

	// //pr_info("%s: Restoring USBC2... %d\n", __func__, global_counter_restore);
	// currentPtr= restore_registers(usbc2_base,usbc_regs_offset, ARRAY_SIZE(usbc_regs_offset), currentPtr);

	// //pr_info("%s: Restoring USBMISC... %d\n", __func__, global_counter_restore);
	// currentPtr= restore_registers(usbmisc_base,usbmisc_regs_offset, ARRAY_SIZE(usbmisc_regs_offset), currentPtr);

	// //pr_info("%s: Restoring USBPHY1... %d\n", __func__, global_counter_restore);
	// currentPtr= restore_registers(usbphy1_base,usbphy_regs_offset, ARRAY_SIZE(usbphy_regs_offset), currentPtr);

	// //pr_info("%s: Restoring USBPHY2... %d\n", __func__, global_counter_restore);
	// currentPtr= restore_registers(usbphy2_base,usbphy_regs_offset, ARRAY_SIZE(usbphy_regs_offset), currentPtr);

	// //pr_info("%s: Restoring USBANALOG ... %d\n", __func__, global_counter_restore);
	// currentPtr= restore_registers(usb_analog_base,usb_analog_regs_offset, ARRAY_SIZE(usb_analog_regs_offset), currentPtr);
	usbotg_force_bsession(0);

	/** GPIO **/
	//pr_info("%s: Restoring GPIO registers... %d \n", __func__, global_counter_restore);
	currentPtr = restore_registers(gpio1, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = restore_registers(gpio2, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = restore_registers(gpio3, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = restore_registers(gpio4, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = restore_registers(gpio5, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);
	currentPtr = restore_registers(gpio6, gpio_regs_offset, ARRAY_SIZE(gpio_regs_offset), currentPtr);



	// /** UART ***/
	//pr_info("%s: Restoring UART4 registers... %d \n", __func__, global_counter_restore);
	// currentPtr = restore_registers(uart4, uart_regs_offset, ARRAY_SIZE(uart_regs_offset), currentPtr);

	//pr_info("%s: Restoring UART1 registers... %d \n", __func__, global_counter_restore);
	// currentPtr = restore_registers(uart1, uart_regs_offset, ARRAY_SIZE(uart_regs_offset), currentPtr);

	//pr_info("%s: Restoring UART2 registers... %d \n", __func__, global_counter_restore);
	// currentPtr = restore_registers(uart2, uart_regs_offset, ARRAY_SIZE(uart_regs_offset), currentPtr);


	/* SNVS GPR */
	//pr_info("%s: Restoring SNVS GPR registers... %d \n", __func__, global_counter_restore);
	currentPtr= restore_registers(snvs_gpr, snvs_gpr_regs_offset, ARRAY_SIZE(snvs_gpr_regs_offset), currentPtr);

	/* SNVS */
	//pr_info("%s: Restoring SNVS registers... %d \n", __func__, global_counter_restore);
	currentPtr= restore_registers(snvs_base, snvs_regs_offset, ARRAY_SIZE(snvs_regs_offset), currentPtr);


	/* I2C */
	//pr_info("%s: Restoring I2C1 registers... %d \n", __func__, global_counter_restore);
	currentPtr= restore_registers_16(i2c1_base, i2c_regs_offset, ARRAY_SIZE(i2c_regs_offset), currentPtr);

	//pr_info("%s: Restoring I2C2 registers... %d \n", __func__, global_counter_restore);
	currentPtr= restore_registers_16(i2c2_base, i2c_regs_offset, ARRAY_SIZE(i2c_regs_offset), currentPtr);

	//pr_info("%s: Restoring I2C3 registers... %d \n", __func__, global_counter_restore);
	currentPtr= restore_registers_16(i2c3_base, i2c_regs_offset, ARRAY_SIZE(i2c_regs_offset), currentPtr);

	
	/** USDHC **/
	//pr_info("%s: Restoring usdhc1 registers... %d \n", __func__, global_counter_restore);
	// currentPtr= restore_registers(usdhc1, usdhci_regs_offset, ARRAY_SIZE(usdhci_regs_offset), currentPtr);

	//pr_info("%s: Restoring usdhc2 registers... %d \n", __func__, global_counter_restore);
#if defined(CONFIG_TOI)
	if (is_hibernation)
		currentPtr= restore_registers(usdhc2, usdhci_regs_offset, ARRAY_SIZE(usdhci_regs_offset), currentPtr);
#endif

	//pr_info("%s: Restoring usdhc3 registers... %d \n", __func__, global_counter_restore);
	// currentPtr= restore_registers(usdhc3, usdhci_regs_offset, ARRAY_SIZE(usdhci_regs_offset), currentPtr);

	//pr_info("%s: local_timer_resume... %d \n", __func__, global_counter_restore);
	local_timer_resume();

	/** GIC **/
	//pr_info("%s: Restoring GIC registers... %d \n", __func__, global_counter_restore);
	gic_resume();



	/* SRAM */
	//pr_info("%s: Restoring IRAM... %d\n", __func__, global_counter_save);
    memcpy((u32 *)sram_base, save_sram, IMX6SLL_IRAM_SAVE_SIZE);

	/* L2_CACHE */
	// //pr_info("%s: Restoring L2_CACHE... %d\n", __func__, global_counter_save);
 	// memcpy((u32 *)l2_cache_base, save_l2_cache, L2_CACHE_SIZE);


	/** Cleanup **/
	global_counter_save = 0;
	global_counter_restore = 0;

    flush_tlb_all();
}

void imx6sll_hibernation_init(void)
{
    struct device_node *np;

	//pr_info("Initialization for hibernation device node imx6sll\n");

	storageBuffer = (u32 *)kmalloc(32*1024, GFP_ATOMIC);
    if(storageBuffer == NULL) {
            //pr_info("%s: fail to allocate register save area\n", __func__);
            return ;
    }
    memset(storageBuffer, 0x0, 32*1024);

	save_sram = (u32 *)kmalloc(IMX6SLL_IRAM_SAVE_SIZE, GFP_ATOMIC);
    if(save_sram == NULL) {
            //pr_info("%s: fail to save_sram save area\n", __func__);
            return ;
    }
    memset(save_sram, 0x0, IMX6SLL_IRAM_SAVE_SIZE);


	sram_base = (u32)ioremap(MX6Q_IRAM_BASE_ADDR, IMX6SLL_IRAM_SAVE_SIZE);
    if((u32 *)sram_base == NULL) {
            //pr_info(KERN_ERR "Can't get SRAM base addr\n");
            return ;
    }


	save_l2_cache = (u32 *)kmalloc(L2_CACHE_SIZE, GFP_ATOMIC);
    if(save_l2_cache == NULL) {
            //pr_info("%s: fail to save_l2_cache save area\n", __func__);
            return ;
    }
    memset(save_l2_cache, 0x0, L2_CACHE_SIZE);


	l2_cache_base = (u32)ioremap(L2_CACHE_BASE_ADDR, L2_CACHE_SIZE);
    if((u32 *)l2_cache_base == NULL) {
            //pr_info(KERN_ERR "Can't get l2 cache base addr\n");
            return ;
    }


	scu_base = (u32)ioremap(SCU_BASE_ADDR, SZ_256);
	if((u32 *)scu_base == NULL) {
		//pr_info(KERN_ERR "Can't get scu_base addr\n");
		return;
	}

	wdt_base = (u32)ioremap(IMX6SLL_WDT_BASE, SZ_4K);
	if((u32 *)wdt_base == NULL) {
		//pr_info(KERN_ERR "Can't get wdt_base addr\n");
		return;
	}

	src_base = (u32)ioremap(SRC_BASE_ADDR, SZ_16K);
	if((u32 *)src_base == NULL) {
		//pr_info(KERN_ERR "Can't get src base addr\n");
		return ;
	}

	local_twd_base = (u32)ioremap(LOCAL_TWD_ADDR, SZ_256);
	if((u32 *)local_twd_base == NULL) {
		//pr_info(KERN_ERR "Can't get local timer base addr\n");
		return ;
	}


	ccm = map_registers(CCM_DEVICE_TREE_PATH);  

	ccm_analog = map_registers(CCM_ANALOG_DEVICE_TREE_PATH);  

	ocotp_base = map_registers(OCOTP_BASE_DEVICE_TREE_PATH);  


	iomux = map_registers(IOMUX_DEVICE_TREE_PATH);  

	iomux_gpr = map_registers(IOMUX_GPR_DEVICE_TREE_PATH);  

	// iomuxc_snvs = map_registers(IOMUXC_SNVS_DEVICE_TREE_PATH);

	gpt = map_registers(GPT_DEVICE_TREE_PATH);  

	gpc = map_registers(GPC_DEVICE_TREE_PATH);  

	gic_dist_base = map_registers1(GIC_DIST_BASE_DEVICE_TREE_PATH);

	gic_cpu_base =  map_registers1(GIC_CPU_BASE_DEVICE_TREE_PATH);

	// usbc1_base = map_registers(USBC1_BASE_DEVICE_TREE_PATH);  

	// usbc2_base = map_registers(USBC2_BASE_DEVICE_TREE_PATH);  

	// usbmisc_base = map_registers(USBMISC_BASE_DEVICE_TREE_PATH);  

	// usbphy1_base = map_registers(USBPHY1_BASE_DEVICE_TREE_PATH);  

	// usbphy2_base = map_registers(USBPHY2_BASE_DEVICE_TREE_PATH);  

	gpio1 = map_registers(GPIO1_DEVICE_TREE_PATH);  

	gpio2 = map_registers(GPIO2_DEVICE_TREE_PATH);  

	gpio3 = map_registers(GPIO3_DEVICE_TREE_PATH);  

	gpio4 = map_registers(GPIO4_DEVICE_TREE_PATH);  

	gpio5 = map_registers(GPIO5_DEVICE_TREE_PATH);  

	gpio6 = map_registers(GPIO6_DEVICE_TREE_PATH);  

	snvs_gpr = map_registers(SNVS_GPR_DEVICE_TREE_PATH);

	snvs_base = map_registers(SNVS_BASE_DEVICE_TREE_PATH);  


	i2c1_base = map_registers(I2C1_BASE_DEVICE_TREE_PATH);  
	i2c2_base = map_registers(I2C2_BASE_DEVICE_TREE_PATH);  
	i2c3_base = map_registers(I2C3_BASE_DEVICE_TREE_PATH);  

	usdhc1 = map_registers(USDHC1_DEVICE_TREE_PATH);  

	usdhc2 = map_registers(USDHC2_DEVICE_TREE_PATH);  

	usdhc3 = map_registers(USDHC3_DEVICE_TREE_PATH);  


	np = of_find_node_by_path(UART4_DEVICE_TREE_PATH);
	uart4 = of_iomap(np, 0);
	of_node_put(np);

	np = of_find_node_by_path(UART1_DEVICE_TREE_PATH);
	uart1 = of_iomap(np, 0);
	of_node_put(np);

	np = of_find_node_by_path(UART2_DEVICE_TREE_PATH);
	uart2 = of_iomap(np, 0);
	of_node_put(np);

}
EXPORT_SYMBOL(imx6sll_hibernation_init);


void cpu_save_state(void) {
	if (cpu_is_imx6sll()) {
		imx6sll_save_state();
	}

}

void cpu_restore_state(void) {
	if (cpu_is_imx6sll()) {
		imx6sll_restore_state();
	}
}

EXPORT_SYMBOL(cpu_save_state);
EXPORT_SYMBOL(cpu_restore_state);

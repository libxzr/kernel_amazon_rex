/*
 * bd71827-power.c
 * @file ROHM BD71827 Charger driver
 *
 * Copyright 2016.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

//#define DEBUG
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/mfd/bd71827.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/mfd/pmic-notifier.h>
#include <linux/mfd/bd71827_events.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/reboot.h>
#if defined(CONFIG_LAB126)
extern void dump_lastk_to_mmc(void);
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_LAB126
#include <linux/cpufreq.h>
#endif

#if defined(CONFIG_AMAZON_METRICS_LOG)
#include <linux/metricslog.h>
char bd71827_metric_buf[BD718xx_METRIC_BUFFER_SIZE];
#endif
#ifdef CONFIG_PM_AUTOSLEEP
static char* bd71827_VBUS_WAKE_LOCK_NAME = "bd71827_vbus";
#endif


extern int idme_hwid_value;
#define HWID_DVT_VAL	8

#define USE_SOC_AVERAGE			1

#define SCREEN_UPDATE_DELAY		2500		/*delay 2.5 seconds after screen update to calculate soc*/
#define JITTER_DEFAULT			60000		/* hope 60s is enough */
#define DCIN_STATUS_DELAY       	100             /* DCIN delay*/
#define DCIN_SAFE_CHARGING_DELAY       	604800000      /* DCIN safe charging delay -- 7days==7*24*60*60*1000*/
#define JITTER_REPORT_CAP		3000000		/* 1000 seconds */
#define PWRKEY_SKIP_INTERVAL    	500             /* in milli-seconds */
#define BATTERY_CAP_MAH_DEFAULT		910
#define BD71827_BATTERY_CAP		mAh_A10s(BATTERY_CAP_MAH_DEFAULT)
#define MAX_VOLTAGE_DEFAULT		ocv_table_default[0]
#define MIN_VOLTAGE_DEFAULT		3400000
#define THR_VOLTAGE_DEFAULT		4100000
#define MAX_CURRENT_DCP_DEFAULT			900000		/* uA */
#define MAX_CURRENT_SDP_DEFAULT			450000		/* uA, @TODO fine tune this value */
#define MAX_CURRENT_CHG_OUTPUT_DCP_DEFAULT	1000000		/* uA */
#define MAX_CURRENT_CHG_OUTPUT_SDP_DEFAULT	550000		/* uA, @TODO fine tune this value */
#define AC_NAME				"bd71827_ac"
#define BAT_NAME			"bd71827_bat"
#define BATTERY_FULL_DEFAULT		100

#define LOW_BATT_VOLT_LEVEL             0
#define CRIT_BATT_VOLT_LEVEL            1
#define FG_LOW_BATT_CT			3
#define SYS_LOW_VOLT_THRESH             3400    /* 10% */
#define SYS_CRIT_VOLT_THRESH            3200    /* 3% */
#define BY_BAT_VOLT			0
#define BY_VBATLOAD_REG			1
#define INIT_COULOMB			BY_VBATLOAD_REG
#define FASTBOOT			1
#define CALIB_CURRENT_A2A3		0xCE9E
#define INIT_OCV_BY_PWRON

//VBAT Low voltage detection Threshold
#define VBAT_LOW_TH			0x00D4 // 0x00D4*16mV = 212*0.016 = 3.392v
#define FULL_SOC			1000

#define BAT_OPEN	0x7

//In full charge case, BD71827 current ADC has some offset that causes pmic
////reporting current not exact reflecting the true battery current
////this threshold is used as a workaround to ignore the current
#define BD71827_FULLCHARGE_CURRENT_ADC_TOLERANCE 3000


static int soc_buf[10];
static int soc_buf_counter=0;

static int debug_soc_enable=1;

/* system reset type (warm or cold). default=cold */
static bool default_warm_reset;

#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
extern u8 is_hibernation;
#endif

//#ifdef CONFIG_LAB126
#define JITTER_CHK_UDC		1000*120 // only check USB UDC every 2 minutes
extern int usb_udc_connected(void);
void heisenberg_battery_lobat_event(struct device  *dev, int crit_level);
static void heisenberg_battery_overheat_event(struct device *dev);
//static void bd71827_verify_soc_with_vcell(struct bd71827_power* pwr, int soc_to_verify);
static struct delayed_work pwrkey_skip_work;		/** delayed work for powerkey skip */
static DEFINE_MUTEX(pwrkey_lock);
static DEFINE_MUTEX(bd_work_lock);
static bool heisenberg_pwrkey_press_skip = 0;
static bool heisenberg_offline_event = 0;
static bool heisenberg_pwrkey_enabled = 0;
static bool _heisenberg_pwrkey_initialized = 0;
static int metrics_counter = 0;
#define METRICS_TIMER_MIN	 30
#define BATTERY_METRICS_TIMER (METRICS_TIMER_MIN * 20)
extern int gpio_hallsensor_detect(void);
bool heisenberg_pwrkey_initialized(void) {
	return _heisenberg_pwrkey_initialized;
}
EXPORT_SYMBOL(heisenberg_pwrkey_initialized);
extern unsigned long bd71827_total_suspend_time(void);
extern unsigned long bd71827_total_wake_time(void);
#define POWER_METRIC_LOG_PERIOD	10000 //log metrics every 10000*3 seconds

#ifdef CONFIG_AMAZON_METRICS_LOG
#define LOG_SYSTEM_WAIT_TIME (1000*120)         /* 120s wait */
static struct delayed_work log_system_work;	/* delayed work for logging system metrics */
static void log_system_work_func(struct work_struct *work);
#endif

static int bd71827_get_online(struct bd71827_power* pwr);

//#endif /* CONFIG_LAB126 */
#define RS_30mOHM
#ifdef RS_30mOHM
#define A10s_mAh(s)		((s) * 1000 / (360 * 3))
#define mAh_A10s(m)		((m) * (360 * 3) / 1000)
#else
#define A10s_mAh(s)		((s) * 1000 / 360)
#define mAh_A10s(m)		((m) * 360 / 1000)
#endif

#define THR_RELAX_CURRENT_DEFAULT	5		/* mA */
#define THR_RELAX_TIME_DEFAULT		(60 * 60)	/* sec. */

#define DGRD_CYC_CAP_DEFAULT		26	/* 1 micro Ah unit */

#define DGRD_TEMP_H_DEFAULT			45	/* 1 degrees C unit */
#define DGRD_TEMP_M_DEFAULT			25	/* 1 degrees C unit */
#define DGRD_TEMP_L_DEFAULT			5	/* 1 degrees C unit */
#define DGRD_TEMP_VL_DEFAULT		0	/* 1 degrees C unit */

#define SOC_EST_MAX_NUM_DEFAULT 	1
#define DGRD_TEMP_CAP_H_DEFAULT		(0)	/* 1 micro Ah unit */
#define DGRD_TEMP_CAP_M_DEFAULT		(1187)	/* 1 micro Ah unit */
#define DGRD_TEMP_CAP_L_DEFAULT		(5141)	/* 1 micro Ah unit */

#define PWRCTRL_NORMAL				0x22
#define PWRCTRL_RESET				0x23

#define SOFT_REBOOT			0xAA
#define POWEROFF			0x55
#define HIBERNATION			0x77

#define HALL_ONEVENT_BIT    (1 << 3)
#define RTC0_ONEVENT_BIT    (1 << 2)

#ifdef CONFIG_LAB126
#define PWRCTRL_NORMAL_BATCUT_COLDRST_WDG_WARMRST			0x20
#define PWRCTRL_RESET_BATCUT_COLDRST_WDG_WARMRST			0x21
#define BUCK1_VOL_900       0x04
#define BUCK5_LP_ON         BIT(0)
#define CHG_IPRE_70MA_500MA 0x7A
#define D71827_PWRON_PRESSED 0x3C
#define BD71827_PRECHG_TIME_30MIN	0x1D
#define BD71827_CHG_TIME_600MIN		0xAB
#define BD71827_PWRCTRL2_HALL_INV	BIT(0)
#define CHARGE_TOP_OFF				0x0E
#define CHARGE_DONE					0x0F
#define BD71827_REG_BUCK1_VOLT_H_DEFAULT        0xD4
#define BD71827_REG_BUCK1_VOLT_L_DEFAULT        0x14
#define BD71827_REG_BUCK2_VOLT_H_DEFAULT        0x94
#define BD71827_REG_BUCK2_VOLT_L_DEFAULT        0x14

#if !defined(power_dbg)

#ifdef dev_info
#undef dev_info
#define dev_info dev_dbg
#endif //dev_info

#endif //!defined(power_dbg)

#endif //CONFIG_LAB126

static u16 reset_reason;

enum reset_reason_bits {
	RR_SOFTWARE_RESTART = 0,	/* Software reboot; 0xAA in Reserve0 */
	RR_WATCHDOG_RST,		/* Interrupt Status register 3 bit 6 is set */
	RR_PWRON_LONGPRESS,		/* Interrupt Status register 3 bit 2 is set */
	RR_LOW_BAT_SHUTDOWN,		/* Interrupt Status register 4 bit 3 is set */
	RR_THERMAL_SHUTDOWN,		/* Interrupt Status register 11 bit 3 is set */
	RR_POWER_OFF,			/* System is powered off; 0x55 in Reserve0 */
	RR_HIBERNATION,			/* System in hibernate; 0x77 in Reserve0 */

	RR_MAX,
};

enum bd71827_workarounds {
	BOOTUP_WORKAROUND = 0,
	SHIPMODE_WORKAROUND,
};

#define RR_DESC_ITEM(N,D) [RR_##N] = { #N, D },
static const char *reset_reason_desc[][2] = {
	RR_DESC_ITEM(SOFTWARE_RESTART,	"Software Shutdown")
	RR_DESC_ITEM(WATCHDOG_RST,	"Watchdog Triggered Reset")
	RR_DESC_ITEM(PWRON_LONGPRESS,	"Long Pressed Power Button Shutdown")
	RR_DESC_ITEM(LOW_BAT_SHUTDOWN,	"Low Battery Shutdown")
	RR_DESC_ITEM(THERMAL_SHUTDOWN,	"PMIC Overheated Thermal Shutdown")
	RR_DESC_ITEM(POWER_OFF,		"System Powered Off")
	RR_DESC_ITEM(HIBERNATION,	"System Hibernation")
};

extern int gpio_epd_init_pins(void);
extern void gpio_epd_enable_hv(int enable);
extern void gpio_epd_enable_vcom(int enable);
extern int gpio_epd_free_pins(void);
#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
extern void cancel_toi_hibernation(void);
#endif

#define USE_BAT_PARAMS_DEFAULT	0
#define USE_BAT_PARAMS_15KOHM	1
#define USE_BAT_PARAMS_22KOHM 	2
#define USE_BAT_PARAMS_15KOHM_1500MAH	3
#define USE_BAT_PARAMS_47KOHM_1500MAH 	4

static int ocv_table_15kohm[23] = {
	4350000,
	4325138,
	4265151,
	4211064,
	4159675,
	4109493,
	4073406,
	4001485,
	3969549,
	3934141,
	3884880,
	3852478,
	3829435,
	3811091,
	3796191,
	3783269,
	3766630,
	3747633,
	3724839,
	3696889,
	3688726,
	3503077,
	2866784
};	/* unit 1 micro V */

static int vdr_table_h_15kohm[23] = {
	100,
	100,
	101,
	101,
	102,
	103,
	103,
	104,
	104,
	105,
	106,
	106,
	107,
	108,
	108,
	109,
	110,
	110,
	113,
	114,
	131,
	193,
	912
};

static int vdr_table_m_15kohm[23] = {
	100,
	100,
	100,
	100,
	101,
	101,
	101,
	101,
	102,
	102,
	102,
	102,
	103,
	103,
	103,
	103,
	103,
	104,
	107,
	110,
	133,
	176,
	673
};

static int vdr_table_l_15kohm[23] = {
	100,
	100,
	101,
	102,
	104,
	105,
	106,
	107,
	108,
	109,
	111,
	112,
	113,
	114,
	115,
	117,
	118,
	119,
	128,
	140,
	166,
	225,
	673
};

static int vdr_table_vl_15kohm[23] = {
	100,
	100,
	101,
	102,
	104,
	105,
	106,
	107,
	109,
	110,
	111,
	112,
	113,
	115,
	116,
	117,
	118,
	119,
	127,
	140,
	170,
	218,
	673
};

static int ocv_table_22kohm[23] = {
	4350000,
	4321618,
	4262995,
	4208650,
	4156446,
	4106944,
	4060658,
	4017968,
	3978289,
	3934350,
	3888741,
	3857477,
	3834200,
	3815130,
	3799186,
	3785002,
	3763852,
	3742232,
	3716694,
	3686268,
	3667923,
	3497718,
	2928401
};	/* unit 1 micro V */

static int vdr_table_h_22kohm[23] = {
	100,
	100,
	101,
	101,
	102,
	103,
	103,
	104,
	104,
	105,
	106,
	106,
	107,
	108,
	108,
	109,
	110,
	110,
	113,
	114,
	131,
	193,
	912
};

static int vdr_table_m_22kohm[23] = {
	100,
	100,
	102,
	103,
	105,
	107,
	108,
	110,
	112,
	114,
	115,
	117,
	119,
	120,
	122,
	124,
	125,
	127,
	129,
	131,
	152,
	175,
	379
};

static int vdr_table_l_22kohm[23] = {
	100,
	100,
	103,
	106,
	110,
	113,
	116,
	119,
	122,
	125,
	129,
	132,
	135,
	138,
	141,
	144,
	148,
	151,
	160,
	181,
	270,
	348,
	408
};

static int vdr_table_vl_22kohm[23] = {
	100,
	100,
	104,
	107,
	111,
	114,
	118,
	121,
	125,
	128,
	132,
	136,
	139,
	143,
	146,
	150,
	153,
	157,
	160,
	194,
	272,
	360,
	486
};

static int ocv_table_47kohm_1500mah[] = {
	4350000,
	4332314,
	4268128,
	4212674,
	4159545,
	4108647,
	4063581,
	4013678,
	3973377,
	3942003,
	3898042,
	3850201,
	3828981,
	3811035,
	3796514,
	3784875,
	3771757,
	3749146,
	3728759,
	3691612,
	3688559,
	3546894,
	2680219
};	/* unit 1 micro V */

static int vdr_table_h_47kohm_1500mah[] = {
	100,
	100,
	102,
	105,
	107,
	109,
	113,
	116,
	122,
	138,
	142,
	104,
	109,
	112,
	117,
	126,
	106,
	108,
	111,
	110,
	126,
	190,
	668
};

static int vdr_table_m_47kohm_1500mah[] = {
	100,
	100,
	100,
	100,
	101,
	103,
	107,
	112,
	119,
	139,
	133,
	94,
	98,
	102,
	107,
	112,
	110,
	97,
	106,
	111,
	130,
	177,
	555
};

static int vdr_table_l_47kohm_1500mah[] = {
	100,
	100,
	98,
	97,
	95,
	95,
	101,
	98,
	103,
	105,
	93,
	86,
	87,
	89,
	93,
	99,
	111,
	122,
	130,
	150,
	185,
	234,
	418
};

static int vdr_table_vl_47kohm_1500mah[] = {
	100,
	100,
	99,
	98,
	96,
	96,
	101,
	96,
	99,
	99,
	90,
	87,
	89,
	92,
	96,
	105,
	116,
	128,
	139,
	158,
	148,
	351,
	372
};


static int ocv_table_15kohm_1500mah[] = {
	4350000,
	4325945,
	4255935,
	4197476,
	4142843,
	4090615,
	4047113,
	3987352,
	3957835,
	3920815,
	3879834,
	3827010,
	3807239,
	3791379,
	3779925,
	3775038,
	3773530,
	3756695,
	3734099,
	3704867,
	3635377,
	3512942,
	3019825
};	/* unit 1 micro V */

static int vdr_table_h_15kohm_1500mah[] = {
	100,
	100,
	102,
	104,
	105,
	108,
	111,
	115,
	122,
	138,
	158,
	96,
	108,
	112,
	117,
	123,
	137,
	109,
	131,
	150,
	172,
	136,
	218
};

static int vdr_table_m_15kohm_1500mah[] = {
	100,
	100,
	100,
	100,
	102,
	104,
	114,
	110,
	127,
	141,
	139,
	96,
	102,
	106,
	109,
	113,
	130,
	134,
	149,
	188,
	204,
	126,
	271
};

static int vdr_table_l_15kohm_1500mah[] = {
	100,
	100,
	98,
	96,
	96,
	96,
	105,
	94,
	108,
	105,
	95,
	89,
	90,
	92,
	99,
	112,
	129,
	143,
	155,
	162,
	156,
	119,
	326
};

static int vdr_table_vl_15kohm_1500mah[] = {
	100,
	100,
	98,
	96,
	95,
	97,
	101,
	92,
	100,
	97,
	91,
	89,
	90,
	93,
	103,
	115,
	128,
	139,
	148,
	148,
	156,
	246,
	336
};


static int ocv_table_default[23] = {
	4200000,
	4167456,
	4109781,
	4065242,
	4025618,
	3989877,
	3958031,
	3929302,
	3900935,
	3869637,
	3838475,
	3815196,
	3799778,
	3788385,
	3779627,
	3770675,
	3755368,
	3736049,
	3713545,
	3685118,
	3645278,
	3465599,
	2830610
};	/* unit 1 micro V */

static int soc_table_default[23] = {
	1000,
	1000,
	950,
	900,
	850,
	800,
	750,
	700,
	650,
	600,
	550,
	500,
	450,
	400,
	350,
	300,
	250,
	200,
	150,
	100,
	50,
	0,
	-50
	/* unit 0.1% */
};

static int vdr_table_h_default[23] = {
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100
};

static int vdr_table_m_default[23] = {
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100
};

static int vdr_table_l_default[23] = {
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100
};

static int vdr_table_vl_default[23] = {
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100,
	100
};

int use_load_bat_params = USE_BAT_PARAMS_DEFAULT;

int battery_cap_mah;
static int battery_cap;
int max_voltage;
int min_voltage;
int thr_voltage;
int max_current;
unsigned int battery_full;

unsigned int thr_relax_current;
unsigned int thr_relax_time;

int dgrd_cyc_cap;

int dgrd_temp_h;
int dgrd_temp_m;
int dgrd_temp_l;
int dgrd_temp_vl;

int soc_est_max_num;

int dgrd_temp_cap_h;
int dgrd_temp_cap_m;
int dgrd_temp_cap_l;

static unsigned int battery_cycle = 0;

int ocv_table[23];
int soc_table[23];
int vdr_table_h[23];
int vdr_table_m[23];
int vdr_table_l[23];
int vdr_table_vl[23];

/** @brief power deivce */
struct bd71827_power {
	struct device *dev;
	struct bd71827 *mfd;			/**< parent for access register */
	struct power_supply *ac;			/**< alternating current power */
	struct power_supply *bat;		/**< battery power */
	struct delayed_work bd_work;		/**< delayed work for timed work */
	struct delayed_work bd_soc_update_work;		/**< delayed work for soc update */
	struct delayed_work bd_lobat_check_work;		/**< delayed work for timed work */
	struct delayed_work bd_power_work;		/** delayed work for power work*/
	struct delayed_work bd_bat_work;		/** delayed work for battery */
	struct delayed_work bd_ums_work;		/** delayed work for usb mass storage callback **/
	struct delayed_work bd_safe_charging_work;	/** delayed work for usb safe charging control **/
#ifdef CONFIG_PM_AUTOSLEEP
	struct wakeup_source *vbus_wakeup_source;	/* keep device awake while vbus is present */
#endif
	struct int_status_reg irq_status[12];
	int gauge_delay;		/**< Schedule to call gauge algorithm */

	int	reg_index;			/**< register address saved for sysfs */

	int    vbus_status;			/**< last vbus status */
	int    charge_status;			/**< last charge status */
	int    bat_status;			/**< last bat status */

	int	hw_ocv1;			/**< HW ocv1 */
	int	hw_ocv2;			/**< HW ocv2 */
	int	hw_ocv_pwron;			/**< HW ocv_pwron */
	int	bat_online;			/**< battery connect */
	int	charger_online;			/**< charger connect */
	int	vcell;				/**< battery voltage */
	int	vsys;				/**< system voltage */
	int	vcell_min;			/**< minimum battery voltage */
	int	vsys_min;			/**< minimum system voltage */
	int	rpt_status;			/**< battery status report */
	int	prev_rpt_status;		/**< previous battery status report */
	int	bat_health;			/**< battery health */
	int	designed_cap;			/**< battery designed capacity */
	int	full_cap;			/**< battery capacity */
	int	curr;				/**< battery current from DS-ADC */
	int	curr_sar;			/**< battery current from VM_IBAT */
	int	temp;				/**< battery tempature */
	u32	coulomb_cnt;			/**< Coulomb Counter */
	int	state_machine;			/**< initial-procedure state machine */

	u32	soc_org;			/**< State Of Charge using designed capacity without by load */
	u32	soc_norm;			/**< State Of Charge using full capacity without by load */
	u32	soc;				/**< State Of Charge using full capacity with by load */
	u32	clamp_soc;			/**< Clamped State Of Charge using full capacity with by load */

	int	relax_time;			/**< Relax Time */

	u32	cycle;				/**< Charging and Discharging cycle number */
	volatile int calib_current;		/**< calibration current */
};

struct bd71827 *pmic_data;
struct bd71827_power *pmic_pwr;
int heisenberg_critbat_event = 0;
int heisenberg_lobat_event = 0;

#ifdef CONFIG_LAB126
void (*ums_cb)(bool connected);
#define JITTER_UMS_DELAY 1000
/*This function is called after binding fsg...*/
void bd71827_ums_callback(void (*func)(bool connected))
{
	ums_cb = func;
	cancel_delayed_work(&pmic_pwr->bd_ums_work);
	schedule_delayed_work(&pmic_pwr->bd_ums_work, msecs_to_jiffies(JITTER_UMS_DELAY));
}
EXPORT_SYMBOL(bd71827_ums_callback);
#endif

#define CALIB_NORM			0
#define CALIB_START			1
#define CALIB_GO			2

enum {
	STAT_POWER_ON,
	STAT_INITIALIZED,
};
static int pmic_power_button_event_handler(struct notifier_block * this, unsigned long event, void *ptr)
{
	struct bd71827* mfd = pmic_data;
	struct input_dev *input;
	input = pmic_data->input;


	if (event == EVENT_DCIN_PWRON_PRESS) {
		printk(KERN_INFO "Power button pressed\n");
		pr_debug("The event EVENT_DCIN_PWRON_PRESS 0x%0x happens\n",EVENT_DCIN_PWRON_PRESS);
		//input_event(input, EV_KEY, KEY_POWER, 1);
		input_report_key(input,  KEY_POWER, 1);
		input_sync(input);
	}
#ifdef CONFIG_REX_HALL
	if (event == EVENT_DCIN_PWRON_SHORT) {
		if (gpio_hallsensor_detect()) {
			printk(KERN_INFO "KERNEL: I pmic:pwrkey:: pwron_short skipped");
			return 0;
		}
		if (heisenberg_offline_event){
			heisenberg_offline_event = 0;
		} else {
#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
			cancel_toi_hibernation();
#endif
		}
		kobject_uevent(&(mfd->dev->kobj), KOBJ_ONLINE);
		printk(KERN_INFO "Power button pressed, send user event KOBJ_ONLINE\n");
		pr_debug("The event EVENT_DCIN_PWRON_SHORT 0x%0x happens\n",EVENT_DCIN_PWRON_SHORT);

		{
		//input_event(input, EV_KEY, KEY_POWER, 0);
		input_report_key(input, KEY_POWER, 0);
		input_sync(input);
		}
	}
#endif
	if (event == EVENT_DCIN_PWRON_MID) {
		kobject_uevent(&(mfd->dev->kobj), KOBJ_OFFLINE);
		heisenberg_offline_event = 1;
		printk(KERN_INFO "Power button pressed, send user event KOBJ_OFFLINE\n");
		pr_debug("The event EVENT_DCIN_PWRON_MID 0x%0x happens\n",EVENT_DCIN_PWRON_MID);

		{
		/* Android */
		input_event(input, EV_KEY, KEY_POWER, 0);
		input_sync(input);
		}
	}

	return 0;
}

static int pmic_battery_event_handler(struct notifier_block * this, unsigned long event, void *ptr)
{
	pr_debug(" Entering %s \n", __func__);
	pr_debug(" In %s event number is 0x%0x\n", __func__, event);
	return 0;
}

static int pmic_charging_event_handler(struct notifier_block * this, unsigned long event, void *ptr)
{
	pr_debug(" Entering %s \n", __func__);
	pr_debug(" In %s event number is 0x%0x\n", __func__, event);

	return 0;
}

static int pmic_temp_event_handler(struct notifier_block * this, unsigned long event, void *ptr)
{
	struct bd71827* mfd = pmic_data;

	if (event == EVENT_TMP_OVTMP_DET) {
		char *envp[] = { "BATTERY=temp_hi", NULL };
		printk(KERN_CRIT "KERNEL: I pmic:fg battery temperature high event\n");
		kobject_uevent_env(&(mfd->dev->kobj), KOBJ_CHANGE, envp);
		printk("\n~~~ Overtemp Detected ... \n");
	}

	if (event == EVENT_TMP_LOTMP_DET) {
		char *envp[] = { "BATTERY=temp_lo", NULL };
		printk(KERN_CRIT "KERNEL: I pmic:fg battery temperature low event\n");
		kobject_uevent_env(&(mfd->dev->kobj), KOBJ_CHANGE, envp);
		printk("\n~~~ Lowtemp Detected ... \n");
	}

	return 0;
}

static int pmic_fg_event_handler(struct notifier_block * this, unsigned long event, void *ptr)
{
	pr_debug(" Entering %s \n", __func__);
	pr_debug(" In %s event number is 0x%0x\n", __func__, event);
	return 0;
}

static int pmic_batmon_event_handler(struct notifier_block * this, unsigned long event, void *ptr)
{
	pr_debug(" Entering %s \n", __func__);
	pr_debug(" In %s event number is 0x%0x\n", __func__, event);
	return 0;
}


static struct power_supply_desc bd71827_ac_desc;

int pmic_current_event_handler(unsigned long event, void *ptr)
{
	struct bd71827 *mfd = pmic_data;
	struct bd71827_power *pwr = pmic_pwr;
	enum power_supply_type type_new = event;
	int dc_input_cur = (int) ptr; // mA units
	int chg_output_cur = 0;

	if (mfd == NULL || pwr == NULL || pwr->ac == NULL) {
		return -ENODEV;
	}
//Per latest discussion between Rohm/Lab, we set the current limit as:
//SDP: DCIN 450mA, IFST 550mA
//DCP: DCIN 900mA, IFST 1000mA
	// "MAX_CURRENT" is maximum charging current in uA units

	switch (event) {
	case POWER_SUPPLY_TYPE_USB:
		dc_input_cur = min(dc_input_cur, MAX_CURRENT_SDP_DEFAULT / 1000);
		chg_output_cur = MAX_CURRENT_CHG_OUTPUT_SDP_DEFAULT / 1000;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB_DCP:
		dc_input_cur = min(dc_input_cur, MAX_CURRENT_DCP_DEFAULT / 1000);
		chg_output_cur = MAX_CURRENT_CHG_OUTPUT_DCP_DEFAULT / 1000;
		break;
	case POWER_SUPPLY_TYPE_UNKNOWN:
	default:
		type_new = POWER_SUPPLY_TYPE_MAINS;
		dc_input_cur = min(dc_input_cur, MAX_CURRENT_SDP_DEFAULT / 1000);
		chg_output_cur = MAX_CURRENT_CHG_OUTPUT_SDP_DEFAULT / 1000;
	}

	dc_input_cur = min(dc_input_cur, CHG_DCIN_UPPER_LIMIT);

	if (max_current > 0) {
		dc_input_cur = min(dc_input_cur, (max_current / 1000));
	}

	pr_info("%s:%d max dcin current:%dmA, ifst:%dmA\n", __func__, __LINE__, dc_input_cur, chg_output_cur);
	bd71827_update_bits(mfd, BD71827_REG_DCIN_SET, CHG_DCIN_SET_MASK, ((dc_input_cur - 1) / CHG_DCIN_SET_STEP));
	bd71827_update_bits(mfd, BD71827_REG_CHG_IFST, CHG_IFST_MASK, (chg_output_cur / CHG_IFST_STEP));

	// Hold CHGRST bit for 10ms to avoid PMIC stucks in SUSPEND or Battart Assist Mode
	bd71827_set_bits(mfd, BD71827_REG_SYS_INIT, CHGRST);
	msleep(10);
	bd71827_clear_bits(mfd, BD71827_REG_SYS_INIT, CHGRST);

	if (type_new != bd71827_ac_desc.type) {
		bd71827_ac_desc.type = type_new;
		bd71827_get_online(pwr);
		power_supply_changed(pwr->ac);
	}

	return 0;
}
EXPORT_SYMBOL(pmic_current_event_handler);

static struct notifier_block pmic_power_button_notifier =
{
	.notifier_call = pmic_power_button_event_handler,
};

static struct notifier_block pmic_battery_notifier =
{
	.notifier_call = pmic_battery_event_handler,
};

static struct notifier_block pmic_charging_notifier =
{
	.notifier_call = pmic_charging_event_handler,
};

static struct notifier_block pmic_temp_notifier =
{
	.notifier_call = pmic_temp_event_handler,
};

static struct notifier_block pmic_fg_notifier =
{
	.notifier_call = pmic_fg_event_handler,
};

static struct notifier_block pmic_batmon_notifier =
{
	.notifier_call = pmic_batmon_event_handler,
};

static int register_power_button_notifier (void)
{
	int err;

	err = register_pmic_power_button_notifier(&pmic_power_button_notifier);
	if (err) {
		pr_debug(" Register pmic_power_notifier failed\n");
		return -1;
	} else
		pr_debug(" Register pmic_power_notifier completed\n");

	return err;

}

static int unregister_power_button_notifier (void)
{
	int err;

	err = unregister_pmic_power_button_notifier(&pmic_power_button_notifier);
	if (err) {
		pr_debug(" Unregister pmic_power_notifier failed\n");
		return -1;
	} else
		pr_debug(" Unregister pmic_power_notifier completed\n");

	return err;

}

static int register_battery_notifier (void)
{
	int err;

	err = register_pmic_battery_notifier(&pmic_battery_notifier);
	if (err) {
		pr_debug(" Register pmic_battery_notifier failed\n");
		return -1;
	} else
		pr_debug(" Register pmic_battery_notifier completed\n");

	return err;

}

static int register_charging_notifier (void)
{
	int err;

	err = register_pmic_charging_notifier(&pmic_charging_notifier);
	if (err) {
		pr_debug(" Register pmic_charging_notifier failed\n");
		return -1;
	} else
		pr_debug(" Register pmic_charging_notifier completed\n");

	return err;

}

static int register_temp_notifier (void)
{
	int err;

	err = register_pmic_temp_notifier(&pmic_temp_notifier);
	if (err) {
		pr_debug(" Register pmic_temp_notifier failed\n");
		return -1;
	} else
		pr_debug(" Register pmic_temp_notifier completed\n");

	return err;

}

static int register_fg_notifier (void)
{
	int err;

	err = register_pmic_fg_notifier(&pmic_fg_notifier);
	if (err) {
		pr_debug(" Register pmic_fg_notifier failed\n");
		return -1;
	} else
		pr_debug(" Register pmic_fg_notifier completed\n");

	return err;

}

static int register_batmon_notifier (void)
{
	int err;

	err = register_pmic_batmon_notifier(&pmic_batmon_notifier);
	if (err) {
		pr_debug(" Register pmic_batmon_notifier failed\n");
		return -1;
	} else
		pr_debug(" Register pmic_batmon_notifier completed\n");

	return err;

}

/*
 *	Notifier for system reboot
 */
static int reboot_notify_handler(struct notifier_block *this, unsigned long code, void *cmd)
{
	/* Always set software reboot flag will be overwrite if power off */
	ext_bd71827_reg_write8(BD71827_REG_RESERVE_0, SOFT_REBOOT);
	if (cmd && strcmp(cmd, "bootloader") == 0) {
		ext_bd71827_reg_write8(BD71827_REG_RESERVE_7, FASTBOOT);
		pr_info("System reboot to fastboot mode\n");
	}
	return 0;
}

/*
 *	The reboot handler needs to learn about the reasons of the reboot
 *	if the bootloader is in the command, set fastboot flag
 */
static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_notify_handler,
};

static void heisenberg_pwrkey_skip_work(struct work_struct *work)
{
	heisenberg_pwrkey_press_skip = 0;
	return;
}

int heisenberg_pwrkey_ctrl(int enable)
{
	if (!_heisenberg_pwrkey_initialized) {
		printk(KERN_WARNING "%s: Attempt to control power button before driver online\n", __func__);
		return -1;
	}

	mutex_lock(&pwrkey_lock);
	if (enable && !heisenberg_pwrkey_enabled) {
		cancel_delayed_work_sync(&pwrkey_skip_work);
		register_power_button_notifier();
		schedule_delayed_work(&pwrkey_skip_work, msecs_to_jiffies(PWRKEY_SKIP_INTERVAL));
		heisenberg_pwrkey_enabled = 1;
	} else if (!enable && heisenberg_pwrkey_enabled) {
		cancel_delayed_work_sync(&pwrkey_skip_work);
		unregister_power_button_notifier();
		heisenberg_pwrkey_press_skip = 1;
		heisenberg_pwrkey_enabled = 0;
	}
	mutex_unlock(&pwrkey_lock);
	return 0;
}

#ifdef CONFIG_AMAZON_METRICS_LOG
/*
 * log_system_work_func delayed work function
 * Log system information to system metrics
 */
static void log_system_work_func(struct work_struct *work)
{
	char metric_buf[BD718xx_METRIC_BUFFER_SIZE] = {0};
	int i;

	/* log last reset reason to system metrics */
	for (i = 0; i < ARRAY_SIZE(reset_reason_desc); i++) {
		if (reset_reason & (1 << i)) {
			snprintf(metric_buf, sizeof(metric_buf) - 1,
				 "kernel:pmic-bd71827:last_reset_reason=%s;DV;1:NR",
				 reset_reason_desc[i][0]);
			log_to_metrics(ANDROID_LOG_INFO, "system", metric_buf);
		}
	}
}
#endif

static ssize_t pwrkey_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) <= 0) {
		return -EINVAL;
	}
	if (value >= 0) {
		heisenberg_pwrkey_ctrl(1);

	} else {
		heisenberg_pwrkey_ctrl(0);
	}
	return count;
}

static ssize_t pwrkey_ctrl_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	        return sprintf(buf, "%d\n", heisenberg_pwrkey_enabled);
}

static DEVICE_ATTR(pwrkey_ctrl, S_IWUSR | S_IRUGO, pwrkey_ctrl_show, pwrkey_ctrl_store);

static int bd71827_calc_soc_org(struct bd71827_power* pwr);

/** @brief read a register group once
 *  @param mfd bd71827 device
 *  @param reg	 register address of lower register
 *  @return register value
 */

u8 ext_bd71827_reg_read8(u8 reg)
{
	struct bd71827* mfd = pmic_data;
	u8 v;
	v = (u8)bd71827_reg_read(mfd, reg);

	return v;
}

int ext_bd71827_reg_write8(int reg, u8 val)
{
	struct bd71827* mfd = pmic_data;
	return bd71827_reg_write(mfd, reg, val);
}

/** @brief read a register group once
 *  @param mfd bd71827 device
 *  @param reg	 register address of lower register
 *  @return register value
 */
#ifdef __BD71827_REGMAP_H__
u16 ext_bd71827_reg_read16(int reg)
{
	struct bd71827* mfd = pmic_data;
	u16 v;

	v = (u16)bd71827_reg_read(mfd, reg) << 8;
	v |= (u16)bd71827_reg_read(mfd, reg + 1) << 0;
	return v;
}
#else
u16 ext_bd71827_reg_read16(int reg)
{
	struct bd71827* mfd = pmic_data;
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return be16_to_cpu(u.long_type);
}
#endif

/** @brief write a register group once
 * @param mfd bd71827 device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
int ext_bd71827_reg_write16(int reg, u16 val)
{
	struct bd71827* mfd = pmic_data;
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	u.long_type = cpu_to_be16(val);
	// printk("write16 0x%.4X 0x%.4X\n", val, u.long_type);
#ifdef __BD71827_REGMAP_H__
	r = mfd->write(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return 0;
}

/** @brief read quad register once
 *  @param mfd bd71827 device
 *  @param reg	 register address of lower register
 *  @return register value
 */
int ext_bd71827_reg_read32(int reg)
{
	struct bd71827* mfd = pmic_data;
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

#ifdef __BD71827_REGMAP_H__
	r = mfd->read(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return be32_to_cpu(u.long_type);
}

#ifdef __BD71827_REGMAP_H__
static u16 bd71827_reg_read16(struct bd71827* mfd, int reg)
{
	u16 v;

	v = (u16)bd71827_reg_read(mfd, reg) << 8;
	v |= (u16)bd71827_reg_read(mfd, reg + 1) << 0;
	return v;
}
#else
static u16 bd71827_reg_read16(struct bd71827* mfd, int reg)
{
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return be16_to_cpu(u.long_type);
}
#endif

/** @brief write a register group once
 * @param mfd bd71827 device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd71827_reg_write16(struct bd71827 *mfd, int reg, u16 val)
{
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	u.long_type = cpu_to_be16(val);
	// printk("write16 0x%.4X 0x%.4X\n", val, u.long_type);
#ifdef __BD71827_REGMAP_H__
	r = mfd->write(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return 0;
}

/** @brief read quad register once
 *  @param mfd bd71827 device
 *  @param reg	 register address of lower register
 *  @return register value
 */
static int bd71827_reg_read32(struct bd71827 *mfd, int reg)
{
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

#ifdef __BD71827_REGMAP_H__
	r = mfd->read(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return be32_to_cpu(u.long_type);
}

#if 0
/** @brief write quad register once
 * @param mfd bd71827 device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd71827_reg_write32(struct bd71827 *mfd, int reg, unsigned val)
{
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

	u.long_type = cpu_to_be32(val);
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return 0;
}
#endif

#if INIT_COULOMB == BY_VBATLOAD_REG
/** @brief get initial battery voltage and current
 * @param pwr power device
 * @return 0
 */
static int bd71827_get_init_bat_stat(struct bd71827_power *pwr)
{
	struct bd71827 *mfd = pwr->mfd;
	int vcell;

	vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_OCV_PRE_U) * 1000;
	dev_dbg(pwr->dev, "VM_OCV_PRE = %d\n", vcell);
	pwr->hw_ocv1 = vcell;

	vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_OCV_PST_U) * 1000;
	dev_dbg(pwr->dev, "VM_OCV_PST = %d\n", vcell);
	pwr->hw_ocv2 = vcell;

	vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_OCV_PWRON_U) * 1000;
        dev_dbg(pwr->dev, "VM_OCV_PWRON = %d\n", vcell);
        pwr->hw_ocv_pwron = vcell;

	return 0;
}
#endif

#if INIT_COULOMB == BY_BAT_VOLT
/** @brief get battery average voltage and current
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @param curr  pointer to return back current in unit uA.
 * @return 0
 */
static int bd71827_get_vbat_curr(struct bd71827_power *pwr, int *vcell, int *curr)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;

	tmp_vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VBAT_U);

	*vcell = tmp_vcell * 1000;
	*curr = 0;

	return 0;
}
#endif

/** @brief get battery average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd71827_get_vbat(struct bd71827_power *pwr, int *vcell)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;
	tmp_vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VBAT_U);

	*vcell = tmp_vcell * 1000;

	return 0;
}

/** @brief get battery current and battery average current from DS-ADC
 * @param pwr power device
 * @param current in unit uA
 * @param average current in unit uA
 * @return 0
 */
static int bd71827_get_current_ds_adc(struct bd71827_power *pwr, int *curr, int *curr_avg)
{
	int tmp_curr, tmp_curr_avg;

	tmp_curr = 0;
	tmp_curr_avg = 0;
	tmp_curr = bd71827_reg_read16(pwr->mfd, BD71827_REG_CC_CURCD_U);
	if (tmp_curr & CURDIR_Discharging) {
		tmp_curr = -(tmp_curr & ~CURDIR_Discharging);
	}
#ifdef RS_30mOHM
	*curr = tmp_curr * 1000 / 3;
#else
	*curr = tmp_curr * 1000;
#endif

	tmp_curr_avg = bd71827_reg_read16(pwr->mfd, BD71827_REG_CC_SA_CURCD_U);
	if (tmp_curr_avg & CURDIR_Discharging) {
		tmp_curr_avg = -(tmp_curr_avg & ~CURDIR_Discharging);
	}
#ifdef RS_30mOHM
	*curr_avg = tmp_curr_avg * 1000 /3;
#else
	*curr_avg = tmp_curr_avg * 1000;
#endif

	return 0;
}

/** @brief get system average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd71827_get_vsys(struct bd71827_power *pwr, int *vsys)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vsys;

	tmp_vsys = 0;

	tmp_vsys = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VSYS_U);

	*vsys = tmp_vsys * 1000;

	return 0;
}

/** @brief get battery minimum average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd71827_get_vbat_min(struct bd71827_power *pwr, int *vcell)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;

	tmp_vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VBAT_MIN_U);
	bd71827_set_bits(pwr->mfd, BD71827_REG_VM_SA_MINMAX_CLR, VBAT_SA_MIN_CLR);

	*vcell = tmp_vcell * 1000;

	return 0;
}

/** @brief get system minimum average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd71827_get_vsys_min(struct bd71827_power *pwr, int *vcell)
{
	struct bd71827* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;

	tmp_vcell = bd71827_reg_read16(mfd, BD71827_REG_VM_SA_VSYS_MIN_U);
	bd71827_set_bits(pwr->mfd, BD71827_REG_VM_SA_MINMAX_CLR, VSYS_SA_MIN_CLR);

	*vcell = tmp_vcell * 1000;

	return 0;
}

/** @brief get battery capacity
 * @param ocv open circuit voltage
 * @return capcity in unit 0.1 percent
 */
static int bd71827_voltage_to_capacity(int ocv)
{
	int i = 0;
	int soc;

	if (ocv > ocv_table[0]) {
		soc = soc_table[0];
	} else {
		i = 0;
		while (soc_table[i] != -50) {
			if ((ocv <= ocv_table[i]) && (ocv > ocv_table[i+1])) {
				soc = (soc_table[i] - soc_table[i+1]) * (ocv - ocv_table[i+1]) / (ocv_table[i] - ocv_table[i+1]);
				soc += soc_table[i+1];
				break;
			}
			i++;
		}
		if (soc_table[i] == -50)
			soc = soc_table[i];
	}
	return soc;
}

/** @brief get battery temperature
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd71827_get_temp(struct bd71827_power *pwr)
{
	struct bd71827* mfd = pwr->mfd;
	int t;

	t = 200 - (int)bd71827_reg_read(mfd, BD71827_REG_VM_BTMP);

	// battery temperature error
	t = (t > 200)? 200: t;

	return t;
}

int bd71827_get_temperature(void)
{
	if(pmic_pwr) {
		return bd71827_get_temp(pmic_pwr);
	}
	else
		return 25;
}
EXPORT_SYMBOL(bd71827_get_temperature);

int bd71827_reset_soc_buf_counter(void)
{
	soc_buf_counter = 0;
}
EXPORT_SYMBOL(bd71827_reset_soc_buf_counter);

static int bd71827_reset_coulomb_count(struct bd71827_power* pwr);

/** @brief get battery charge status
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd71827_charge_status(struct bd71827_power *pwr)
{
	u8 state;
	int ret = 1;

	pwr->prev_rpt_status = pwr->rpt_status;

	state = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_STATE);
	dev_dbg(pwr->dev, "%s(): CHG_STATE %d\n", __func__, state);

	switch (state) {
	case 0x00:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x0E:
		pwr->rpt_status = POWER_SUPPLY_STATUS_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		heisenberg_critbat_event = 0;
		heisenberg_lobat_event = 0;
		break;
	case 0x0F:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_FULL;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x13:
	case 0x14:
	case 0x20:
	case 0x21:
	case 0x22:
	case 0x23:
	case 0x24:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case 0x30:
	case 0x31:
	case 0x32:
	case 0x40:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x7f:
	default:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_DEAD;
		break;
	}

	bd71827_reset_coulomb_count(pwr);

	return ret;
}

#if INIT_COULOMB == BY_BAT_VOLT
static int bd71827_calib_voltage(struct bd71827_power* pwr, int* ocv)
{
	int r, curr, volt;

	bd71827_get_vbat_curr(pwr, &volt, &curr);

	r = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_STATE);
	if (r >= 0 && curr > 0) {
		// voltage increment caused by battery inner resistor
		if (r == 3) volt -= 100 * 1000;
		else if (r == 2) volt -= 50 * 1000;
	}
	*ocv = volt;

	return 0;
}
#endif

/** @brief set initial coulomb counter value from battery voltage
 * @param pwr power device
 * @return 0
 */
static int calibration_coulomb_counter(struct bd71827_power* pwr)
{
	u32 bcap;
	int soc, ocv;

#if INIT_COULOMB == BY_VBATLOAD_REG
	/* Get init OCV by HW */
	bd71827_get_init_bat_stat(pwr);

#ifdef INIT_OCV_BY_PWRON
	ocv = pwr->hw_ocv_pwron;
#else
	ocv = (pwr->hw_ocv1 >= pwr->hw_ocv2)? pwr->hw_ocv1: pwr->hw_ocv2;
#endif

	dev_dbg(pwr->dev, "ocv %d\n", ocv);
#elif INIT_COULOMB == BY_BAT_VOLT
	bd71827_calib_voltage(pwr, &ocv);
#endif

	/* Get init soc from ocv/soc table */
	soc = bd71827_voltage_to_capacity(ocv);
	dev_dbg(pwr->dev, "soc %d[0.1%%]\n", soc);
	if (soc < 0)
		soc = 0;
	bcap = pwr->designed_cap * soc / 1000;

	bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
	bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, ((bcap + pwr->designed_cap / 200) & 0x0FFFUL));

	pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
	dev_dbg(pwr->dev, "%s() CC_CCNTD = %d\n", __func__, pwr->coulomb_cnt);

	/* Start canceling offset of the DS ADC. This needs 1 second at least */
	bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCCALIB);

	return 0;
}

/** @brief adjust coulomb counter values at relaxed state
 * @param pwr power device
 * @return 0
 */
static int bd71827_adjust_coulomb_count(struct bd71827_power* pwr)
{
	int relax_ocv=0;

	relax_ocv = bd71827_reg_read16(pwr->mfd, BD71827_REG_REX_SA_VBAT_U) * 1000;
	dev_dbg(pwr->dev,  "%s(): relax_ocv = 0x%x\n", __func__, relax_ocv);
	if (relax_ocv != 0) {
		u32 bcap;
		int soc;

		/* Clear Relaxed Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_REX_CTRL_1, REX_CLR);

		/* Get soc at relaxed state from ocv/soc table */
		soc = bd71827_voltage_to_capacity(relax_ocv);
		dev_dbg(pwr->dev,  "soc %d[0.1%%]\n", soc);
		if (soc < 0)
			soc = 0;

		bcap = pwr->designed_cap * soc / 1000;
		bcap = (bcap + pwr->designed_cap / 200) & 0x0FFFUL;

		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, bcap);

		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		dev_dbg(pwr->dev,  "Adjust Coulomb Counter at Relaxed State\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);
		dev_dbg(pwr->dev, "relaxed_ocv:%d, bcap:%d, soc:%d, coulomb_cnt:0x%d\n", relax_ocv, bcap, soc, pwr->coulomb_cnt);
		printk(KERN_INFO "relaxed_ocv:%d, bcap:%d, soc:%d, coulomb_cnt:0x%d\n", relax_ocv, bcap, soc, pwr->coulomb_cnt);
		bd71827_reset_soc_buf_counter();
		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		/* If the following commented out code is enabled, the SOC is not clamped at the relax time. */
		/* Reset SOCs */
		/* bd71827_calc_soc_org(pwr); */
		/* pwr->soc_norm = pwr->soc_org; */
		/* pwr->soc = pwr->soc_norm; */
		/* pwr->clamp_soc = pwr->soc; */
	}

	return 0;
}

/** @brief reset coulomb counter values at full charged state
 * @param pwr power device
 * @return 0
 */
static int bd71827_reset_coulomb_count(struct bd71827_power* pwr)
{
	u32 full_charged_coulomb_cnt;

	full_charged_coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_FULL_CCNTD_3) & 0x0FFFFFFFUL;
	dev_dbg(pwr->dev, "%s(): full_charged_coulomb_cnt=0x%x\n", __func__, full_charged_coulomb_cnt);
	if (full_charged_coulomb_cnt != 0) {
		int diff_coulomb_cnt;

		/* Clear Full Charged Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_FULL_CTRL, FULL_CLR);

		diff_coulomb_cnt = full_charged_coulomb_cnt - (bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL);
		diff_coulomb_cnt = diff_coulomb_cnt >> 16;
		if (diff_coulomb_cnt > 0) {
			diff_coulomb_cnt = 0;
		}
		dev_dbg(pwr->dev,  "diff_coulomb_cnt = %d\n", diff_coulomb_cnt);

		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, ((pwr->designed_cap + pwr->designed_cap / 200) & 0x0FFFUL) + diff_coulomb_cnt);

		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		dev_dbg(pwr->dev,  "Reset Coulomb Counter at POWER_SUPPLY_STATUS_FULL\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
	}

	return 0;
}

/** @brief get battery parameters, such as voltages, currents, temperatures.
 * @param pwr power device
 * @return 0
 */
static int bd71827_get_voltage_current(struct bd71827_power* pwr)
{
	int p_id;

	p_id = bd71827_reg_read(pwr->mfd, BD71827_REG_PRODUCT) & PRODUCT_VERSION;

	if (p_id == 0) { /* BD71827 */
		/* Read detailed vcell and current */
	}
	else { /* BD7182x */
		/* Read detailed vcell and current */
		bd71827_get_vbat(pwr, &pwr->vcell);

		bd71827_get_current_ds_adc(pwr, &pwr->curr_sar, &pwr->curr);
	}
	/* Read detailed vsys */
	bd71827_get_vsys(pwr, &pwr->vsys);
	dev_dbg(pwr->dev,  "VM_VSYS = %d\n", pwr->vsys);

	/* Read detailed vbat_min */
	bd71827_get_vbat_min(pwr, &pwr->vcell_min);
	dev_dbg(pwr->dev,  "VM_VBAT_MIN = %d\n", pwr->vcell_min);

	/* Read detailed vsys_min */
	bd71827_get_vsys_min(pwr, &pwr->vsys_min);
	dev_dbg(pwr->dev,  "VM_VSYS_MIN = %d\n", pwr->vsys_min);

	/* Get tempature */
	pwr->temp = bd71827_get_temp(pwr);
	// dev_dbg(pwr->dev,  "Temperature %d degrees C\n", pwr->temp);

	return 0;
}

/** @brief adjust coulomb counter values at relaxed state by SW
 * @param pwr power device
 * @return 0
 */
static int bd71827_adjust_coulomb_count_sw(struct bd71827_power* pwr)
{
	int tmp_curr_mA;

	tmp_curr_mA = pwr->curr / 1000;
	if ((tmp_curr_mA * tmp_curr_mA) <= (thr_relax_current * thr_relax_current)) { /* No load */
		pwr->relax_time += (JITTER_DEFAULT / 1000);
	}
	else {
		pwr->relax_time = 0;
	}
	dev_dbg(pwr->dev,  "%s(): pwr->relax_time = 0x%x\n", __func__, pwr->relax_time);
	if (pwr->relax_time >= thr_relax_time) { /* Battery is relaxed. */
		u32 bcap;
		int soc, ocv;

		pwr->relax_time = 0;

		/* Get OCV */
		ocv = pwr->vcell;

		/* Get soc at relaxed state from ocv/soc table */
		soc = bd71827_voltage_to_capacity(ocv);
		dev_dbg(pwr->dev,  "soc %d[0.1%%]\n", soc);
		if (soc < 0)
			soc = 0;

		bcap = pwr->designed_cap * soc / 1000;

		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, ((bcap + pwr->designed_cap / 200) & 0x0FFFUL));

		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		dev_dbg(pwr->dev,  "Adjust Coulomb Counter by SW at Relaxed State\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		/* If the following commented out code is enabled, the SOC is not clamped at the relax time. */
		/* Reset SOCs */
		/* bd71827_calc_soc_org(pwr); */
		/* pwr->soc_norm = pwr->soc_org; */
		/* pwr->soc = pwr->soc_norm; */
		/* pwr->clamp_soc = pwr->soc; */
	}

	return 0;
}

/** @brief get coulomb counter values
 * @param pwr power device
 * @return 0
 */
static int bd71827_coulomb_count(struct bd71827_power* pwr)
{
	dev_dbg(pwr->dev, "%s(): pwr->state_machine = 0x%x\n", __func__, pwr->state_machine);
	if (pwr->state_machine == STAT_POWER_ON) {
		pwr->state_machine = STAT_INITIALIZED;
		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
	} else if (pwr->state_machine == STAT_INITIALIZED) {
		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		// dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);
	}
	return 0;
}

/** @brief calc cycle
 * @param pwr power device
 * @return 0
 */
static int bd71827_update_cycle(struct bd71827_power* pwr)
{
	int charged_coulomb_cnt;

	charged_coulomb_cnt = bd71827_reg_read16(pwr->mfd, BD71827_REG_CCNTD_CHG_3);
	dev_dbg(pwr->dev, "%s(): charged_coulomb_cnt = 0x%x\n", __func__, charged_coulomb_cnt);
	if (charged_coulomb_cnt >= pwr->designed_cap) {
		pwr->cycle++;
		dev_dbg(pwr->dev,  "Update cycle = %d\n", pwr->cycle);
		battery_cycle = pwr->cycle;
		charged_coulomb_cnt -= pwr->designed_cap;
		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CCNTD_CHG_3, charged_coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
	}
	return 0;
}

/** @brief calc full capacity value by Cycle and Temperature
 * @param pwr power device
 * @return 0
 */
static int bd71827_calc_full_cap(struct bd71827_power* pwr)
{
	u32 designed_cap_uAh;
	u32 full_cap_uAh;

	/* Calculate full capacity by cycle */
	designed_cap_uAh = A10s_mAh(pwr->designed_cap) * 1000;
	full_cap_uAh = designed_cap_uAh - dgrd_cyc_cap * pwr->cycle;
	pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	dev_dbg(pwr->dev,  "Calculate full capacity by cycle\n");
	dev_dbg(pwr->dev,  "%s() pwr->full_cap = %d\n", __func__, pwr->full_cap);

	/* Calculate full capacity by temperature */
	dev_dbg(pwr->dev,  "Temperature = %d\n", pwr->temp);
	if (pwr->temp >= dgrd_temp_m) {
		full_cap_uAh += (pwr->temp - dgrd_temp_m) * dgrd_temp_cap_h;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}
	else if (pwr->temp >= dgrd_temp_l) {
		full_cap_uAh += (pwr->temp - dgrd_temp_m) * dgrd_temp_cap_m;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}
	else {
		full_cap_uAh += (dgrd_temp_l - dgrd_temp_m) * dgrd_temp_cap_m;
		full_cap_uAh += (pwr->temp - dgrd_temp_l) * dgrd_temp_cap_l;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}

	if (pwr->full_cap < 1)
		pwr->full_cap = 1;

	dev_dbg(pwr->dev,  "Calculate full capacity by cycle and temperature\n");
	dev_dbg(pwr->dev,  "%s() pwr->full_cap = %d\n", __func__, pwr->full_cap);

	return 0;
}

/** @brief calculate SOC values by designed capacity
 * @param pwr power device
 * @return 0
 */
static int bd71827_calc_soc_org(struct bd71827_power* pwr)
{
	pwr->soc_org = (pwr->coulomb_cnt >> 16) * 100 /  pwr->designed_cap;
	if (pwr->soc_org > 100) {
		pwr->soc_org = 100;
		/* Stop Coulomb Counter */
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);

		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_1, 0);
		bd71827_reg_write16(pwr->mfd, BD71827_REG_CC_CCNTD_3, ((pwr->designed_cap + pwr->designed_cap / 200) & 0x0FFFUL));

		pwr->coulomb_cnt = bd71827_reg_read32(pwr->mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
		dev_dbg(pwr->dev,  "Limit Coulomb Counter\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
	}
	dev_dbg(pwr->dev, "%s(): pwr->soc_org = %d\n", __func__, pwr->soc_org);
	return 0;
}

/** @brief calculate SOC values by full capacity
 * @param pwr power device
 * @return 0
 */
static int bd71827_calc_soc_norm(struct bd71827_power* pwr)
{
	int lost_cap;
	int mod_coulomb_cnt;

	lost_cap = pwr->designed_cap - pwr->full_cap;
	dev_dbg(pwr->dev,  "%s() lost_cap = %d\n", __func__, lost_cap);
	mod_coulomb_cnt = (pwr->coulomb_cnt >> 16) - lost_cap;
	if ((mod_coulomb_cnt > 0) && (pwr->full_cap > 0)) {
		pwr->soc_norm = mod_coulomb_cnt * 100 /  pwr->full_cap;
	}
	else {
		pwr->soc_norm = 0;
	}
	if (pwr->soc_norm > 100) {
		pwr->soc_norm = 100;
	}
		dev_dbg(pwr->dev,  "%s() pwr->soc_norm = %d\n", __func__, pwr->soc_norm);
	return 0;
}

/** @brief get OCV value by SOC
 * @param pwr power device
 * @return 0
 */
int bd71827_get_ocv(struct bd71827_power* pwr, int dsoc)
{
	int i = 0;
	int ocv = 0;

	if (dsoc > soc_table[0]) {
		ocv = max_voltage;
	}
	else if (dsoc == 0) {
			ocv = ocv_table[21];
	}
	else {
		i = 0;
		while (i < 22) {
			if ((dsoc <= soc_table[i]) && (dsoc > soc_table[i+1])) {
				ocv = (ocv_table[i] - ocv_table[i+1]) * (dsoc - soc_table[i+1]) / (soc_table[i] - soc_table[i+1]) + ocv_table[i+1];
				break;
			}
			i++;
		}
		if (i == 22)
			ocv = ocv_table[22];
	}
	dev_dbg(pwr->dev,  "%s() ocv = %d\n", __func__, ocv);
	return ocv;
}


/** @brief get VDR(Voltage Drop Rate) value by SOC
 * @param pwr power device
 * @return 0
 */
static int bd71827_get_vdr(struct bd71827_power* pwr, int dsoc)
{
	int i = 0;
	int vdr = 100;
	int vdr_table[23];

	/* Calculate VDR by temperature */
	if (pwr->temp >= dgrd_temp_h) {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_h[i];
		}
	}
	else if (pwr->temp >= dgrd_temp_m) {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_m[i] +
			(pwr->temp - dgrd_temp_m) * (vdr_table_h[i] - vdr_table_m[i]) / (dgrd_temp_h - dgrd_temp_m);
		}
	}
	else if (pwr->temp >= dgrd_temp_l) {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_l[i] +
			(pwr->temp - dgrd_temp_l) * (vdr_table_m[i] - vdr_table_l[i]) / (dgrd_temp_m - dgrd_temp_l);
		}
	}
	else if (pwr->temp >= dgrd_temp_vl) {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_vl[i] +
			(pwr->temp - dgrd_temp_vl) * (vdr_table_l[i] - vdr_table_vl[i]) / (dgrd_temp_l - dgrd_temp_vl);
		}
	}
	else {
		for (i = 0; i < 23; i++) {
			vdr_table[i] = vdr_table_vl[i];
		}
	}

	if (dsoc > soc_table[0]) {
		vdr = 100;
	}
	else if (dsoc == 0) {
		vdr = vdr_table[21];
	}
	else {
		i = 0;
		while (i < 22) {
			if ((dsoc <= soc_table[i]) && (dsoc > soc_table[i+1])) {
				vdr = (vdr_table[i] - vdr_table[i+1]) * (dsoc - soc_table[i+1]) / (soc_table[i] - soc_table[i+1]) + vdr_table[i+1];
				break;
			}
			i++;
		}
		if (i == 22)
			vdr = vdr_table[22];
	}
	dev_dbg(pwr->dev, "%s() vdr = %d\n", __func__, vdr);
	return vdr;
}

/** @brief calculate SOC value by full_capacity and load
 * @param pwr power device
 * @return OCV
 */
static int bd71827_calc_soc(struct bd71827_power* pwr)
{
	int ocv_table_load[23];

	pwr->soc = pwr->soc_norm;

	switch (pwr->rpt_status) { /* Adjust for 0% between thr_voltage and min_voltage */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->vsys_min <= thr_voltage) {
			int i;
			int ocv;
			int lost_cap;
			int mod_coulomb_cnt;
			int dsoc;

			lost_cap = pwr->designed_cap - pwr->full_cap;
			mod_coulomb_cnt = (pwr->coulomb_cnt >> 16) - lost_cap;
			dsoc = mod_coulomb_cnt * 1000 /  pwr->full_cap;
			dev_dbg(pwr->dev,  "%s() dsoc = %d\n", __func__, dsoc);
			ocv = bd71827_get_ocv(pwr, dsoc);
			for (i = 1; i < 23; i++) {
				ocv_table_load[i] = ocv_table[i] - (ocv - pwr->vsys_min);
				if (ocv_table_load[i] <= min_voltage) {
					dev_dbg(pwr->dev,  "%s() ocv_table_load[%d] = %d\n", __func__, i, ocv_table_load[i]);
					break;
				}
			}
			if (i < 23) {
				int j, k, m;
				int dv;
				int lost_cap2, new_lost_cap2;
				int mod_coulomb_cnt2, mod_full_cap;
				int dsoc0;
				int vdr, vdr0;
				dv = (ocv_table_load[i-1] - ocv_table_load[i]) / 5;
				for (j = 1; j < 5; j++){
					if ((ocv_table_load[i] + dv * j) > min_voltage) {
						break;
					}
				}
				lost_cap2 = ((21 - i) * 5 + (j - 1)) * pwr->full_cap / 100;
				dev_dbg(pwr->dev, "%s() lost_cap2-1 = %d\n", __func__, lost_cap2);
				for (m = 0; m < soc_est_max_num; m++) {
					new_lost_cap2 = lost_cap2;
				dsoc0 = lost_cap2 * 1000 / pwr->full_cap;
				if (dsoc >= 0) {
					if (dsoc0 > dsoc)
						dsoc0 = dsoc;
				}
				else {
					if (dsoc0 < dsoc)
						dsoc0 = dsoc;
				}
				dev_dbg(pwr->dev, "%s() dsoc0(%d) = %d\n", __func__, m, dsoc0);

				vdr = bd71827_get_vdr(pwr, dsoc);
				vdr0 = bd71827_get_vdr(pwr, dsoc0);

				for (k = 1; k < 23; k++) {
					ocv_table_load[k] = ocv_table[k] - (ocv - pwr->vsys_min) * vdr0 / vdr;
					if (ocv_table_load[k] <= min_voltage) {
						dev_dbg(pwr->dev, "%s() ocv_table_load[%d] = %d\n", __func__, k, ocv_table_load[k]);
						break;
					}
				}
				if (k < 23) {
					dv = (ocv_table_load[k-1] - ocv_table_load[k]) / 5;
					for (j = 1; j < 5; j++){
						if ((ocv_table_load[k] + dv * j) > min_voltage) {
							break;
						}
					}
						new_lost_cap2 = ((21 - k) * 5 + (j - 1)) * pwr->full_cap / 100;
						if (soc_est_max_num == 1) {
						lost_cap2 = new_lost_cap2;
						}
						else {
							lost_cap2 += (new_lost_cap2 - lost_cap2) / (2 * (soc_est_max_num - m));
						}
						dev_dbg(pwr->dev, "%s() lost_cap2-2(%d) = %d\n", __func__, m, lost_cap2);
				}
					if (new_lost_cap2 == lost_cap2) {
						break;
					}
				}
				mod_coulomb_cnt2 = mod_coulomb_cnt - lost_cap2;
				mod_full_cap = pwr->full_cap - lost_cap2;
				if ((mod_coulomb_cnt2 > 0) && (mod_full_cap > 0)) {
					pwr->soc = mod_coulomb_cnt2 * 100 / mod_full_cap;
				}
				else {
					pwr->soc = 0;
				}
				dev_dbg(pwr->dev,  "%s() pwr->soc(by load) = %d\n", __func__, pwr->soc);
			}
		}
		break;
	default:
		break;
	}

	switch (pwr->rpt_status) {/* Adjust for 0% and 100% */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->vsys_min <= min_voltage) {
			pwr->soc = 0;
		}
		else {
			if (pwr->soc == 0) {
				pwr->soc = 1;
			}
		}
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		if (pwr->soc == 100) {
			pwr->soc = 99;
		}
		break;
	case POWER_SUPPLY_STATUS_FULL:
		//when pmic report charging full and current is less than the threshold
		//while charger is online, consider battery is full
		if ( (abs(pwr->curr_sar) <= BD71827_FULLCHARGE_CURRENT_ADC_TOLERANCE)
				&& (pwr->charger_online != 0)) {
			pwr->soc = 100;
		}
		break;
	default:
		break;
	}
	dev_dbg(pwr->dev,  "%s() pwr->soc = %d\n", __func__, pwr->soc);
	return 0;
}


/** @brief calculate Clamped SOC value by full_capacity and load
 * @param pwr power device
 * @return OCV
 */
static int bd71827_calc_soc_clamp(struct bd71827_power* pwr)
{
	static u32 last_soc = 0;
#if USE_SOC_AVERAGE
	soc_buf[soc_buf_counter%10] = pwr->soc;
	soc_buf_counter++;
	if (soc_buf_counter == 200) soc_buf_counter = 100; //this is to prevent soc_buf_counter overflow
	if(soc_buf_counter >= 10){
	//buf is full and can be used to get the average soc
		int i=0, total_soc = 0;
		for(i=0;i<10;i++)
			total_soc +=soc_buf[i];
		pwr->clamp_soc = total_soc/10;
#if DEBUG
		{
		//DUMP data for test
	        printk(KERN_INFO "the last 10 SOC sampling:\n\n");
	        for(i=soc_buf_counter%10;i<10;i++)
			printk(KERN_INFO ":%d:\n",soc_buf[i]);
	        for(i=0;i<soc_buf_counter%10;i++)
			printk(KERN_INFO ":%d:\n",soc_buf[i]);
	        printk(KERN_INFO "counter=%d\n\n",soc_buf_counter);
//	        for(i=0;i<10;i++)
//        	        printk(KERN_ERR ":%d:\n",soc_buf[i]);
	        printk(KERN_INFO "counter=%d, clamp_soc=%d\n\n",soc_buf_counter,pwr->clamp_soc);
		}
#endif
	} else {
		pwr->clamp_soc = pwr->soc;
	}
	if(last_soc != pwr->clamp_soc)
	{
		printk(KERN_INFO "kernel:pmic:clamp_soc:%d,vbat:%d\n",pwr->clamp_soc,pwr->vcell);
		last_soc = pwr->clamp_soc;
	}
#else
	switch (pwr->rpt_status) {/* Adjust for 0% and 100% */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->soc <= pwr->clamp_soc) {
			pwr->clamp_soc = pwr->soc;
		}
		break;
	default:
		pwr->clamp_soc = pwr->soc;
		break;
	}
#endif
	dev_dbg(pwr->dev,  "%s() pwr->clamp_soc = %d\n", __func__, pwr->clamp_soc);
	return 0;
}

/** @brief get battery and DC online status
 * @param pwr power device
 * @return 0
 */
static int bd71827_get_online(struct bd71827_power* pwr)
{
	int r;

#if 0
#define TS_THRESHOLD_VOLT	0xD9
	r = bd71827_reg_read(pwr->mfd, BD71827_REG_VM_VTH);
	pwr->bat_online = (r > TS_THRESHOLD_VOLT);
#endif
#if 0
	r = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_STAT);
	if (r >= 0 && (r & BAT_DET_DONE)) {
		pwr->bat_online = (r & BAT_DET) != 0;
	}
#endif
#if 1
	r = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_TEMP);
	pwr->bat_online = (r != BAT_OPEN);
#endif
	r = bd71827_reg_read(pwr->mfd, BD71827_REG_DCIN_STAT);
	if (r >= 0) {
		pwr->charger_online = (r & DCIN_DET) != 0;
	}
	dev_dbg(pwr->dev, "%s(): pwr->bat_online = %d, pwr->charger_online = %d\n", __func__, pwr->bat_online, pwr->charger_online);
	return 0;
}

/**@ brief bd71827_get_battery_soc
 * @ param none
 * @ this function return the current battery soc
 */
int bd71827_get_battery_soc(void)
{
//	struct bd71827* mfd = pmic_data;
//	struct power_supply *psy = dev_get_drvdata(mfd->dev);
	if(pmic_pwr) {
		/* On page update, display driver will call this function, so driver can piggy back an internal pmic update to the high
		   current situation. We can keep the regular update interval longer.*/
                if(!schedule_delayed_work(&pmic_pwr->bd_soc_update_work, msecs_to_jiffies(SCREEN_UPDATE_DELAY))) {
			printk(KERN_INFO "screen update triggered SOC update skipped\n");
		}
		return pmic_pwr->clamp_soc;
	}
	else
		return 0;
}
EXPORT_SYMBOL(bd71827_get_battery_soc);

/**@ brief bd71827_get_battery_mah
 * @ param none
 * @ this function return the current battery mah
 */
int bd71827_get_battery_mah(void)
{
//	struct bd71827* mfd = pmic_data;
//	struct power_supply *psy = dev_get_drvdata(mfd->dev);
	if(pmic_pwr)
	{
		printk(KERN_ERR "FRED: pmic_pwr->clamp_soc:%d, designed_cap:%d\n",pmic_pwr->clamp_soc, pmic_pwr->designed_cap);

		return pmic_pwr->clamp_soc * pmic_pwr->designed_cap/100;
	}
	else
		return 0;
}
EXPORT_SYMBOL(bd71827_get_battery_mah);

static void log_reset_reason(struct device *dev)
{
	int i;

	dev_notice(dev, "RESET_REASON = 0x%X\n", reset_reason);
	if (reset_reason == 0) {
		dev_err(dev, "Reboot Reason: UNKNOWN\n");
	} else {
		for (i = 0; i < ARRAY_SIZE(reset_reason_desc); i++) {
			if (reset_reason & (1<<i)) {
				dev_err(dev, "Reboot Reason: %-20s %s\n", reset_reason_desc[i][0], reset_reason_desc[i][1]);
			}
		}
	}
}

/**@ brief bd71827_get_events_recorder
 * @ this function read all the interrupts status registers and clear
 * @ all the interrupt status registers
 * @ return 0
 */
int bd71827_get_events_recorder(struct bd71827 *mfd)
{
	int ret;
	int r;
	unsigned long addr;
	u8 events_recorder[TOTAL_IRQ_STATUS_REGS+2];

	int i;

	/* read and clean the software reset flag register */
	r = bd71827_reg_read(mfd, BD71827_REG_RESERVE_0);
	ret = bd71827_reg_write(mfd, BD71827_REG_RESERVE_0, 0x00);
	if (ret)
		return ret;

	reset_reason = 0;

	if (r == SOFT_REBOOT) {
		reset_reason |= BIT(RR_SOFTWARE_RESTART);	/* software reset */
		dev_dbg(mfd->dev, "The last system restart was initiated by software !!!!!\n");
	} else if (r == POWEROFF) {
		reset_reason |= BIT(RR_POWER_OFF);		/* power off */
		dev_dbg(mfd->dev, "The system was powered off !!!!!\n");
	} else if (r == HIBERNATION) {
		reset_reason |= BIT(RR_HIBERNATION);		/* hibernation */
		dev_dbg(mfd->dev, "The system was in hibernation !!!!!\n");
	}

	/* record and clear all INT_STAT */
	/* bd71827 has another interrupt stat 13 */
	for (i = 0; i <= TOTAL_IRQ_STATUS_REGS+1; i++) {
		if (i == TOTAL_IRQ_STATUS_REGS+1)
			addr = BD71827_REG_INT_STAT_13;
		else
			addr = BD71827_REG_INT_STAT + i;
		events_recorder[i] = bd71827_reg_read(mfd, addr);
		ret = bd71827_reg_write(mfd, addr, events_recorder[i]);
		if (ret)
			return ret;
		dev_notice(mfd->dev, "BD71827_REG_INT_STAT_%02d=0x%02X ", i, events_recorder[i]);
	}

	if ((events_recorder[3] & BIT(6)) && (!(reset_reason & BIT(RR_SOFTWARE_RESTART))))
		reset_reason |= BIT(RR_WATCHDOG_RST);		/* watchdog reset */

	if (events_recorder[3] & BIT(2))
		reset_reason |= BIT(RR_PWRON_LONGPRESS);	/* power button long press reset */

	if (events_recorder[4] & BIT(3))
		reset_reason |= BIT(RR_LOW_BAT_SHUTDOWN);	/* battery low voltage reset */

	if (events_recorder[11] & BIT(3))
		reset_reason |= BIT(RR_THERMAL_SHUTDOWN); 	/* battery temperature high reset */

	log_reset_reason(mfd->dev);
	return ret;
}
EXPORT_SYMBOL(bd71827_get_events_recorder);

static void bd71827_safe_charging_control(struct bd71827_power *pwr, int on_off)
{
	printk(KERN_INFO "%s on_off:%d\n",__func__, on_off);
	if(on_off == 1) {
	/*safe charge is on, set charging voltage to 4V*/
		bd71827_reg_write(pwr->mfd, BD71827_REG_CHG_VBAT_1, BD71827_CHG_VBAT_4V);
		bd71827_reg_write(pwr->mfd, BD71827_REG_CHG_VBAT_2, BD71827_CHG_VBAT_4V);
		bd71827_reg_write(pwr->mfd, BD71827_REG_CHG_VBAT_3, BD71827_CHG_VBAT_4V);
	} else {
	/*safe charge is off, restore the default setting*/
		bd71827_reg_write(pwr->mfd, BD71827_REG_CHG_VBAT_1, BD71827_CHG_VBAT_CHG_1_DEFAULT);
		bd71827_reg_write(pwr->mfd, BD71827_REG_CHG_VBAT_2, BD71827_CHG_VBAT_CHG_2_DEFAULT);
		bd71827_reg_write(pwr->mfd, BD71827_REG_CHG_VBAT_3, BD71827_CHG_VBAT_CHG_3_DEFAULT);
	}
}

/*
 * This function will be used to perform the workarounds suggested by ROHMC
 * The sequence goes like this:
 * - Enter the PMIC test mode
 * - workaround #1: Extend the vsys_min sampling window to reduce the impact
 *                  of sudden vsys dip
 * - workaround #2: Enable OFF sequence. This should already be in initialization
 * -                code for SUSPEND latency workaround
 * - workaround X...: if more
 * - (IMPORTANT) Exit test mode and back to PMIC normal mode.
 * During the duration of this process, all other I2C registers access
 * are invalid hence we need to do this "atomically".
 */
static int bd71827_testmode_workaround(struct bd71827 *bd71827, enum bd71827_workarounds workaround)
{
	struct i2c_client *client = bd71827->i2c_client;
	int ret;

	if (unlikely(!client))
		return -EINVAL;

	/*
	 * lock at the adapter level explicitly. If there are any other processes
	 * (including threaded IRQ handlers) who wish to access the i2c, it will
	 * need to wait.
	 * This is a poor man's way to ensure atomicity in executing a
	 * a sequence of i2c operations back to back.
	 */
	i2c_lock_adapter(client->adapter);

	/* Enter PMIC test mode */
	ret = bd71827_enter_test_mode(client);
	if (ret) {
		printk(KERN_ERR "bd71827 entering test mode failed in %s",__func__);
		goto out;
	}
	switch (workaround)
	{
#if 0 //BOOTUP workaround is moved to u-boot
		case BOOTUP_WORKAROUND:
			//workaround #1: Extend the vsys_min sampling window to reduce the impact of sudden vsys dip
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_SARADC_REG, BD71827_TEST_REG_SARADC_VALUE_256);
			//workaround #2: Enable OFF sequence This should already be in initialization code for SUSPEND latency workaround
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_POWER_SEQ_REG, BD71827_TEST_REG_ENABLE_OFF_SEQUENCE);
			//workaround #3: BATFET ON to avoid DCIN insertion failure inside latency
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_BATFET_CFG_REG, BD71827_TEST_REG_BATFET_ON);
			// Enable TS monitor
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_TS_MONITOR_CFG_REG, BD71827_TEST_REG_TS_MONITOR_ON);
		break;
#endif
		case SHIPMODE_WORKAROUND:
			// Disable OFF sequence to minimize latency 1.64sec to 480usec
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_POWER_SEQ_REG, BD71827_TEST_REG_DISABLE_OFF_SEQUENCE);
			// Clear button press counter to avoid button press failure inside latency
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_BUTTON_PRESS_COUNTER_REG, BD71827_TEST_REG_BUTTON_PRESS_COUNTER_CLEAR);
			mdelay(20);
			// Resume normal button press counter
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_BUTTON_PRESS_COUNTER_REG, BD71827_TEST_REG_BUTTON_PRESS_COUNTER_NORMAL);
			// BATFET automatic control for ship mode
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_BATFET_CFG_REG, BD71827_TEST_REG_BATFET_OFF);
			// Disable TS monitor
			bd71827_write_i2c_reg(client, BD71827_TEST_REG_TS_MONITOR_CFG_REG, BD71827_TEST_REG_TS_MONITOR_OFF);


		break;
		default:
			printk(KERN_INFO "unknown bd71827 workaround:%d, do nothing\n", workaround);
		break;
	}
	/* Exit PMIC test mode, back to normal */
	bd71827_exit_test_mode(client);

out:
	i2c_unlock_adapter(client->adapter);

	/* everything went ok, delay a bit for test setting to take effect */
	if (!ret)
		udelay(2000);

	return ret;
}

/** @brief init bd71827 sub module charger
 * @param pwr power device
 * @return 0
 */
static int bd71827_init_hardware(struct bd71827_power *pwr)
{
	struct bd71827 *mfd = pwr->mfd;
	int r = 0, rtc_year = 0;
	u8 val = 0;

	r = bd71827_reg_write(mfd, BD71827_REG_DCIN_CLPS, 0x36);
        rtc_year = bd71827_reg_read(mfd,BD71827_REG_YEAR);
#define XSTB		0x02
	r = bd71827_reg_read(mfd, BD71827_REG_CONF);

#if 0
	for (i = 0; i < 300; i++) {
		r = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_STAT);
		if (r >= 0 && (r & BAT_DET_DONE)) {
			break;
		}
		msleep(5);
	}
#endif
	if ((r & XSTB) == 0x00  && rtc_year == 0) {
	//if (r & BAT_DET) {
		/* Init HW, when the battery is inserted. */

		bd71827_reg_write(mfd, BD71827_REG_CONF, r | XSTB);

		/* Stop Coulomb Counter */
		bd71827_clear_bits(mfd, BD71827_REG_CC_CTRL, CCNTENB);

		/* Set Coulomb Counter Reset bit*/
		bd71827_set_bits(mfd, BD71827_REG_CC_CTRL, CCNTRST);

		/* Clear Coulomb Counter Reset bit*/
		bd71827_clear_bits(mfd, BD71827_REG_CC_CTRL, CCNTRST);

		/* Clear Relaxed Coulomb Counter */
		bd71827_set_bits(mfd, BD71827_REG_REX_CTRL_1, REX_CLR);

		/* Set default Battery Capacity */
		pwr->designed_cap = battery_cap;
		pwr->full_cap = battery_cap;

		/* Set initial Coulomb Counter by HW OCV */
		calibration_coulomb_counter(pwr);

		/* WDT_FST auto set */
		bd71827_set_bits(mfd, BD71827_REG_CHG_SET1, WDT_AUTO);

#ifdef CONFIG_AMAZON_METRICS_LOG

		memset(bd71827_metric_buf,0,sizeof(bd71827_metric_buf));
                snprintf(bd71827_metric_buf, sizeof(bd71827_metric_buf),
                        "kernel:pmic-bd71827:hw_ocv1=%d;CT;1,hw_ocv2=%d;CT;1,hw_ocv_pwron=%d;CT;1:NR",
                        pwr->hw_ocv1, pwr->hw_ocv2, pwr->hw_ocv_pwron);

                log_to_metrics(ANDROID_LOG_INFO, "battery", bd71827_metric_buf);
#endif
		/* VBAT Low voltage detection Setting, added by John Zhang*/
		bd71827_reg_write16(mfd, BD71827_REG_ALM_VBAT_TH_U, VBAT_LOW_TH);

		/* Set Battery Capacity Monitor threshold1 as 95% */
		bd71827_reg_write16(mfd, BD71827_REG_CC_BATCAP1_TH_U, (battery_cap * 95 / 100));
		dev_dbg(pwr->dev, "BD71827_REG_CC_BATCAP1_TH = %d\n", (battery_cap * 95 / 100));

		/* Enable LED ON when charging */
		bd71827_set_bits(pwr->mfd, BD71827_REG_LED_CTRL, CHGDONE_LED_EN);

		pwr->state_machine = STAT_POWER_ON;
	} else {
		pwr->designed_cap = battery_cap;
		pwr->full_cap = battery_cap;	// bd71827_reg_read16(pwr->mfd, BD71827_REG_CC_BATCAP_U);
		pwr->state_machine = STAT_INITIALIZED;	// STAT_INITIALIZED
	}

#ifdef CONFIG_LAB126
	/* VSYS_MIN register OTP value should be 0x2F, but it is actually 0x35,
	 * so that device stop responding to power key when battery drop below 3.4v
	 * set the value to 0x2F so that the limit is 3.0v. This is SW workaround
	 * should check next revision chip. */
	bd71827_reg_write(mfd, BD71827_REG_VSYS_MIN, 0x2F);

	bd71827_reg_write(mfd, BD71827_REG_LDO_PD_DIS, 0x0);
#if 0 //BOOTUP workaround is moved to uboot
	bd71827_testmode_workaround(mfd, BOOTUP_WORKAROUND);
#endif

	/* Clear Relax PMU STATE mask, Relax decision is by PMU state*/
	bd71827_clear_bits(pwr->mfd, BD71827_REG_REX_CTRL_1, REX_PMU_STATE_MASK);
	/* Clear Relax current thredshold */
	bd71827_reg_write(pwr->mfd, BD71827_REG_REX_CTRL_2, 0);
	/* Set Relax DUR to 60 Min*/
	bd71827_set_bits(pwr->mfd, BD71827_REG_REX_CTRL_1, 0x1);

	bd71827_safe_charging_control(pwr, false);

	/* VBAT Low voltage detection Setting, added by John Zhang*/
	bd71827_reg_write16(mfd, BD71827_REG_ALM_VBAT_TH_U, VBAT_LOW_TH);

	r = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_TEMP);
	if (r != BAT_OPEN) {
		/* Set Battery Capacity Monitor threshold1 as 95% */
		bd71827_reg_write16(mfd, BD71827_REG_CC_BATCAP1_TH_U, (battery_cap * 95 / 100));
		dev_info(pwr->dev, "BD71827_REG_CC_BATCAP1_TH = %d\n", (battery_cap * 95 / 100));
		//set CHG_DONE to 4.35-0.016
		bd71827_reg_write(pwr->mfd, BD71827_REG_BAT_SET_2, 0x46);
		//set recharge threshold to 4.35-0.05
		bd71827_reg_write(pwr->mfd, BD71827_REG_BAT_SET_3, 0x62);
		/* Enable LED ON when charging */
		bd71827_set_bits(pwr->mfd, BD71827_REG_LED_CTRL, CHGDONE_LED_EN);
	}

	r = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_IFST_TERM);
	if (r <= 0x07) {
#define CHG_TERM_66_7_MA	0x08
		r = CHG_TERM_66_7_MA;
		bd71827_reg_write(pwr->mfd, BD71827_REG_CHG_IFST_TERM, r);
	}
	/* set the VDD_SOC to 0.9 V when in suspend mode */
	bd71827_reg_write(mfd, BD71827_REG_BUCK1_VOLT_SUSP, BUCK1_VOL_900);

	/* Turn off eMMC voltage when in suspend mode */
	val = bd71827_reg_read(mfd, BD71827_REG_BUCK5_MODE);
	val &=~(BUCK5_LP_ON);
	bd71827_reg_write(mfd, BD71827_REG_BUCK5_MODE, val);

	bd71827_reg_write(mfd, BD71827_REG_CHG_WDT_PRE, BD71827_PRECHG_TIME_30MIN);

	bd71827_reg_write(mfd, BD71827_REG_CHG_WDT_FST, BD71827_CHG_TIME_600MIN);

	bd71827_reg_write(mfd, BD71827_REG_INT_EN_05, 0x00);

	/* Clear HALL_INV */
	bd71827_clear_bits(pwr->mfd, BD71827_REG_PWRCTRL2, BD71827_PWRCTRL2_HALL_INV);
#endif
//        if (bd71827_reg_read(pwr->mfd, BD71827_REG_PWRCTRL) & RESTARTEN)
	{ /* Enable CC when out of ship mode in previous version of chip
	   * Now change to enable CC every time device reboot since
	   * 1. CC was disabled before enter ship mode, need to reenable when out of ship mode
	   * 2. CC was not disabled on regular reboot, enable CC does not change anything
	   * 3. in 71827, did not see which register indicates out of ship mode */

                /* Start Coulomb Counter */
                bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB);
                dev_dbg(pwr->dev, "%s: Start Coulomb Counter\n", __func__);

                /* moved from regulator driver.. */
                bd71827_clear_bits(pwr->mfd, BD71827_REG_PWRCTRL, RESTARTEN);
        }
	if (default_warm_reset) {
		bd71827_reg_write(mfd, BD71827_REG_PWRCTRL, PWRCTRL_NORMAL_BATCUT_COLDRST_WDG_WARMRST);
	} else {
		bd71827_reg_write(mfd, BD71827_REG_PWRCTRL, PWRCTRL_NORMAL);
	}
	pwr->temp = bd71827_get_temp(pwr);
	dev_dbg(pwr->dev,  "Temperature = %d\n", pwr->temp);
	bd71827_adjust_coulomb_count(pwr);
	bd71827_reset_coulomb_count(pwr);
	pwr->coulomb_cnt = bd71827_reg_read32(mfd, BD71827_REG_CC_CCNTD_3) & 0x0FFFFFFFUL;
	bd71827_calc_soc_org(pwr);
	pwr->soc_norm = pwr->soc_org;
	pwr->soc = pwr->soc_norm;
	pwr->clamp_soc = pwr->soc;
	dev_dbg(pwr->dev,  "%s() CC_CCNTD = %d\n", __func__, pwr->coulomb_cnt);
	dev_dbg(pwr->dev,  "%s() pwr->soc = %d\n", __func__, pwr->soc);
	dev_dbg(pwr->dev,  "%s() pwr->clamp_soc = %d\n", __func__, pwr->clamp_soc);

	pwr->cycle = battery_cycle;
	pwr->curr = 0;
	pwr->relax_time = 0;

	return 0;
}

/** @brief set bd71827 battery parameters
 * @param pwr power device
 * @return 0
 */
static int bd71827_set_battery_parameters(struct bd71827_power *pwr)
{
	//struct bd71827 *mfd = pwr->mfd;
	int i;
	int batId=0;

	if(idme_hwid_value < HWID_DVT_VAL)
		use_load_bat_params = USE_BAT_PARAMS_22KOHM;
	else {
		batId=bd71827_reg_read(pwr->mfd, BD71827_REG_BATID);
		if( batId>=0x20 && batId<=0x25 ) {
			use_load_bat_params = USE_BAT_PARAMS_15KOHM_1500MAH;
			bd71827_set_bits(pwr->mfd, BD71827_REG_CHG_SET1, CHG_EN);
		}
		else if( batId>=0x50 && batId<=0x55 ) {
			use_load_bat_params = USE_BAT_PARAMS_47KOHM_1500MAH;
			bd71827_set_bits(pwr->mfd, BD71827_REG_CHG_SET1, CHG_EN);
		}
		else {
			printk(KERN_INFO "%s !!!unknown battID:0x%x detected, disabling charging!!!\n",__func__, batId);
			bd71827_clear_bits(pwr->mfd, BD71827_REG_CHG_SET1, CHG_EN);
		}
	}
	printk(KERN_INFO "%s found battType:%d, battID:0x%x\n",__func__, use_load_bat_params, batId);

	if (use_load_bat_params == USE_BAT_PARAMS_22KOHM)
	{
		max_voltage = 4350000;
		min_voltage = 3400000;
		thr_voltage = 4250000;
		if(idme_hwid_value >= HWID_DVT_VAL)
			battery_cap_mah = 1500;
		else
			battery_cap_mah = 1023;
		dgrd_cyc_cap = 15;
		soc_est_max_num = 5;
		dgrd_temp_cap_h = (0);
		dgrd_temp_cap_m = (0);
		dgrd_temp_cap_l = (0);

		for (i = 0; i < 23; i++) {
			ocv_table[i] = ocv_table_22kohm[i];
			vdr_table_h[i] = vdr_table_h_22kohm[i];
			vdr_table_m[i] = vdr_table_m_22kohm[i];
			vdr_table_l[i] = vdr_table_l_22kohm[i];
			vdr_table_vl[i] = vdr_table_vl_22kohm[i];
		}
	}
	else if (use_load_bat_params == USE_BAT_PARAMS_15KOHM)
	{
		max_voltage = 4350000;
		min_voltage = 3400000;
		thr_voltage = 4250000;
		if(idme_hwid_value >= HWID_DVT_VAL)
			battery_cap_mah = 1500;
		else
			battery_cap_mah = 1026;
		dgrd_cyc_cap = 15;
		soc_est_max_num = 5;
		dgrd_temp_cap_h = (0);
		dgrd_temp_cap_m = (0);
		dgrd_temp_cap_l = (0);

		for (i = 0; i < 23; i++) {
			ocv_table[i] = ocv_table_15kohm[i];
			vdr_table_h[i] = vdr_table_h_15kohm[i];
			vdr_table_m[i] = vdr_table_m_15kohm[i];
			vdr_table_l[i] = vdr_table_l_15kohm[i];
			vdr_table_vl[i] = vdr_table_vl_15kohm[i];
		}
	}
	else if (use_load_bat_params == USE_BAT_PARAMS_15KOHM_1500MAH)
	{
		max_voltage = 4350000;
		min_voltage = 3400000;
		thr_voltage = 4250000;
		battery_cap_mah = 1529;
		dgrd_cyc_cap = 88;
		soc_est_max_num = 5;
		dgrd_temp_cap_h = (0);
		dgrd_temp_cap_m = (0);
		dgrd_temp_cap_l = (0);

		for (i = 0; i < 23; i++) {
			ocv_table[i] = ocv_table_15kohm_1500mah[i];
			vdr_table_h[i] = vdr_table_h_15kohm_1500mah[i];
			vdr_table_m[i] = vdr_table_m_15kohm_1500mah[i];
			vdr_table_l[i] = vdr_table_l_15kohm_1500mah[i];
			vdr_table_vl[i] = vdr_table_vl_15kohm_1500mah[i];
		}
	}
	else if (use_load_bat_params == USE_BAT_PARAMS_47KOHM_1500MAH)
	{
		max_voltage = 4350000;
		min_voltage = 3400000;
		thr_voltage = 4250000;
		battery_cap_mah = 1508;
		dgrd_cyc_cap = 169;
		soc_est_max_num = 5;
		dgrd_temp_cap_h = (0);
		dgrd_temp_cap_m = (0);
		dgrd_temp_cap_l = (0);

		for (i = 0; i < 23; i++) {
			ocv_table[i] = ocv_table_47kohm_1500mah[i];
			vdr_table_h[i] = vdr_table_h_47kohm_1500mah[i];
			vdr_table_m[i] = vdr_table_m_47kohm_1500mah[i];
			vdr_table_l[i] = vdr_table_l_47kohm_1500mah[i];
			vdr_table_vl[i] = vdr_table_vl_47kohm_1500mah[i];
		}
	} else {
		max_voltage = MAX_VOLTAGE_DEFAULT;
		min_voltage = MIN_VOLTAGE_DEFAULT;
		thr_voltage = THR_VOLTAGE_DEFAULT;
		battery_cap_mah = BATTERY_CAP_MAH_DEFAULT;
		dgrd_cyc_cap = DGRD_CYC_CAP_DEFAULT;
		soc_est_max_num = SOC_EST_MAX_NUM_DEFAULT;
		dgrd_temp_cap_h = DGRD_TEMP_CAP_H_DEFAULT;
		dgrd_temp_cap_m = DGRD_TEMP_CAP_M_DEFAULT;
		dgrd_temp_cap_l = DGRD_TEMP_CAP_L_DEFAULT;

		for (i = 0; i < 23; i++) {
			ocv_table[i] = ocv_table_default[i];
			vdr_table_h[i] = vdr_table_h_default[i];
			vdr_table_m[i] = vdr_table_m_default[i];
			vdr_table_l[i] = vdr_table_l_default[i];
			vdr_table_vl[i] = vdr_table_vl_default[i];
		}
	}
	max_current = MAX_CURRENT_DCP_DEFAULT;
	battery_full = BATTERY_FULL_DEFAULT;
	thr_relax_current = THR_RELAX_CURRENT_DEFAULT;
	thr_relax_time = THR_RELAX_TIME_DEFAULT;
	dgrd_temp_h = DGRD_TEMP_H_DEFAULT;
	dgrd_temp_m = DGRD_TEMP_M_DEFAULT;
	dgrd_temp_l = DGRD_TEMP_L_DEFAULT;
	dgrd_temp_vl = DGRD_TEMP_VL_DEFAULT;
	for (i = 0; i < 23; i++) {
		soc_table[i] = soc_table_default[i];
	}

	battery_cap = mAh_A10s(battery_cap_mah);

	return 0;
}

static void pmic_lobat_check_work(struct work_struct *work)
{
	struct bd71827_power *pwr;
	struct delayed_work *delayed_work;

	static int low_bat_count = 0;


	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71827_power, bd_lobat_check_work);
	bd71827_get_voltage_current(pwr);
	printk(KERN_INFO "%s vsys: %d, lo_bat_count%d\n",__func__, pwr->vsys, low_bat_count);
	if ( pwr->vsys/1000 <= SYS_CRIT_VOLT_THRESH)
		heisenberg_battery_lobat_event(pwr->dev, CRIT_BATT_VOLT_LEVEL);
	else if (pwr->vsys/1000 <= SYS_LOW_VOLT_THRESH) {
		low_bat_count++;
		if(low_bat_count > FG_LOW_BATT_CT)
			heisenberg_battery_lobat_event(pwr->dev, LOW_BATT_VOLT_LEVEL);
		else
			schedule_delayed_work(&pwr->bd_lobat_check_work, msecs_to_jiffies(2000));
	}
	else
		low_bat_count = 0;
}
/**@brief timed work function called by system
 *  read battery capacity,
 *  sense change of charge status, etc.
 * @param work work struct
 * @return  void
 */

static void bd_work_callback(struct work_struct *work)
{
	struct bd71827_power *pwr;
	struct delayed_work *delayed_work;
	int status, changed = 0, vbus_changed = 0;
	static int cap_counter = 0;
	static int chk_udc_counter = 0, power_metric_counter = 0;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71827_power, bd_work);

	mutex_lock(&bd_work_lock);
	status = bd71827_reg_read(pwr->mfd, BD71827_REG_DCIN_STAT);
	if (status != pwr->vbus_status) {
    	dev_dbg(pwr->dev,"DCIN_STAT CHANGED from 0x%X to 0x%X\n", pwr->vbus_status, status);
		pwr->vbus_status = status;
#ifdef CONFIG_LAB126
                if ((status & 0x1) != 0) //DCIN connected
                {
#ifdef CONFIG_PM_AUTOSLEEP
                        __pm_stay_awake(pwr->vbus_wakeup_source);
#endif
                } else {
#ifdef CONFIG_PM_AUTOSLEEP
                        __pm_relax(pwr->vbus_wakeup_source);
#endif
                }
#endif

		changed = 1;
	}

	status = bd71827_reg_read(pwr->mfd, BD71827_REG_BAT_STAT);
	status &= ~BAT_DET_DONE;
	if (status != pwr->bat_status) {
		dev_dbg(pwr->dev, "BAT_STAT CHANGED from 0x%X to 0x%X\n", pwr->bat_status, status);
		pwr->bat_status = status;
		changed = 1;
	}

	status = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_STATE);
	if (status != pwr->charge_status) {
		dev_dbg(pwr->dev, "CHG_STATE CHANGED from 0x%X to 0x%X\n", pwr->charge_status, status);
		pwr->charge_status = status;
		//changed = 1;
	}

	bd71827_get_voltage_current(pwr);
	bd71827_adjust_coulomb_count(pwr);
	bd71827_reset_coulomb_count(pwr);
//	bd71827_adjust_coulomb_count_sw(pwr);
	bd71827_coulomb_count(pwr);
	bd71827_update_cycle(pwr);
	bd71827_calc_full_cap(pwr);
	bd71827_calc_soc_org(pwr);
	bd71827_calc_soc_norm(pwr);
	bd71827_calc_soc(pwr);
	bd71827_calc_soc_clamp(pwr);
	bd71827_get_online(pwr);
	bd71827_charge_status(pwr);

        if ( pwr->vsys/1000 <= SYS_CRIT_VOLT_THRESH)  {
                heisenberg_battery_lobat_event(pwr->dev, CRIT_BATT_VOLT_LEVEL);
        } else if (pwr->vsys/1000 <= SYS_LOW_VOLT_THRESH) {
		schedule_delayed_work(&pwr->bd_lobat_check_work, msecs_to_jiffies(2000));
	}
        if(changed && pwr->charger_online != 0) //DCIN connected
        {
	       kobject_uevent(&(pwr->dev->kobj), KOBJ_ADD);
		//If charger is online, we use current thredshold to decide if relax snapshot
		//should be taken. This is a SW workaround to make sure after full charge if
		//charger is still connected, relax snapshot can be taken so that the PMIC
		//current ADC offset(a HW limit) is not causing incorrect SOC drain.
		/* Clear Relax PMU STATE mask, Relax decision is by PMU state*/
		bd71827_set_bits(pwr->mfd, BD71827_REG_REX_CTRL_1, REX_PMU_STATE_MASK);
		/* set the Relax current thredshold to be 1.98mA = 6*0.33 */
		bd71827_reg_write(pwr->mfd, BD71827_REG_REX_CTRL_2, 0x6);
        }
        else if(changed && pwr->charger_online == 0)
        {
		kobject_uevent(&(pwr->dev->kobj), KOBJ_REMOVE);

		//no charger, set the relax snapshot condition back to PMU state
		/* Clear Relax PMU STATE mask, Relax decision is by PMU state*/
		bd71827_clear_bits(pwr->mfd, BD71827_REG_REX_CTRL_1, REX_PMU_STATE_MASK);
		/* Clear Relax current thredshold */
		bd71827_reg_write(pwr->mfd, BD71827_REG_REX_CTRL_2, 0);
        }

	if (changed /*|| cap_counter++ > JITTER_REPORT_CAP / JITTER_DEFAULT*/) {
		power_supply_changed(pwr->ac);
		power_supply_changed(pwr->bat);
		cap_counter = 0;
	}

	if (pwr->calib_current == CALIB_NORM) {
		pwr->gauge_delay = JITTER_DEFAULT;
		schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(JITTER_DEFAULT));
	} else if (pwr->calib_current == CALIB_START) {
		pwr->calib_current = CALIB_GO;
	}
#ifdef CONFIG_LAB126
	if(!vbus_changed && (chk_udc_counter * JITTER_DEFAULT >= JITTER_CHK_UDC))
	//if vbus changed, then we start/stop B session based on the vbus status
	//if vbus did not change, we check usb udc status every JITTER_CHK_UDC(120 seconds) and disable usb B session if usb is not connected to a host
	{
		if(!usb_udc_connected())
		{
			pr_debug("USB UDC not connected\n");
		}
		else
			pr_debug("USB UDC connected\n");
		chk_udc_counter = 0; //reset the counter for next time to check the usb udc
	}
#ifdef CONFIG_AMAZON_METRICS_LOG
	power_metric_counter++;
	if(power_metric_counter >= POWER_METRIC_LOG_PERIOD)
	{
		unsigned long total_suspend=bd71827_total_suspend_time();
		unsigned long total_wake=bd71827_total_wake_time();
		memset(bd71827_metric_buf,0,sizeof(bd71827_metric_buf));
                snprintf(bd71827_metric_buf, sizeof(bd71827_metric_buf),
                        "kernel:pmic-bd71827:total_suspend_time=%d;CT;1,total_wake_time=%d;CT;1,total_up_time=%d;CT;1:NR",
			total_suspend, total_wake, total_suspend+total_wake);

                log_to_metrics(ANDROID_LOG_INFO, "battery", bd71827_metric_buf);
		power_metric_counter = 0;
	}
	metrics_counter ++;
	if (metrics_counter >= BATTERY_METRICS_TIMER) {
		metrics_counter = 0;
		memset(bd71827_metric_buf,0,sizeof(bd71827_metric_buf));
		snprintf(bd71827_metric_buf, sizeof(bd71827_metric_buf),
			"kernel:pmic-bd71827:battery_capacity=%d;CT;1,battery_voltage=%d;CT;1:NR",
			pwr->clamp_soc, pwr->vcell);

		log_to_metrics(ANDROID_LOG_INFO, "battery", bd71827_metric_buf);
	}
#endif

#endif
	mutex_unlock(&bd_work_lock);
}


static void bd_soc_update_work_callback(struct work_struct *work)
{
	struct bd71827_power *pwr;
	struct delayed_work *delayed_work;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71827_power, bd_soc_update_work);

	mutex_lock(&bd_work_lock);


	bd71827_get_voltage_current(pwr);
	bd71827_adjust_coulomb_count(pwr);
	bd71827_reset_coulomb_count(pwr);
	bd71827_coulomb_count(pwr);
	bd71827_update_cycle(pwr);
	bd71827_calc_full_cap(pwr);
	bd71827_calc_soc_org(pwr);
	bd71827_calc_soc_norm(pwr);
	bd71827_calc_soc(pwr);
	bd71827_calc_soc_clamp(pwr);

	printk(KERN_INFO "%s: clamp_soc=%d\n",__FUNCTION__, pwr->clamp_soc);
	mutex_unlock(&bd_work_lock);
}



/*
* register 0x9A
* #define EVENT_DCIN_SWRESET			ENCODE_EVENT(BD71827_REG_INT_STAT_03, BIT(7))
* #define EVENT_DCIN_WDOGB				ENCODE_EVENT(BD71827_REG_INT_STAT_03, BIT(6))
* #define EVENT_DCIN_PWRON_PRESS		ENCODE_EVENT(BD71827_REG_INT_STAT_03, BIT(5))
* #define EVENT_DCIN_PWRON_SHORT		ENCODE_EVENT(BD71827_REG_INT_STAT_03, BIT(4))
* #define EVENT_DCIN_PWRON_MID			ENCODE_EVENT(BD71827_REG_INT_STAT_03, BIT(3))
* #define EVENT_DCIN_PWRON_LONG			ENCODE_EVENT(BD71827_REG_INT_STAT_03, BIT(2))
* #define EVENT_DCIN_MON_DET			ENCODE_EVENT(BD71827_REG_INT_STAT_03, BIT(1))
* #define EVENT_DCIN_MON_RES			ENCODE_EVENT(BD71827_REG_INT_STAT_03, BIT(0))
*/
static void pmic_power_button_work(struct work_struct *work)
{
	struct bd71827_power *pwr;
	struct delayed_work *delayed_work;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71827_power, bd_power_work);

	mutex_lock(&pwr->irq_status[BD71827_IRQ_DCIN_03].lock);

	if (pwr->irq_status[BD71827_IRQ_DCIN_03].reg & EVENT_DCIN_PWRON_SHORT) {
		pr_debug("The event EVENT_DCIN_PWRON_SHORT 0x%0x happens\n",EVENT_DCIN_PWRON_SHORT);
		pmic_power_button_notifier_call_chain(EVENT_DCIN_PWRON_SHORT, NULL);
	}

	if (pwr->irq_status[BD71827_IRQ_DCIN_03].reg & EVENT_DCIN_PWRON_MID) {
		pr_debug("The event EVENT_DCIN_PWRON_MID 0x%0x happens\n",EVENT_DCIN_PWRON_MID);
		pmic_power_button_notifier_call_chain(EVENT_DCIN_PWRON_MID, NULL);
	}

	if (pwr->irq_status[BD71827_IRQ_DCIN_03].reg & EVENT_DCIN_PWRON_LONG) {
		pr_debug("The event EVENT_DCIN_PWRON_LONG 0x%0x happens\n",EVENT_DCIN_PWRON_LONG);
		pmic_power_button_notifier_call_chain(EVENT_DCIN_PWRON_LONG, NULL);
	}

	if (pwr->irq_status[BD71827_IRQ_DCIN_03].reg & EVENT_DCIN_PWRON_PRESS) {
		pr_debug("The event EVENT_DCIN_PWRON_LONG 0x%0x happens\n",EVENT_DCIN_PWRON_PRESS);
		pmic_power_button_notifier_call_chain(EVENT_DCIN_PWRON_PRESS, NULL);
	}

	mutex_unlock(&pwr->irq_status[BD71827_IRQ_DCIN_03].lock);

}
static void pmic_battery_work(struct work_struct *work)
{

	struct bd71827_power *pwr;
	struct delayed_work *delayed_work;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71827_power, bd_bat_work);

	mutex_lock(&pwr->irq_status[BD71827_IRQ_BAT_MON_08].lock);

	if (pwr->irq_status[BD71827_IRQ_BAT_MON_08].reg & VBAT_MON_DET) {
		pr_debug("\n~~~ VBAT LOW Detected ... \n");

	} else if (pwr->irq_status[BD71827_IRQ_BAT_MON_08].reg & VBAT_MON_RES) {
		pr_debug("\n~~~ VBAT LOW Resumed ... \n");
	}

	if (pwr->irq_status[BD71827_IRQ_BAT_MON_08].reg & EVENT_VBAT_MON_DET) {
		pr_debug("The event EVENT_DCIN_PWRON_PRESS 0x%0x happens\n",EVENT_VBAT_MON_DET);
		pmic_battery_notifier_call_chain(EVENT_VBAT_MON_DET, NULL);
	}

	if (pwr->irq_status[BD71827_IRQ_BAT_MON_08].reg & EVENT_VBAT_MON_RES) {
		pr_debug("The event EVENT_DCIN_PWRON_PRESS 0x%0x happens\n",EVENT_VBAT_MON_RES);
		pmic_battery_notifier_call_chain(EVENT_VBAT_MON_RES, NULL);
	}
	mutex_unlock(&pwr->irq_status[BD71827_IRQ_BAT_MON_08].lock);

}


static void ums_work_callback(struct work_struct *work)
{

	struct bd71827_power *pwr;
	struct delayed_work *delayed_work;
	int status = 0;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71827_power, bd_ums_work);

	status = bd71827_reg_read(pwr->mfd, BD71827_REG_DCIN_STAT);
	if (ums_cb) {
		pr_debug("bd71827 calling ums_cb DCIN_STAT=0x%x\n",status);
		if (status & 0x1)
			ums_cb(true);
		else
			ums_cb(false);
	}
}

static void bd_safe_charging_work_callback(struct work_struct *work)
{

	struct bd71827_power *pwr;
	struct delayed_work *delayed_work;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71827_power, bd_safe_charging_work);
	//DC has been inserted for DCIN_SAFT_CHARGING_DELAY (7days), need to set up safe charging
	bd71827_safe_charging_control(pwr, true);
}

extern void usb_gadget_chg_det_sync(int delay_ms);
/**@brief bd71827 power interrupt
 * @param irq system irq
 * @param pwrsys bd71827 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t bd71827_power_interrupt(int irq, void *pwrsys)
{
        struct device *dev = pwrsys;
        struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	    struct bd71827_power *pwr = dev_get_drvdata(dev);
        int reg, r;

        reg = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_03);
        if (reg <= 0)
                return IRQ_NONE;

	if (reg & POWERON_LONG) {
#if defined(CONFIG_LAB126)
        printk(KERN_INFO "POWERON_LONG pressed, INT_STAT_03 = 0x%0x \n", reg);
		dump_lastk_to_mmc();
#endif
		dev_notice(dev, "POWERON_LONG interrupt\n");
		/* expected reset here, so delay clear the INT_STAT */
		udelay(1000);
	}

handle_again:
        r = bd71827_reg_write(mfd, BD71827_REG_INT_STAT_03, reg);
        if (r)
                return IRQ_NONE;

        if (reg & POWERON_PRESS) {
#ifdef CONFIG_CPU_FREQ_OVERRIDE_LAB126
            cpufreq_override(4);
#endif
                printk(KERN_INFO "Power button pressed, INT_STAT_03 = 0x%0x \n", reg);
        }

	mutex_lock(&pwr->irq_status[BD71827_IRQ_DCIN_03].lock);
	pwr->irq_status[BD71827_IRQ_DCIN_03].reg = (int)reg;
	mutex_unlock(&pwr->irq_status[BD71827_IRQ_DCIN_03].lock);

	//TODO: Check if this really needs to be scheduled in the global work queue.
	if ( reg & POWERON_PRESS || reg & POWERON_SHORT
	  || reg & POWERON_MID   || reg & POWERON_LONG )
		schedule_delayed_work(&pwr->bd_power_work, msecs_to_jiffies(0));

        if (reg & DCIN_MON_DET || reg & DCIN_MON_RES) {
                /* BD7181X_REG_DCIN_STAT register does not reflect the change immediately, so scheduling bd_work after a delay */
                if(!schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(DCIN_STATUS_DELAY))) {
                        cancel_delayed_work(&pwr->bd_work);
                        schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(DCIN_STATUS_DELAY));
                }
		if(reg & DCIN_MON_DET) {
			// DC removed
            cancel_delayed_work(&pwr->bd_safe_charging_work);
			bd71827_safe_charging_control(pwr, false);
			usb_gadget_chg_det_sync(2000);
		} else if (reg & DCIN_MON_RES) {
			//DC inserted
			if(!schedule_delayed_work(&pwr->bd_safe_charging_work, msecs_to_jiffies(DCIN_SAFE_CHARGING_DELAY))) {
				cancel_delayed_work(&pwr->bd_safe_charging_work);
				schedule_delayed_work(&pwr->bd_safe_charging_work, msecs_to_jiffies(DCIN_SAFE_CHARGING_DELAY));
			}
			usb_gadget_chg_det_sync(0);
		}
        }

        reg = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_03);
        if (reg > 0)
        {
                printk(KERN_ERR "Handle INT again\n");
                goto handle_again;
        }

        return IRQ_HANDLED;
}

/**@brief bd71827 vbat low voltage detection interrupt
 * @param irq system irq
 * @param pwrsys bd71827 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 * added by John Zhang at 2015-07-22
 */
static irqreturn_t bd71827_vbat_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	// struct bd71827_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	dev_info(mfd->dev, "bd71827_vbat_interrupt() in.\n");

	reg = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_08);
	if (reg < 0)
		return IRQ_NONE;

	dev_info(mfd->dev, "INT_STAT_08 = 0x%.2X\n", reg);

	r = bd71827_reg_write(mfd, BD71827_REG_INT_STAT_08, reg);
	if (r)
		return IRQ_NONE;

	if (reg & VBAT_MON_DET) {
		dev_info(mfd->dev, "\n~~~ VBAT LOW Detected ... \n");

	} else if (reg & VBAT_MON_RES) {
		dev_info(mfd->dev, "\n~~~ VBAT LOW Resumed ... \n");
	}

	return IRQ_HANDLED;

}

/**@brief bd71827 int_stat_11 detection interrupt
 * @param irq system irq
 * @param pwrsys bd71827 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 * added 2015-12-26
 */
static irqreturn_t bd71827_int_11_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	// struct bd71827_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	dev_info(mfd->dev, "bd71827_int_11_interrupt() in.\n");

	reg = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_11);
	if (reg < 0)
		return IRQ_NONE;

	dev_info(mfd->dev, "INT_STAT_11 = 0x%.2X\n", reg);

	r = bd71827_reg_write(mfd, BD71827_REG_INT_STAT_11, reg);
	if (r) {
		return IRQ_NONE;
	}

	if (reg & INT_STAT_11_VF_DET) {
		dev_info(mfd->dev, "\n~~~ VF Detected ... \n");
	} else if (reg & INT_STAT_11_VF_RES) {
		dev_info(mfd->dev, "\n~~~ VF Resumed ... \n");
	} else if (reg & INT_STAT_11_VF125_DET) {
		dev_info(mfd->dev, "\n~~~ VF125 Detected ... \n");
	} else if (reg & INT_STAT_11_VF125_RES) {
		dev_info(mfd->dev, "\n~~~ VF125 Resumed ... \n");
	} else if (reg & INT_STAT_11_OVTMP_DET) {
		dev_info(mfd->dev, "\n~~~ Overtemp Detected ... \n");
	} else if (reg & INT_STAT_11_OVTMP_RES) {
		dev_info(mfd->dev, "\n~~~ Overtemp Detected ... \n");
	} else if (reg & INT_STAT_11_LOTMP_DET) {
		dev_info(mfd->dev, "\n~~~ Lowtemp Detected ... \n");
	} else if (reg & INT_STAT_11_LOTMP_RES) {
		dev_info(mfd->dev, "\n~~~ Lowtemp Detected ... \n");
	}

	return IRQ_HANDLED;

}

/* TODO: The charger interrupt handler is no in use right now.
* we might need to use it later on, so just commented it out
*/
#if 0
/**@brief bd71827 bd71827_charging state interrupt
 * @param irq system irq
 * @param pwrsys bd71827 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 * added 2016-02-19
 */
static irqreturn_t bd71827_charger_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	struct bd71827_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	reg = bd71827_reg_read(mfd, BD71827_REG_INT_STAT_05);

	if (reg <= 0)
		return IRQ_NONE;

	printk("INT_STAT_5 = 0x%.2X\n", reg);

	r = bd71827_reg_write(mfd, BD71827_REG_INT_STAT_05, reg);
	if (r) {
		return IRQ_NONE;
	}

	if (reg & CHARGE_TRNS) {
		reg = bd71827_reg_read(mfd, BD71827_REG_CHG_STATE);
		if ((reg == CHARGE_TOP_OFF) || (reg == CHARGE_DONE)) {
			/* Set Coulomb Counter to full charged */
			bd71827_force_reset_coulomb_count(pwr);
		}
	}
	return IRQ_HANDLED;

}
#endif

/** @brief get property of power supply ac
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */
static int bd71827_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd71827_power *pwr = dev_get_drvdata(psy->dev.parent);
	u32 vot;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->charger_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd71827_reg_read16(pwr->mfd, BD71827_REG_VM_DCIN_U);
		val->intval = 5000 * vot;		// 5 milli volt steps
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief get property of power supply bat
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */

static int bd71827_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd71827_power *pwr = dev_get_drvdata(psy->dev.parent);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		bd71827_charge_status(pwr);
		val->intval = pwr->rpt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = pwr->bat_health;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (pwr->rpt_status == POWER_SUPPLY_STATUS_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		bd71827_get_voltage_current(pwr);
		val->intval = pwr->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = pwr->clamp_soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		{
		u32 t;

		t = pwr->coulomb_cnt >> 16;
		t = A10s_mAh(t);
		if (t > A10s_mAh(pwr->designed_cap)) t = A10s_mAh(pwr->designed_cap);
		val->intval = t * 1000;		/* uA to report */
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = battery_full * A10s_mAh(pwr->designed_cap) * 10;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = battery_full * A10s_mAh(pwr->full_cap) * 10;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		bd71827_get_voltage_current(pwr);
		val->intval = pwr->curr_sar;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = pwr->curr;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = pwr->temp;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = max_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = min_voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = max_current;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief ac properties */
static enum power_supply_property bd71827_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/** @brief bat properies */
static enum power_supply_property bd71827_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

/** @brief directly set raw value to chip register, format: 'register value' */
static ssize_t bd71827_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		pwr->reg_index = -1;
		dev_err(pwr->dev, "registers set: <reg> <value>\n");
		return count;
	}

	if (ret == 1 && reg <= BD71827_MAX_REGISTER) {
		pwr->reg_index = reg;
		dev_dbg(pwr->dev, "registers set: reg=0x%x\n", reg);
		return count;
	}

	if (reg > BD71827_MAX_REGISTER || val > 255)
		return -EINVAL;

	dev_dbg(pwr->dev, "registers set: reg=0x%x, val=0x%x\n", reg, val);
	ret = bd71827_reg_write(pwr->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/** @brief print value of chip register, format: 'register=value' */
static ssize_t bd71827_sysfs_print_reg(struct bd71827_power *pwr,
				       u8 reg,
				       char *buf)
{
	int ret = bd71827_reg_read(pwr->mfd, reg);

	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/** @brief show all raw values of chip register, format per line: 'register=value' */
static ssize_t bd71827_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	int i;

	dev_dbg(pwr->dev, "register: index[%d]\n", pwr->reg_index);
	if (pwr->reg_index >= 0) {
		ret += bd71827_sysfs_print_reg(pwr, pwr->reg_index, buf + ret);
	} else {
		for (i = 0; i < BD71827_MAX_REGISTER; i++) {
			ret += bd71827_sysfs_print_reg(pwr, i, buf + ret);
		}
	}
	return ret;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bd71827_sysfs_show_registers, bd71827_sysfs_set_registers);

/** @brief directly set charging status, set 1 to enable charging, set 0 to disable charging */
static ssize_t bd71827_sysfs_set_charging(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	//unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x", &val);
	printk(KERN_ERR "%s val=%x\n",__FUNCTION__, val);
	if (ret < 1) {
		return count;
	}

	if (ret == 1 && val >1) {
		return count;
	}

	if(val == 1)
		bd71827_set_bits(pwr->mfd, BD71827_REG_CHG_SET1, CHG_EN);
	else
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CHG_SET1, CHG_EN);

	return count;

}
/** @brief show charging status' */
static ssize_t bd71827_sysfs_show_charging(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	int reg_value=0;
	//ssize_t ret = 0;
	//unsigned int reg;

	reg_value = bd71827_reg_read(pwr->mfd, BD71827_REG_CHG_SET1);

//	printk(KERN_ERR "%s charger_online:%x, reg_value:%x\n",__FUNCTION__, pwr->charger_online, reg_value);
	return sprintf(buf, "%x\n", pwr->charger_online && reg_value & CHG_EN);
}

static DEVICE_ATTR(charging, S_IWUSR | S_IRUGO,
		bd71827_sysfs_show_charging, bd71827_sysfs_set_charging);

/** @brief show reset reasons */
static ssize_t bd71827_sysfs_show_reset_reasons(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
	log_reset_reason(dev);
	return 0;
}

static DEVICE_ATTR(reset_reasons, S_IRUGO, bd71827_sysfs_show_reset_reasons, NULL);

#ifdef CONFIG_AMAZON_SIGN_OF_LIFE_BD71827
#define DEV_SOL_PROC_NAME   "life_cycle_reason"

static int life_cycle_metrics_show(struct seq_file *m, void *v)
{
	u8 i;
	for ( i = 0; i < ARRAY_SIZE(reset_reason_desc); i++) {
		if (reset_reason & (1<<i)) {
			seq_printf(m, "%s", reset_reason_desc[i][1]);
			return 0;
		}
	}
	seq_printf(m, "Life Cycle Reason Not Available");
	return 0;
}

static int life_cycle_metrics_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, life_cycle_metrics_show, NULL);
}

static const struct file_operations life_cycle_metrics_proc_fops = {
	.open = life_cycle_metrics_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct proc_dir_entry *life_cycle_metrics_file;
void life_cycle_metrics_proc_init(void)
{
	life_cycle_metrics_file = proc_create(DEV_SOL_PROC_NAME,
				  0444, NULL, &life_cycle_metrics_proc_fops);
	if (life_cycle_metrics_file == NULL) {
		printk(KERN_ERR "%s: Can't create life cycle metrics proc entry\n", __func__);
	}
}
EXPORT_SYMBOL(life_cycle_metrics_proc_init);
#endif

static ssize_t gpio_epd_en_set(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	unsigned int val;
	ssize_t ret = 0;

	ret = sscanf(buf, "%x", &val);
	printk(KERN_ERR "%s val=%x\n",__FUNCTION__, val);
	if (ret < 1) {
		return count;
	}

	if (ret == 1 && val >1) {
		return count;
	}

	if(val == 1)
		gpio_epd_enable_hv(1);
	else
		gpio_epd_enable_hv(0);

	return count;
}

static DEVICE_ATTR(gpio_epd_en, S_IWUSR, NULL, gpio_epd_en_set);

static ssize_t gpio_epd_enop_set(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	unsigned int val;
	ssize_t ret = 0;

	ret = sscanf(buf, "%x", &val);
	printk(KERN_ERR "%s val=%x\n",__FUNCTION__, val);
	if (ret < 1) {
		return count;
	}

	if (ret == 1 && val >1) {
		return count;
	}

	if(val == 1)
		gpio_epd_enable_vcom(1);
	else
		gpio_epd_enable_vcom(0);

	return count;
}

static DEVICE_ATTR(gpio_epd_enop, S_IWUSR, NULL, gpio_epd_enop_set);


static ssize_t gpio_epd_pin_set(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	unsigned int val;
	ssize_t ret = 0;

	ret = sscanf(buf, "%x", &val);
	printk(KERN_ERR "%s val=%x\n",__FUNCTION__, val);
	if (ret < 1) {
		return count;
	}

	if (ret == 1 && val >1) {
		return count;
	}

	if(val == 1)
		gpio_epd_init_pins();
	else
		gpio_epd_free_pins();

	return count;
}

static DEVICE_ATTR(gpio_epd_pin, S_IWUSR, NULL, gpio_epd_pin_set);

// will use clamp_soc * designed_cap to show the mah, so that the mah logged in kdm is consistent with
// the soc show on the device.
static ssize_t bd71827_sysfs_show_battery_mah(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	return sprintf(buf, "%d\n", pwr->clamp_soc * pwr->designed_cap/100);
}
static DEVICE_ATTR(battery_mah, S_IRUGO,bd71827_sysfs_show_battery_mah, NULL);

int (*display_temp_fp)(void);
EXPORT_SYMBOL(display_temp_fp);

static ssize_t bd71827_sysfs_show_battery_id(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
	int batId = 0;
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);

	batId=bd71827_reg_read(pwr->mfd, BD71827_REG_BATID);
	if( batId>=0x20 && batId<=0x25  ||  batId>=0x50 && batId<=0x55 ) {
		return sprintf(buf, "%d", batId);
	}
	else {
		printk(KERN_INFO "%s !!!unknown battID:0x%x detected, disabling charging!!!\n",__func__, batId);
		bd71827_clear_bits(pwr->mfd, BD71827_REG_CHG_SET1, CHG_EN);
		return sprintf(buf, "%s", "Invalid Battery");
	}
}

static DEVICE_ATTR(battery_id, S_IRUGO, bd71827_sysfs_show_battery_id, NULL);

static int first_offset(struct bd71827_power *pwr)
{
	unsigned char ra2, ra3, ra6, ra7;
	unsigned char ra2_temp;
	struct bd71827 *mfd = pwr->mfd;

	bd71827_reg_write_nocheck(mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_1);
	bd71827_reg_write_nocheck(mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_2);
	bd71827_reg_write_nocheck(mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_3);


	ra2 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_11);	// I want to know initial A2 & A3.
	ra3 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_12);	// I want to know initial A2 & A3.
	ra6 = bd71827_reg_read_nocheck(mfd, 0xA6);
	ra7 = bd71827_reg_read_nocheck(mfd, 0xA7);

	bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_11, 0x00);
	bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_12, 0x00);

	dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2);
	dev_info(pwr->dev, "TEST[A3] = 0x%.2X\n", ra3);
	dev_info(pwr->dev, "TEST[A6] = 0x%.2X\n", ra6);
	dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);

	//-------------- First Step -------------------
	dev_info(pwr->dev, "Frist Step begginning \n");

	// delay some time , Make a state of IBAT=0mA
	// mdelay(1000 * 10);

	ra2_temp = ra2;

	if (ra7 != 0) {
		//if 0<0xA7<20 decrease the Test register 0xA2[7:3] until 0xA7 becomes 0x00.
		if ((ra7 > 0) && (ra7 < 20)) {
			do {
				ra2 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_11);
				ra2_temp = ra2 >> 3;
				ra2_temp -= 1;
				ra2_temp <<= 3;
				bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_11, ra2_temp);
				dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2_temp);

				ra7 = bd71827_reg_read_nocheck(mfd, 0xA7);
				dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);
				mdelay(1000);	// 1sec?
			} while (ra7);

			dev_info(pwr->dev, "A7 becomes 0 . \n");

		}		// end if((ra7 > 0)&&(ra7 < 20))
		else if ((ra7 > 0xDF) && (ra7 < 0xFF))
			//if DF<0xA7<FF increase the Test register 0xA2[7:3] until 0xA7 becomes 0x00.
		{
			do {
				ra2 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_11);
				ra2_temp = ra2 >> 3;
				ra2_temp += 1;
				ra2_temp <<= 3;

				bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_11, ra2_temp);
				dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2_temp);

				ra7 = bd71827_reg_read_nocheck(mfd, 0xA7);
				dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);
				mdelay(1000);	// 1sec?
			} while (ra7);

			dev_info(pwr->dev, "A7 becomes 0 . \n");
		}
	}

	// please use "ra2_temp" at step2.
	return ra2_temp;
}

static int second_step(struct bd71827_power *pwr, u8 ra2_temp)
{
	u16 ra6, ra7;
	u8 aft_ra2, aft_ra3;
	u8 r79, r7a;
	unsigned int LNRDSA_FUSE;
	long ADC_SIGN;
	long DSADGAIN1_INI;
	struct bd71827 *mfd = pwr->mfd;

	//-------------- Second Step -------------------
	dev_info(pwr->dev, "Second Step begginning \n");

	// need to change boad setting ( input 1A tio 10mohm)
	// delay some time , Make a state of IBAT=1000mA
	// mdelay(1000 * 10);

// rough adjust
	dev_info(pwr->dev, "ra2_temp = 0x%.2X\n", ra2_temp);

	ra6 = bd71827_reg_read_nocheck(mfd, 0xA6);
	ra7 = bd71827_reg_read_nocheck(mfd, 0xA7);
	ra6 <<= 8;
	ra6 |= ra7;		// [0xA6 0xA7]
	dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);

	bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_11, ra2_temp);	// this value from step1
	bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_12, 0x00);

	bd71827_reg_write_nocheck(mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_USER_AREA);

	r79 = bd71827_reg_read(mfd, 0x79);
	r7a = bd71827_reg_read(mfd, 0x7A);

	ADC_SIGN = r79 >> 7;
	ADC_SIGN = 1 - (2 * ADC_SIGN);
	DSADGAIN1_INI = r79 << 8;
	DSADGAIN1_INI = DSADGAIN1_INI + r7a;
	DSADGAIN1_INI = DSADGAIN1_INI & 0x7FFF;
	DSADGAIN1_INI = DSADGAIN1_INI * ADC_SIGN; //  unit 0.001

	// unit 0.000001
	DSADGAIN1_INI *= 1000;
	{
	if (DSADGAIN1_INI > 1000001) {
		DSADGAIN1_INI = 2048000000UL - (DSADGAIN1_INI - 1000000) * 8187;
	} else if (DSADGAIN1_INI < 999999) {
		DSADGAIN1_INI = -(DSADGAIN1_INI - 1000000) * 8187;
	} else {
		DSADGAIN1_INI = 0;
	}
	}

	LNRDSA_FUSE = (int) DSADGAIN1_INI / 1000000;

	dev_info(pwr->dev, "LNRDSA_FUSE = 0x%.8X\n", LNRDSA_FUSE);

	aft_ra2 = (LNRDSA_FUSE >> 8) & 255;
	aft_ra3 = (LNRDSA_FUSE) & 255;

	aft_ra2 = aft_ra2 + ra2_temp;

	bd71827_reg_write_nocheck(mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_1);
	bd71827_reg_write_nocheck(mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_2);
	bd71827_reg_write_nocheck(mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_3);

	bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_11, aft_ra2);
	bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_12, aft_ra3);

	return 0;
}

static int third_step(struct bd71827_power *pwr, unsigned thr)
{
	u16 ra2_a3, ra6, ra7;
	u8 ra2, ra3;
	u8 aft_ra2, aft_ra3;
	struct bd71827 *mfd = pwr->mfd;

// fine adjust
	ra2 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_11);	//
	ra3 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_12);	//

	ra6 = bd71827_reg_read_nocheck(mfd, 0xA6);
	ra7 = bd71827_reg_read_nocheck(mfd, 0xA7);
	ra6 <<= 8;
	ra6 |= ra7;		// [0xA6 0xA7]
	dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);


	if (ra6 > thr) {
		do {
			ra2_a3 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_11);
			ra2_a3 <<= 8;
			ra3 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_12);
			ra2_a3 |= ra3;
			//ra2_a3 >>= 3; // ? 0xA3[7:3] , or 0xA3[7:0]

			ra2_a3 -= 1;
			//ra2_a3 <<= 3;
			ra3 = ra2_a3;
			bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_12, ra3);

			ra2_a3 >>= 8;
			ra2 = ra2_a3;
			bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_11, ra2);

			dev_info(pwr->dev, "TEST[A2] = 0x%.2X , TEST[A3] = 0x%.2X \n", ra2, ra3);

			mdelay(1000);	// 1sec?

			ra6 = bd71827_reg_read_nocheck(mfd, 0xA6);
			ra7 = bd71827_reg_read_nocheck(mfd, 0xA7);
			ra6 <<= 8;
			ra6 |= ra7;	// [0xA6 0xA7]
			dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);
		} while (ra6 > thr);
	} else if (ra6 < thr) {
		do {
			ra2_a3 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_11);
			ra2_a3 <<= 8;
			ra3 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_12);
			ra2_a3 |= ra3;
			//ra2_a3 >>= 3; // ? 0xA3[7:3] , or 0xA3[7:0]

			ra2_a3 += 1;
			//ra2_a3 <<= 3;
			ra3 = ra2_a3;
			bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_12, ra3);

			ra2_a3 >>= 8;
			ra2 = ra2_a3;
			bd71827_reg_write_nocheck(mfd, BD71827_REG_INT_STAT_11, ra2);

			dev_info(pwr->dev, "TEST[A2] = 0x%.2X , TEST[A3] = 0x%.2X \n", ra2, ra3);

			mdelay(1000);	// 1sec?

			ra6 = bd71827_reg_read_nocheck(mfd, 0xA6);
			ra7 = bd71827_reg_read_nocheck(mfd, 0xA7);
			ra6 <<= 8;
			ra6 |= ra7;	// [0xA6 0xA7]
			dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);

		} while (ra6 < thr);
	}

	dev_info(pwr->dev, "[0xA6 0xA7] becomes [0x%.4X] . \n", thr);
	dev_info(pwr->dev, " Calibation finished ... \n\n");

	aft_ra2 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_11);	//
	aft_ra3 = bd71827_reg_read_nocheck(mfd, BD71827_REG_INT_STAT_12);	// I want to know initial A2 & A3.

	dev_info(pwr->dev, "TEST[A2,A3] = 0x%.2X%.2X\n", aft_ra2, aft_ra3);

	// bd71827_reg_write_nocheck(mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_USER_AREA);

	return 0;
}

static ssize_t bd71827_sysfs_set_calibrate(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	unsigned int val, mA;
	static u8 rA2;

	ret = sscanf(buf, "%d %d", &val, &mA);
	if (ret < 1) {
		dev_err(pwr->dev, "error: write a integer string");
		return count;
	}

	if (val == 1) {
		pwr->calib_current = CALIB_START;
		while (pwr->calib_current != CALIB_GO) {
			msleep(500);
		}
		rA2 = first_offset(pwr);
	}
	if (val == 2) {
		second_step(pwr, rA2);
	}
	if (val == 3) {
		if (ret <= 1) {
			dev_err(pwr->dev, "error: Fine adjust need a mA argument!");
		} else {
		unsigned int ra6_thr;

		ra6_thr = mA * 0xFFFF / 20000;
		dev_info(pwr->dev, "Fine adjust at %d mA, ra6 threshold %d(0x%X)\n", mA, ra6_thr, ra6_thr);
		third_step(pwr, ra6_thr);
		}
	}
	if (val == 4) {
		bd71827_reg_write_nocheck(pwr->mfd, BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_USER_AREA);
		pwr->calib_current = CALIB_NORM;
		pwr->gauge_delay = 0;
		schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(0));
	}

	return count;
}

static ssize_t bd71827_sysfs_show_calibrate(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	// struct power_supply *psy = dev_get_drvdata(dev);
	// struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;

	ret = 0;
	ret += sprintf(buf + ret, "write string value\n"
		"\t1      0 mA for step one\n"
		"\t2      1000 mA for rough adjust\n"
		"\t3 <mA> for fine adjust\n"
		"\t4      exit current calibration\n");
	return ret;
}

static DEVICE_ATTR(calibrate, S_IWUSR | S_IRUGO,
		bd71827_sysfs_show_calibrate, bd71827_sysfs_set_calibrate);

/*
 * Post a low battery or a critical battery event to the userspace
 */
void heisenberg_battery_lobat_event(struct device  *dev, int crit_level)
{
	printk(KERN_ERR "%d %s heisenberg_lobat_event %d heisenberg_critbat_event %d",
		__LINE__, __func__, heisenberg_lobat_event, heisenberg_critbat_event);
	if (!crit_level) {
		if (!heisenberg_lobat_event) {
			char *envp[] = { "BATTERY=low", NULL };
			printk(KERN_CRIT "KERNEL: I pmic:fg battery valrtmin::lowbat event\n");
			kobject_uevent_env(&(dev->kobj), KOBJ_CHANGE, envp);
			heisenberg_lobat_event = 1;
		}
	} else {
		if (!heisenberg_critbat_event) {
			char *envp[] = { "BATTERY=critical", NULL };
			printk(KERN_CRIT "KERNEL: I pmic:fg battery mbattlow::critbat event\n");
			kobject_uevent_env(&(dev->kobj), KOBJ_CHANGE, envp);
			heisenberg_critbat_event = 1;
		}
	}
}

static void heisenberg_battery_overheat_event(struct device *dev)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	char *envp[] = { "BATTERY=overheat", NULL };
	printk(KERN_CRIT "KERNEL: E pmic:fg battery temp::overheat event temp=%dC\n",
			pwr->temp);
	kobject_uevent_env(&(dev->kobj), KOBJ_CHANGE, envp);
	return;
}
#ifdef DEVELOPMENT_MODE
static ssize_t
battery_store_send_lobat_uevent(struct device *dev, struct attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) <= 0) {
		return -EINVAL;
	}

	if (value == 1) {
		heisenberg_battery_lobat_event(dev, 0);
	} else if (value == 2) {
		heisenberg_battery_lobat_event(dev, 1);
	} else {
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(send_lobat_uevent, S_IWUSR, NULL, battery_store_send_lobat_uevent);

static ssize_t
battery_store_send_overheat_uevent(struct device *dev, struct attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) <= 0) {
		return -EINVAL;
	}
	if (value > 0) {
		heisenberg_battery_overheat_event(dev);
	} else {
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(send_overheat_uevent, S_IWUSR, NULL, battery_store_send_overheat_uevent);

static ssize_t bd71827_sysfs_show_battery_soc(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return sprintf(buf, "%d\n", bd71827_get_battery_soc());
}
static DEVICE_ATTR(battery_soc, S_IRUGO,bd71827_sysfs_show_battery_soc, NULL);
#endif

static ssize_t
debug_soc_func(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value, i;
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);

	if (sscanf(buf, "%d", &value) <= 0) {
		return -EINVAL;
	}
	if (value > 0) {
		debug_soc_enable=value;
		printk(KERN_ERR "hw_ocv1:%d, hw_ocv2:%d, hw_ocv_pwron:%d\n",pwr->hw_ocv1, pwr->hw_ocv2, pwr->hw_ocv_pwron);
		printk(KERN_ERR "coulomb_cnt:%d, designed_cap:%d,full_cap:%d\n",pwr->coulomb_cnt, pwr->designed_cap,pwr->full_cap);
		printk(KERN_ERR "vsys_min:%d\n",pwr->vsys_min);
		for(i=0;i<23;i++)
			printk(KERN_ERR "\t\tsoc_table[%d]=%d\t\tocv_table[%d]=%d\n",i,soc_table[i],i,ocv_table[i]);
	} else {
		debug_soc_enable=0;
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(debug_soc, S_IWUSR, NULL, debug_soc_func);


static ssize_t bd71827_battery_cycle_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	return sprintf(buf, "%d\n", battery_cycle);
}

static ssize_t
bd71827_battery_cycle_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	int value;
        struct power_supply *psy = dev_get_drvdata(dev);
        struct bd71827_power *pwr = power_supply_get_drvdata(psy);

	if (sscanf(buf, "%d", &value) <= 0) {
		return -EINVAL;
	}
	if (value >= 0) {
		battery_cycle=value;
		pwr->cycle=battery_cycle;

	} else {
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(battery_cycle, S_IWUSR | S_IRUGO, bd71827_battery_cycle_show, bd71827_battery_cycle_store);

static ssize_t bd7181x_pmic_pwrctrl_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int reg = 0;

	if (pmic_pwr)
		reg = bd71827_reg_read(pmic_pwr->mfd, BD71827_REG_PWRCTRL);
	return sprintf(buf, "0x%x\n", reg);
}

static ssize_t
bd71827_watchdog_use_warm_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	int value;
        struct power_supply *psy = dev_get_drvdata(dev);
        struct bd71827_power *pwr = power_supply_get_drvdata(psy);

	if (sscanf(buf, "%d", &value) <= 0) {
		return -EINVAL;
	}
	if (value > 0) {
		bd71827_reg_write(pwr->mfd, BD71827_REG_PWRCTRL, PWRCTRL_NORMAL_BATCUT_COLDRST_WDG_WARMRST);
	} else {
		bd71827_reg_write(pwr->mfd, BD71827_REG_PWRCTRL, PWRCTRL_NORMAL);
	}
	return count;
}

static DEVICE_ATTR(watchdog_use_warm_reset, S_IWUSR | S_IRUGO, bd7181x_pmic_pwrctrl_show,
		   bd71827_watchdog_use_warm_reset);

static ssize_t bd71827_sysfs_set_gauge(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;
	int delay = -1;

	ret = sscanf(buf, "%d", &delay);
	if (ret < 1) {
		dev_err(pwr->dev, "error: write a integer string");
		return count;
	}

	if (delay == -1) {
		dev_info(pwr->dev, "Gauge schedule cancelled\n");
		cancel_delayed_work(&pwr->bd_work);
		return count;
	}

	dev_info(pwr->dev, "Gauge schedule in %d\n", delay);
	pwr->gauge_delay = delay;
	schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(delay));

	return count;
}

static ssize_t bd71827_sysfs_show_gauge(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71827_power *pwr = power_supply_get_drvdata(psy);
	ssize_t ret = 0;

	ret = 0;
	ret += sprintf(buf + ret, "Gauge schedule in %d\n", pwr->gauge_delay);
	return ret;
}

static DEVICE_ATTR(gauge, S_IWUSR | S_IRUGO,
		bd71827_sysfs_show_gauge, bd71827_sysfs_set_gauge);

static struct attribute *bd71827_sysfs_attributes[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use pwr supply class props.
	 */
	&dev_attr_registers.attr,
	&dev_attr_charging.attr,
#ifdef DEVELOPMENT_MODE
	&dev_attr_calibrate.attr,
	&dev_attr_send_overheat_uevent.attr,
	&dev_attr_send_lobat_uevent.attr,
	&dev_attr_battery_soc.attr,
#endif
	&dev_attr_debug_soc.attr,
	&dev_attr_reset_reasons.attr,
	&dev_attr_battery_id.attr,
	&dev_attr_battery_cycle.attr,
	&dev_attr_watchdog_use_warm_reset.attr,
	&dev_attr_pwrkey_ctrl.attr,
	&dev_attr_battery_mah.attr,
	&dev_attr_gpio_epd_pin.attr,
	&dev_attr_gpio_epd_en.attr,
	&dev_attr_gpio_epd_enop.attr,
	&dev_attr_gauge.attr,
	NULL,
};

static const struct attribute_group bd71827_sysfs_attr_group = {
	.attrs = bd71827_sysfs_attributes,
};

/** @brief powers supplied by bd71827_ac */
static char *bd71827_ac_supplied_to[] = {
	BAT_NAME,
};

static struct power_supply_desc bd71827_ac_desc = {
	.name		= AC_NAME,
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= bd71827_charger_props,
	.num_properties	= ARRAY_SIZE(bd71827_charger_props),
	.get_property	= bd71827_charger_get_property,
};

static const struct power_supply_desc bd71827_battery_desc = {
	.name		= BAT_NAME,
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties	= bd71827_battery_props,
	.num_properties	= ARRAY_SIZE(bd71827_battery_props),
	.get_property	= bd71827_battery_get_property,
};

/* called from pm inside machine_halt */
void bd71827_chip_hibernate(void)
{
    /* Disable Coulomb Counter before entering ship mode*/
    ext_bd71827_reg_write8(BD71827_REG_CC_CTRL,0x0);
	/* programming sequence in EANAB-151 */
	ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_NORMAL);
	ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_RESET);

}

/* called from pm inside machine_power_off */
#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
void bd71827_chip_poweroff(void)
{
	int i = 0;
    mutex_lock(&bd_work_lock);

    if (!is_hibernation) {
	    ext_bd71827_reg_write8(BD71827_REG_BUCK1_VOLT_RUN, BD71827_REG_BUCK1_VOLT_H_DEFAULT);
	    ext_bd71827_reg_write8(BD71827_REG_BUCK1_VOLT_SUSP, BD71827_REG_BUCK1_VOLT_L_DEFAULT);
	    ext_bd71827_reg_write8(BD71827_REG_BUCK2_VOLT_RUN,BD71827_REG_BUCK2_VOLT_H_DEFAULT);
	    ext_bd71827_reg_write8(BD71827_REG_BUCK2_VOLT_SUSP,BD71827_REG_BUCK2_VOLT_L_DEFAULT);
	    /* Disable ALM0 RTC interrupts in shutdown */
	    ext_bd71827_reg_write8(BD71827_REG_ALM0_MASK, 0x0);
	    /* Disable Coulomb Counter before entering ship mode*/
	    ext_bd71827_reg_write8(BD71827_REG_CC_CTRL,0x0);
	}

    if (is_hibernation) {
    	/*Enable ALM0 RTC interrupts in hibernation*/
	    ext_bd71827_reg_write8(BD71827_REG_ALM0_MASK, 0x77);

		/* Set Vsys_MIN (regadd:0x46) to 3.392V (0x35) before HBNT */
		printk("Set Vsys_MIN (regadd:0x46) to 3.392V (0x35) before HBNT\n");
		ext_bd71827_reg_write8(0x46, 0x35);
		printk("Reading back value of Vsys_MIN = 0x%X\n", ext_bd71827_reg_read8(0x46));

		printk("Disabling wifi card.\n");
		extern int wifi_card_disable(void);
   		wifi_card_disable();
	}

    if (default_warm_reset) {
	    ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_NORMAL_BATCUT_COLDRST_WDG_WARMRST);
	    ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_RESET_BATCUT_COLDRST_WDG_WARMRST);
    } else {
	    ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_NORMAL);
	    ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_RESET);
    }

    if (!is_hibernation) {
	    /* Disable Hall sensor and RTC0 but keep power button and DCIN enabled for ONEVENT */
	    ext_bd71827_reg_write8(BD71827_REG_ONEVNT_MODE_2,
	                ext_bd71827_reg_read8(BD71827_REG_ONEVNT_MODE_2) &
	                ~(HALL_ONEVENT_BIT | RTC0_ONEVENT_BIT));
	}

	printk("BD71827_REG_ALM0_MASK = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ALM0_MASK) );
	printk("BD71827_REG_ALM0_SEC = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ALM0_SEC) );
	printk("BD71827_REG_ALM0_MIN = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ALM0_MIN) );
	printk("BD71827_REG_ALM0_HOUR = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ALM0_HOUR) );
	printk("BD71827_REG_ALM0_WEEK = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ALM0_WEEK) );
	printk("BD71827_REG_ALM0_DAY = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ALM0_DAY) );
	printk("BD71827_REG_ALM0_MONTH = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ALM0_MONTH) );
	printk("BD71827_REG_ALM0_YEAR = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ALM0_YEAR) );

	printk("BD71827_REG_CC_CTRL = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_CC_CTRL) );
	printk("BD71827_REG_ONEVNT_MODE_1 = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ONEVNT_MODE_1) );
	printk("BD71827_REG_ONEVNT_MODE_2 = 0x%X\n", ext_bd71827_reg_read8(BD71827_REG_ONEVNT_MODE_2) );

    if (is_hibernation) {
        pr_info("Enable HALL_DET\n");
        ext_bd71827_reg_write8(BD71827_REG_INT_EN_13, 0x02);

	/* set hibernation flag */
	ext_bd71827_reg_write8(BD71827_REG_RESERVE_0, HIBERNATION);

    	printk("Enter hibernation mode\n");
	    /* PWRCTRL3 bit 1 to enter hibernation mode from RUN mode */
	    for(i = 0; i < 5; i++) {
		    ext_bd71827_reg_write8(BD71827_REG_PWRCTRL3, 0x02);
		    /*sleep 3 seconds to ensure device go into hibernation mode*/
		    msleep(3000);
		    printk("We should never see this message!\n");
	    }
    } else {
	/* set poweroff flag */
	ext_bd71827_reg_write8(BD71827_REG_RESERVE_0, POWEROFF);

	while(true)
	{
		vm_dcin_value = ext_bd71827_reg_read16(BD71827_REG_VM_DCIN_U);
		printk(KERN_INFO "%s: DCIN=0x%x\n", __func__, vm_dcin_value);
		if(vm_dcin_value < 0xFA) {
			printk(KERN_INFO "DCIN=0x%x below threshold, proceed with poweroff\n", vm_dcin_value);
			break;
		}
		msleep(200);
	}
	/*First disable all pmic interrupts*/
	/*mask all pmic interrputs*/
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_01, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_02, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_03, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_04, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_05, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_06, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_07, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_08, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_09, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_10, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_11, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_12, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_13, 0x0);

	mdelay(100);

	if(pmic_data)
		bd71827_testmode_workaround(pmic_data, SHIPMODE_WORKAROUND);

	ext_bd71827_reg_write8(BD71827_REG_LDO_PD_DIS, 0x01);
	    /* PWRCTRL3 bit 0 to enter ship mode from RUN mode */
	ext_bd71827_reg_write8(BD71827_REG_PWRCTRL3, 0x01);
    }
    mutex_unlock(&bd_work_lock);
}
#else
void bd71827_chip_poweroff(void)
{
	u16 vm_dcin_value;
	/* Make sure EMMC data cache completely write-back to NAND flash */
	extern void mmc_shipmode(void);

	/* set poweroff flag */
	ext_bd71827_reg_write8(BD71827_REG_RESERVE_0, POWEROFF);

	ext_bd71827_reg_write8(BD71827_REG_BUCK1_VOLT_RUN, BD71827_REG_BUCK1_VOLT_H_DEFAULT);
	ext_bd71827_reg_write8(BD71827_REG_BUCK1_VOLT_SUSP, BD71827_REG_BUCK1_VOLT_L_DEFAULT);
	ext_bd71827_reg_write8(BD71827_REG_BUCK2_VOLT_RUN,BD71827_REG_BUCK2_VOLT_H_DEFAULT);
	ext_bd71827_reg_write8(BD71827_REG_BUCK2_VOLT_SUSP,BD71827_REG_BUCK2_VOLT_L_DEFAULT);
	/* Disable ALM0 RTC interrupts in shutdown */
	ext_bd71827_reg_write8(BD71827_REG_ALM0_MASK, 0x0);
	/* Disable Coulomb Counter before entering ship mode*/
	ext_bd71827_reg_write8(BD71827_REG_CC_CTRL,0x0);
	if (default_warm_reset) {
		ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_NORMAL_BATCUT_COLDRST_WDG_WARMRST);
		ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_RESET_BATCUT_COLDRST_WDG_WARMRST);
	} else {
		ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_NORMAL);
		ext_bd71827_reg_write8(BD71827_REG_PWRCTRL, PWRCTRL_RESET);
	}
	/* Disable Hall sensor and RTC0 but keep power button and DCIN enabled for ONEVENT */
	ext_bd71827_reg_write8(BD71827_REG_ONEVNT_MODE_2,
		ext_bd71827_reg_read8(BD71827_REG_ONEVNT_MODE_2) &
		~(HALL_ONEVENT_BIT | RTC0_ONEVENT_BIT));

	while(true)
	{
		vm_dcin_value = ext_bd71827_reg_read16(BD71827_REG_VM_DCIN_U);
		printk(KERN_INFO "%s: DCIN=0x%x\n", __func__, vm_dcin_value);
		if(vm_dcin_value < 0xFA) {
			printk(KERN_INFO "DCIN=0x%x below threshold, proceed with poweroff.\n", vm_dcin_value);
			break;
		}
		msleep(200);
	}

	/* Make sure EMMC data cache completely write-back to NAND flash */
	extern void mmc_shipmode(void);
	mmc_shipmode();
	msleep(100);

	/*First disable all pmic interrupts*/
	/*mask all pmic interrputs*/
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_01, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_02, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_03, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_04, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_05, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_06, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_07, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_08, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_09, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_10, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_11, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_12, 0x0);
	ext_bd71827_reg_write8(BD71827_REG_INT_EN_13, 0x0);

	mdelay(100);

	if(pmic_data)
		bd71827_testmode_workaround(pmic_data, SHIPMODE_WORKAROUND);

	ext_bd71827_reg_write8(BD71827_REG_LDO_PD_DIS, 0x01);
	/* PWRCTRL3 bit 0 to enter ship mode from RUN mode */

	ext_bd71827_reg_write8(BD71827_REG_PWRCTRL3, 0x01);
}
#endif


/** @brief probe pwr device
 * @param pdev platform deivce of bd71827_power
 * @retval 0 success
 * @retval negative fail
 */
static int __init bd71827_power_probe(struct platform_device *pdev)
{
	struct bd71827 *bd71827 = dev_get_drvdata(pdev->dev.parent);
	struct input_dev *input = NULL;
	struct bd71827_power *pwr;
	struct power_supply_config ac_cfg = {};
	struct power_supply_config bat_cfg = {};
	int irq, ret;
	int i, j;

	pwr = kzalloc(sizeof(*pwr), GFP_KERNEL);
	if (pwr == NULL)
		return -ENOMEM;
	pmic_pwr = pwr;
	pwr->dev = &pdev->dev;
	pwr->mfd = bd71827;

	platform_set_drvdata(pdev, pwr);

	if (battery_cycle <= 0) {
		battery_cycle = 0;
	}
	dev_info(pwr->dev, "battery_cycle = %d\n", battery_cycle);

	/* If the product often power up/down and the power down time is long, the Coulomb Counter may have a drift. */
	/* If so, it may be better accuracy to enable Coulomb Counter using following commented out code */
	/* for counting Coulomb when the product is power up(including sleep). */
	/* The condition  */
	/* (1) Product often power up and down, the power down time is long and there is no power consumed in power down time. */
	/* (2) Kernel must call this routin at power up time. */
	/* (3) Kernel must call this routin at charging time. */
	/* (4) Must use this code with "Stop Coulomb Counter" code in bd71827_power_remove() function */
	/* Start Coulomb Counter */
	/* bd71827_set_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB); */

	bd71827_set_battery_parameters(pwr);
#ifdef DEBUG_PMIC
	printk(KERN_ERR "========bd71827 register dump========\n");
	for (i = 0; i < 0x10; i++) {
		for(j = 0; j < 0x10; j++) {
			bootreg[j] = (u8)bd71827_reg_read(pwr->mfd, i*16+j);
		}
		printk(KERN_ERR "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				bootreg[0],bootreg[1],bootreg[2],bootreg[3],
				bootreg[4],bootreg[5],bootreg[6],bootreg[7],
				bootreg[8],bootreg[9],bootreg[10],bootreg[11],
				bootreg[12],bootreg[13],bootreg[14],bootreg[15]);
	}
	printk(KERN_ERR "========end bd71827 register dump========\n");
#endif

	bd71827_init_hardware(pwr);
#ifdef DEBUG_PMIC_ACCELERATE
	accelerate_snapshot_2000x(pwr->mfd);
#endif

	bat_cfg.drv_data 			= pwr;
	pwr->bat = power_supply_register(&pdev->dev, &bd71827_battery_desc, &bat_cfg);
	if (IS_ERR(pwr->bat)) {
		ret = PTR_ERR(pwr->bat);
		dev_err(&pdev->dev, "failed to register bat: %d\n", ret);
		goto fail_register_bat;
	}

	ac_cfg.supplied_to			= bd71827_ac_supplied_to;
	ac_cfg.num_supplicants		= ARRAY_SIZE(bd71827_ac_supplied_to);
	ac_cfg.drv_data 			= pwr;
	pwr->ac = power_supply_register(&pdev->dev, &bd71827_ac_desc, &ac_cfg);
	if (IS_ERR(pwr->ac)) {
		ret = PTR_ERR(pwr->ac);
		dev_err(&pdev->dev, "failed to register ac: %d\n", ret);
		goto fail_register_ac;
	}

	INIT_DELAYED_WORK(&pwr->bd_work, bd_work_callback);
	INIT_DELAYED_WORK(&pwr->bd_soc_update_work, bd_soc_update_work_callback);
	INIT_DELAYED_WORK(&pwr->bd_power_work, pmic_power_button_work);
	INIT_DELAYED_WORK(&pwr->bd_bat_work, pmic_battery_work);
	INIT_DELAYED_WORK(&pwr->bd_lobat_check_work, pmic_lobat_check_work);

	INIT_DELAYED_WORK(&pwr->bd_ums_work, ums_work_callback);
	INIT_DELAYED_WORK(&pwr->bd_safe_charging_work, bd_safe_charging_work_callback);

	for (i=0; i < BD71827_IRQ_ALARM_12; i++)
		mutex_init(&pwr->irq_status[i].lock);

	register_power_button_notifier();
	register_battery_notifier();
	register_temp_notifier();

	pmic_data = bd71827;

	input = devm_input_allocate_device(&pdev->dev);
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		goto err_input;
       }

	input->name = pdev->name;
	input->phys = "pwrkey/input0";
	input->id.bustype = BUS_HOST;
	input->evbit[0] = BIT_MASK(EV_KEY);

	input_set_capability(input, EV_KEY, KEY_POWER/*116*/);

	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		input_free_device(input);
		goto err_input;
	}
	bd71827->input = input;

	/*Add DC_IN Inserted and Remove ISR */
	irq  = platform_get_irq(pdev, 0); // get irq number
#ifdef __BD71827_REGMAP_H__
	irq += bd71827->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd71827_power_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/* Configure wakeup capable */
	device_set_wakeup_capable(pwr->dev, 1);
	device_set_wakeup_enable(pwr->dev , 1);

	/*add VBAT Low Voltage detection, John Zhang*/
	irq  = platform_get_irq(pdev, 1);
#ifdef __BD71827_REGMAP_H__
	irq += bd71827->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd71827_vbat_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/* add INT_STAT_11 */
	irq  = platform_get_irq(pdev, 2);
#ifdef __BD71827_REGMAP_H__
	irq += bd71827->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd71827_int_11_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	ret = sysfs_create_group(&pwr->bat->dev.kobj, &bd71827_sysfs_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register sysfs interface\n");
	}

	pwr->reg_index = -1;

	/* Schedule timer to check current status */
	pwr->calib_current = CALIB_NORM;
	pwr->gauge_delay = 0;
	schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(0));
	pm_power_off = bd71827_chip_poweroff;
	pm_power_hibernate = bd71827_chip_hibernate;

	heisenberg_pwrkey_enabled = 1;
	_heisenberg_pwrkey_initialized = 1;


#ifdef CONFIG_AMAZON_SIGN_OF_LIFE_BD71827
		life_cycle_metrics_proc_init();
#endif
	ret = register_reboot_notifier(&reboot_notifier);
	if (ret)
		goto failed_0;

#ifdef CONFIG_LAB126
    usb_gadget_chg_det_sync(2000);
#endif
   /* Enable Hall sensor, RTC0, power button and DCIN for ONEVENT */
    ext_bd71827_reg_write8(BD71827_REG_ONEVNT_MODE_2,
        ext_bd71827_reg_read8(BD71827_REG_ONEVNT_MODE_2) |
        (HALL_ONEVENT_BIT | RTC0_ONEVENT_BIT));

    return 0;

failed_0:
	unregister_reboot_notifier(&reboot_notifier);
err_input: // Shall be put below failed_0
//error_exit:
	power_supply_unregister(pwr->ac);
fail_register_ac:
	power_supply_unregister(pwr->bat);
fail_register_bat:
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);

	return ret;
}

/** @brief remove pwr device
 * @param pdev platform deivce of bd71827_power
 * @return 0
 */

static int __exit bd71827_power_remove(struct platform_device *pdev)
{
	struct bd71827_power *pwr = platform_get_drvdata(pdev);

	/* If the product often power up/down and the power down time is long, the Coulomb Counter may have a drift. */
	/* If so, it may be better accuracy to disable Coulomb Counter using following commented out code */
	/* for stopping counting Coulomb when the product is power down(without sleep). */
	/* The condition  */
	/* (1) Product often power up and down, the power down time is long and there is no power consumed in power down time. */
	/* (2) Kernel must call this routin at power down time. */
	/* (3) Must use this code with "Start Coulomb Counter" code in bd71827_power_probe() function */
	/* Stop Coulomb Counter */
	/* bd71827_clear_bits(pwr->mfd, BD71827_REG_CC_CTRL, CCNTENB); */

	sysfs_remove_group(&pwr->bat->dev.kobj, &bd71827_sysfs_attr_group);

	pwr->gauge_delay = -1;
	cancel_delayed_work(&pwr->bd_work);
	cancel_delayed_work(&pwr->bd_soc_update_work);
	cancel_delayed_work(&pwr->bd_lobat_check_work);
	cancel_delayed_work(&pwr->bd_power_work);
	cancel_delayed_work(&pwr->bd_bat_work);
	cancel_delayed_work(&pwrkey_skip_work);

#ifdef CONFIG_PM_AUTOSLEEP
	wakeup_source_unregister(pwr->vbus_wakeup_source);
#endif
	input_unregister_device(pwr->mfd->input); /* Android */

	power_supply_unregister(pwr->bat);
	power_supply_unregister(pwr->ac);
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);
	pmic_pwr = NULL;

	_heisenberg_pwrkey_initialized = 0;
	unregister_reboot_notifier(&reboot_notifier);

	return 0;
}


#if defined(CONFIG_LAB126) && ( defined(CONFIG_TOI) || defined(CONFIG_FALCON) )
static int bd71827_power_suspend(struct platform_device *pdev)
{
	return 0;
}

/*
* This is workaround for missing power-button-pressed-event which caused by HW issue.
* Once we know the root cause, it should be removed.
*/
static int bd71827_power_resume(struct platform_device *pdev)
{
	struct bd71827* mfd = pmic_data;
	int i = 0;
	u8 reg;
	int send_powerbutton_event = 0;

	/* Check HALL_DET (moved to RESERV_1 in uboot) and clear */
	reg = ext_bd71827_reg_read8(BD71827_REG_RESERVE_1);
	if (reg) {
		pr_info("Cover open, send event KOBJ_ONLINE\n");
		send_powerbutton_event = 1;
		ext_bd71827_reg_write8(BD71827_REG_RESERVE_1, 0x0);
	}

	// For power button pending bit
	reg = ext_bd71827_reg_read8(BD71827_REG_INT_STAT_03);
	if ( reg & POWERON_PRESS || reg & POWERON_SHORT ) {
		pr_info("PMIC-WORKAROUND: Power button pressed, send MISSING user event KOBJ_ONLINE\n");
		send_powerbutton_event = 1;
		for(i = 0; i < 5; i++) {
			ext_bd71827_reg_write8(BD71827_REG_INT_STAT_03, reg);
			reg = ext_bd71827_reg_read8(BD71827_REG_INT_STAT_03);
			if (!reg) 
				break;
		}
	}

	if (send_powerbutton_event)
		kobject_uevent(&(mfd->dev->kobj), KOBJ_ONLINE);

	// For DCIN pending bit
	reg = ext_bd71827_reg_read8(BD71827_REG_INT_STAT_02);
	if ( reg ) {
		pr_info("PMIC-WORKAROUND: DCIN pending\n");
		for(i = 0; i < 5; i++) {
			ext_bd71827_reg_write8(BD71827_REG_INT_STAT_02, reg);
			reg = ext_bd71827_reg_read8(BD71827_REG_INT_STAT_02);
			if (!reg) 
				break;
		}
	}

	// For RTC pending bit
	reg = ext_bd71827_reg_read8(BD71827_REG_INT_STAT_12);
	if ( reg ) {
		pr_info("PMIC-WORKAROUND: RTC pending\n");
		for(i = 0; i < 5; i++) {
			ext_bd71827_reg_write8(BD71827_REG_INT_STAT_12, reg);
			reg = ext_bd71827_reg_read8(BD71827_REG_INT_STAT_12);
			if (!reg) 
				break;
		}
	}
	bd71827_reset_soc_buf_counter();
	if(!schedule_delayed_work(&pmic_pwr->bd_soc_update_work, msecs_to_jiffies(10))) {
		cancel_delayed_work(&pmic_pwr->bd_soc_update_work);
		schedule_delayed_work(&pmic_pwr->bd_soc_update_work, msecs_to_jiffies(10));
	}

	return 0;
}
static SIMPLE_DEV_PM_OPS(bd71827_power_pm_ops, bd71827_power_suspend, bd71827_power_resume);
#endif

static struct platform_driver bd71827_power_driver = {
	.driver = {
		.name = "bd71827-power",
		.owner = THIS_MODULE,
#if defined(CONFIG_LAB126) && ( defined(CONFIG_TOI) || defined(CONFIG_FALCON) )
		.pm	= &bd71827_power_pm_ops,
#endif
	},
	.remove = __exit_p(bd71827_power_remove),
};

/** @brief module initialize function */
static int __init bd71827_power_init(void)
{
	return platform_driver_probe(&bd71827_power_driver, bd71827_power_probe);
}

module_init(bd71827_power_init);

/** @brief module deinitialize function */
static void __exit bd71827_power_exit(void)
{
	platform_driver_unregister(&bd71827_power_driver);
}

/*-------------------------------------------------------*/


#define PROCFS_NAME 		"bd71827_rev"
#define BD71827_REV			"BD71827 Driver: Rev009\n"

#define BD71827_BUF_SIZE	1024
static char procfs_buffer[BD71827_BUF_SIZE];
/**
 * This function is called then the /proc file is read
 *
 */
static int onetime = 0;
static ssize_t bd71827_proc_read (struct file *file, char __user *buffer, size_t count, loff_t *data)
{
	int ret = 0, error = 0;
	if(onetime==0) {
		onetime = 1;
		memset( procfs_buffer, 0, BD71827_BUF_SIZE);
		sprintf(procfs_buffer, "%s", BD71827_REV);
		ret = strlen(procfs_buffer);
		error = copy_to_user(buffer, procfs_buffer, strlen(procfs_buffer));
	} else {
		//Clear for next time
		onetime = 0;
	}
	return (error!=0)?0:ret;
}

#if 0
int bd71827_debug_mask = 0;
static ssize_t bd71827_proc_write (struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	sscanf(buffer, "0x%x", &bd71827_debug_mask);
	printk("BD71827: bd71827_debug_mask=0x%08x\n", bd71827_debug_mask);
	return count;
}
#endif

static const struct file_operations bd71827_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= bd71827_proc_read,
	//.write		= bd71827_proc_write,
};

/**
 *This function is called when the module is loaded
 *
 */
int bd71827_revision_init(void)
{
	struct proc_dir_entry *bd71827_proc_entry;

	/* create the /proc/bd71827_rev */
	bd71827_proc_entry = proc_create(PROCFS_NAME, 0644, NULL, &bd71827_proc_fops);
	if (bd71827_proc_entry == NULL) {
		printk("Error: Could not initialize /proc/%s\n", PROCFS_NAME);
		return -ENOMEM;
	}

	return 0;
}
module_init(bd71827_revision_init);
/*-------------------------------------------------------*/

module_exit(bd71827_power_exit);

module_param(use_load_bat_params, int, S_IRUGO);
MODULE_PARM_DESC(use_load_bat_params, "use_load_bat_params:Use loading battery parameters");

module_param(battery_cap_mah, int, S_IRUGO);
MODULE_PARM_DESC(battery_cap_mah, "battery_cap_mah:Battery capacity (mAh)");

module_param(dgrd_cyc_cap, int, S_IRUGO);
MODULE_PARM_DESC(dgrd_cyc_cap, "dgrd_cyc_cap:Degraded capacity per cycle (uAh)");

module_param(soc_est_max_num, int, S_IRUGO);
MODULE_PARM_DESC(soc_est_max_num, "soc_est_max_num:SOC estimation max repeat number");

module_param(dgrd_temp_cap_h, int, S_IRUGO);
MODULE_PARM_DESC(dgrd_temp_cap_h, "dgrd_temp_cap_h:Degraded capacity at high temperature (uAh)");

module_param(dgrd_temp_cap_m, int, S_IRUGO);
MODULE_PARM_DESC(dgrd_temp_cap_m, "dgrd_temp_cap_m:Degraded capacity at middle temperature (uAh)");

module_param(dgrd_temp_cap_l, int, S_IRUGO);
MODULE_PARM_DESC(dgrd_temp_cap_l, "dgrd_temp_cap_l:Degraded capacity at low temperature (uAh)");

module_param(battery_cycle, uint, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(battery_parameters, "battery_cycle:battery charge/discharge cycles");

module_param_array(ocv_table, int, NULL, S_IRUGO);
MODULE_PARM_DESC(ocv_table, "ocv_table:Open Circuit Voltage table (uV)");

module_param_array(vdr_table_h, int, NULL, S_IRUGO);
MODULE_PARM_DESC(vdr_table_h, "vdr_table_h:Voltage Drop Ratio temperatyre high area table");

module_param_array(vdr_table_m, int, NULL, S_IRUGO);
MODULE_PARM_DESC(vdr_table_m, "vdr_table_m:Voltage Drop Ratio temperatyre middle area table");

module_param_array(vdr_table_l, int, NULL, S_IRUGO);
MODULE_PARM_DESC(vdr_table_l, "vdr_table_l:Voltage Drop Ratio temperatyre low area table");

module_param_array(vdr_table_vl, int, NULL, S_IRUGO);
MODULE_PARM_DESC(vdr_table_vl, "vdr_table_vl:Voltage Drop Ratio temperatyre very low area table");

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_DESCRIPTION("BD71827 Battery Charger Power driver");
MODULE_LICENSE("GPL");

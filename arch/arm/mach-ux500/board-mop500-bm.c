/*
 * Copyright (C) ST-Ericsson SA 2011
 * Copyright (C) 2012 Sony Mobile Communications AB.
 * All rights, including trade secret rights, reserved.
 *
 * License terms:  GNU General Public License (GPL), version 2
 *
 * U8500 board specific charger and battery initialization parameters.
 *
 * Author: Johan Palsson <johan.palsson@stericsson.com> for ST-Ericsson.
 * Author: Johan Gardsmark <johan.gardsmark@stericsson.com> for ST-Ericsson.
 *
 */

#include <linux/power_supply.h>
#include <linux/mfd/abx500/ab8500-bm.h>
#include <linux/mfd/abx500/ab8500-pwmleds.h>
#include "board-mop500-bm.h"


/* Discharge curve for 10mA load for Lowe batteries */
static struct v_to_cap cap_tbl_A_Lowe[] = {
	{4179,	100},
	{4117,	 95},
	{4073,	 90},
	{4028,	 85},
	{3988,	 80},
	{3956,	 75},
	{3925,	 70},
	{3897,	 65},
	{3869,	 60},
	{3827,	 55},
	{3806,	 50},
	{3791,	 45},
	{3780,	 40},
	{3773,	 35},
	{3766,	 30},
	{3754,	 25},
	{3734,	 20},
	{3695,	 15},
	{3677,	 10},
	{3637,	  5},
	{3251,	  0},
};

/* 100mA discharge curve at 23degC */
static struct v_to_cap cap_tbl_A_Lowe_100mA[] = {
	{4174,	100},
	{4142,	99},
	{4128,	98},
	{4117,	97},
	{4107,	96},
	{4098,	95},
	{4088,	94},
	{4078,	93},
	{4069,	92},
	{4061,	91},
	{4053,	90},
	{4044,	89},
	{4036,	88},
	{4028,	87},
	{4019,	86},
	{4009,	85},
	{4001,	84},
	{3993,	83},
	{3984,	82},
	{3976,	81},
	{3969,	80},
	{3962,	79},
	{3955,	78},
	{3947,	77},
	{3941,	76},
	{3935,	75},
	{3928,	74},
	{3921,	73},
	{3916,	72},
	{3909,	71},
	{3903,	70},
	{3897,	69},
	{3891,	68},
	{3885,	67},
	{3878,	66},
	{3872,	65},
	{3867,	64},
	{3860,	63},
	{3853,	62},
	{3847,	61},
	{3839,	60},
	{3832,	59},
	{3824,	58},
	{3817,	57},
	{3811,	56},
	{3805,	55},
	{3800,	54},
	{3796,	53},
	{3791,	52},
	{3787,	51},
	{3784,	50},
	{3780,	49},
	{3776,	48},
	{3773,	47},
	{3770,	46},
	{3767,	45},
	{3764,	44},
	{3761,	43},
	{3759,	42},
	{3756,	41},
	{3754,	40},
	{3751,	39},
	{3749,	38},
	{3747,	37},
	{3745,	36},
	{3743,	35},
	{3741,	34},
	{3739,	33},
	{3737,	32},
	{3736,	31},
	{3734,	30},
	{3732,	29},
	{3730,	28},
	{3728,	27},
	{3726,	26},
	{3724,	25},
	{3721,	24},
	{3718,	23},
	{3715,	22},
	{3710,	21},
	{3706,	20},
	{3701,	19},
	{3696,	18},
	{3689,	17},
	{3681,	16},
	{3671,	15},
	{3663,	14},
	{3651,	13},
	{3646,	12},
	{3643,	11},
	{3640,	10},
	{3637,	9},
	{3634,	8},
	{3630,	7},
	{3624,	6},
	{3617,	5},
	{3603,	4},
	{3580,	3},
	{3533,	2},
	{3472,	1},
	{3391,	0},
};

/* Discharge curve for 10mA load for Laurel batteries */
static struct v_to_cap cap_tbl_A_Laurel_100mA[] = {
	{4167, 100},
	{4135, 99},
	{4119, 98},
	{4106, 97},
	{4095, 96},
	{4084, 95},
	{4075, 94},
	{4065, 93},
	{4056, 92},
	{4049, 91},
	{4040, 90},
	{4031, 89},
	{4023, 88},
	{4013, 87},
	{4004, 86},
	{3995, 85},
	{3988, 84},
	{3980, 83},
	{3972, 82},
	{3965, 81},
	{3959, 80},
	{3953, 79},
	{3946, 78},
	{3940, 77},
	{3933, 76},
	{3927, 75},
	{3920, 74},
	{3914, 73},
	{3908, 72},
	{3902, 71},
	{3895, 70},
	{3890, 69},
	{3883, 68},
	{3878, 67},
	{3873, 66},
	{3866, 65},
	{3860, 64},
	{3854, 63},
	{3847, 62},
	{3841, 61},
	{3835, 60},
	{3828, 59},
	{3822, 58},
	{3816, 57},
	{3809, 56},
	{3804, 55},
	{3799, 54},
	{3795, 53},
	{3791, 52},
	{3787, 51},
	{3783, 50},
	{3780, 49},
	{3777, 48},
	{3773, 47},
	{3771, 46},
	{3768, 45},
	{3766, 44},
	{3763, 43},
	{3761, 42},
	{3759, 41},
	{3757, 40},
	{3755, 39},
	{3753, 38},
	{3752, 37},
	{3751, 36},
	{3750, 35},
	{3749, 34},
	{3748, 33},
	{3747, 32},
	{3746, 31},
	{3744, 30},
	{3743, 29},
	{3741, 28},
	{3739, 27},
	{3737, 26},
	{3735, 25},
	{3731, 24},
	{3727, 23},
	{3724, 22},
	{3720, 21},
	{3716, 20},
	{3711, 19},
	{3707, 18},
	{3701, 17},
	{3693, 16},
	{3685, 15},
	{3676, 14},
	{3666, 13},
	{3655, 12},
	{3650, 11},
	{3646, 10},
	{3643, 9},
	{3639, 8},
	{3635, 7},
	{3630, 6},
	{3623, 5},
	{3612, 4},
	{3590, 3},
	{3549, 2},
	{3486, 1},
	{3397, 0},
};

#ifdef CONFIG_AB8500_BATTERY_THERM_ON_BATCTRL
/*
 * These are the defined batteries that uses a NTC and ID resistor placed
 * inside of the battery pack.
 * Note that the res_to_temp table must be strictly sorted by falling resistance
 * values to work.
 */
/* SEMC Type 1 battery */
static struct res_to_temp temp_tbl_A[] = {
	{-20, 67400},
	{  0, 49200},
	{  5, 44200},
	{ 10, 39400},
	{ 15, 35000},
	{ 20, 31000},
	{ 25, 27400},
	{ 30, 24300},
	{ 35, 21700},
	{ 40, 19400},
	{ 45, 17500},
	{ 50, 15900},
	{ 55, 14600},
	{ 60, 13500},
	{ 65, 12500},
	{ 70, 11800},
	{100,  9200},
};
/* SEMC Type 2 battery */
static struct res_to_temp temp_tbl_B[] = {
	{-20, 180700},
	{  0, 160000},
	{  5, 152700},
	{ 10, 144900},
	{ 15, 136800},
	{ 20, 128700},
	{ 25, 121000},
	{ 30, 113800},
	{ 35, 107300},
	{ 40, 101500},
	{ 45,  96500},
	{ 50,  92200},
	{ 55,  88600},
	{ 60,  85600},
	{ 65,  83000},
	{ 70,  80900},
	{100,  73900},
};

static struct v_to_cap cap_tbl_B[] = {
	{4161,	100},
	{4124,	 98},
	{4044,	 90},
	{4003,	 85},
	{3966,	 80},
	{3933,	 75},
	{3888,	 67},
	{3849,	 60},
	{3813,	 55},
	{3787,	 47},
	{3772,	 30},
	{3751,	 25},
	{3718,	 20},
	{3681,	 16},
	{3660,	 14},
	{3589,	 10},
	{3546,	  7},
	{3495,	  4},
	{3404,	  2},
	{3250,	  0},
};
#endif
static struct v_to_cap cap_tbl[] = {
	{4186,	100},
	{4163,	 99},
	{4114,	 95},
	{4068,	 90},
	{3990,	 80},
	{3926,	 70},
	{3898,	 65},
	{3866,	 60},
	{3833,	 55},
	{3812,	 50},
	{3787,	 40},
	{3768,	 30},
	{3747,	 25},
	{3730,	 20},
	{3705,	 15},
	{3699,	 14},
	{3684,	 12},
	{3672,	  9},
	{3657,	  7},
	{3638,	  6},
	{3556,	  4},
	{3424,	  2},
	{3317,	  1},
	{3094,	  0},
};

/*
 * Note that the res_to_temp table must be strictly sorted by falling
 * resistance values to work.
 */
static struct res_to_temp temp_tbl[] = {
	{-5, 214834},
	{ 0, 162943},
	{ 5, 124820},
	{10,  96520},
	{15,  75306},
	{20,  59254},
	{25,  47000},
	{30,  37566},
	{35,  30245},
	{40,  24520},
	{45,  20010},
	{50,  16432},
	{55,  13576},
	{60,  11280},
	{65,   9425},
};

#ifdef CONFIG_AB8500_BATTERY_THERM_ON_BATCTRL
/*
 * Note that the batres_vs_temp table must be strictly sorted by falling
 * temperature values to work.
 */
static struct batres_vs_temp temp_to_batres_tbl[] = {
	{ 40, 120},
	{ 30, 135},
	{ 20, 165},
	{ 10, 230},
	{ 00, 325},
	{-10, 445},
	{-20, 595},
};
#else
/*
 * Note that the batres_vs_temp table must be strictly sorted by falling
 * temperature values to work.
 */
static struct batres_vs_temp temp_to_batres_tbl[] = {
	{ 60, 300},
	{ 30, 300},
	{ 20, 300},
	{ 10, 300},
	{ 00, 300},
	{-10, 300},
	{-20, 300},
};
#endif
static const struct battery_type bat_type[] = {
	[BATTERY_UNKNOWN] = {
		/* First element always represent the UNKNOWN battery */
		.name = POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
		.resis_high = 0,
		.resis_low = 0,
		.charge_full_design = 612,
		.nominal_voltage = 3700,
		.termination_vol = 4050,
		.termination_curr = 200,
		.recharge_cap = 95,
		.normal_cur_lvl = 400,
		.normal_vol_lvl = 4100,
		.maint_a_cur_lvl = 400,
		.maint_a_vol_lvl = 4050,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 400,
		.maint_b_vol_lvl = 4000,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
		.curve_load = 10,
	},

#ifdef CONFIG_AB8500_BATTERY_THERM_ON_BATCTRL
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 70000,
		.resis_low = 8200,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.recharge_cap = 95,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 1050, /* 0.7C */
		.maint_a_vol_lvl = 4114, /* 95% */
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 1050, /* 0.7C */
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 400,
		.low_high_vol_lvl = 4000,
		.batt_vbat_offset = 8, /* +8mV to Battery voltage */
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl_A),
		.r_to_t_tbl = temp_tbl_A,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
		.curve_load = 100,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LiMn,
		.resis_high = 180000,
		.resis_low = 70001,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.recharge_cap = 95,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 1050, /* 0.7C */
		.maint_a_vol_lvl = 4094, /* 95% */
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 1050, /* 0.7C */
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 400,
		.low_high_vol_lvl = 4000,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl_B),
		.r_to_t_tbl = temp_tbl_B,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
		.curve_load = 10,
	},
#else
/*
 * These are the batteries that doesn't have an internal NTC resistor to measure
 * its temperature. The temperature in this case is measure with a NTC placed
 * near the battery but on the PCB.
 */
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 76000,
		.resis_low = 53000,
		.charge_full_design = 900,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.termination_curr = 100,
		.recharge_cap = 95,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4114, /* 95% */
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
		.curve_load = 10,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LION,
		.resis_high = 30000,
		.resis_low = 10000,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.termination_curr = 100,
		.recharge_cap = 95,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4100,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
		.curve_load = 10,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LION,
		.resis_high = 95000,
		.resis_low = 76001,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.termination_curr = 100,
		.recharge_cap = 95,
		.normal_cur_lvl = 700,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4100,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
		.curve_load = 10,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LION,
		.resis_high = 7650,
		.resis_low = 7350,
		.charge_full_design = 950,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.termination_curr = 100,
		.recharge_cap = 95,
		.normal_cur_lvl = 100,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 100,
		.maint_a_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 100,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 100,
		.low_high_vol_lvl = 4000,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl),
		.v_to_cap_tbl = cap_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
	},
#endif
};

static char *ab8500_charger_supplied_to[] = {
	"ab8500_chargalg",
	"ab8500_fg",
	"ab8500_btemp",
};

static char *ab8500_btemp_supplied_to[] = {
	"ab8500_chargalg",
	"ab8500_fg",
};

static char *ab8500_fg_supplied_to[] = {
	"ab8500_chargalg",
	"ab8500_usb",
};

static char *ab8500_chargalg_supplied_to[] = {
	"ab8500_fg",
};

struct ab8500_charger_platform_data ab8500_charger_plat_data = {
	.supplied_to = ab8500_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_charger_supplied_to),
	.autopower_cfg		= false,
};

struct ab8500_btemp_platform_data ab8500_btemp_plat_data = {
	.supplied_to = ab8500_btemp_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_btemp_supplied_to),
};

struct ab8500_fg_platform_data ab8500_fg_plat_data = {
	.supplied_to = ab8500_fg_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_fg_supplied_to),
	.ddata = &device_data,
};

struct ab8500_chargalg_platform_data ab8500_chargalg_plat_data = {
	.supplied_to = ab8500_chargalg_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_chargalg_supplied_to),
	.ddata = &device_data,
};


static struct ab8500_led_pwm leds_pwm_data[] = {
	[0] = {
		.pwm_id = 1,
		.blink_en = 1,
	},
	[1] = {
		.pwm_id = 2,
		.blink_en = 0,
	},
	[2] = {
		.pwm_id = 3,
		.blink_en = 0,
	},
};

struct ab8500_pwmled_platform_data ab8500_pwmled_plat_data = {
	.num_pwm = 3,
	.leds = leds_pwm_data,
};

static const struct ab8500_bm_capacity_levels cap_levels = {
	.critical	= 2,
	.low		= 10,
	.normal		= 70,
	.high		= 95,
	.full		= 100,
};

static const struct ab8500_fg_parameters fg = {
	.recovery_sleep_timer = 10,
	.recovery_total_time = 210,
	.init_timer = 1,
	.init_discard_time = 5,
	.init_total_time = 40,
	.high_curr_time = 30,
	.accu_charging = 30,
	.accu_high_curr = 5,
	.high_curr_threshold = 125,
	.high_curr_exceed_thr = 60,
	.battok_falling_th_sel0 = 2860,
	.battok_raising_th_sel1 = 2860,
	.maint_thres = 95,
	.user_cap_limit = 15,
};

static const struct ab8500_maxim_parameters maxi_params = {
	.ena_maxi = true,
	.chg_curr = 1500,
	.wait_cycles = 10,
	.charger_curr_step = 100,
};

static const struct ab8500_bm_charger_parameters chg = {
	.usb_volt_max		= 5500,
	.usb_curr_max		= 1500,
	.usb_curr_max_nc	= 500,
	.ac_volt_max		= 7500,
	.ac_curr_max		= 1500,
};

struct ab8500_bm_data ab8500_bm_data = {
	.temp_under		= 5,
	.temp_low		= 5,
	.temp_high		= 45,
	.temp_over		= 55,
	.main_safety_tmr_h	= 4,
	.temp_interval_chg	= 20,
	.temp_interval_nochg	= 120,
	.usb_safety_tmr_h	= 24,
	.bkup_bat_v		= BUP_VCH_SEL_2P6V,
	.bkup_bat_i		= BUP_ICH_SEL_150UA,
	.no_maintenance		= true,
	.capacity_scaling	= true,
#ifdef CONFIG_AB8500_BATTERY_THERM_ON_BATCTRL
	.adc_therm		= ADC_THERM_BATCTRL,
#else
	.adc_therm		= ADC_THERM_BATTEMP,
#endif
	.chg_unknown_bat	= false,
	.enable_overshoot	= false,
	.fg_res			= 100,
	.cap_levels		= &cap_levels,
	.bat_type		= bat_type,
	.n_btypes		= ARRAY_SIZE(bat_type),
	.batt_id		= 0,
	.interval_charging	= 5,
	.interval_not_charging	= 120,
	.temp_hysteresis	= 3,
	.gnd_lift_resistance	= 34,
	.maxi			= &maxi_params,
	.chg_params		= &chg,
	.fg_params		= &fg,
	.curves			= {
		{.cap_tbl = cap_tbl_A_Lowe_100mA,
		.num = ARRAY_SIZE(cap_tbl_A_Lowe_100mA)},
		{.cap_tbl = cap_tbl_A_Laurel_100mA,
		.num = ARRAY_SIZE(cap_tbl_A_Laurel_100mA)},
		{.cap_tbl = cap_tbl_B,
		.num = ARRAY_SIZE(cap_tbl_B)},
	},
};

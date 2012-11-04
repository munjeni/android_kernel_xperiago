/*
 * Copyright (C) ST-Ericsson AB 2010
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Main and Back-up battery management driver.
 *
 * Note: Backup battery management is required in case of Li-Ion battery and not
 * for capacitive battery. HREF boards have capacitive battery and hence backup
 * battery management is not used and the supported code is available in this
 * driver.
 *
 * License Terms: GNU General Public License v2
 * Author: Johan Palsson <johan.palsson@stericsson.com>
 * Author: Karl Komierowski <karl.komierowski@stericsson.com>
 * Author: Imre Sunyi <imre.sunyi@sonymobile.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/kobject.h>
#include <linux/mfd/ab8500.h>
#include <linux/mfd/abx500.h>
#include <linux/slab.h>
#include <linux/mfd/abx500/ab8500-bm.h>
#include <linux/delay.h>
#include <linux/mfd/abx500/ab8500-gpadc.h>
#include <linux/mfd/abx500.h>
#include <linux/time.h>
#include <linux/completion.h>
#include <linux/kernel.h>

#define MILLI_TO_MICRO			1000
#define FG_LSB_IN_MA			1627
#define QLSB_NANO_AMP_HOURS_X10		1129
#define CCEOC_IRQ_SKIP_CNT	1

#define SEC_TO_SAMPLE(S)		(S * 4)

#define NBR_AVG_SAMPLES			20

#define LOW_BAT_CHECK_INTERVAL		500 /* ms */
#define LOW_BAT_CHECK_SAMPLES		5


#define VALID_CAPACITY_SEC		(45 * 60) /* 45 minutes */
#define INS_CURR_TIMEOUT		600		/* ms */

/* FG constants */
#define BATT_OVV			0x01

#define BATT_OK_MIN			2360 /* mV */
#define BATT_OK_INCREMENT		50 /* mV */
#define BATT_OK_MAX_NR_INCREMENTS	0xE

#define interpolate(x, x1, y1, x2, y2) \
	((y1) + ((((y2) - (y1)) * ((x) - (x1))) / ((x2) - (x1))));

#define to_ab8500_fg_device_info(x) container_of((x), \
	struct ab8500_fg, fg_psy);

extern int sysfs_attr_on; /* flag, get_property called from sysfs */

/**
 * struct ab8500_fg_interrupts - ab8500 fg interupts
 * @name:	name of the interrupt
 * @isr		function pointer to the isr
 */
struct ab8500_fg_interrupts {
	char *name;
	irqreturn_t (*isr)(int irq, void *data);
};

enum ab8500_fg_discharge_state {
	AB8500_FG_DISCHARGE_INIT,
	AB8500_FG_DISCHARGE_INITMEASURING,
	AB8500_FG_DISCHARGE_START,
	AB8500_FG_DISCHARGE_INIT_RECOVERY,
	AB8500_FG_DISCHARGE_RECOVERY,
	AB8500_FG_DISCHARGE_READOUT_INIT,
	AB8500_FG_DISCHARGE_READOUT,
};

static char *discharge_state[] = {
	"DISCHARGE_INIT",
	"DISCHARGE_INITMEASURING",
	"DISCHARGE_START",
	"DISCHARGE_INIT_RECOVERY",
	"DISCHARGE_RECOVERY",
	"DISCHARGE_READOUT_INIT",
	"DISCHARGE_READOUT",
};

enum ab8500_fg_charge_state {
	AB8500_FG_CHARGE_INIT,
	AB8500_FG_CHARGE_READOUT,
};

static char *charge_state[] = {
	"CHARGE_INIT",
	"CHARGE_READOUT",
};

enum ab8500_fg_calibration_state {
	AB8500_FG_CALIB_INIT,
	AB8500_FG_CALIB_WAIT,
	AB8500_FG_CALIB_END,
};

struct ab8500_fg_avg_cap {
	int avg;
	int samples[NBR_AVG_SAMPLES];
	__kernel_time_t time_stamps[NBR_AVG_SAMPLES];
	int pos;
	int nbr_samples;
	int sum;
};

struct ab8500_fg_cap_scaling {
	bool enable;
	int cap_to_scale[2];
	int disable_cap_level;
	int scaled_cap;
};

struct ab8500_fg_battery_capacity {
	int max_mah_design;
	int max_mah;
	int mah;
	int permille;
	int level;
	int prev_mah;
	int prev_percent;
	int prev_level;
	int user_mah;
	int deltav;
	int unusable_permille;
	int rm_permille;
	struct ab8500_fg_cap_scaling cap_scale;
};

struct ab8500_fg_flags {
	bool fg_enabled;
	bool conv_done;
	bool charging;
	bool fully_charged;
	bool force_full;
	bool low_bat_delay;
	bool low_bat;
	bool bat_ovv;
	bool batt_unknown;
	bool calibrate;
	bool user_cap;
	bool batt_id_received;
	bool fg_re_enable;
};

struct inst_curr_result_list {
	struct list_head list;
	int *result;
};

/**
 * struct ab8500_fg - ab8500 FG device information
 * @dev:		Pointer to the structure device
 * @node:		a list of AB8500 FGs, hence prepared for reentrance
 * @irq			holds the CCEOC interrupt number
 * @cc_irq:		average current irq number
 * @vbat:		Battery voltage in mV
 * @vbat_nom:		Nominal battery voltage in mV
 * @inst_curr:		Instantenous battery current in mA
 * @avg_curr:		Average battery current in mA
 * @bat_temp		battery temperature
 * @fg_samples:		Number of samples used in the FG accumulation
 * @accu_charge:	Accumulated charge from the last conversion
 * @missed_accu_charge:	Accumulated charge that may be missed between
 *			resume/suspend
 * @recovery_cnt_ms:	Counter for recovery mode
 * @high_curr_cnt_ms:	Counter for high current mode
 * @init_cnt:		Counter for init mode
 * @low_bat_cnt		Counter for number of consecutive low battery measures
 * @nbr_cceoc_irq_cnt	Counter for number of CCEOC irqs to skip
 * @high_curr_thr_pc	Percent counter of exceeded high current threshold
 * @calculate_missed_accu:
 *			Indicate if missing accu logic should be activated
 * @recovery_needed:	Indicate if recovery is needed
 * @high_curr_mode:	Indicate if we're in high current mode
 * @init_capacity:	Indicate if initial capacity measuring should be done
 * @prohibit_uncomp_voltage_replace:
			True when capacity is not allowed to be replaced with
			uncompensated voltage
 * @calib_state		State during offset calibration
 * @discharge_state:	Current discharge state
 * @charge_state:	Current charge state
 * @ab8500_fg_complete	Completion struct used for the instant current reading
 * @flags:		Structure for information about events triggered
 * @bat_cap:		Structure for battery capacity specific parameters
 * @avg_cap:		Average capacity filter
 * @parent:		Pointer to the struct ab8500
 * @gpadc:		Pointer to the struct gpadc
 * @pdata:		Pointer to the ab8500_fg platform data
 * @bat:		Pointer to the ab8500_bm platform data
 * @fg_psy:		Structure that holds the FG specific battery properties
 * @fg_wq:		Work queue for running the FG algorithm
 * @avg_curr_wq:	Work queue for running the avg current calculation
 * @shutdown_wq:	Work queue for running shutdown determination
 * @fg_periodic_work:	Work to run the FG algorithm periodically
 * @fg_low_bat_work:	Work to check low bat condition
 * @fg_reinit_work	Work used to reset and reinitialise the FG algorithm
 * @fg_work:		Work to run the FG algorithm instantly
 * @fg_acc_cur_work:	Work to read the FG accumulator
 * @fg_check_hw_failure_work:	Work for checking HW state
 * @cc_lock:		Mutex for locking the CC
 * @fg_kobject:		Structure of type kobject
 * @resume_start:	Time when FG resume started
 * @recovery_start:	Time when FG recovery starts
 * @discharge_start:	Time when discharge started
 * @high_curr_start:	Time when high current mode started
 */
struct ab8500_fg {
	struct device *dev;
	struct list_head node;
	int irq;
	int cc_irq;
	int vbat;
	int vbat_nom;
	int inst_curr;
	int avg_curr;
	int bat_temp;
	int fg_samples;
	int accu_charge;
	int missed_accu_charge;
	int recovery_cnt_ms;
	int high_curr_cnt_ms;
	int init_cnt;
	int low_bat_cnt;
	int nbr_cceoc_irq_cnt;
	int high_curr_thr_pc;
	bool calculate_missed_accu;
	bool recovery_needed;
	bool high_curr_mode;
	bool init_capacity;
	bool prohibit_uncomp_voltage_replace;
	enum ab8500_fg_calibration_state calib_state;
	enum ab8500_fg_discharge_state discharge_state;
	enum ab8500_fg_charge_state charge_state;
	struct completion ab8500_fg_complete;
	struct completion accu_done;
	struct ab8500_fg_flags flags;
	struct ab8500_fg_battery_capacity bat_cap;
	struct ab8500_fg_avg_cap avg_cap;
	struct ab8500 *parent;
	struct ab8500_gpadc *gpadc;
	struct ab8500_fg_platform_data *pdata;
	struct ab8500_bm_data *bat;
	struct power_supply fg_psy;
	struct workqueue_struct *fg_wq;
	struct workqueue_struct *avg_curr_wq;
	struct workqueue_struct *shutdown_wq;
	struct delayed_work fg_periodic_work;
	struct delayed_work fg_low_bat_work;
	struct delayed_work fg_reinit_work;
	struct work_struct fg_work;
	struct work_struct fg_acc_cur_work;
	struct delayed_work fg_check_hw_failure_work;
	struct mutex cc_lock;
	struct mutex shutdown_lock;
	struct kobject fg_kobject;
	struct timespec resume_start;
	struct timespec recovery_start;
	struct timespec discharge_start;
	struct timespec high_curr_start;
};
static LIST_HEAD(ab8500_fg_list);

static void ab8500_fg_update_capacity(struct ab8500_fg *di, int uah);

/**
 * ab8500_fg_get() - returns a reference to the primary AB8500 fuel gauge
 * (i.e. the first fuel gauge in the instance list)
 */
struct ab8500_fg *ab8500_fg_get(void)
{
	struct ab8500_fg *fg;

	if (list_empty(&ab8500_fg_list))
		return NULL;

	fg = list_first_entry(&ab8500_fg_list, struct ab8500_fg, node);
	return fg;
}

/* Main battery properties */
static enum power_supply_property ab8500_fg_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

/*
 * This array maps the raw hex value to lowbat voltage used by the AB8500
 * Values taken from the UM0836
 */
static int ab8500_fg_lowbat_voltage_map[] = {
	2300 ,
	2325 ,
	2350 ,
	2375 ,
	2400 ,
	2425 ,
	2450 ,
	2475 ,
	2500 ,
	2525 ,
	2550 ,
	2575 ,
	2600 ,
	2625 ,
	2650 ,
	2675 ,
	2700 ,
	2725 ,
	2750 ,
	2775 ,
	2800 ,
	2825 ,
	2850 ,
	2875 ,
	2900 ,
	2925 ,
	2950 ,
	2975 ,
	3000 ,
	3025 ,
	3050 ,
	3075 ,
	3100 ,
	3125 ,
	3150 ,
	3175 ,
	3200 ,
	3225 ,
	3250 ,
	3275 ,
	3300 ,
	3325 ,
	3350 ,
	3375 ,
	3400 ,
	3425 ,
	3450 ,
	3475 ,
	3500 ,
	3525 ,
	3550 ,
	3575 ,
	3600 ,
	3625 ,
	3650 ,
	3675 ,
	3700 ,
	3725 ,
	3750 ,
	3775 ,
	3800 ,
	3825 ,
	3850 ,
	3850 ,
};

static u8 ab8500_volt_to_regval(int voltage)
{
	int i;

	if (voltage < ab8500_fg_lowbat_voltage_map[0])
		return 0;

	for (i = 0; i < ARRAY_SIZE(ab8500_fg_lowbat_voltage_map); i++) {
		if (voltage < ab8500_fg_lowbat_voltage_map[i])
			return (u8) i - 1;
	}

	/* If not captured above, return index of last element */
	return (u8) ARRAY_SIZE(ab8500_fg_lowbat_voltage_map) - 1;
}

static struct timespec ab8500_fg_get_time(void)
{
	unsigned long long ctime;
	struct timespec time;
	static int cpu = -1;

	if (cpu == -1)
		cpu = smp_processor_id();

	ctime = cpu_clock(cpu);
	time.tv_nsec = sector_div(ctime, NSEC_PER_SEC);
	time.tv_sec = (__kernel_time_t)ctime;
	return time;
}

static inline int ab8500_fg_timespec_to_ms(struct timespec time)
{
	return (int)(time.tv_sec * MSEC_PER_SEC + time.tv_nsec / NSEC_PER_MSEC);
}

/**
 * ab8500_fg_is_low_curr() - Low or high current mode
 * @di:		pointer to the ab8500_fg structure
 * @curr:	the current to base or our decision on
 *
 * Low current mode if the current consumption is below a certain threshold
 */
static int ab8500_fg_is_low_curr(struct ab8500_fg *di, int curr)
{
	/*
	 * We want to know if we're in low current mode
	 */
	if (curr > -di->bat->fg_params->high_curr_threshold)
		return true;
	else
		return false;
}

/**
 * ab8500_fg_add_cap_sample() - Add capacity to average filter
 * @di:		pointer to the ab8500_fg structure
 * @sample:	the capacity in mAh to add to the filter
 *
 * A capacity is added to the filter and a new mean capacity is calculated and
 * returned
 */
static int ab8500_fg_add_cap_sample(struct ab8500_fg *di, int sample)
{
	struct timespec ts;
	struct ab8500_fg_avg_cap *avg = &di->avg_cap;

	getnstimeofday(&ts);

	do {
		avg->sum += sample - avg->samples[avg->pos];
		avg->samples[avg->pos] = sample;
		avg->time_stamps[avg->pos] = ts.tv_sec;
		avg->pos++;

		if (avg->pos == NBR_AVG_SAMPLES)
			avg->pos = 0;

		if (avg->nbr_samples < NBR_AVG_SAMPLES)
			avg->nbr_samples++;

		/*
		 * Check the time stamp for each sample. If too old,
		 * replace with latest sample
		 */
	} while (ts.tv_sec - VALID_CAPACITY_SEC > avg->time_stamps[avg->pos]);

	avg->avg = avg->sum / avg->nbr_samples;

	return avg->avg;
}

/**
 * ab8500_fg_clear_cap_samples() - Clear average filter
 * @di:		pointer to the ab8500_fg structure
 *
 * The capacity filter is is reset to zero.
 */
static void ab8500_fg_clear_cap_samples(struct ab8500_fg *di)
{
	int i;
	struct ab8500_fg_avg_cap *avg = &di->avg_cap;

	avg->pos = 0;
	avg->nbr_samples = 0;
	avg->sum = 0;
	avg->avg = 0;

	for (i = 0; i < NBR_AVG_SAMPLES; i++) {
		avg->samples[i] = 0;
		avg->time_stamps[i] = 0;
	}
}

/**
 * ab8500_fg_fill_cap_sample() - Fill average filter
 * @di:		pointer to the ab8500_fg structure
 * @sample:	the capacity in mAh to fill the filter with
 *
 * The capacity filter is filled with a capacity in mAh
 */
static void ab8500_fg_fill_cap_sample(struct ab8500_fg *di, int sample)
{
	int i;
	struct timespec ts;
	struct ab8500_fg_avg_cap *avg = &di->avg_cap;

	getnstimeofday(&ts);

	for (i = 0; i < NBR_AVG_SAMPLES; i++) {
		avg->samples[i] = sample;
		avg->time_stamps[i] = ts.tv_sec;
	}

	avg->pos = 0;
	avg->nbr_samples = NBR_AVG_SAMPLES;
	avg->sum = sample * NBR_AVG_SAMPLES;
	avg->avg = sample;
}

/**
 * ab8500_fg_coulomb_counter() - enable coulomb counter
 * @di:		pointer to the ab8500_fg structure
 * @samples: number of samples to program
 * @enable:	enable/disable
 *
 * Enable/Disable coulomb counter.
 * On failure returns negative value.
 */
static int
ab8500_fg_coulomb_counter(struct ab8500_fg *di, int samples, bool enable)
{
	int ret = 0;
	mutex_lock(&di->cc_lock);
	if (enable) {
		if (di->fg_samples == samples)
			/*
			 * don't need to reprogram it again
			 */
			goto unlock_and_leave;

		/* To be able to reprogram the number of samples, we have to
		 * first stop the CC and then enable it again */
		ret = abx500_set_register_interruptible(di->dev, AB8500_RTC,
			AB8500_RTC_CC_CONF_REG, 0x00);
		if (ret)
			goto cc_err;

		/* Program the samples */
		ret = abx500_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_NCOV_ACCU,
			samples);
		if (ret)
			goto cc_err;

		/* Start the CC */
		ret = abx500_set_register_interruptible(di->dev, AB8500_RTC,
			AB8500_RTC_CC_CONF_REG,
			(CC_DEEP_SLEEP_ENA | CC_PWR_UP_ENA));
		if (ret)
			goto cc_err;

		if (!di->flags.fg_enabled) {
			di->flags.fg_enabled = true;
			enable_irq(di->cc_irq);
		}

		di->fg_samples = samples;

		/*
		 * how many CCEOC to skip after enabling CC
		 */
		di->nbr_cceoc_irq_cnt = CCEOC_IRQ_SKIP_CNT;
	} else {
		if (di->flags.fg_enabled) {
			di->flags.fg_enabled = false;
			disable_irq(di->cc_irq);
		}

		/* Clear any pending read requests */
		ret = abx500_mask_and_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
			(RESET_ACCU | READ_REQ), CC_MUXOFFSET);
		if (ret)
			goto cc_err;

		/* reset ResetNconvAccu accumulator */
		ret = abx500_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_NCOV_ACCU_CTRL,
			RESET_ACCU);
		if (ret)
			goto cc_err;

		/* Stop the CC */
		ret = abx500_set_register_interruptible(di->dev, AB8500_RTC,
			AB8500_RTC_CC_CONF_REG, 0);
		if (ret)
			goto cc_err;

		di->flags.conv_done = false;
		di->fg_samples = 0;
		di->accu_charge = 0;
		di->avg_curr = 0;
	}
	dev_dbg(di->dev, " CC enabled: %d Samples: %d\n",
		enable, di->fg_samples);

unlock_and_leave:
	mutex_unlock(&di->cc_lock);
	return ret;
cc_err:
	dev_err(di->dev, "%s Enabling coulomb counter failed\n", __func__);
	mutex_unlock(&di->cc_lock);
	return ret;
}

/**
 * ab8500_fg_inst_curr_start() - start battery instantaneous current
 * @di:         pointer to the ab8500_fg structure
 *
 * Returns 0 or error code
 * Note: This is part "one" and has to be called before
 * ab8500_fg_inst_curr_finalize()
 */
static int
ab8500_fg_inst_curr_start(struct ab8500_fg *di)
{
	u8 reg_val;
	int ret;

	mutex_lock(&di->cc_lock);

	ret = abx500_get_register_interruptible(di->dev, AB8500_RTC,
		AB8500_RTC_CC_CONF_REG, &reg_val);
	if (ret < 0)
		goto fail;

	/*
	 * The following condition can be triggered when there are some
	 * I2C problems.
	 * This prevents us from measuring current with disabled FG.
	 */
	if (!(reg_val & CC_PWR_UP_ENA)) {
		if (!di->flags.fg_enabled) {
			dev_err(di->dev,
				"%s Got here from illegal condition\n",
				__func__);
			ret = -EPERM;
		} else if (!di->flags.fg_re_enable) {
			dev_warn(di->dev,
				 "%s FG should be enabled but is not\n",
				 __func__);
			di->flags.fg_re_enable = true;
			queue_work(di->fg_wq, &di->fg_work);
			ret = -EAGAIN;
		} else {
			dev_info(di->dev, "%s FG is about to be re-enabled\n",
				 __func__);
			ret = -EBUSY;
		}

		goto fail;
	}

	/* Return and WFI */
	INIT_COMPLETION(di->ab8500_fg_complete);
	enable_irq(di->irq);

	/* Note: cc_lock is still locked */
	return 0;
fail:
	mutex_unlock(&di->cc_lock);
	return ret;
}

/**
 * ab8500_fg_inst_curr_done() - check if fg conversion is done
 * @di:         pointer to the ab8500_fg structure
 *
 * Returns 1 if conversion done, 0 if still waiting
 */
int ab8500_fg_inst_curr_done(struct ab8500_fg *di)
{
	return completion_done(&di->ab8500_fg_complete);
}

int ab8500_fg_get_inst_curr(struct ab8500_fg *di)
{
	return di->inst_curr;
}

/**
 * ab8500_fg_inst_curr_finalize() - battery instantaneous current
 * @di:         pointer to the ab8500_fg structure
 * @res:	battery instantenous current(on success)
 *
 * Returns 0 or an error code
 * Note: This is part "two" and has to be called at earliest 250 ms
 * after ab8500_fg_inst_curr_start()
 */
static int
ab8500_fg_inst_curr_finalize(struct ab8500_fg *di, int *res)
{
	unsigned long timeout;
	u8 low, high;
	int val;
	int ret;

	/*
	 * Under heavy load, i see that we may wait for seconds
	 * till CCEOC comes. The worst case is ~4 sec, whereas
	 * the normal conversion time is 250 ms for one sample.
	 */
	timeout = wait_for_completion_timeout(&di->ab8500_fg_complete,
			msecs_to_jiffies(INS_CURR_TIMEOUT));

	/* disable CCEOC irq */
	disable_irq(di->irq);

	if (timeout == 0) {
		dev_dbg(di->dev, "%s:%d: waiting for 'CCEOC' irq timeout, "
				"use previous value\n", __func__, __LINE__);
		val = di->inst_curr;
		goto leave;
	}

	ret = abx500_mask_and_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
			READ_REQ, READ_REQ);

	/*
	 * in order to update 0x0C07 and 0x0C08 registers,
	 * we need to wait 100us.
	 */
	udelay(100);

	/* Read CC Sample conversion value Low and high */
	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_SMPL_CNVL_REG,  &low);
	if (ret < 0)
		goto fail;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_SMPL_CNVH_REG,  &high);
	if (ret < 0)
		goto fail;

	/*
	 * negative value for Discharging
	 * convert 2's compliment into decimal
	 */
	if (high & 0x10)
		val = (low | (high << 8) | 0xFFFFE000);
	else
		val = (low | (high << 8));

	/*
	 * Convert to unit value in mA
	 * Full scale input voltage is
	 * 66.660mV => LSB = 66.660mV/(4096*res) = 1.627mA
	 * Given a 250ms conversion cycle time the LSB corresponds
	 * to 112.9 nAh. Convert to current by dividing by the conversion
	 * time in hours (250ms = 1 / (3600 * 4)h)
	 * 112.9nAh assumes 10mOhm, but fg_res is in 0.1mOhm
	 */
	val = (val * QLSB_NANO_AMP_HOURS_X10 * 36 * 4) /
		(1000 * di->bat->fg_res);

leave:
	mutex_unlock(&di->cc_lock);
	(*res) = val;

	return 0;
fail:
	mutex_unlock(&di->cc_lock);
	return ret;
}

/**
 * ab8500_fg_inst_curr_blocking() - battery instantaneous current
 * @di:         pointer to the ab8500_fg structure
 * @res:	battery instantenous current(on success)
 *
 * Returns 0 else error code
 */
static int
ab8500_fg_inst_curr_blocking(struct ab8500_fg *di)
{
	int res = 0;
	int ret;

	ret = ab8500_fg_inst_curr_start(di);
	if (ret) {
		dev_err(di->dev, "Failed to initialize fg_inst\n");
		return 0;
	}

	ret = ab8500_fg_inst_curr_finalize(di, &res);
	if (ret) {
		dev_err(di->dev, "Failed to finalize fg_inst\n");
		return 0;
	}

	dev_dbg(di->dev, "%s instant current: %d", __func__, res);
	return res;
}

/**
 * ab8500_fg_acc_cur_work() - average battery current
 * @work:	pointer to the work_struct structure
 *
 * Updated the average battery current obtained from the
 * coulomb counter.
 */
static void ab8500_fg_acc_cur_work(struct work_struct *work)
{
	int val;
	int ret;
	u8 low, med, high;

	struct ab8500_fg *di = container_of(work,
		struct ab8500_fg, fg_acc_cur_work);

	mutex_lock(&di->cc_lock);
	if (!di->flags.fg_enabled) {
		dev_info(di->dev,
			 "%s: Registers are not valid since FG is disabled\n",
			 __func__);
		mutex_unlock(&di->cc_lock);
		return;
	}

	ret = abx500_set_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU_CTRL, RD_NCONV_ACCU_REQ | RESET_ACCU);
	if (ret < 0)
		goto exit;

	udelay(100);

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU_LOW,  &low);
	if (ret < 0)
		goto exit;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU_MED,  &med);
	if (ret < 0)
		goto exit;

	ret = abx500_get_register_interruptible(di->dev, AB8500_GAS_GAUGE,
		AB8500_GASG_CC_NCOV_ACCU_HIGH, &high);
	if (ret < 0)
		goto exit;

	/* Check for sign bit in case of negative value, 2's compliment */
	if (high & 0x10)
		val = (low | (med << 8) | (high << 16) | 0xFFE00000);
	else
		val = (low | (med << 8) | (high << 16));

	/*
	 * Convert to uAh
	 * Given a 250ms conversion cycle time the LSB corresponds
	 * to 112.9 nAh.
	 * 112.9nAh assumes 10mOhm, but fg_res is in 0.1mOhm
	 */
	di->accu_charge = (val * QLSB_NANO_AMP_HOURS_X10) /
		(100 * di->bat->fg_res);

	/*
	 * calculate "missing capacity", use 2 sample (500ms)
	 */
	if (di->calculate_missed_accu) {
		struct timespec diff;

		di->missed_accu_charge += di->accu_charge;
		diff = timespec_sub(ab8500_fg_get_time(), di->resume_start);

		if (diff.tv_sec > di->bat->fg_params->accu_high_curr) {
			int samples = SEC_TO_SAMPLE(
				di->bat->fg_params->accu_high_curr);

			ab8500_fg_update_capacity(di, di->missed_accu_charge);

			di->missed_accu_charge = 0;
			di->calculate_missed_accu = false;

			/* just reprogram number of samples */
			ret = abx500_set_register_interruptible(di->dev,
				AB8500_GAS_GAUGE, AB8500_GASG_CC_NCOV_ACCU,
				samples);
			if (ret) {
				dev_err(di->dev,
					"%s failed to set FG samples\n",
					__func__);
				goto exit;
			}

			di->fg_samples = samples;
			dev_info(di->dev, "Stopping missed accu logic\n");
		}
	} else {
		/*
		 * Convert to unit value in mA
		 * Full scale input voltage is
		 * 66.660mV => LSB = 66.660mV/(4096*res) = 1.627mA
		 * Given a 250ms conversion cycle time the LSB corresponds
		 * to 112.9 nAh. Convert to current by dividing
		 * by the conversion time in hours (= samples / (3600 * 4)h)
		 * 112.9nAh assumes 10mOhm, but fg_res is in 0.1mOhm
		 */
		di->avg_curr = (di->accu_charge * 36) /
			((di->fg_samples / 4) * 10);

		di->flags.conv_done = true;
		queue_work(di->fg_wq, &di->fg_work);
	}

exit:
	complete(&di->accu_done);
	mutex_unlock(&di->cc_lock);

	if (ret < 0) {
		dev_err(di->dev,
			"Failed to read or write gas gauge registers\n");
		queue_work(di->fg_wq, &di->fg_work);
	}

	return;
}

/**
 * ab8500_fg_bat_voltage() - get battery voltage
 * @di:		pointer to the ab8500_fg structure
 *
 * Returns battery voltage(on success) else error code
 */
static int ab8500_fg_bat_voltage(struct ab8500_fg *di)
{
	int vbat;
	static int prev;

	vbat = ab8500_gpadc_convert(di->gpadc, MAIN_BAT_V);
	if (vbat < 0) {
		dev_err(di->dev,
			"%s gpadc conversion failed, using previous value\n",
			__func__);
		return prev;
	}
	vbat += di->bat->bat_type[di->bat->batt_id].batt_vbat_offset;
	prev = vbat;
	return vbat;
}

/**
 * ab8500_fg_get_synced_vbat_curr() - get Vbat and Current
 * @di: pointer to the ab8500_fg structure
 * @v:  holds measured voltage
 * @c:  holds measured current
 *
 * Return 0 on success else error code
 */
static int
ab8500_fg_get_synced_vbat_curr(struct ab8500_fg *di, int *v, int *c)
{
	int vbat = 0;
	int i = 0;
	int ret = ab8500_fg_inst_curr_start(di);

	if (ret < 0) {
		dev_err(di->dev, "Failed to get synced vbat and curr\n");
		return ret;
	}

	do {
		/*
		 * make battery measurement as much as possible
		 * during 250ms. It's important, because while
		 * resuming the battery voltage goes up and down
		 * very frequently.
		 */
		vbat += ab8500_fg_bat_voltage(di);
		usleep_range(10000, 10000);
		i++;
	} while (!ab8500_fg_inst_curr_done(di));

	ab8500_fg_inst_curr_finalize(di, c);
	*v = vbat / i;

	dev_dbg(di->dev, "%s Vbat: %dmV (samples: %d), Current: %dmA\n",
			__func__, *v, i, *c);
	return 0;
}

/**
 * ab8500_fg_volt_to_capacity() - Voltage based capacity
 * @di:		pointer to the ab8500_fg structure
 * @voltage:	The voltage to convert to a capacity
 *
 * Returns battery capacity in per mille based on voltage
 */
static int ab8500_fg_volt_to_capacity(struct ab8500_fg *di, int voltage)
{
	int i, tbl_size;
	struct v_to_cap *tbl;
	int cap = 0;

	if (di->pdata->ddata && di->pdata->ddata->b_chem < BAT_CURVE_MAX_NBR) {
		tbl = di->bat->curves[di->pdata->ddata->b_chem].cap_tbl;
		tbl_size = di->bat->curves[di->pdata->ddata->b_chem].num;
	} else {
		/* If battery type is undefined, take Lowe battery by default */
		tbl = di->bat->curves[TYPE1_BAT_CURVE_LOWE].cap_tbl;
		tbl_size = di->bat->curves[TYPE1_BAT_CURVE_LOWE].num;
	}

	for (i = 0; i < tbl_size; ++i) {
		if (voltage > tbl[i].voltage)
			break;
	}

	if ((i > 0) && (i < tbl_size)) {
		cap = interpolate(voltage,
			tbl[i].voltage,
			tbl[i].capacity * 10,
			tbl[i-1].voltage,
			tbl[i-1].capacity * 10);
	} else if (i == 0) {
		cap = 1000;
	} else {
		cap = 0;
	}

	dev_dbg(di->dev, "%s Vbat: %d, Cap: %d per mille\n",
		__func__, voltage, cap);

	return cap;
}

/**
 * ab8500_fg_capacity_to_volt() - Voltage based capacity
 * @di:		pointer to the ab8500_fg structure
 * @cap:	The capacity to convert to a voltage
 *
 * Returns battery voltage in mV based on capacity in percent
 */
static int ab8500_fg_capacity_to_volt(struct ab8500_fg *di, int cap)
{
	unsigned int i;
	int tbl_size;
	struct v_to_cap *tbl;
	int volt = 0;

	if (di->pdata->ddata && di->pdata->ddata->b_chem < BAT_CURVE_MAX_NBR) {
		tbl = di->bat->curves[di->pdata->ddata->b_chem].cap_tbl;
		tbl_size = di->bat->curves[di->pdata->ddata->b_chem].num;
	} else {
		/* If battery type is undefined, take Lowe battery by default */
		tbl = di->bat->curves[TYPE1_BAT_CURVE_LOWE].cap_tbl;
		tbl_size = di->bat->curves[TYPE1_BAT_CURVE_LOWE].num;
	}

	for (i = 0; i < tbl_size; i++)
		if (cap > tbl[i].capacity)
			break;

	if (i > 0 && i < tbl_size) {
		volt = interpolate(cap,
				   tbl[i].capacity,
				   tbl[i].voltage,
				   tbl[i-1].capacity,
				   tbl[i-1].voltage);
	} else if (!i) {
		volt = tbl[0].voltage;
	} else {
		volt = tbl[tbl_size - 1].voltage;
	}

	dev_dbg(di->dev, "%s Cap: %d, Vbat: %d\n", __func__, cap, volt);

	return volt;
}

/**
 * ab8500_fg_uncomp_volt_to_capacity() - Uncompensated voltage based capacity
 * @di:		pointer to the ab8500_fg structure
 *
 * Returns battery capacity based on battery voltage that is not compensated
 * for the voltage drop due to the load
 */
static int ab8500_fg_uncomp_volt_to_capacity(struct ab8500_fg *di)
{
	di->vbat = ab8500_fg_bat_voltage(di);
	return ab8500_fg_volt_to_capacity(di, di->vbat);
}

/**
 * ab8500_fg_battery_resistance() - Returns the battery inner resistance
 * @di:		pointer to the ab8500_fg structure
 *
 * Returns battery inner resistance added with the fuel gauge resistor value
 * to get the total resistance in the whole link from gnd to bat+ node.
 */
static int ab8500_fg_battery_resistance(struct ab8500_fg *di)
{
	int i, tbl_size;
	struct batres_vs_temp *tbl;
	int resist = 0;

	tbl = di->bat->bat_type[di->bat->batt_id].batres_tbl;
	tbl_size = di->bat->bat_type[di->bat->batt_id].n_batres_tbl_elements;

	for (i = 0; i < tbl_size; ++i) {
		if (di->bat_temp / 10 > tbl[i].temp)
			break;
	}

	if ((i > 0) && (i < tbl_size)) {
		resist = interpolate(di->bat_temp / 10,
			tbl[i].temp,
			tbl[i].resist,
			tbl[i-1].temp,
			tbl[i-1].resist);
	} else if (i == 0) {
		resist = tbl[0].resist;
	} else {
		resist = tbl[tbl_size - 1].resist;
	}

	dev_dbg(di->dev, "%s Temp: %d battery internal resistance: %d"
		" fg resistance %d, total: %d (mOhm)\n",
		__func__, di->bat_temp, resist, di->bat->fg_res / 10,
		(di->bat->fg_res / 10) + resist);

	/* fg_res variable is in 0.1mOhm */
	resist += di->bat->fg_res / 10;
	/*
	 * Batt_resistance vs Batt_temp table is not up to date.
	 * It was found that battery internal resistance seems to be higher
	 * than one specified in the table we have got.
	 * Fuel gauge accuracy becomes higher when using +150mOhm as an offset.
	 */
	resist += 150;

	return resist;
}

/**
 * ab8500_fg_load_comp_volt_to_capacity() - Load compensated voltage based capacity
 * @di:		pointer to the ab8500_fg structure
 *
 * Returns battery capacity based on battery voltage that
 * is load compensated for the voltage drop.
 */
static int ab8500_fg_load_comp_volt_to_capacity(struct ab8500_fg *di)
{
	int vbat_comp = 0;
	int res = 0;

	res = ab8500_fg_battery_resistance(di);

	/*
	 * Use Ohms law to get the load compensated voltage.
	 * Add offset to instant current because we use 'curve_load'
	 */
	vbat_comp = di->vbat - ((di->inst_curr +
		 di->bat->bat_type[di->bat->batt_id].curve_load) * res) / 1000;

	dev_dbg(di->dev, "%s Measured Vbat: %dmV, "
			"Compensated Vbat %dmV, "
			"R: %dmOhm, Current: %dmA\n",
			__func__, di->vbat, vbat_comp, res,
			di->inst_curr);

	return ab8500_fg_volt_to_capacity(di, vbat_comp);
}

/**
 * ab8500_fg_convert_mah_to_permille() - Capacity in mAh to permille
 * @di:		pointer to the ab8500_fg structure
 * @cap_mah:	capacity in mAh
 *
 * Converts capacity in mAh to capacity in permille
 */
static int ab8500_fg_convert_mah_to_permille(struct ab8500_fg *di, int cap_mah)
{
	int unusable_mah = DIV_ROUND_CLOSEST(
		di->bat_cap.unusable_permille * di->bat_cap.max_mah_design,
		1000);

	if (unusable_mah >= cap_mah)
		di->bat_cap.rm_permille = 0;
	else
		di->bat_cap.rm_permille = ((cap_mah - unusable_mah) * 1000) /
			(di->bat_cap.max_mah_design - unusable_mah);

	return (cap_mah * 1000) / di->bat_cap.max_mah_design;
}

/**
 * ab8500_fg_convert_permille_to_mah() - Capacity in permille to mAh
 * @di:		pointer to the ab8500_fg structure
 * @cap_pm:	capacity in permille
 *
 * Converts capacity in permille to capacity in mAh
 */
static int ab8500_fg_convert_permille_to_mah(struct ab8500_fg *di, int cap_pm)
{
	return cap_pm * di->bat_cap.max_mah_design / 1000;
}

/**
 * ab8500_fg_convert_mah_to_uwh() - Capacity in mAh to uWh
 * @di:		pointer to the ab8500_fg structure
 * @cap_mah:	capacity in mAh
 *
 * Converts capacity in mAh to capacity in uWh
 */
static int ab8500_fg_convert_mah_to_uwh(struct ab8500_fg *di, int cap_mah)
{
	u64 div_res;
	u32 div_rem;

	div_res = ((u64) cap_mah) * ((u64) di->vbat_nom);
	div_rem = do_div(div_res, 1000);

	/* Make sure to round upwards if necessary */
	if (div_rem >= 1000 / 2)
		div_res++;

	return (int) div_res;
}

/**
 * ab8500_fg_calc_cap_charging() - Calculate remaining capacity while charging
 * @di:		pointer to the ab8500_fg structure
 *
 * Return the capacity in mAh based on previous calculated capcity and the FG
 * accumulator register value. The filter is filled with this capacity
 */
static int ab8500_fg_calc_cap_charging(struct ab8500_fg *di)
{
	dev_dbg(di->dev, "%s cap_mah %d accu_charge %d\n",
		__func__,
		di->bat_cap.mah,
		di->accu_charge);

	/* Capacity should not be less than 0 */
	if (di->bat_cap.mah + di->accu_charge > 0)
		di->bat_cap.mah += di->accu_charge;
	else
		di->bat_cap.mah = 0;

	if (di->bat_cap.mah >= di->bat_cap.max_mah_design) {
		di->bat_cap.mah = di->bat_cap.max_mah_design;
		di->bat_cap.max_mah = di->bat_cap.max_mah_design;
	}

	ab8500_fg_fill_cap_sample(di, di->bat_cap.mah);
	di->bat_cap.permille =
		ab8500_fg_convert_mah_to_permille(di, di->bat_cap.mah);

	/* We need to update battery voltage and inst current when charging */
	di->vbat = ab8500_fg_bat_voltage(di);
	di->inst_curr = ab8500_fg_inst_curr_blocking(di);

	return di->bat_cap.mah;
}

/**
 * ab8500_fg_calc_cap_discharge_voltage() - Capacity in discharge with voltage
 * @di:		pointer to the ab8500_fg structure
 * @comp:	if voltage should be load compensated before capacity calc
 *
 * Return the capacity in mAh based on the battery voltage.
 * The voltage can either be load compensated or not. This value is
 * added to the filter and a new mean value is calculated and returned.
 */
static int ab8500_fg_calc_cap_discharge_voltage(struct ab8500_fg *di, bool comp)
{
	int permille, mah;

	if (comp)
		permille = ab8500_fg_load_comp_volt_to_capacity(di);
	else
		permille = ab8500_fg_uncomp_volt_to_capacity(di);

	mah = ab8500_fg_convert_permille_to_mah(di, permille);
	di->bat_cap.mah = ab8500_fg_add_cap_sample(di, mah);
	di->bat_cap.permille =
		ab8500_fg_convert_mah_to_permille(di, di->bat_cap.mah);

	return di->bat_cap.mah;
}

/**
 * ab8500_fg_calc_cap_discharge_fg() - Capacity in discharge with FG
 * @di:		pointer to the ab8500_fg structure
 *
 * Return the capacity in mAh based on previous calculated capcity and the FG
 * accumulator register value. This value is added to the filter and a
 * new mean value is calculated and returned.
 */
static int ab8500_fg_calc_cap_discharge_fg(struct ab8500_fg *di)
{
	int permille_volt, permille;
	int diff;

	dev_dbg(di->dev, "%s cap_mah %d accu_charge %d\n",
		__func__,
		di->bat_cap.mah,
		di->accu_charge);

	/* Capacity should not be less than 0 */
	if (di->bat_cap.mah + di->accu_charge > 0)
		di->bat_cap.mah += di->accu_charge;
	else
		di->bat_cap.mah = 0;

	if (di->bat_cap.mah >= di->bat_cap.max_mah_design) {
		di->bat_cap.mah = di->bat_cap.max_mah_design;
		di->bat_cap.max_mah = di->bat_cap.max_mah_design;
	}

	/*
	 * Check against voltage based capacity. It can not be lower
	 * than what the uncompensated voltage says
	 */
	permille = ab8500_fg_convert_mah_to_permille(di, di->bat_cap.mah);
	if (di->prohibit_uncomp_voltage_replace) {
		struct timespec diff =
			timespec_sub(ab8500_fg_get_time(), di->discharge_start);
		permille_volt = permille;
		if (diff.tv_sec > di->bat->fg_params->recovery_total_time)
			di->prohibit_uncomp_voltage_replace = false;
		else
			dev_dbg(di->dev,
				"%s: Uncompensated voltage not allowed to "
				"check\n", __func__);
	}

	if (!di->prohibit_uncomp_voltage_replace) {
		int vbat =
			max(0, ab8500_fg_bat_voltage(di) - di->bat_cap.deltav);
		permille_volt = ab8500_fg_volt_to_capacity(di, vbat);
	}

	/* get differences in % */
	diff = DIV_ROUND_CLOSEST(permille_volt - permille, 10);

	/* not allow to go up more than 1 % */
	if (permille < permille_volt && diff > 0) {
		di->bat_cap.permille += 10;
		di->bat_cap.mah = ab8500_fg_convert_permille_to_mah(di,
			di->bat_cap.permille);

		dev_dbg(di->dev, "%s voltage based: perm %d perm_volt %d\n",
			__func__,
			permille,
			permille_volt);

		ab8500_fg_fill_cap_sample(di, di->bat_cap.mah);
	} else {
		ab8500_fg_fill_cap_sample(di, di->bat_cap.mah);
		di->bat_cap.permille =
			ab8500_fg_convert_mah_to_permille(di, di->bat_cap.mah);
	}

	return di->bat_cap.mah;
}

static void ab8500_fg_update_unusable_permille(struct ab8500_fg *di, int deltav)
{
	di->bat_cap.deltav = deltav;
	di->bat_cap.unusable_permille = ab8500_fg_volt_to_capacity(di,
			      di->pdata->ddata->lowbat_threshold + deltav);
	dev_info(di->dev, "Updated unusable permille to %d. dV = %d mV\n",
		 di->bat_cap.unusable_permille, deltav);
}

static void ab8500_fg_calculate_unusable_permille(struct ab8500_fg *di)
{
	int cap = DIV_ROUND_CLOSEST(
		ab8500_fg_convert_mah_to_permille(di, di->bat_cap.mah), 10);
	int ocv = ab8500_fg_capacity_to_volt(di, cap);
	/* Take the maximum vbat to lower chance of measure during voltage
	 * spike i.e. caused by GSM.
	 */
	int deltav = ocv - max(ab8500_fg_bat_voltage(di), di->vbat);

	if (deltav < 0 || deltav <= di->bat_cap.deltav)
		return;

	ab8500_fg_update_unusable_permille(di, deltav);
}

/**
 * ab8500_fg_capacity_level() - Get the battery capacity level
 * @di:		pointer to the ab8500_fg structure
 *
 * Get the battery capacity level based on the capacity in percent
 */
static int ab8500_fg_capacity_level(struct ab8500_fg *di)
{
	int ret, percent;

	percent = DIV_ROUND_CLOSEST(di->bat_cap.rm_permille, 10);

	if (percent <= di->bat->cap_levels->critical ||
		di->flags.low_bat)
		ret = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else if (percent <= di->bat->cap_levels->low)
		ret = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (percent <= di->bat->cap_levels->normal)
		ret = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (percent <= di->bat->cap_levels->high)
		ret = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else
		ret = POWER_SUPPLY_CAPACITY_LEVEL_FULL;

	return ret;
}

/**
 * ab8500_fg_calculate_scaled_capacity() - Capacity scaling
 * @di:		pointer to the ab8500_fg structure
 *
 * Calculates the capacity to be shown to upper layers. Scales the capacity
 * to have 100% as a reference from the actual capacity upon removal of charger
 * when charging is in maintenance mode.
 */
static int ab8500_fg_calculate_scaled_capacity(struct ab8500_fg *di)
{
	struct ab8500_fg_cap_scaling *cs = &di->bat_cap.cap_scale;
	int capacity = di->bat_cap.prev_percent;

	if (!cs->enable)
		return capacity;

	/*
	 * As long as we are in fully charge mode scale the capacity
	 * to show 100%.
	 */
	if (di->flags.fully_charged) {
		cs->cap_to_scale[0] = 100;
		cs->cap_to_scale[1] =
			max(capacity, di->bat->fg_params->maint_thres);
		dev_dbg(di->dev, "Scale cap with %d/%d\n",
			 cs->cap_to_scale[0], cs->cap_to_scale[1]);
	}

	/* Calculates the scaled capacity. */
	if ((cs->cap_to_scale[0] != cs->cap_to_scale[1])
					&& (cs->cap_to_scale[1] > 0))
		capacity = min(100,
				 DIV_ROUND_CLOSEST(di->bat_cap.prev_percent *
						 cs->cap_to_scale[0],
						 cs->cap_to_scale[1]));

	if (di->flags.charging) {
		if (capacity < cs->disable_cap_level) {
			cs->disable_cap_level = capacity;
			dev_dbg(di->dev, "Cap to stop scale lowered %d%%\n",
				cs->disable_cap_level);
		} else if (!di->flags.fully_charged) {
			if (di->bat_cap.prev_percent >=
			    cs->disable_cap_level) {
				dev_dbg(di->dev, "Disabling scaled capacity\n");
				cs->enable = false;
				capacity = di->bat_cap.prev_percent;
			} else {
				dev_dbg(di->dev,
					"Waiting in cap to level %d%%\n",
					cs->disable_cap_level);
				capacity = cs->disable_cap_level;
			}
		}
	}

	return capacity;
}

/**
 * ab8500_fg_update_cap_scalers() - Capacity scaling
 * @di:		pointer to the ab8500_fg structure
 *
 * To be called when state change from charge<->discharge to update
 * the capacity scalers.
 */
static void ab8500_fg_update_cap_scalers(struct ab8500_fg *di)
{
	struct ab8500_fg_cap_scaling *cs = &di->bat_cap.cap_scale;

	if (!cs->enable)
		return;
	if (di->flags.charging) {
		di->bat_cap.cap_scale.disable_cap_level =
			di->bat_cap.cap_scale.scaled_cap;
		dev_dbg(di->dev, "Cap to stop scale at charge %d%%\n",
				di->bat_cap.cap_scale.disable_cap_level);
	} else {
		if (cs->scaled_cap != 100) {
			cs->cap_to_scale[0] = cs->scaled_cap;
			cs->cap_to_scale[1] = di->bat_cap.prev_percent;
		} else {
			cs->cap_to_scale[0] = 100;
			cs->cap_to_scale[1] =
				max(di->bat_cap.prev_percent,
				    di->bat->fg_params->maint_thres);
		}

		dev_dbg(di->dev, "Cap to scale at discharge %d/%d\n",
				cs->cap_to_scale[0], cs->cap_to_scale[1]);
	}
}

/**
 * ab8500_fg_check_capacity_limits() - Check if capacity has changed
 * @di:		pointer to the ab8500_fg structure
 * @init:	capacity is allowed to go up in init mode
 *
 * Check if capacity or capacity limit has changed and notify the system
 * about it using the power_supply framework
 */
static void ab8500_fg_check_capacity_limits(struct ab8500_fg *di, bool init)
{
	bool changed = false;
	int percent = DIV_ROUND_CLOSEST(di->bat_cap.rm_permille, 10);

	di->bat_cap.level = ab8500_fg_capacity_level(di);

	if (di->bat_cap.level != di->bat_cap.prev_level) {
		/*
		 * We do not allow reported capacity level to go up
		 * unless we're charging or if we're in init
		 */
		if (!(!di->flags.charging && di->bat_cap.level >
			di->bat_cap.prev_level) || init) {
			dev_dbg(di->dev, "level changed from %d to %d\n",
				di->bat_cap.prev_level,
				di->bat_cap.level);
			di->bat_cap.prev_level = di->bat_cap.level;
			changed = true;
		} else {
			dev_dbg(di->dev, "level not allowed to go up "
				"since no charger is connected: %d to %d\n",
				di->bat_cap.prev_level,
				di->bat_cap.level);
		}
	}

	/*
	 * If we have received the LOW_BAT IRQ, set capacity to 0 to initiate
	 * shutdown
	 */
	if (di->flags.low_bat) {
		dev_dbg(di->dev, "Battery low, set capacity to 0\n");
		di->bat_cap.prev_percent = 0;
		di->bat_cap.permille = 0;
		di->bat_cap.rm_permille = 0;
		percent = 0;
		di->bat_cap.prev_mah = 0;
		di->bat_cap.mah = 0;
		changed = true;
	} else if (di->flags.fully_charged) {
		/*
		 * We report 100% if algorithm reported fully charged
		 * and show 100% during maintenance charging (scaling).
		 */
		if (di->flags.force_full) {
			ab8500_fg_update_unusable_permille(di,
					   di->pdata->ddata->lowbat_hysteresis);
			/* Updates remaining permille */
			(void)ab8500_fg_convert_mah_to_permille(di,
							di->bat_cap.mah);

			di->bat_cap.prev_percent =
				DIV_ROUND_CLOSEST(di->bat_cap.rm_permille, 10);
			di->bat_cap.prev_mah = di->bat_cap.mah;

			changed = true;

			if (!di->bat_cap.cap_scale.enable) {
				di->bat_cap.cap_scale.enable = true;
				di->bat_cap.cap_scale.cap_to_scale[0] = 100;
				di->bat_cap.cap_scale.cap_to_scale[1] =
					di->bat_cap.prev_percent;
				di->bat_cap.cap_scale.disable_cap_level = 100;
			}
		} else if (di->bat_cap.prev_percent != percent) {
			di->bat_cap.prev_percent = percent;
			di->bat_cap.prev_mah = di->bat_cap.mah;

			changed = true;
		}
	} else if (di->bat_cap.prev_percent != percent) {
		if (percent == 0) {
			/*
			 * We will not report 0% unless we've got
			 * the LOW_BAT IRQ, no matter what the FG
			 * algorithm says.
			 */
			di->bat_cap.prev_percent = 1;
			percent = 1;

			changed = true;
		} else if (di->flags.charging ||
			   (!di->flags.charging &&
			    (percent < di->bat_cap.prev_percent ||
			     percent > (di->bat_cap.prev_percent + 1))) ||
			   init) {
			/*
			 * When not charging we only allow reported capacity to
			 * go up when in init or if new capacity is 2 units
			 * greater than previous.
			 */
			dev_dbg(di->dev,
				"capacity changed from %d to %d (%d)\n",
				di->bat_cap.prev_percent,
				percent,
				di->bat_cap.rm_permille);
			di->bat_cap.prev_percent = percent;
			di->bat_cap.prev_mah = di->bat_cap.mah;

			if (!changed) {
				di->bat_cap.prev_level = di->bat_cap.level;
				changed = true;
			}
		} else {
			dev_dbg(di->dev, "capacity not allowed to go up since "
				"no charger is connected: %d to %d (%d)\n",
				di->bat_cap.prev_percent, percent,
				di->bat_cap.permille);
		}
	}

	if (changed) {
		if (di->bat->capacity_scaling) {
			di->bat_cap.cap_scale.scaled_cap =
				ab8500_fg_calculate_scaled_capacity(di);

			dev_info(di->dev, "capacity=%d (%d)\n",
				di->bat_cap.prev_percent,
				di->bat_cap.cap_scale.scaled_cap);
		}
		power_supply_changed(&di->fg_psy);
		sysfs_notify(&di->fg_kobject, NULL, "charge_now");
		if (di->flags.fully_charged && di->flags.force_full) {
			dev_dbg(di->dev, "Battery full, notifying.\n");
			di->flags.force_full = false;
			sysfs_notify(&di->fg_kobject, NULL, "charge_full");
		}
		sysfs_notify(&di->fg_kobject, NULL, "charge_now");
	}
}

static void
ab8500_fg_check_calc_cap_discharge_fg(struct ab8500_fg *di)
{
	int conv_flag = 0;

	mutex_lock(&di->cc_lock);
	if (di->flags.conv_done) {
		conv_flag = 1;
		di->flags.conv_done = false;
	}
	mutex_unlock(&di->cc_lock);

	if (conv_flag) {
		ab8500_fg_calc_cap_discharge_fg(di);
		ab8500_fg_check_capacity_limits(di, false);
	}
}

static void
ab8500_fg_update_capacity(struct ab8500_fg *di, int uah)
{
	int prev;

	prev = di->bat_cap.mah;
	di->bat_cap.mah = clamp(di->bat_cap.mah + uah,
		0, di->bat_cap.max_mah_design);

	if (di->bat_cap.mah == di->bat_cap.max_mah_design)
		di->bat_cap.max_mah = di->bat_cap.max_mah_design;

	dev_dbg(di->dev,
		"Updated cap with %d uAh (was %d, now %d)\n",
		uah, prev, di->bat_cap.mah);

	ab8500_fg_fill_cap_sample(di, di->bat_cap.mah);
	di->bat_cap.permille = ab8500_fg_convert_mah_to_permille(di,
		di->bat_cap.mah);
	ab8500_fg_check_capacity_limits(di, false);
}

static void ab8500_fg_charge_state_to(struct ab8500_fg *di,
	enum ab8500_fg_charge_state new_state)
{
	if (di->charge_state == new_state)
		return;

	dev_dbg(di->dev, "Charge state from %d [%s] to %d [%s]\n",
		di->charge_state,
		charge_state[di->charge_state],
		new_state,
		charge_state[new_state]);

	di->charge_state = new_state;
}

static void ab8500_fg_discharge_state_to(struct ab8500_fg *di,
	enum ab8500_fg_charge_state new_state)
{
	if (di->discharge_state == new_state)
		return;

	dev_dbg(di->dev, "Disharge state from %d [%s] to %d [%s]\n",
		di->discharge_state,
		discharge_state[di->discharge_state],
		new_state,
		discharge_state[new_state]);

	di->discharge_state = new_state;
}

/**
 * ab8500_fg_algorithm_charging() - FG algorithm for when charging
 * @di:		pointer to the ab8500_fg structure
 *
 * Battery capacity calculation state machine for when we're charging
 */
static void ab8500_fg_algorithm_charging(struct ab8500_fg *di)
{
	int samples;

	/*
	 * If we change to discharge mode
	 * we should start with recovery
	 */
	if (di->discharge_state != AB8500_FG_DISCHARGE_START)
		ab8500_fg_discharge_state_to(di,
			AB8500_FG_DISCHARGE_START);

	switch (di->charge_state) {
	case AB8500_FG_CHARGE_INIT:
		if (di->calculate_missed_accu) {
			di->calculate_missed_accu = false;

			if (di->missed_accu_charge) {
				ab8500_fg_update_capacity(di,
						di->missed_accu_charge);
				di->missed_accu_charge = 0;
			}
		}

		samples = SEC_TO_SAMPLE(
			di->bat->fg_params->accu_charging);

		ab8500_fg_coulomb_counter(di, samples, true);
		ab8500_fg_charge_state_to(di, AB8500_FG_CHARGE_READOUT);
		break;

	case AB8500_FG_CHARGE_READOUT:
		/*
		 * Read the FG and calculate the new capacity
		 */
		mutex_lock(&di->cc_lock);
		if (!di->flags.conv_done && !di->flags.force_full) {
			/* Wasn't the CC IRQ that got us here */
			mutex_unlock(&di->cc_lock);
			dev_dbg(di->dev, "%s CC conv not done\n",
				__func__);

			break;
		}
		di->flags.conv_done = false;
		mutex_unlock(&di->cc_lock);

		ab8500_fg_calc_cap_charging(di);

		break;

	default:
		break;
	}

	/* Check capacity limits */
	ab8500_fg_check_capacity_limits(di, false);
}

static void force_capacity(struct ab8500_fg *di)
{
	int cap;

	ab8500_fg_clear_cap_samples(di);
	cap = di->bat_cap.user_mah;
	if (cap > di->bat_cap.max_mah_design) {
		dev_dbg(di->dev, "Remaining cap %d can't be bigger than total"
			" %d\n", cap, di->bat_cap.max_mah_design);
		cap = di->bat_cap.max_mah_design;
	}
	ab8500_fg_fill_cap_sample(di, di->bat_cap.user_mah);
	di->bat_cap.permille = ab8500_fg_convert_mah_to_permille(di, cap);
	di->bat_cap.mah = cap;
	ab8500_fg_check_capacity_limits(di, true);
}

static bool check_sysfs_capacity(struct ab8500_fg *di)
{
	int cap, lower, upper;
	int cap_permille;

	cap = di->bat_cap.user_mah;

	cap_permille = ab8500_fg_convert_mah_to_permille(di,
		di->bat_cap.user_mah);

	lower = di->bat_cap.permille - di->bat->fg_params->user_cap_limit * 10;
	upper = di->bat_cap.permille + di->bat->fg_params->user_cap_limit * 10;

	if (lower < 0)
		lower = 0;
	/* 1000 is permille, -> 100 percent */
	if (upper > 1000)
		upper = 1000;

	dev_dbg(di->dev, "Capacity limits:"
		" (Lower: %d User: %d Upper: %d) [user: %d, was: %d]\n",
		lower, cap_permille, upper, cap, di->bat_cap.mah);

	/* If within limits, use the saved capacity and exit estimation...*/
	if (cap_permille > lower && cap_permille < upper) {
		dev_dbg(di->dev, "OK! Using users cap %d uAh now\n", cap);
		force_capacity(di);
		return true;
	}
	dev_dbg(di->dev, "Capacity from user out of limits, ignoring\n");
	return false;
}

/**
 * ab8500_fg_algorithm_discharging() - FG algorithm for when discharging
 * @di:		pointer to the ab8500_fg structure
 *
 * Battery capacity calculation state machine for when we're discharging
 */
static void ab8500_fg_algorithm_discharging(struct ab8500_fg *di)
{
	struct timespec now;
	struct timespec diff;
	int sleep_time;
	int samples;

	/* If we change to charge mode we should start with init */
	if (di->charge_state != AB8500_FG_CHARGE_INIT)
		ab8500_fg_charge_state_to(di, AB8500_FG_CHARGE_INIT);

	di->vbat = ab8500_fg_bat_voltage(di);

	switch (di->discharge_state) {
	case AB8500_FG_DISCHARGE_INIT:
		/* We use the FG IRQ to work on */
		di->init_cnt = 0;
		samples = SEC_TO_SAMPLE(di->bat->fg_params->init_timer);
		ab8500_fg_coulomb_counter(di, samples, true);
		ab8500_fg_discharge_state_to(di,
			AB8500_FG_DISCHARGE_INITMEASURING);

		/* Intentional fallthrough */
	case AB8500_FG_DISCHARGE_INITMEASURING:
		/*
		 * Discard a number of samples during startup.
		 * After that, use compensated voltage for a few
		 * samples to get an initial capacity.
		 * Then go to READOUT
		 */
		sleep_time = di->bat->fg_params->init_timer;

		/* Discard the first [x] seconds */
		if (di->init_cnt >
			di->bat->fg_params->init_discard_time) {
			if (ab8500_fg_get_synced_vbat_curr(di, &di->vbat,
							&di->inst_curr))
				break;

			ab8500_fg_calc_cap_discharge_voltage(di, true);
			ab8500_fg_check_capacity_limits(di, true);
		}

		di->init_cnt += sleep_time;
		if (di->init_cnt > di->bat->fg_params->init_total_time) {
			samples = SEC_TO_SAMPLE(
				di->bat->fg_params->accu_high_curr);
			ab8500_fg_coulomb_counter(di, samples, true);
			ab8500_fg_discharge_state_to(di,
			AB8500_FG_DISCHARGE_READOUT_INIT);
		}

		break;

	case AB8500_FG_DISCHARGE_START:
		di->prohibit_uncomp_voltage_replace = true;
		di->high_curr_mode = false;
		di->discharge_start = ab8500_fg_get_time();
		ab8500_fg_discharge_state_to(di,
			AB8500_FG_DISCHARGE_INIT_RECOVERY);

		/* Intentional fallthrough */

	case AB8500_FG_DISCHARGE_INIT_RECOVERY:
		di->recovery_cnt_ms = 0;
		di->recovery_needed = true;
		di->high_curr_thr_pc = 0;
		ab8500_fg_discharge_state_to(di,
			AB8500_FG_DISCHARGE_RECOVERY);
		di->recovery_start = ab8500_fg_get_time();

		/* Intentional fallthrough */

	case AB8500_FG_DISCHARGE_RECOVERY:
		ab8500_fg_check_calc_cap_discharge_fg(di);
		sleep_time = di->bat->fg_params->recovery_sleep_timer;

		now = ab8500_fg_get_time();
		diff = timespec_sub(now, di->recovery_start);
		di->recovery_start = now;

		/*
		 * We should check the power consumption
		 * If low, go to READOUT (after x min) or
		 * RECOVERY_SLEEP if time left.
		 * If high, go to READOUT
		 */
		di->inst_curr = ab8500_fg_inst_curr_blocking(di);

		if (ab8500_fg_is_low_curr(di, di->inst_curr)) {
			di->recovery_cnt_ms += ab8500_fg_timespec_to_ms(diff);
			dev_dbg(di->dev,
				"FG recovered %ld.%ld sec. Total %d.%d sec\n",
				diff.tv_sec, diff.tv_nsec / 1000,
				di->recovery_cnt_ms / 1000,
				di->recovery_cnt_ms % 1000);

			/* Add 500 to compare with nearest second */
			if ((di->recovery_cnt_ms + 500) >
				di->bat->fg_params->recovery_total_time *
				1000) {
				di->recovery_needed = false;

				/* Need to clear mode since we can immediately
				 * end up in the high current path at
				 * DISCHARGE_READOUT and that will incorrecly
				 * put us back to AB8500_FG_DISCHARGE_RECOVERY
				 * again since high current counter is not
				 * reset.
				 */
				if (di->high_curr_mode)
					di->high_curr_mode = false;

				sleep_time = 0;
				ab8500_fg_discharge_state_to(di,
					     AB8500_FG_DISCHARGE_READOUT);
			}
		} else {
			ab8500_fg_calculate_unusable_permille(di);
			/*
			 * In order to prevent RECOVERY mode from being
			 * interrupted and started over again by single current
			 * spikes, we need to specify acceptable range in
			 * percents, and do not start RECOVERY again if we are
			 * in that range
			 */
			di->high_curr_thr_pc += (-di->inst_curr * 100 /
				di->bat->fg_params->high_curr_threshold) - 100;

			 if (di->high_curr_thr_pc >
			     di->bat->fg_params->high_curr_exceed_thr) {
				/*
				 * Need to clear counter since we can end up
				 * in suspend before entering DISCHARGE_READOUT
				 * state again.
				 */
				di->recovery_cnt_ms = 0;
				di->high_curr_thr_pc = 0;

				sleep_time = 0;
				ab8500_fg_discharge_state_to(di,
					AB8500_FG_DISCHARGE_READOUT);
			 } else {
				 dev_dbg(di->dev, "Instant current is higher "
					 "than threshold in recovery mode: %d\n",
					 di->high_curr_thr_pc);
			 }
		}

		queue_delayed_work(di->fg_wq,
				   &di->fg_periodic_work,
				   sleep_time * HZ);
		break;

	case AB8500_FG_DISCHARGE_READOUT_INIT:
		ab8500_fg_discharge_state_to(di,
				AB8500_FG_DISCHARGE_READOUT);
		break;

	case AB8500_FG_DISCHARGE_READOUT:
		ab8500_fg_check_calc_cap_discharge_fg(di);
		if (ab8500_fg_get_synced_vbat_curr(di, &di->vbat,
						   &di->inst_curr))
			break;

		if (ab8500_fg_is_low_curr(di, di->inst_curr)) {
			/*
			 * Assume, we are in low current mode
			 */
			if (di->high_curr_mode)
				di->high_curr_mode = false;

			if (di->recovery_needed) {
				ab8500_fg_discharge_state_to(di,
					AB8500_FG_DISCHARGE_INIT_RECOVERY);

				queue_delayed_work(di->fg_wq,
					&di->fg_periodic_work, 0);

				break;
			}

			/*
			 * we do not want to compensate when current
			 * is positive, thus, small sanity check.
			 */
			if (di->inst_curr <= 0) {
				di->missed_accu_charge = 0;
				ab8500_fg_calc_cap_discharge_voltage(di, true);
				ab8500_fg_check_capacity_limits(di, false);
			}
		} else {
			ab8500_fg_calculate_unusable_permille(di);
			/* Detect mode change */
			if (!di->high_curr_mode) {
				di->high_curr_mode = true;
				di->high_curr_cnt_ms = 0;
				di->high_curr_start = ab8500_fg_get_time();
			}

			if (di->recovery_needed)
				break;

			now = ab8500_fg_get_time();
			diff = timespec_sub(now, di->high_curr_start);
			di->high_curr_start = now;

			di->high_curr_cnt_ms +=
				ab8500_fg_timespec_to_ms(diff);
			dev_dbg(di->dev, "Time spent in high curr: %d ms\n",
				di->high_curr_cnt_ms);
			/* Add 500 to compare with nearest second */
			if ((di->high_curr_cnt_ms + 500) >
				di->bat->fg_params->high_curr_time * 1000) {
				di->recovery_needed = true;
				di->recovery_cnt_ms = 0;
			} else {
				/* Try again to make new measurement on instant
				 * current and see if capacity can be based on
				 * voltage.
				 */
				queue_delayed_work(di->fg_wq,
						   &di->fg_periodic_work, HZ);
			}
		}

		break;
	default:
		break;
	}
}

/**
 * ab8500_fg_algorithm_calibrate() - Internal columb counter offset calibration
 * @di:		pointer to the ab8500_fg structure
 *
 */
static void ab8500_fg_algorithm_calibrate(struct ab8500_fg *di)
{
	int ret;

	switch (di->calib_state) {
	case AB8500_FG_CALIB_INIT:
		dev_dbg(di->dev, "Calibration ongoing...\n");

		ret = abx500_mask_and_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
			CC_INT_CAL_N_AVG_MASK, CC_INT_CAL_SAMPLES_8);
		if (ret < 0)
			goto err;

		ret = abx500_mask_and_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
			CC_INTAVGOFFSET_ENA, CC_INTAVGOFFSET_ENA);
		if (ret < 0)
			goto err;
		di->calib_state = AB8500_FG_CALIB_WAIT;
		break;
	case AB8500_FG_CALIB_END:
		ret = abx500_mask_and_set_register_interruptible(di->dev,
			AB8500_GAS_GAUGE, AB8500_GASG_CC_CTRL_REG,
			CC_MUXOFFSET, CC_MUXOFFSET);
		if (ret < 0)
			goto err;
		di->flags.calibrate = false;
		dev_dbg(di->dev, "Calibration done...\n");
		queue_delayed_work(di->fg_wq, &di->fg_periodic_work, 0);
		break;
	case AB8500_FG_CALIB_WAIT:
		dev_dbg(di->dev, "Calibration WFI\n");
	default:
		break;
	}
	return;
err:
	/* Something went wrong, don't calibrate then */
	dev_err(di->dev, "failed to calibrate the CC\n");
	di->flags.calibrate = false;
	di->calib_state = AB8500_FG_CALIB_INIT;
	queue_delayed_work(di->fg_wq, &di->fg_periodic_work, 0);
}

/**
 * ab8500_fg_algorithm() - Entry point for the FG algorithm
 * @di:		pointer to the ab8500_fg structure
 *
 * Entry point for the battery capacity calculation state machine
 */
static void ab8500_fg_algorithm(struct ab8500_fg *di)
{
	if (di->flags.calibrate)
		ab8500_fg_algorithm_calibrate(di);
	else {
		if (di->flags.fg_re_enable) {
			int samples = di->fg_samples;
			di->flags.fg_re_enable = false;
			di->fg_samples = 0;
			if (ab8500_fg_coulomb_counter(di, samples, true) < 0)
				BUG();
		}
		if (di->flags.charging)
			ab8500_fg_algorithm_charging(di);
		else
			ab8500_fg_algorithm_discharging(di);
	}

	dev_dbg(di->dev, "[FG_DATA] %d %d %d %d %d %d %d %d %d %d %d %d "
		"%d %d %d %d %d %d %d\n",
		di->bat_cap.max_mah_design,
		di->bat_cap.max_mah,
		di->bat_cap.mah,
		di->bat_cap.permille,
		di->bat_cap.level,
		di->bat_cap.unusable_permille,
		di->bat_cap.deltav,
		di->bat_cap.prev_mah,
		di->bat_cap.prev_percent,
		di->bat_cap.prev_level,
		di->vbat,
		di->inst_curr,
		di->avg_curr,
		di->accu_charge,
		di->flags.charging,
		di->charge_state,
		di->discharge_state,
		di->high_curr_mode,
		di->recovery_needed);
}

/**
 * ab8500_fg_periodic_work() - Run the FG state machine periodically
 * @work:	pointer to the work_struct structure
 *
 * Work queue function for periodic work
 */
static void ab8500_fg_periodic_work(struct work_struct *work)
{
	struct ab8500_fg *di = container_of(work, struct ab8500_fg,
		fg_periodic_work.work);

	if (di->init_capacity) {
		/* Get an initial capacity calculation */
		if (ab8500_fg_get_synced_vbat_curr(di, &di->vbat,
						   &di->inst_curr))
			return;
		ab8500_fg_calc_cap_discharge_voltage(di, true);
		ab8500_fg_check_capacity_limits(di, true);
		di->init_capacity = false;

		queue_delayed_work(di->fg_wq, &di->fg_periodic_work, 0);
	} else if (di->flags.user_cap) {
		if (check_sysfs_capacity(di)) {
			ab8500_fg_check_capacity_limits(di, true);
			if (di->flags.charging)
				ab8500_fg_charge_state_to(di,
					AB8500_FG_CHARGE_INIT);
			else
				ab8500_fg_discharge_state_to(di,
					AB8500_FG_DISCHARGE_READOUT_INIT);
		}
		di->flags.user_cap = false;
		queue_delayed_work(di->fg_wq, &di->fg_periodic_work, 0);
	} else
		ab8500_fg_algorithm(di);


}


/**
 * ab8500_fg_check_hw_failure_work() - Check OVV_BAT condition
 * @work:	pointer to the work_struct structure
 *
 * Work queue function for checking the OVV_BAT condition
 */
static void ab8500_fg_check_hw_failure_work(struct work_struct *work)
{
	int ret;
	u8 reg_value;

	struct ab8500_fg *di = container_of(work, struct ab8500_fg,
		fg_check_hw_failure_work.work);

	/*
	 * If we have had a battery over-voltage situation,
	 * check ovv-bit to see if it should be reset.
	 */
	ret = abx500_get_register_interruptible(di->dev,
		AB8500_CHARGER, AB8500_CH_STAT_REG,
		&reg_value);
	if (ret < 0) {
		dev_err(di->dev, "%s ab8500 read failed\n", __func__);
		return;
	}
	if ((reg_value & BATT_OVV) == BATT_OVV) {
		if (!di->flags.bat_ovv) {
			dev_dbg(di->dev, "Battery OVV\n");
			di->flags.bat_ovv = true;
			power_supply_changed(&di->fg_psy);
		}
		queue_delayed_work(di->fg_wq, &di->fg_check_hw_failure_work,
				   HZ);
	} else {
		dev_dbg(di->dev, "Battery recovered from OVV\n");
		di->flags.bat_ovv = false;
		power_supply_changed(&di->fg_psy);
	}
}

/**
 * ab8500_fg_low_bat_work() - Check LOW_BAT condition
 * @work:	pointer to the work_struct structure
 *
 * Work queue function for checking the LOW_BAT condition
 */
static void ab8500_fg_low_bat_work(struct work_struct *work)
{
	struct ab8500_fg *di = container_of(work, struct ab8500_fg,
		fg_low_bat_work.work);
	int cut_level = 0;

	di->vbat = ab8500_fg_bat_voltage(di);
	mutex_lock(&di->shutdown_lock);

	/* Check if LOW_BAT still fulfilled */
	if (di->vbat < di->pdata->ddata->lowbat_threshold +
			di->bat->bat_type[di->bat->batt_id].batt_vbat_offset +
			di->pdata->ddata->lowbat_hysteresis) {
		/*
		 * if capacity is between 15% - 2%, and we get LOW
		 * battery interrupt, it means our FG is out of the
		 * track
		 */
		if (di->bat_cap.permille < 160 && di->bat_cap.permille > 20) {
			cut_level = ab8500_fg_convert_permille_to_mah(di, 10);
			ab8500_fg_update_capacity(di, -cut_level);
		}

		if (--di->low_bat_cnt == 0)
			di->flags.low_bat = true;

		dev_warn(di->dev, "( %d )Battery voltage still LOW: %dmV\n",
			di->low_bat_cnt, di->vbat);

		di->flags.low_bat_delay = true;

		/*
		 * We need to re-schedule this check to be able to detect
		 * if the voltage increases again during charging
		 */
		if (!di->flags.low_bat)
			queue_delayed_work(di->shutdown_wq,
				&di->fg_low_bat_work,
				msecs_to_jiffies(LOW_BAT_CHECK_INTERVAL));
	} else {
		di->flags.low_bat_delay = false;
		di->flags.low_bat = false;
		dev_warn(di->dev, "Battery voltage OK again: %dmV\n", di->vbat);
	}
	mutex_unlock(&di->shutdown_lock);
	/* This is needed to dispatch LOW_BAT */
	ab8500_fg_check_capacity_limits(di, false);
}

/**
 * ab8500_fg_battok_calc - calculate the bit pattern corresponding
 * to the target voltage.
 * @di:       pointer to the ab8500_fg structure
 * @target    target voltage
 *
 * Returns bit pattern closest to the target voltage
 * valid return values are 0-14. (0-BATT_OK_MAX_NR_INCREMENTS)
 */

static int ab8500_fg_battok_calc(struct ab8500_fg *di, int target)
{
	if (target > BATT_OK_MIN +
		(BATT_OK_INCREMENT * BATT_OK_MAX_NR_INCREMENTS))
		return BATT_OK_MAX_NR_INCREMENTS;
	if (target < BATT_OK_MIN)
		return 0;
	return (target - BATT_OK_MIN) / BATT_OK_INCREMENT;
}

/**
 * ab8500_fg_battok_init_hw_register - init battok levels
 * @di:       pointer to the ab8500_fg structure
 *
 */

static int ab8500_fg_battok_init_hw_register(struct ab8500_fg *di)
{
	int selected;
	int sel0;
	int sel1;
	int cbp_sel0;
	int cbp_sel1;
	int ret;
	int new_val;

	sel0 = di->bat->fg_params->battok_falling_th_sel0;
	sel1 = di->bat->fg_params->battok_raising_th_sel1;

	cbp_sel0 = ab8500_fg_battok_calc(di, sel0);
	cbp_sel1 = ab8500_fg_battok_calc(di, sel1);

	selected = BATT_OK_MIN + cbp_sel0 * BATT_OK_INCREMENT;

	if (selected != sel0)
		dev_warn(di->dev, "Invalid voltage step:%d, using %d %d\n",
			sel0, selected, cbp_sel0);

	selected = BATT_OK_MIN + cbp_sel1 * BATT_OK_INCREMENT;

	if (selected != sel1)
		dev_warn(di->dev, "Invalid voltage step:%d, using %d %d\n",
			sel1, selected, cbp_sel1);

	new_val = cbp_sel0 | (cbp_sel1 << 4);

	dev_dbg(di->dev, "using: %x %d %d\n", new_val, cbp_sel0, cbp_sel1);
	ret = abx500_set_register_interruptible(di->dev, AB8500_SYS_CTRL2_BLOCK,
		AB8500_BATT_OK_REG, new_val);
	return ret;
}

/**
 * ab8500_fg_instant_work() - Run the FG state machine instantly
 * @work:	pointer to the work_struct structure
 *
 * Work queue function for instant work
 */
static void ab8500_fg_instant_work(struct work_struct *work)
{
	struct ab8500_fg *di = container_of(work, struct ab8500_fg, fg_work);

	ab8500_fg_algorithm(di);
}

/**
 * ab8500_fg_cc_data_end_handler() - to get battery inst current.
 * @irq:       interrupt number
 * @_di:       pointer to the ab8500_fg structure
 *
 * Returns IRQ status(IRQ_HANDLED)
 */
static irqreturn_t ab8500_fg_cc_data_end_handler(int irq, void *_di)
{
	struct ab8500_fg *di = _di;

	if (di->nbr_cceoc_irq_cnt > 0)
		di->nbr_cceoc_irq_cnt--;
	else
		complete(&di->ab8500_fg_complete);

	return IRQ_HANDLED;
}

/**
 * ab8500_fg_cc_int_calib_handler() - isr to get calibration done event.
 * @irq:       interrupt number
 * @_di:       pointer to the ab8500_fg structure
 *
 * Returns IRQ status(IRQ_HANDLED)
 */
static irqreturn_t ab8500_fg_cc_int_calib_handler(int irq, void *_di)
{
	struct ab8500_fg *di = _di;
	di->calib_state = AB8500_FG_CALIB_END;
	queue_delayed_work(di->fg_wq, &di->fg_periodic_work, 0);
	return IRQ_HANDLED;
}

/**
 * ab8500_fg_cc_convend_handler() - isr to get battery avg current.
 * @irq:       interrupt number
 * @_di:       pointer to the ab8500_fg structure
 *
 * Returns IRQ status(IRQ_HANDLED)
 */
static irqreturn_t ab8500_fg_cc_convend_handler(int irq, void *_di)
{
	struct ab8500_fg *di = _di;

	INIT_COMPLETION(di->accu_done);
	queue_work(di->avg_curr_wq, &di->fg_acc_cur_work);

	return IRQ_HANDLED;
}

/**
 * ab8500_fg_batt_ovv_handler() - Battery OVV occured
 * @irq:       interrupt number
 * @_di:       pointer to the ab8500_fg structure
 *
 * Returns IRQ status(IRQ_HANDLED)
 */
static irqreturn_t ab8500_fg_batt_ovv_handler(int irq, void *_di)
{
	struct ab8500_fg *di = _di;

	dev_dbg(di->dev, "Battery OVV\n");

	/* Schedule a new HW failure check */
	queue_delayed_work(di->fg_wq, &di->fg_check_hw_failure_work, 0);

	return IRQ_HANDLED;
}

/**
 * ab8500_fg_lowbatf_handler() - Battery voltage is below LOW threshold
 * @irq:       interrupt number
 * @_di:       pointer to the ab8500_fg structure
 *
 * Returns IRQ status(IRQ_HANDLED)
 */
static irqreturn_t ab8500_fg_lowbatf_handler(int irq, void *_di)
{
	struct ab8500_fg *di = _di;

	mutex_lock(&di->shutdown_lock);
	/* Initiate handling in ab8500_fg_low_bat_work()
	 * if not already initiated.
	 */
	if (!di->flags.low_bat_delay) {
		dev_warn(di->dev, "Battery voltage is below LOW threshold\n");
		di->flags.low_bat_delay = true;
		di->low_bat_cnt = LOW_BAT_CHECK_SAMPLES;
		/*
		 * Start a timer to check LOW_BAT again after some time
		 * This is done to avoid shutdown on single voltage dips
		 */
		queue_delayed_work(di->shutdown_wq, &di->fg_low_bat_work,
			msecs_to_jiffies(LOW_BAT_CHECK_INTERVAL));
	}
	mutex_unlock(&di->shutdown_lock);
	return IRQ_HANDLED;
}

/**
 * ab8500_fg_get_property() - get the fg properties
 * @psy:	pointer to the power_supply structure
 * @psp:	pointer to the power_supply_property structure
 * @val:	pointer to the power_supply_propval union
 *
 * This function gets called when an application tries to get the
 * fg properties by reading the sysfs files.
 * voltage_now:		battery voltage
 * current_now:		battery instant current
 * current_avg:		battery average current
 * charge_full_design:	capacity where battery is considered full
 * charge_now:		battery capacity in nAh
 * capacity:		capacity in percent
 * capacity_level:	capacity level
 *
 * Returns error code in case of failure else 0 on success
 */
static int ab8500_fg_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	struct ab8500_fg *di;

	di = to_ab8500_fg_device_info(psy);

	/*
	 * If battery is identified as unknown and charging of unknown
	 * batteries is disabled, we always report 100% capacity and
	 * capacity level UNKNOWN, since we can't calculate
	 * remaining capacity
	 */

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (di->flags.bat_ovv)
			val->intval = BATT_OVV_VALUE * 1000;
		else
			val->intval = di->vbat * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->inst_curr * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->avg_curr * 1000;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = ab8500_fg_convert_mah_to_uwh(di,
				di->bat_cap.max_mah_design);
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = ab8500_fg_convert_mah_to_uwh(di,
				di->bat_cap.max_mah);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		if (di->flags.batt_unknown && !di->bat->chg_unknown_bat &&
				di->flags.batt_id_received)
			val->intval = ab8500_fg_convert_mah_to_uwh(di,
					di->bat_cap.max_mah);
		else
			val->intval = ab8500_fg_convert_mah_to_uwh(di,
					di->bat_cap.prev_mah);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = di->bat_cap.max_mah_design;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = di->bat_cap.max_mah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (di->flags.batt_unknown && !di->bat->chg_unknown_bat &&
				di->flags.batt_id_received)
			val->intval = di->bat_cap.max_mah;
		else
			val->intval = di->bat_cap.prev_mah;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (di->flags.batt_unknown && !di->bat->chg_unknown_bat &&
				di->flags.batt_id_received)
			val->intval = 100;
		else
			val->intval = di->bat_cap.prev_percent;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (di->flags.batt_unknown && !di->bat->chg_unknown_bat &&
				di->flags.batt_id_received)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		else
			val->intval = di->bat_cap.prev_level;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int ab8500_fg_get_ext_psy_data(struct device *dev, void *data)
{
	struct power_supply *psy;
	struct power_supply *ext;
	struct ab8500_fg *di;
	union power_supply_propval ret;
	int i, j;
	bool psy_found = false;

	psy = (struct power_supply *)data;
	ext = dev_get_drvdata(dev);
	di = to_ab8500_fg_device_info(psy);

	/*
	 * For all psy where the name of your driver
	 * appears in any supplied_to
	 */
	for (i = 0; i < ext->num_supplicants; i++) {
		if (!strcmp(ext->supplied_to[i], psy->name))
			psy_found = true;
	}

	if (!psy_found)
		return 0;

	/* Go through all properties for the psy */
	for (j = 0; j < ext->num_properties; j++) {
		enum power_supply_property prop;
		prop = ext->properties[j];

		if (ext->get_property(ext, prop, &ret))
			continue;

		switch (prop) {
		case POWER_SUPPLY_PROP_STATUS:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				switch (ret.intval) {
				case POWER_SUPPLY_STATUS_UNKNOWN:
				case POWER_SUPPLY_STATUS_DISCHARGING:
				case POWER_SUPPLY_STATUS_NOT_CHARGING:
					if (!di->flags.charging)
						break;
					di->flags.charging = false;
					di->flags.fully_charged = false;
					if (di->bat->capacity_scaling)
						ab8500_fg_update_cap_scalers(di);
					queue_work(di->fg_wq, &di->fg_work);
					break;
				case POWER_SUPPLY_STATUS_FULL:
					if (di->flags.fully_charged)
						break;
					di->flags.fully_charged = true;
					di->flags.force_full = true;
					/* Save current capacity as maximum */
					di->bat_cap.max_mah = di->bat_cap.mah;
					di->bat_cap.mah =
						di->bat_cap.max_mah_design;
					ab8500_fg_fill_cap_sample(di,
							  di->bat_cap.mah);
					queue_work(di->fg_wq, &di->fg_work);
					break;
				case POWER_SUPPLY_STATUS_CHARGING:
					if (di->flags.charging &&
						!di->flags.fully_charged)
						break;
					di->flags.charging = true;
					di->flags.fully_charged = false;
					if (di->bat->capacity_scaling)
						ab8500_fg_update_cap_scalers(di);

					queue_work(di->fg_wq, &di->fg_work);
					break;
				};
			default:
				break;
			};
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				if (!di->flags.batt_id_received)
					di->flags.batt_id_received = true;
				if (ret.intval)
					di->flags.batt_unknown = false;
				else
					di->flags.batt_unknown = true;
				break;
			default:
				break;
			}
			break;
		case POWER_SUPPLY_PROP_TEMP:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				if (di->flags.batt_id_received)
					di->bat_temp = ret.intval;
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
	}
	return 0;
}

/**
 * ab8500_fg_init_hw_registers() - Set up FG related registers
 * @di:		pointer to the ab8500_fg structure
 *
 * Set up battery OVV, low battery voltage registers
 */
static int ab8500_fg_init_hw_registers(struct ab8500_fg *di)
{
	int ret;

	/* Set VBAT OVV threshold */
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_CHARGER,
		AB8500_BATT_OVV,
		BATT_OVV_TH_4P75,
		BATT_OVV_TH_4P75);
	if (ret) {
		dev_err(di->dev, "failed to set BATT_OVV\n");
		goto out;
	}

	/* Enable VBAT OVV detection */
	ret = abx500_mask_and_set_register_interruptible(di->dev,
		AB8500_CHARGER,
		AB8500_BATT_OVV,
		BATT_OVV_ENA,
		BATT_OVV_ENA);
	if (ret) {
		dev_err(di->dev, "failed to enable BATT_OVV\n");
		goto out;
	}

	/* Low Battery Voltage */
	ret = abx500_set_register_interruptible(di->dev,
		AB8500_SYS_CTRL2_BLOCK,
		AB8500_LOW_BAT_REG,
		ab8500_volt_to_regval(
			di->pdata->ddata->lowbat_threshold) << 1 |
		LOW_BAT_ENABLE);
	if (ret) {
		dev_err(di->dev, "%s write failed\n", __func__);
		goto out;
	}

	/* Battery OK threshold */
	ret = ab8500_fg_battok_init_hw_register(di);
	if (ret) {
		dev_err(di->dev, "BattOk init write failed.\n");
		goto out;
	}

out:
	return ret;
}

/**
 * ab8500_fg_external_power_changed() - callback for power supply changes
 * @psy:       pointer to the structure power_supply
 *
 * This function is the entry point of the pointer external_power_changed
 * of the structure power_supply.
 * This function gets executed when there is a change in any external power
 * supply that this driver needs to be notified of.
 */
static void ab8500_fg_external_power_changed(struct power_supply *psy)
{
	struct ab8500_fg *di = to_ab8500_fg_device_info(psy);

	class_for_each_device(power_supply_class, NULL,
		&di->fg_psy, ab8500_fg_get_ext_psy_data);
}

/**
 * abab8500_fg_reinit_work() - work to reset the FG algorithm
 * @work:	pointer to the work_struct structure
 *
 * Used to reset the current battery capacity to be able to
 * retrigger a new voltage base capacity calculation. For
 * test and verification purpose.
 */
static void ab8500_fg_reinit_work(struct work_struct *work)
{
	struct ab8500_fg *di = container_of(work, struct ab8500_fg,
		fg_reinit_work.work);

	if (di->flags.calibrate == false) {
		dev_dbg(di->dev, "Resetting FG state machine to init.\n");
		ab8500_fg_clear_cap_samples(di);
		if (ab8500_fg_get_synced_vbat_curr(di, &di->vbat,
						   &di->inst_curr)) {
			queue_delayed_work(di->fg_wq, &di->fg_reinit_work,
					   round_jiffies(1));
			return;
		}
		ab8500_fg_calc_cap_discharge_voltage(di, true);
		ab8500_fg_charge_state_to(di, AB8500_FG_CHARGE_INIT);
		ab8500_fg_discharge_state_to(di, AB8500_FG_DISCHARGE_INIT);
		queue_delayed_work(di->fg_wq, &di->fg_periodic_work, 0);

	} else {
		dev_err(di->dev, "Residual offset calibration ongoing "
			"retrying..\n");
		/* Wait one second until next try*/
		queue_delayed_work(di->fg_wq, &di->fg_reinit_work,
			round_jiffies(1));
	}
}

/**
 * ab8500_fg_reinit() - forces FG algorithm to reinitialize with current values
 *
 * This function can be used to force the FG algorithm to recalculate a new
 * voltage based battery capacity.
 */
void ab8500_fg_reinit(void)
{
	struct ab8500_fg *di = ab8500_fg_get();
	/* User won't be notified if a null pointer returned. */
	if (di != NULL)
		queue_delayed_work(di->fg_wq, &di->fg_reinit_work, 0);
}

/* Exposure to the sysfs interface */

struct ab8500_fg_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct ab8500_fg *, char *);
	ssize_t (*store)(struct ab8500_fg *, const char *, size_t);
};

static ssize_t charge_full_show(struct ab8500_fg *di, char *buf)
{
	return sprintf(buf, "%d\n", di->bat_cap.max_mah);
}

static ssize_t charge_full_store(struct ab8500_fg *di, const char *buf,
				 size_t count)
{
	unsigned long charge_full;
	ssize_t ret = -EINVAL;

	ret = strict_strtoul(buf, 10, &charge_full);

	dev_dbg(di->dev, "Ret %d charge_full %lu\n", ret, charge_full);

	if (!ret) {
		di->bat_cap.max_mah = (int) charge_full;
		ret = count;
	}
	return ret;
}

static ssize_t charge_now_show(struct ab8500_fg *di, char *buf)
{
	return sprintf(buf, "%d\n", di->bat_cap.prev_mah);
}

static ssize_t charge_now_store(struct ab8500_fg *di, const char *buf,
				 size_t count)
{
	unsigned long charge_now;
	ssize_t ret;

	ret = strict_strtoul(buf, 10, &charge_now);

	dev_dbg(di->dev, "Ret %d charge_now %lu was %d\n",
		ret, charge_now, di->bat_cap.prev_mah);

	if (!ret) {
		di->bat_cap.user_mah = (int) charge_now;
		di->flags.user_cap = true;
		ret = count;
		queue_delayed_work(di->fg_wq, &di->fg_periodic_work, 0);
	}
	return ret;
}

static struct ab8500_fg_sysfs_entry charge_full_attr =
	__ATTR(charge_full, 0644, charge_full_show, charge_full_store);

static struct ab8500_fg_sysfs_entry charge_now_attr =
	__ATTR(charge_now, 0644, charge_now_show, charge_now_store);

static ssize_t
ab8500_fg_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct ab8500_fg_sysfs_entry *entry;
	struct ab8500_fg *di;

	entry = container_of(attr, struct ab8500_fg_sysfs_entry, attr);
	di = container_of(kobj, struct ab8500_fg, fg_kobject);

	if (!entry->show)
		return -EIO;

	return entry->show(di, buf);
}
static ssize_t
ab8500_fg_store(struct kobject *kobj, struct attribute *attr, const char *buf,
		size_t count)
{
	struct ab8500_fg_sysfs_entry *entry;
	struct ab8500_fg *di;

	entry = container_of(attr, struct ab8500_fg_sysfs_entry, attr);
	di = container_of(kobj, struct ab8500_fg, fg_kobject);

	if (!entry->store)
		return -EIO;

	return entry->store(di, buf, count);
}

const struct sysfs_ops ab8500_fg_sysfs_ops = {
	.show = ab8500_fg_show,
	.store = ab8500_fg_store,
};

static struct attribute *ab8500_fg_attrs[] = {
	&charge_full_attr.attr,
	&charge_now_attr.attr,
	NULL,
};

static struct kobj_type ab8500_fg_ktype = {
	.sysfs_ops = &ab8500_fg_sysfs_ops,
	.default_attrs = ab8500_fg_attrs,
};

/**
 * ab8500_chargalg_sysfs_exit() - de-init of sysfs entry
 * @di:                pointer to the struct ab8500_chargalg
 *
 * This function removes the entry in sysfs.
 */
static void ab8500_fg_sysfs_exit(struct ab8500_fg *di)
{
	kobject_del(&di->fg_kobject);
}

/**
 * ab8500_chargalg_sysfs_init() - init of sysfs entry
 * @di:                pointer to the struct ab8500_chargalg
 *
 * This function adds an entry in sysfs.
 * Returns error code in case of failure else 0(on success)
 */
static int ab8500_fg_sysfs_init(struct ab8500_fg *di)
{
	int ret = 0;

	sysfs_attr_on = 0;
	ret = kobject_init_and_add(&di->fg_kobject,
		&ab8500_fg_ktype,
		NULL, "ab8500_fg");
	if (ret < 0)
		dev_err(di->dev, "failed to create sysfs entry\n");

	return ret;
}

static ssize_t ab8500_show_capacity(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct ab8500_fg *di;
	int capacity;

	di = to_ab8500_fg_device_info(psy);

	if (di->bat->capacity_scaling)
		capacity = di->bat_cap.cap_scale.scaled_cap;
	else
		capacity = di->bat_cap.prev_percent;

	return scnprintf(buf, PAGE_SIZE, "%d\n", capacity);
}

static struct device_attribute ab8500_fg_sysfs_psy_attrs[] = {
	__ATTR(capacity, S_IRUGO, ab8500_show_capacity, NULL),
};

static int ab8500_fg_sysfs_psy_create_attrs(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ab8500_fg_sysfs_psy_attrs); i++)
		if (device_create_file(dev, &ab8500_fg_sysfs_psy_attrs[i]))
			goto sysfs_psy_create_attrs_failed;

	return 0;

sysfs_psy_create_attrs_failed:
	dev_err(dev, "Failed creating sysfs psy attrs.\n");
	while (i--)
		device_remove_file(dev, &ab8500_fg_sysfs_psy_attrs[i]);

	return -EIO;
}

static void ab8500_fg_sysfs_psy_remove_attrs(struct device *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ab8500_fg_sysfs_psy_attrs); i++)
		(void)device_remove_file(dev, &ab8500_fg_sysfs_psy_attrs[i]);
}
/* Exposure to the sysfs interface <<END>> */

#if defined(CONFIG_PM)
static int ab8500_fg_resume(struct platform_device *pdev)
{
	struct ab8500_fg *di = platform_get_drvdata(pdev);
	int samples = 0;

	/*
	 * enable CC, if it was disabled
	 */
	if (!di->flags.charging) {
		di->calculate_missed_accu = true;
		INIT_COMPLETION(di->accu_done);
		samples = 2;			/* 500 ms */
	} else {
		samples = SEC_TO_SAMPLE(di->bat->fg_params->accu_charging);
	}

	(void) ab8500_fg_coulomb_counter(di, samples, true);
	di->resume_start = ab8500_fg_get_time();

	/*
	 * Change state if we're not charging. If we're charging we will wake
	 * up on the FG IRQ
	 */
	if (!di->flags.charging) {
		if (di->recovery_needed) {
			ab8500_fg_discharge_state_to(di,
					     AB8500_FG_DISCHARGE_RECOVERY);
		} else {
			ab8500_fg_discharge_state_to(di,
					     AB8500_FG_DISCHARGE_READOUT);

			if (di->high_curr_mode)
				di->high_curr_start = ab8500_fg_get_time();
		}

		queue_work(di->fg_wq, &di->fg_work);
	}

	return 0;
}

static int ab8500_fg_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct ab8500_fg *di = platform_get_drvdata(pdev);
	unsigned long timeout;

	mutex_lock(&di->shutdown_lock);
	if (di->flags.low_bat_delay) {
		mutex_unlock(&di->shutdown_lock);
		return -EAGAIN;
	}
	mutex_unlock(&di->shutdown_lock);

	/* Do not allow suspend when there is any actions to complete */
	if (di->flags.calibrate || di->init_capacity || di->flags.user_cap) {
		dev_info(di->dev,
			 "Job(s) active: Calib: %s, InitCap: %s, UserCap: %s\n",
			 di->flags.calibrate ? "yes" : "no",
			 di->init_capacity ? "yes" : "no",
			 di->flags.user_cap ? "yes" : "no");
		return -EAGAIN;
	}

	/*
	 * In order to prevent from having any new running
	 * jobs, FG has to be disabled first, and right after
	 * clean any running/pending jobs
	 */
	if (!di->flags.charging) {
		/*
		 * We guarantee reading accumulated charge
		 * at least once, between short resume/suspend
		 */
		timeout = wait_for_completion_timeout(&di->accu_done,
			msecs_to_jiffies(500));

		if (timeout == 0)
			dev_err(di->dev,
				"failed to read/write 'Gas Gauge' registers\n");

		(void) ab8500_fg_coulomb_counter(di, 0, false);

		/* FG is recovering in suspend */
		if (di->recovery_needed)
			di->recovery_start = ab8500_fg_get_time();
	}

	/* wait for AVG current */
	flush_workqueue(di->avg_curr_wq);

	if (di->calculate_missed_accu && !di->flags.charging) {
		if (di->missed_accu_charge) {
			ab8500_fg_update_capacity(di, di->missed_accu_charge);
			di->missed_accu_charge = 0;
		}
	}

	/* Make sure FG work has been executed */
	flush_workqueue(di->fg_wq);

	/* Make sure any re-arming work in algorithm is stopped */
	cancel_delayed_work_sync(&di->fg_periodic_work);
	return 0;
}
#else
#define ab8500_fg_suspend      NULL
#define ab8500_fg_resume       NULL
#endif

static int __devexit ab8500_fg_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct ab8500_fg *di = platform_get_drvdata(pdev);

	list_del(&di->node);

	/* Disable coulomb counter */
	ret = ab8500_fg_coulomb_counter(di, 0, false);
	if (ret)
		dev_err(di->dev, "failed to disable coulomb counter\n");

	destroy_workqueue(di->fg_wq);
	destroy_workqueue(di->shutdown_wq);
	ab8500_fg_sysfs_exit(di);

	flush_scheduled_work();
	ab8500_fg_sysfs_psy_remove_attrs(di->fg_psy.dev);
	power_supply_unregister(&di->fg_psy);
	platform_set_drvdata(pdev, NULL);
	kfree(di);
	return ret;
}

/* ab8500 fg driver interrupts and their respective isr */
static struct ab8500_fg_interrupts ab8500_fg_irq[] = {
	{"NCONV_ACCU", ab8500_fg_cc_convend_handler},
	{"BATT_OVV", ab8500_fg_batt_ovv_handler},
	{"LOW_BAT_F", ab8500_fg_lowbatf_handler},
	{"CC_INT_CALIB", ab8500_fg_cc_int_calib_handler},
	{"CCEOC", ab8500_fg_cc_data_end_handler},
};

static int __devinit ab8500_fg_probe(struct platform_device *pdev)
{
	int i, irq;
	struct ab8500_platform_data *plat;
	int samples = 0;
	int ret = 0;

	struct ab8500_fg *di =
		kzalloc(sizeof(struct ab8500_fg), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	mutex_init(&di->cc_lock);
	mutex_init(&di->shutdown_lock);

	/* get parent data */
	di->dev = &pdev->dev;
	di->parent = dev_get_drvdata(pdev->dev.parent);
	di->gpadc = ab8500_gpadc_get();

	plat = dev_get_platdata(di->parent->dev);

	/* get fg specific platform data */
	if (!plat->fg) {
		dev_err(di->dev, "no fg platform data supplied\n");
		ret = -EINVAL;
		goto free_device_info;
	}
	di->pdata = plat->fg;

	/* get battery specific platform data */
	if (!plat->battery) {
		dev_err(di->dev, "no battery platform data supplied\n");
		ret = -EINVAL;
		goto free_device_info;
	}
	di->bat = plat->battery;

	di->fg_psy.name = "ab8500_fg";
	di->fg_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	di->fg_psy.properties = ab8500_fg_props;
	di->fg_psy.num_properties = ARRAY_SIZE(ab8500_fg_props);
	di->fg_psy.get_property = ab8500_fg_get_property;
	di->fg_psy.supplied_to = di->pdata->supplied_to;
	di->fg_psy.num_supplicants = di->pdata->num_supplicants;
	di->fg_psy.external_power_changed = ab8500_fg_external_power_changed;

	di->bat_cap.max_mah_design = MILLI_TO_MICRO *
		di->pdata->ddata->charge_full_design;

	di->bat_cap.max_mah = di->bat_cap.max_mah_design;
	di->vbat_nom = di->bat->bat_type[di->bat->batt_id].nominal_voltage;

	ab8500_fg_update_unusable_permille(di,
					   di->pdata->ddata->lowbat_hysteresis);

	di->init_capacity = true;

	ab8500_fg_charge_state_to(di, AB8500_FG_CHARGE_INIT);
	ab8500_fg_discharge_state_to(di, AB8500_FG_DISCHARGE_INIT);

	/* Create a work queue for running the FG algorithm */
	di->fg_wq = create_singlethread_workqueue("ab8500_fg_wq");
	if (di->fg_wq == NULL) {
		dev_err(di->dev, "failed to create work queue\n");
		goto free_device_info;
	}

	/* Create a work queue for running the shutdown algorithm */
	di->shutdown_wq =
		create_singlethread_workqueue("ab8500_fg_shutdown_wq");
	if (di->shutdown_wq == NULL) {
		dev_err(di->dev, "failed to create work queue\n");
		goto free_fg_wq;
	}

	/* work for average current */
	di->avg_curr_wq =
		create_singlethread_workqueue("ab8500_fg_avg_curr_wq");
	if (di->avg_curr_wq == NULL) {
		dev_err(di->dev, "failed to create work queue\n");
		goto free_shutdown_wq;
	}

	/* Init work for running the fg algorithm instantly */
	INIT_WORK(&di->fg_work, ab8500_fg_instant_work);

	/* Init work for getting the battery accumulated current */
	INIT_WORK(&di->fg_acc_cur_work, ab8500_fg_acc_cur_work);

	/* Init work for reinitialising the fg algorithm */
	INIT_DELAYED_WORK_DEFERRABLE(&di->fg_reinit_work,
		ab8500_fg_reinit_work);

	/* Work delayed Queue to run the state machine */
	INIT_DELAYED_WORK_DEFERRABLE(&di->fg_periodic_work,
		ab8500_fg_periodic_work);

	/* Work to check low battery condition */
	INIT_DELAYED_WORK_DEFERRABLE(&di->fg_low_bat_work,
		ab8500_fg_low_bat_work);

	/* Init work for HW failure check */
	INIT_DELAYED_WORK_DEFERRABLE(&di->fg_check_hw_failure_work,
		ab8500_fg_check_hw_failure_work);

	/* Initialize OVV, and other registers */
	ret = ab8500_fg_init_hw_registers(di);
	if (ret) {
		dev_err(di->dev, "failed to initialize registers\n");
		goto free_shutdown_wq;
	}

	/* Consider battery unknown until we're informed otherwise */
	di->flags.batt_unknown = true;
	di->flags.batt_id_received = false;

	/* Register FG power supply class */
	ret = power_supply_register(di->dev, &di->fg_psy);
	if (ret) {
		dev_err(di->dev, "failed to register FG psy\n");
		goto free_shutdown_wq;
	}

	/*
	 * Initialize completion used to notify completion and start
	 * of inst current
	 */
	init_completion(&di->ab8500_fg_complete);
	init_completion(&di->accu_done);

	/* Register interrupts */
	for (i = 0; i < ARRAY_SIZE(ab8500_fg_irq); i++) {
		irq = platform_get_irq_byname(pdev, ab8500_fg_irq[i].name);
		ret = request_threaded_irq(irq, NULL, ab8500_fg_irq[i].isr,
			IRQF_SHARED | IRQF_NO_SUSPEND,
			ab8500_fg_irq[i].name, di);

		if (ret != 0) {
			dev_err(di->dev, "failed to request %s IRQ %d: %d\n"
				, ab8500_fg_irq[i].name, irq, ret);
			goto free_irq;
		}
		dev_dbg(di->dev, "Requested %s IRQ %d: %d\n",
			ab8500_fg_irq[i].name, irq, ret);
	}
	di->irq = platform_get_irq_byname(pdev, "CCEOC");
	disable_irq(di->irq);

	di->cc_irq = platform_get_irq_byname(pdev, "NCONV_ACCU");
	disable_irq(di->cc_irq);

	samples = SEC_TO_SAMPLE(di->bat->fg_params->accu_high_curr);
	ab8500_fg_coulomb_counter(di, samples, true);

	platform_set_drvdata(pdev, di);

	ret = ab8500_fg_sysfs_init(di);
	if (ret) {
		dev_err(di->dev, "failed to create sysfs entry\n");
		goto free_irq;
	}

	ret = ab8500_fg_sysfs_psy_create_attrs(di->fg_psy.dev);
	if (ret) {
		dev_err(di->dev, "failed to create FG psy\n");
		ab8500_fg_sysfs_exit(di);
		goto free_irq;
	}

	/* Calibrate the fg first time */
	di->flags.calibrate = true;
	di->calib_state = AB8500_FG_CALIB_INIT;

	/* Use room temp as default value until we get an update from driver. */
	di->bat_temp = 210;

	/* Run the FG algorithm */
	queue_delayed_work(di->fg_wq, &di->fg_periodic_work, 0);

	list_add_tail(&di->node, &ab8500_fg_list);

	return ret;

free_irq:
	power_supply_unregister(&di->fg_psy);

	/* We also have to free all successfully registered irqs */
	for (i = i - 1; i >= 0; i--) {
		irq = platform_get_irq_byname(pdev, ab8500_fg_irq[i].name);
		free_irq(irq, di);
	}

	destroy_workqueue(di->avg_curr_wq);
free_shutdown_wq:
	destroy_workqueue(di->shutdown_wq);
free_fg_wq:
	destroy_workqueue(di->fg_wq);
free_device_info:
	kfree(di);

	return ret;
}

static struct platform_driver ab8500_fg_driver = {
	.probe = ab8500_fg_probe,
	.remove = __devexit_p(ab8500_fg_remove),
	.suspend = ab8500_fg_suspend,
	.resume = ab8500_fg_resume,
	.driver = {
		.name = "ab8500-fg",
		.owner = THIS_MODULE,
	},
};

static int __init ab8500_fg_init(void)
{
	return platform_driver_register(&ab8500_fg_driver);
}

static void __exit ab8500_fg_exit(void)
{
	platform_driver_unregister(&ab8500_fg_driver);
}

subsys_initcall_sync(ab8500_fg_init);
module_exit(ab8500_fg_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Johan Palsson, Karl Komierowski");
MODULE_ALIAS("platform:ab8500-fg");
MODULE_DESCRIPTION("AB8500 Fuel Gauge driver");

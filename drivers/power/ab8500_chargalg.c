/*
 * Copyright (C) ST-Ericsson SA 2010
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Charging algorithm driver for AB8500
 *
 * License Terms: GNU General Public License v2
 * Author: Johan Palsson <johan.palsson@stericsson.com>
 * Author: Karl Komierowski <karl.komierowski@stericsson.com>
 * Author: Imre Sunyi <imre.sunyi@sonymobile.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/mfd/ab8500.h>
#include <linux/mfd/abx500/ux500_chargalg.h>
#include <linux/mfd/abx500/ab8500-bm.h>
#include <linux/mfd/abx500/ab8500-gpadc.h>

/* Watchdog kick interval */
#define CHG_WD_INTERVAL			(60 * HZ)

/* End-of-charge criteria counter */
#define EOC_COND_CNT			10

/*
 * temp under over counter is used to not
 * enable charging again if the battery
 * temp is normal but the limit counter
 * is exceeded.
 */
#define MAX_TUO_CNT		3
#define to_ab8500_chargalg_device_info(x) container_of((x), \
	struct ab8500_chargalg, chargalg_psy);

enum ab8500_chargers {
	NO_CHG,
	AC_CHG,
	USB_CHG,
};

struct ab8500_chargalg_charger_info {
	enum ab8500_chargers conn_chg;
	enum ab8500_chargers prev_conn_chg;
	enum ab8500_chargers online_chg;
	enum ab8500_chargers prev_online_chg;
	enum ab8500_chargers charger_type;
	bool usb_chg_ok;
	bool ac_chg_ok;
	int usb_volt;
	int usb_curr;
	int ac_volt;
	int ac_curr;
	int usb_vset;
	int usb_iset;
	int ac_vset;
	int ac_iset;
};

struct ab8500_chargalg_suspension_status {
	bool suspended_change;
	bool ac_suspended;
	bool usb_suspended;
};

struct ab8500_chargalg_battery_data {
	int temp;
	int volt;
	int avg_curr;
	int inst_curr;
	int percent;
};

enum ab8500_chargalg_states {
	STATE_HANDHELD_INIT,
	STATE_HANDHELD,
	STATE_CHG_NOT_OK_INIT,
	STATE_CHG_NOT_OK,
	STATE_HW_TEMP_PROTECT_INIT,
	STATE_HW_TEMP_PROTECT,
	STATE_NORMAL_INIT,
	STATE_NORMAL,
	STATE_WAIT_FOR_RECHARGE_INIT,
	STATE_WAIT_FOR_RECHARGE,
	STATE_MAINTENANCE_A_INIT,
	STATE_MAINTENANCE_A,
	STATE_MAINTENANCE_B_INIT,
	STATE_MAINTENANCE_B,
	STATE_TEMP_UNDEROVER_INIT,
	STATE_TEMP_UNDEROVER,
	STATE_TEMP_LOWHIGH_INIT,
	STATE_TEMP_LOWHIGH,
	STATE_SUSPENDED_INIT,
	STATE_SUSPENDED,
	STATE_OVV_PROTECT_INIT,
	STATE_OVV_PROTECT,
	STATE_SAFETY_TIMER_EXPIRED_INIT,
	STATE_SAFETY_TIMER_EXPIRED,
	STATE_BATT_REMOVED_INIT,
	STATE_BATT_REMOVED,
	STATE_WD_EXPIRED_INIT,
	STATE_WD_EXPIRED,
};

static const char *states[] = {
	"HANDHELD_INIT",
	"HANDHELD",
	"CHG_NOT_OK_INIT",
	"CHG_NOT_OK",
	"HW_TEMP_PROTECT_INIT",
	"HW_TEMP_PROTECT",
	"NORMAL_INIT",
	"NORMAL",
	"WAIT_FOR_RECHARGE_INIT",
	"WAIT_FOR_RECHARGE",
	"MAINTENANCE_A_INIT",
	"MAINTENANCE_A",
	"MAINTENANCE_B_INIT",
	"MAINTENANCE_B",
	"TEMP_UNDEROVER_INIT",
	"TEMP_UNDEROVER",
	"TEMP_LOWHIGH_INIT",
	"TEMP_LOWHIGH",
	"SUSPENDED_INIT",
	"SUSPENDED",
	"OVV_PROTECT_INIT",
	"OVV_PROTECT",
	"SAFETY_TIMER_EXPIRED_INIT",
	"SAFETY_TIMER_EXPIRED",
	"BATT_REMOVED_INIT",
	"BATT_REMOVED",
	"WD_EXPIRED_INIT",
	"WD_EXPIRED",
};

struct ab8500_chargalg_events {
	bool batt_unknown;
	bool mainextchnotok;
	bool batt_ovv;
	bool batt_rem;
	bool btemp_underover;
	bool btemp_lowhigh;
	bool main_thermal_prot;
	bool usb_thermal_prot;
	bool main_ovv;
	bool vbus_ovv;
	bool usbchargernotok;
	bool safety_timer_expired;
	bool maintenance_timer_expired;
	bool ac_wd_expired;
	bool usb_wd_expired;
	bool ac_cv_active;
	bool usb_cv_active;
	bool vbus_collapsed;
	bool force_usb_charging_suspend;
};

/**
 * struct ab8500_charge_curr_maximization - Charger maximization parameters
 * @original_iset:	the non optimized/maximised charger current
 * @current_iset:	the charging current used at this moment
 * @test_delta_i:	the delta between the current we want to charge and the
			current that is really going into the battery
 * @condition_cnt:	number of iterations needed before a new charger current
			is set
 * @max_current:	maximum charger current
 * @wait_cnt:		to avoid too fast current step down in case of charger
 *			voltage collapse, we insert this delay between step
 *			down
 * @level:		tells in how many steps the charging current has been
			increased
 */
struct ab8500_charge_curr_maximization {
	int original_iset;
	int current_iset;
	int test_delta_i;
	int condition_cnt;
	int max_current;
	int wait_cnt;
	u8 level;
};

enum maxim_ret {
	MAXIM_RET_NOACTION,
	MAXIM_RET_CHANGE,
	MAXIM_RET_IBAT_TOO_HIGH,
};

enum maintenance_state {
	MAINT_A,
	MAINT_B,
};

/**
 * struct ab8500_chargalg - ab8500 Charging algorithm device information
 * @dev:		pointer to the structure device
 * @charge_status:	battery operating status
 * @eoc_cnt:		counter used to determine end-of_charge
 * @maintenance_chg:	indicate if maintenance charge is active
 * @maint_state:	indicate what maintenance state we should go to next
 * @t_hyst_norm		temperature hysteresis when the temperature has been
 *			over or under normal limits
 * @t_hyst_lowhigh	temperature hysteresis when the temperature has been
 *			over or under the high or low limits
 * @exceed_temp_cnt not allow charging, if it's exceeded
 * @prev_health:	previous battery health shown
 * @charge_state:	current state of the charging algorithm
 * @ccm			charging current maximization parameters
 * @chg_info:		information about connected charger types
 * @batt_data:		data of the battery
 * @susp_status:	current charger suspension status
 * @parent:		pointer to the struct ab8500
 * @pdata:		pointer to the ab8500_chargalg platform data
 * @bat:		pointer to the ab8500_bm platform data
 * @chargalg_psy:	structure that holds the battery properties exposed by
 *			the charging algorithm
 * @events:		structure for information about events triggered
 * @chargalg_wq:		work queue for running the charging algorithm
 * @chargalg_periodic_work:	work to run the charging algorithm periodically
 * @chargalg_wd_work:		work to kick the charger watchdog periodically
 * @chargalg_work:		work to run the charging algorithm instantly
 * @safety_timer:		charging safety timer
 * @maintenance_timer:		maintenance charging timer
 * @chargalg_kobject:		structure of type kobject
 */
struct ab8500_chargalg {
	struct device *dev;
	int charge_status;
	int eoc_cnt;
	bool maintenance_chg;
	enum maintenance_state maint_state;
	int t_hyst_norm;
	int t_hyst_lowhigh;
	int exceed_temp_cnt;
	int prev_health;
	enum ab8500_chargalg_states charge_state;
	struct ab8500_charge_curr_maximization ccm;
	struct ab8500_chargalg_charger_info chg_info;
	struct ab8500_chargalg_battery_data batt_data;
	struct ab8500_chargalg_suspension_status susp_status;
	struct ab8500 *parent;
	struct ab8500_chargalg_platform_data *pdata;
	struct ab8500_bm_data *bat;
	struct power_supply chargalg_psy;
	struct ux500_charger *ac_chg;
	struct ux500_charger *usb_chg;
	struct ab8500_chargalg_events events;
	struct workqueue_struct *chargalg_wq;
	struct delayed_work chargalg_periodic_work;
	struct delayed_work chargalg_wd_work;
	struct work_struct chargalg_work;
	struct hrtimer safety_timer;
	struct hrtimer maintenance_timer;
	struct kobject chargalg_kobject;
};

/* Main battery properties */
static enum power_supply_property ab8500_chargalg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
};

/**
 * ab8500_chargalg_safety_timer_expired() - Expiration of the safety timer
 * @timer:	pointer to the hrtimer structure
 *
 * This function gets called when the safety timer for the charger
 * expires
 */
static enum hrtimer_restart
ab8500_chargalg_safety_timer_expired(struct hrtimer *timer)
{
	struct ab8500_chargalg *di = container_of(timer, struct ab8500_chargalg,
						  safety_timer);
	dev_err(di->dev, "Safety timer expired\n");
	di->events.safety_timer_expired = true;

	/* Trigger execution of the algorithm instantly */
	queue_work(di->chargalg_wq, &di->chargalg_work);

	return HRTIMER_NORESTART;
}

/**
 * ab8500_chargalg_maintenance_timer_expired() - Expiration of
 * the maintenance timer
 * @timer:	pointer to the timer structure
 *
 * This function gets called when the maintenence timer
 * expires
 */
static enum hrtimer_restart
ab8500_chargalg_maintenance_timer_expired(struct hrtimer *timer)
{
	struct ab8500_chargalg *di = container_of(timer, struct ab8500_chargalg,
						  maintenance_timer);
	dev_dbg(di->dev, "Maintenance timer expired\n");
	di->events.maintenance_timer_expired = true;

	/* Trigger execution of the algorithm instantly */
	queue_work(di->chargalg_wq, &di->chargalg_work);

	return HRTIMER_NORESTART;
}

/**
 * ab8500_chargalg_state_to() - Change charge state
 * @di:		pointer to the ab8500_chargalg structure
 *
 * This function gets called when a charge state change should occur
 */
static void ab8500_chargalg_state_to(struct ab8500_chargalg *di,
	enum ab8500_chargalg_states state)
{
	dev_dbg(di->dev,
		"State changed: %s (From state: [%d] %s =to=> [%d] %s )\n",
		di->charge_state == state ? "NO" : "YES",
		di->charge_state,
		states[di->charge_state],
		state,
		states[state]);

	di->charge_state = state;
}

/**
 * ab8500_chargalg_check_charger_connection() - Check charger connection change
 * @di:		pointer to the ab8500_chargalg structure
 *
 * This function will check if there is a change in the charger connection
 * and change charge state accordingly. AC has precedence over USB.
 */
static int ab8500_chargalg_check_charger_connection(struct ab8500_chargalg *di)
{
	if (di->chg_info.conn_chg != di->chg_info.prev_conn_chg ||
		di->susp_status.suspended_change) {
		/*
		 * Charger state changed or suspension
		 * has changed since last update
		 */
		if ((di->chg_info.conn_chg & AC_CHG) &&
			!di->susp_status.ac_suspended) {
			dev_dbg(di->dev, "Charging source is AC\n");
			if (di->chg_info.charger_type != AC_CHG) {
				di->chg_info.charger_type = AC_CHG;
				ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
			}
		} else if ((di->chg_info.conn_chg & USB_CHG) &&
			!(di->susp_status.usb_suspended ||
			di->events.force_usb_charging_suspend)) {
			dev_dbg(di->dev, "Charging source is USB\n");
			di->chg_info.charger_type = USB_CHG;
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		} else if (di->chg_info.conn_chg &&
			(di->susp_status.ac_suspended ||
			di->susp_status.usb_suspended ||
			di->events.force_usb_charging_suspend)) {
			dev_dbg(di->dev, "Charging is suspended\n");
			di->chg_info.charger_type = NO_CHG;
			ab8500_chargalg_state_to(di, STATE_SUSPENDED_INIT);
		} else {
			dev_dbg(di->dev, "Charging source is OFF\n");
			di->chg_info.charger_type = NO_CHG;
			ab8500_chargalg_state_to(di, STATE_HANDHELD_INIT);

			/*
			 * reset exceeded battery temperature counter
			 * when charge cycle is restarted, i.e. when
			 * USB unplug is happened (physical).
			 */
			di->exceed_temp_cnt = 0;
		}
		di->chg_info.prev_conn_chg = di->chg_info.conn_chg;
		di->susp_status.suspended_change = false;
	}
	return di->chg_info.conn_chg;
}

/**
 * ab8500_chargalg_start_safety_timer() - Start charging safety timer
 * @di:		pointer to the ab8500_chargalg structure
 *
 * The safety timer is used to avoid overcharging of old or bad batteries.
 * There are different timers for AC and USB
 */
static void ab8500_chargalg_start_safety_timer(struct ab8500_chargalg *di)
{
	int timer_expiration = 0;
	ktime_t delta = ktime_set(5 * 60, 0);
	ktime_t expiration;

	switch (di->chg_info.charger_type) {
	case AC_CHG:
		timer_expiration = di->bat->main_safety_tmr_h;
		break;

	case USB_CHG:
		timer_expiration = di->bat->usb_safety_tmr_h;
		break;

	default:
		dev_err(di->dev, "Unknown charger to charge from\n");
		break;
	}

	di->events.safety_timer_expired = false;
	expiration = ktime_set(timer_expiration * 3600, 0);
	hrtimer_set_expires_range(&di->safety_timer, expiration, delta);
	hrtimer_start_expires(&di->safety_timer, HRTIMER_MODE_REL);
}

/**
 * ab8500_chargalg_stop_safety_timer() - Stop charging safety timer
 * @di:		pointer to the ab8500_chargalg structure
 *
 * The safety timer is stopped whenever the NORMAL state is exited
 */
static void ab8500_chargalg_stop_safety_timer(struct ab8500_chargalg *di)
{
	if (hrtimer_try_to_cancel(&di->safety_timer) >= 0)
		di->events.safety_timer_expired = false;
}

/**
 * ab8500_chargalg_start_maintenance_timer() - Start charging maintenance timer
 * @di:		pointer to the ab8500_chargalg structure
 * @duration:	duration of ther maintenance timer in hours
 *
 * The maintenance timer is used to maintain the charge in the battery once
 * the battery is considered full. These timers are chosen to match the
 * discharge curve of the battery
 */
static void ab8500_chargalg_start_maintenance_timer(struct ab8500_chargalg *di,
	int duration)
{
	ktime_t timer_expiration = ktime_set(duration * 3600, 0);
	ktime_t delta = ktime_set(5 * 60, 0);
	hrtimer_set_expires_range(&di->maintenance_timer, timer_expiration,
				  delta);
	di->events.maintenance_timer_expired = false;
	hrtimer_start_expires(&di->maintenance_timer, HRTIMER_MODE_REL);
}

/**
 * ab8500_chargalg_stop_maintenance_timer() - Stop maintenance timer
 * @di:		pointer to the ab8500_chargalg structure
 *
 * The maintenance timer is stopped whenever maintenance ends or when another
 * state is entered
 */
static void ab8500_chargalg_stop_maintenance_timer(struct ab8500_chargalg *di)
{
	if (hrtimer_try_to_cancel(&di->maintenance_timer) >= 0)
		di->events.maintenance_timer_expired = false;
}

/**
 * ab8500_chargalg_kick_watchdog() - Kick charger watchdog
 * @di:		pointer to the ab8500_chargalg structure
 *
 * The charger watchdog have to be kicked periodically whenever the charger is
 * on, else the ABB will reset the system
 */
static int ab8500_chargalg_kick_watchdog(struct ab8500_chargalg *di)
{
	/* Check if charger exists and kick watchdog if charging */
	if (di->ac_chg && di->ac_chg->ops.kick_wd &&
			di->chg_info.online_chg & AC_CHG)
		return di->ac_chg->ops.kick_wd(di->ac_chg);
	else if (di->usb_chg && di->usb_chg->ops.kick_wd &&
			di->chg_info.online_chg & USB_CHG)
		return di->usb_chg->ops.kick_wd(di->usb_chg);

	return -ENXIO;
}

/**
 * ab8500_chargalg_ac_en() - Turn on/off the AC charger
 * @di:		pointer to the ab8500_chargalg structure
 * @enable:	charger on/off
 * @vset:	requested charger output voltage
 * @iset:	requested charger output current
 *
 * The AC charger will be turned on/off with the requested charge voltage and
 * current
 */
static int ab8500_chargalg_ac_en(struct ab8500_chargalg *di, int enable,
	int vset, int iset)
{
	if (!di->ac_chg || !di->ac_chg->ops.enable)
		return -ENXIO;

	/* Select maximum of what both the charger and the battery supports */
	if (di->ac_chg->max_out_volt)
		vset = min(vset, di->ac_chg->max_out_volt);
	if (di->ac_chg->max_out_curr)
		iset = min(iset, di->ac_chg->max_out_curr);

	di->chg_info.ac_iset = iset;
	di->chg_info.ac_vset = vset;

	return di->ac_chg->ops.enable(di->ac_chg, enable, vset, iset);
}

/**
 * ab8500_chargalg_usb_en() - Turn on/off the USB charger
 * @di:		pointer to the ab8500_chargalg structure
 * @enable:	charger on/off
 * @vset:	requested charger output voltage
 * @iset:	requested charger output current
 *
 * The USB charger will be turned on/off with the requested charge voltage and
 * current
 */
static int ab8500_chargalg_usb_en(struct ab8500_chargalg *di, int enable,
	int vset, int iset)
{
	if (!di->usb_chg || !di->usb_chg->ops.enable)
		return -ENXIO;

	/* Select maximum of what both the charger and the battery supports */
	if (di->usb_chg->max_out_volt)
		vset = min(vset, di->usb_chg->max_out_volt);
	if (di->usb_chg->max_out_curr)
		iset = min(iset, di->usb_chg->max_out_curr);

	di->chg_info.usb_iset = iset;
	di->chg_info.usb_vset = vset;

	return di->usb_chg->ops.enable(di->usb_chg, enable, vset, iset);
}

/**
 * ab8500_chargalg_update_chg_curr() - Update charger current
 * @di:		pointer to the ab8500_chargalg structure
 * @iset:	requested charger output current
 *
 * The charger output current will be updated for the charger
 * that is currently in use
 */
static int ab8500_chargalg_update_chg_curr(struct ab8500_chargalg *di,
		int iset)
{
	/* Check if charger exists and update current if charging */
	if (di->ac_chg && di->ac_chg->ops.update_curr &&
			di->chg_info.charger_type & AC_CHG) {
		/*
		 * Select maximum of what both the charger
		 * and the battery supports
		 */
		if (di->ac_chg->max_out_curr)
			iset = min(iset, di->ac_chg->max_out_curr);

		di->chg_info.ac_iset = iset;

		return di->ac_chg->ops.update_curr(di->ac_chg, iset);
	} else if (di->usb_chg && di->usb_chg->ops.update_curr &&
			di->chg_info.charger_type & USB_CHG) {
		/*
		 * Select maximum of what both the charger
		 * and the battery supports
		 */
		if (di->usb_chg->max_out_curr)
			iset = min(iset, di->usb_chg->max_out_curr);

		di->chg_info.usb_iset = iset;

		return di->usb_chg->ops.update_curr(di->usb_chg, iset);
	}

	return -ENXIO;
}

/**
 * ab8500_chargalg_stop_charging() - Stop charging
 * @di:		pointer to the ab8500_chargalg structure
 *
 * This function is called from any state where charging should be stopped.
 * All charging is disabled and all status parameters and timers are changed
 * accordingly
 */
static void ab8500_chargalg_stop_charging(struct ab8500_chargalg *di)
{
	ab8500_chargalg_ac_en(di, false, 0, 0);
	ab8500_chargalg_usb_en(di, false, 0, 0);
	ab8500_chargalg_stop_safety_timer(di);
	ab8500_chargalg_stop_maintenance_timer(di);
	di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	di->maintenance_chg = false;
	cancel_delayed_work(&di->chargalg_wd_work);
	power_supply_changed(&di->chargalg_psy);
}

/**
 * ab8500_chargalg_hold_charging() - Pauses charging
 * @di:		pointer to the ab8500_chargalg structure
 *
 * This function is called in the case where maintenance charging has been
 * disabled and instead a battery voltage mode is entered to check when the
 * battery voltage has reached a certain recharge voltage
 */
static void ab8500_chargalg_hold_charging(struct ab8500_chargalg *di)
{
	ab8500_chargalg_ac_en(di, false, 0, 0);
	ab8500_chargalg_usb_en(di, false, 0, 0);
	ab8500_chargalg_stop_safety_timer(di);
	ab8500_chargalg_stop_maintenance_timer(di);
	di->charge_status = POWER_SUPPLY_STATUS_FULL;
	di->maintenance_chg = false;
	cancel_delayed_work(&di->chargalg_wd_work);
	power_supply_changed(&di->chargalg_psy);
}

/**
 * ab8500_chargalg_start_charging() - Start the charger
 * @di:		pointer to the ab8500_chargalg structure
 * @vset:	requested charger output voltage
 * @iset:	requested charger output current
 *
 * A charger will be enabled depending on the requested charger type that was
 * detected previously.
 */
static void ab8500_chargalg_start_charging(struct ab8500_chargalg *di,
	int vset, int iset)
{
	bool start_chargalg_wd = true;

	switch (di->chg_info.charger_type) {
	case AC_CHG:
		dev_dbg(di->dev,
			"AC parameters: Vset %d, Ich %d\n", vset, iset);
		ab8500_chargalg_ac_en(di, true, vset, iset);
		break;

	case USB_CHG:
		dev_dbg(di->dev,
			"USB parameters: Vset %d, Ich %d\n", vset, iset);
		ab8500_chargalg_usb_en(di, true, vset, iset);
		break;

	default:
		dev_err(di->dev, "Unknown charger to charge from\n");
		start_chargalg_wd = false;
		break;
	}

	if (start_chargalg_wd && !delayed_work_pending(&di->chargalg_wd_work))
		queue_delayed_work(di->chargalg_wq, &di->chargalg_wd_work, 0);
}

/**
 * ab8500_chargalg_check_temp() - Check battery temperature ranges
 * @di:		pointer to the ab8500_chargalg structure
 *
 * The battery temperature is checked against the predefined limits and the
 * charge state is changed accordingly
 */
static void ab8500_chargalg_check_temp(struct ab8500_chargalg *di)
{
	/* Not allowed to change temperature range if exceeded counter has
	 * passed the allowed limit.
	 */
	if (di->exceed_temp_cnt >= MAX_TUO_CNT)
		return;

	if (di->batt_data.temp > (di->bat->temp_low + di->t_hyst_norm) &&
		di->batt_data.temp < (di->bat->temp_high - di->t_hyst_norm)) {
		/* Temp OK! */
		di->events.btemp_underover = false;
		di->events.btemp_lowhigh = false;
		di->t_hyst_norm = 0;
		di->t_hyst_lowhigh = 0;
	} else {
		if (((di->batt_data.temp >= di->bat->temp_high) &&
			(di->batt_data.temp <
				(di->bat->temp_over - di->t_hyst_lowhigh))) ||
			((di->batt_data.temp >
				(di->bat->temp_under + di->t_hyst_lowhigh)) &&
			(di->batt_data.temp <= di->bat->temp_low))) {
			/* TEMP minor!!!!! */
			di->events.btemp_underover = false;
			di->events.btemp_lowhigh = true;
			di->t_hyst_norm = di->bat->temp_hysteresis;
			di->t_hyst_lowhigh = 0;
		} else if (di->batt_data.temp <= di->bat->temp_under ||
			di->batt_data.temp >= di->bat->temp_over) {
			/* TEMP major!!!!! */
			di->events.btemp_underover = true;
			di->events.btemp_lowhigh = false;
			di->t_hyst_norm = 0;
			di->t_hyst_lowhigh = di->bat->temp_hysteresis;
		} else {
		/* Within hysteresis */
		dev_dbg(di->dev, "Within hysteresis limit temp: %d "
				"hyst_lowhigh %d, hyst normal %d\n",
				di->batt_data.temp, di->t_hyst_lowhigh,
				di->t_hyst_norm);
		}
	}
}

/**
 * ab8500_chargalg_check_charger_voltage() - Check charger voltage
 * @di:		pointer to the ab8500_chargalg structure
 *
 * Charger voltage is checked against maximum limit
 */
static void ab8500_chargalg_check_charger_voltage(struct ab8500_chargalg *di)
{
	if (di->chg_info.usb_volt > di->bat->chg_params->usb_volt_max)
		di->chg_info.usb_chg_ok = false;
	else
		di->chg_info.usb_chg_ok = true;

	if (di->chg_info.ac_volt > di->bat->chg_params->ac_volt_max)
		di->chg_info.ac_chg_ok = false;
	else
		di->chg_info.ac_chg_ok = true;

}

/**
 * ab8500_chargalg_end_of_charge() - Check if end-of-charge criteria is fulfilled
 * @di:		pointer to the ab8500_chargalg structure
 *
 * End-of-charge criteria is fulfilled when the battery voltage is above a
 * certain limit and the battery current is below a certain limit for a
 * predefined number of consecutive seconds. If true, the battery is full
 */
static void ab8500_chargalg_end_of_charge(struct ab8500_chargalg *di)
{
	int termination_curr = (di->events.batt_unknown) ?
		di->bat->bat_type[di->bat->batt_id].termination_curr :
		di->pdata->ddata->termination_curr;

	if (di->charge_status == POWER_SUPPLY_STATUS_CHARGING &&
		di->charge_state == STATE_NORMAL &&
		!di->maintenance_chg &&	(di->batt_data.volt >=
		di->bat->bat_type[di->bat->batt_id].termination_vol ||
		di->events.usb_cv_active || di->events.ac_cv_active) &&
		di->batt_data.avg_curr < termination_curr &&
		di->batt_data.avg_curr > 0) {
		if (++di->eoc_cnt >= EOC_COND_CNT) {
			di->eoc_cnt = 0;
			di->charge_status = POWER_SUPPLY_STATUS_FULL;
			di->maintenance_chg = true;
			dev_dbg(di->dev, "EOC reached!\n");
			power_supply_changed(&di->chargalg_psy);
		} else {
			dev_dbg(di->dev,
				" EOC limit reached for the %d"
				" time, out of %d before EOC\n",
				di->eoc_cnt,
				EOC_COND_CNT);
		}
	} else {
		di->eoc_cnt = 0;
	}
}

static void init_maxim_chg_curr(struct ab8500_chargalg *di)
{
	int max_curr = di->events.batt_unknown ?
		di->bat->bat_type[di->bat->batt_id].normal_cur_lvl :
		di->pdata->ddata->normal_cur_lvl;

	di->ccm.original_iset = max_curr;
	di->ccm.current_iset = max_curr;

	di->ccm.test_delta_i = di->bat->maxi->charger_curr_step;
	di->ccm.max_current = di->bat->maxi->chg_curr;
	di->ccm.condition_cnt = di->bat->maxi->wait_cycles;
	di->ccm.level = 0;
}

/**
 * ab8500_chargalg_chg_curr_maxim - increases the charger current to
 *			compensate for the system load
 * @di		pointer to the ab8500_chargalg structure
 *
 * This maximization function is used to raise the charger current to get the
 * battery current as close to the optimal value as possible. The battery
 * current during charging is affected by the system load
 */
static enum maxim_ret ab8500_chargalg_chg_curr_maxim(struct ab8500_chargalg *di)
{
	int delta_i;

	if (!di->bat->maxi->ena_maxi)
		return MAXIM_RET_NOACTION;

	delta_i = di->ccm.original_iset - di->batt_data.inst_curr;

	if (di->events.vbus_collapsed) {
		dev_dbg(di->dev, "Charger voltage has collapsed %d\n",
				di->ccm.wait_cnt);
		if (di->ccm.wait_cnt == 0) {
			dev_dbg(di->dev, "lowering current\n");
			di->ccm.wait_cnt++;
			di->ccm.condition_cnt = di->bat->maxi->wait_cycles;
			di->ccm.max_current =
				di->ccm.current_iset - di->ccm.test_delta_i;
			di->ccm.current_iset = di->ccm.max_current;
			di->ccm.level--;
			return MAXIM_RET_CHANGE;
		} else {
			dev_dbg(di->dev, "waiting\n");
			/* Let's go in here twice before lowering curr again */
			di->ccm.wait_cnt = (di->ccm.wait_cnt + 1) % 3;
			return MAXIM_RET_NOACTION;
		}
	}

	di->ccm.wait_cnt = 0;

	if ((di->batt_data.inst_curr > di->ccm.original_iset)) {
		dev_dbg(di->dev, " Maximization Ibat (%dmA) too high"
			" (limit %dmA) (current iset: %dmA)!\n",
			di->batt_data.inst_curr, di->ccm.original_iset,
			di->ccm.current_iset);

		if (di->ccm.current_iset == di->ccm.original_iset)
			return MAXIM_RET_NOACTION;

		di->ccm.condition_cnt =	di->bat->maxi->wait_cycles;
		di->ccm.current_iset = di->ccm.original_iset;
		di->ccm.level = 0;

		return MAXIM_RET_IBAT_TOO_HIGH;
	}

	if (delta_i > di->ccm.test_delta_i &&
		(di->ccm.current_iset + di->ccm.test_delta_i) <
		di->ccm.max_current) {
		if (di->ccm.condition_cnt-- == 0) {
			/* Increse the iset with cco.test_delta_i */
			di->ccm.condition_cnt =	di->bat->maxi->wait_cycles;
			di->ccm.current_iset += di->ccm.test_delta_i;
			di->ccm.level++;
			dev_dbg(di->dev, " Maximization needed, increase"
				" with %d mA to %dmA (Optimal ibat: %d)"
				" Level %d\n",
				di->ccm.test_delta_i,
				di->ccm.current_iset,
				di->ccm.original_iset,
				di->ccm.level);
			return MAXIM_RET_CHANGE;
		} else {
			return MAXIM_RET_NOACTION;
		}
	}  else {
		di->ccm.condition_cnt =	di->bat->maxi->wait_cycles;
		return MAXIM_RET_NOACTION;
	}
}

static void handle_maxim_chg_curr(struct ab8500_chargalg *di)
{
	enum maxim_ret ret;
	int result;

	ret = ab8500_chargalg_chg_curr_maxim(di);
	switch (ret) {
	case MAXIM_RET_CHANGE:
		result = ab8500_chargalg_update_chg_curr(di,
			di->ccm.current_iset);
		if (result)
			dev_err(di->dev, "failed to set chg curr\n");
		break;
	case MAXIM_RET_IBAT_TOO_HIGH:
		result = ab8500_chargalg_update_chg_curr(di,
			(di->events.batt_unknown) ?
			di->bat->bat_type[di->bat->batt_id].normal_cur_lvl :
			di->pdata->ddata->normal_cur_lvl);
		if (result)
			dev_err(di->dev, "failed to set chg curr\n");
		break;

	case MAXIM_RET_NOACTION:
	default:
		/* Do nothing..*/
		break;
	}
}

static void ab8500_chargalg_check_safety_timer(struct ab8500_chargalg *di)
{
	/*
	 * The safety timer will not be started until the capacity reported
	 * from the FG algorithm is 100%. Then we know that the amount of
	 * charge that's gone into the battery is enough for the battery
	 * to be full. If it has not reached end-of-charge before the safety
	 * timer has expired then we know that the battery is overcharged
	 * and charging will be stopped to protect the battery.
	 */
	if (di->batt_data.percent == 100 &&
	    !hrtimer_active(&di->safety_timer)) {
		ab8500_chargalg_start_safety_timer(di);
		dev_dbg(di->dev, "start safety timer\n");
	} else if (di->batt_data.percent != 100 &&
		    hrtimer_active(&di->safety_timer)) {
		ab8500_chargalg_stop_safety_timer(di);
		dev_dbg(di->dev, "stop safety timer\n");
	}
}

static int ab8500_chargalg_get_ext_psy_data(struct device *dev, void *data)
{
	struct power_supply *psy;
	struct power_supply *ext;
	struct ab8500_chargalg *di;
	union power_supply_propval ret;
	int i, j;
	bool psy_found = false;
	bool capacity_updated = false;

	psy = (struct power_supply *)data;
	ext = dev_get_drvdata(dev);
	di = to_ab8500_chargalg_device_info(psy);

	/* For all psy where the driver name appears in any supplied_to */
	for (i = 0; i < ext->num_supplicants; i++) {
		if (!strcmp(ext->supplied_to[i], psy->name))
			psy_found = true;
	}

	if (!psy_found)
		return 0;

	/*
	 *  If external is not registering 'POWER_SUPPLY_PROP_CAPACITY' to its
	 * property because of handling that sysfs entry on its own, this is
	 * the place to get the battery capacity.
	 */
	if (!ext->get_property(ext, POWER_SUPPLY_PROP_CAPACITY, &ret)) {
		di->batt_data.percent = ret.intval;
		capacity_updated = true;
	}

	/* Go through all properties for the psy */
	for (j = 0; j < ext->num_properties; j++) {
		enum power_supply_property prop;
		prop = ext->properties[j];

		/* Initialize chargers if not already done */
		if (!di->ac_chg &&
			ext->type == POWER_SUPPLY_TYPE_MAINS)
			di->ac_chg = psy_to_ux500_charger(ext);
		else if (!di->usb_chg &&
			ext->type == POWER_SUPPLY_TYPE_USB)
			di->usb_chg = psy_to_ux500_charger(ext);

		if (ext->get_property(ext, prop, &ret))
			continue;

		switch (prop) {
		case POWER_SUPPLY_PROP_PRESENT:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				/* Battery present */
				if (ret.intval)
					di->events.batt_rem = false;
				/* Battery removed */
				else
					di->events.batt_rem = true;
				break;
			case POWER_SUPPLY_TYPE_MAINS:
				/* AC disconnected */
				if (!ret.intval &&
					(di->chg_info.conn_chg & AC_CHG)) {
					di->chg_info.prev_conn_chg =
						di->chg_info.conn_chg;
					di->chg_info.conn_chg &= ~AC_CHG;
				}
				/* AC connected */
				else if (ret.intval &&
					!(di->chg_info.conn_chg & AC_CHG)) {
					di->chg_info.prev_conn_chg =
						di->chg_info.conn_chg;
					di->chg_info.conn_chg |= AC_CHG;
				}
				break;
			case POWER_SUPPLY_TYPE_USB:
				/* USB disconnected */
				if (!ret.intval &&
					(di->chg_info.conn_chg & USB_CHG)) {
					di->chg_info.prev_conn_chg =
						di->chg_info.conn_chg;
					di->chg_info.conn_chg &= ~USB_CHG;
				}
				/* USB connected */
				else if (ret.intval &&
					!(di->chg_info.conn_chg & USB_CHG)) {
					di->chg_info.prev_conn_chg =
						di->chg_info.conn_chg;
					di->chg_info.conn_chg |= USB_CHG;
				}
				break;
			default:
				break;
			}
			break;

		case POWER_SUPPLY_PROP_ONLINE:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				break;
			case POWER_SUPPLY_TYPE_MAINS:
				/* AC offline */
				if (!ret.intval &&
					(di->chg_info.online_chg & AC_CHG)) {
					di->chg_info.prev_online_chg =
						di->chg_info.online_chg;
					di->chg_info.online_chg &= ~AC_CHG;
				}
				/* AC online */
				else if (ret.intval &&
					!(di->chg_info.online_chg & AC_CHG)) {
					di->chg_info.prev_online_chg =
						di->chg_info.online_chg;
					di->chg_info.online_chg |= AC_CHG;
					queue_delayed_work(di->chargalg_wq,
						&di->chargalg_wd_work, 0);
				}
				break;
			case POWER_SUPPLY_TYPE_USB:
				/* USB offline */
				if (!ret.intval &&
					(di->chg_info.online_chg & USB_CHG)) {
					di->chg_info.prev_online_chg =
						di->chg_info.online_chg;
					di->chg_info.online_chg &= ~USB_CHG;
				}
				/* USB online */
				else if (ret.intval &&
					!(di->chg_info.online_chg & USB_CHG)) {
					di->chg_info.prev_online_chg =
						di->chg_info.online_chg;
					di->chg_info.online_chg |= USB_CHG;
					queue_delayed_work(di->chargalg_wq,
						&di->chargalg_wd_work, 0);
				}
				break;
			default:
				break;
			}
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				break;
			case POWER_SUPPLY_TYPE_MAINS:
				switch (ret.intval) {
				case POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
					di->events.mainextchnotok = true;
					di->events.main_thermal_prot = false;
					di->events.main_ovv = false;
					di->events.ac_wd_expired = false;
					break;
				case POWER_SUPPLY_HEALTH_DEAD:
					di->events.ac_wd_expired = true;
					di->events.mainextchnotok = false;
					di->events.main_ovv = false;
					di->events.main_thermal_prot = false;
					break;
				case POWER_SUPPLY_HEALTH_COLD:
				case POWER_SUPPLY_HEALTH_OVERHEAT:
					di->events.main_thermal_prot = true;
					di->events.mainextchnotok = false;
					di->events.main_ovv = false;
					di->events.ac_wd_expired = false;
					break;
				case POWER_SUPPLY_HEALTH_OVERVOLTAGE:
					di->events.main_ovv = true;
					di->events.mainextchnotok = false;
					di->events.main_thermal_prot = false;
					di->events.ac_wd_expired = false;
					break;
				case POWER_SUPPLY_HEALTH_GOOD:
					di->events.main_thermal_prot = false;
					di->events.mainextchnotok = false;
					di->events.main_ovv = false;
					di->events.ac_wd_expired = false;
					break;
				default:
					break;
				}
				break;

			case POWER_SUPPLY_TYPE_USB:
				switch (ret.intval) {
				case POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
					di->events.usbchargernotok = true;
					di->events.usb_thermal_prot = false;
					di->events.vbus_ovv = false;
					di->events.usb_wd_expired = false;
					break;
				case POWER_SUPPLY_HEALTH_DEAD:
					di->events.usb_wd_expired = true;
					di->events.usbchargernotok = false;
					di->events.usb_thermal_prot = false;
					di->events.vbus_ovv = false;
					break;
				case POWER_SUPPLY_HEALTH_COLD:
				case POWER_SUPPLY_HEALTH_OVERHEAT:
					di->events.usb_thermal_prot = true;
					di->events.usbchargernotok = false;
					di->events.vbus_ovv = false;
					di->events.usb_wd_expired = false;
					break;
				case POWER_SUPPLY_HEALTH_OVERVOLTAGE:
					di->events.vbus_ovv = true;
					di->events.usbchargernotok = false;
					di->events.usb_thermal_prot = false;
					di->events.usb_wd_expired = false;
					break;
				case POWER_SUPPLY_HEALTH_GOOD:
					di->events.usbchargernotok = false;
					di->events.usb_thermal_prot = false;
					di->events.vbus_ovv = false;
					di->events.usb_wd_expired = false;
					break;
				case POWER_SUPPLY_HEALTH_UNKNOWN:
				if (!di->events.force_usb_charging_suspend) {
					di->susp_status.suspended_change = true;
					di->events.force_usb_charging_suspend =
						true;
				}
					break;
				default:
					break;
				}

				if (ret.intval != POWER_SUPPLY_HEALTH_UNKNOWN &&
					di->events.force_usb_charging_suspend) {
					di->events.force_usb_charging_suspend =
						false;
					di->susp_status.suspended_change = true;
				}
			default:
				break;
			}
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				di->batt_data.volt = ret.intval / 1000;
				if (di->batt_data.volt >= BATT_OVV_VALUE)
					di->events.batt_ovv = true;
				else
					di->events.batt_ovv = false;
				break;
			case POWER_SUPPLY_TYPE_MAINS:
				di->chg_info.ac_volt = ret.intval / 1000;
				break;
			case POWER_SUPPLY_TYPE_USB:
				di->chg_info.usb_volt = ret.intval / 1000;
				break;
			default:
				break;
			}
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_AVG:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_MAINS:
				/* AVG is used to indicate when we are
				 * in CV mode */
				if (ret.intval)
					di->events.ac_cv_active = true;
				else
					di->events.ac_cv_active = false;

				break;
			case POWER_SUPPLY_TYPE_USB:
				/* AVG is used to indicate when we are
				 * in CV mode */
				if (ret.intval)
					di->events.usb_cv_active = true;
				else
					di->events.usb_cv_active = false;

				break;
			default:
				break;
			}
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				if (ret.intval)
					di->events.batt_unknown = false;
				else
					di->events.batt_unknown = true;

				break;
			default:
				break;
			}
			break;

		case POWER_SUPPLY_PROP_TEMP:
			di->batt_data.temp = ret.intval / 10;
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_MAINS:
					di->chg_info.ac_curr =
						ret.intval / 1000;
					break;
			case POWER_SUPPLY_TYPE_USB:
					di->chg_info.usb_curr =
						ret.intval / 1000;
				break;
			case POWER_SUPPLY_TYPE_BATTERY:
				di->batt_data.inst_curr = ret.intval / 1000;
				break;
			default:
				break;
			}
			break;

		case POWER_SUPPLY_PROP_CURRENT_AVG:
			switch (ext->type) {
			case POWER_SUPPLY_TYPE_BATTERY:
				di->batt_data.avg_curr = ret.intval / 1000;
				break;
			case POWER_SUPPLY_TYPE_USB:
				if (ret.intval)
					di->events.vbus_collapsed = true;
				else
					di->events.vbus_collapsed = false;
				break;
			default:
				break;
			}
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			if (!capacity_updated)
				di->batt_data.percent = ret.intval;
			break;
		default:
			break;
		}
	}
	return 0;
}

/**
 * ab8500_chargalg_external_power_changed() - callback for power supply changes
 * @psy:       pointer to the structure power_supply
 *
 * This function is the entry point of the pointer external_power_changed
 * of the structure power_supply.
 * This function gets executed when there is a change in any external power
 * supply that this driver needs to be notified of.
 */
static void ab8500_chargalg_external_power_changed(struct power_supply *psy)
{
	struct ab8500_chargalg *di = to_ab8500_chargalg_device_info(psy);

	/*
	 * Trigger execution of the algorithm instantly and read
	 * all power_supply properties there instead
	 */
	queue_work(di->chargalg_wq, &di->chargalg_work);
}

/**
 * ab8500_chargalg_algorithm() - Main function for the algorithm
 * @di:		pointer to the ab8500_chargalg structure
 *
 * This is the main control function for the charging algorithm.
 * It is called periodically or when something happens that will
 * trigger a state change
 */
static void ab8500_chargalg_algorithm(struct ab8500_chargalg *di)
{
	int charger_status;

	/* Collect data from all power_supply class devices */
	class_for_each_device(power_supply_class, NULL,
		&di->chargalg_psy, ab8500_chargalg_get_ext_psy_data);

	ab8500_chargalg_end_of_charge(di);
	ab8500_chargalg_check_temp(di);
	ab8500_chargalg_check_charger_voltage(di);
	charger_status = ab8500_chargalg_check_charger_connection(di);

	/*
	 * First check if we have a charger connected.
	 * Also we don't allow charging of unknown batteries if configured
	 * this way
	 */
	if (!charger_status ||
		(di->events.batt_unknown && !di->bat->chg_unknown_bat)) {
		if (di->charge_state != STATE_HANDHELD) {
			di->events.safety_timer_expired = false;
			ab8500_chargalg_state_to(di, STATE_HANDHELD_INIT);
		}
	}

	/* If suspended, we should not continue checking the flags */
	else if (di->charge_state == STATE_SUSPENDED_INIT ||
		di->charge_state == STATE_SUSPENDED) {
		/* We don't do anything here, just don,t continue */
	}

	/* Safety timer expiration */
	else if (di->events.safety_timer_expired) {
		if (di->charge_state !=	STATE_SAFETY_TIMER_EXPIRED)
			ab8500_chargalg_state_to(di,
				STATE_SAFETY_TIMER_EXPIRED_INIT);
	}
	/*
	 * Check if any interrupts has occured
	 * that will prevent us from charging
	 */

	/* Battery removed */
	else if (di->events.batt_rem) {
		if (di->charge_state != STATE_BATT_REMOVED)
			ab8500_chargalg_state_to(di, STATE_BATT_REMOVED_INIT);
	}
	/* Main or USB charger not ok. */
	else if (di->events.mainextchnotok || di->events.usbchargernotok) {
		/*
		 * If vbus_collapsed is set, we have to lower the charger
		 * current, which is done in the normal state below
		 */
		if (di->charge_state != STATE_CHG_NOT_OK &&
				!di->events.vbus_collapsed)
			ab8500_chargalg_state_to(di, STATE_CHG_NOT_OK_INIT);
	}
	/* VBUS, Main or VBAT OVV. */
	else if (di->events.vbus_ovv ||
			di->events.main_ovv ||
			di->events.batt_ovv ||
			!di->chg_info.usb_chg_ok ||
			!di->chg_info.ac_chg_ok) {
		if (di->charge_state != STATE_OVV_PROTECT)
			ab8500_chargalg_state_to(di, STATE_OVV_PROTECT_INIT);
	}
	/* USB Thermal, stop charging */
	else if (di->events.main_thermal_prot ||
		di->events.usb_thermal_prot) {
		if (di->charge_state != STATE_HW_TEMP_PROTECT)
			ab8500_chargalg_state_to(di,
				STATE_HW_TEMP_PROTECT_INIT);
	}
	/* Battery temp over/under */
	else if (di->events.btemp_underover) {
		if (di->charge_state != STATE_TEMP_UNDEROVER) {
			ab8500_chargalg_state_to(di, STATE_TEMP_UNDEROVER_INIT);
			di->exceed_temp_cnt++;
		}
	}
	/* Watchdog expired */
	else if (di->events.ac_wd_expired ||
		di->events.usb_wd_expired) {
		if (di->charge_state != STATE_WD_EXPIRED)
			ab8500_chargalg_state_to(di, STATE_WD_EXPIRED_INIT);
	}
	/* Battery temp high/low */
	else if (di->events.btemp_lowhigh) {
		if (di->charge_state != STATE_TEMP_LOWHIGH)
			ab8500_chargalg_state_to(di, STATE_TEMP_LOWHIGH_INIT);
	}

	dev_dbg(di->dev,
		"[CHARGALG] Vb %d Ib_avg %d Ib_inst %d Tb %d Cap %d Maint %d "
		"State %s Active_chg %d Chg_status %d AC %d USB %d "
		"AC_online %d USB_online %d AC_CV %d USB_CV %d AC_I %d "
		"USB_I %d AC_Vset %d AC_Iset %d USB_Vset %d USB_Iset %d "
		"exceed_temp_cnt: %d\n",
		di->batt_data.volt,
		di->batt_data.avg_curr,
		di->batt_data.inst_curr,
		di->batt_data.temp,
		di->batt_data.percent,
		di->maintenance_chg,
		states[di->charge_state],
		di->chg_info.charger_type,
		di->charge_status,
		di->chg_info.conn_chg & AC_CHG,
		di->chg_info.conn_chg & USB_CHG,
		di->chg_info.online_chg & AC_CHG,
		di->chg_info.online_chg & USB_CHG,
		di->events.ac_cv_active,
		di->events.usb_cv_active,
		di->chg_info.ac_curr,
		di->chg_info.usb_curr,
		di->chg_info.ac_vset,
		di->chg_info.ac_iset,
		di->chg_info.usb_vset,
		di->chg_info.usb_iset,
		di->exceed_temp_cnt);

	switch (di->charge_state) {
	case STATE_HANDHELD_INIT:
		ab8500_chargalg_stop_charging(di);
		di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		ab8500_chargalg_state_to(di, STATE_HANDHELD);
		/* Intentional fallthrough */

	case STATE_HANDHELD:
		break;

	case STATE_SUSPENDED_INIT:
		if (di->susp_status.ac_suspended)
			ab8500_chargalg_ac_en(di, false, 0, 0);
		if (di->susp_status.usb_suspended ||
			di->events.force_usb_charging_suspend)
			ab8500_chargalg_usb_en(di, false, 0, 0);
		ab8500_chargalg_stop_safety_timer(di);
		ab8500_chargalg_stop_maintenance_timer(di);
		di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		di->maintenance_chg = false;
		ab8500_chargalg_state_to(di, STATE_SUSPENDED);
		power_supply_changed(&di->chargalg_psy);
		/* Intentional fallthrough */

	case STATE_SUSPENDED:
		/* CHARGING is suspended */
		break;

	case STATE_BATT_REMOVED_INIT:
		ab8500_chargalg_stop_charging(di);
		ab8500_chargalg_state_to(di, STATE_BATT_REMOVED);
		/* Intentional fallthrough */

	case STATE_BATT_REMOVED:
		if (!di->events.batt_rem)
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		break;

	case STATE_HW_TEMP_PROTECT_INIT:
		ab8500_chargalg_stop_charging(di);
		ab8500_chargalg_state_to(di, STATE_HW_TEMP_PROTECT);
		/* Intentional fallthrough */

	case STATE_HW_TEMP_PROTECT:
		if (!di->events.main_thermal_prot &&
				!di->events.usb_thermal_prot)
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		break;

	case STATE_OVV_PROTECT_INIT:
		ab8500_chargalg_stop_charging(di);
		ab8500_chargalg_state_to(di, STATE_OVV_PROTECT);
		/* Intentional fallthrough */

	case STATE_OVV_PROTECT:
		if (!di->events.vbus_ovv &&
				!di->events.main_ovv &&
				!di->events.batt_ovv &&
				di->chg_info.usb_chg_ok &&
				di->chg_info.ac_chg_ok)
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		break;

	case STATE_CHG_NOT_OK_INIT:
		ab8500_chargalg_stop_charging(di);
		ab8500_chargalg_state_to(di, STATE_CHG_NOT_OK);
		/* Intentional fallthrough */

	case STATE_CHG_NOT_OK:
		if (!di->events.mainextchnotok &&
				!di->events.usbchargernotok)
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		break;

	case STATE_SAFETY_TIMER_EXPIRED_INIT:
		ab8500_chargalg_stop_charging(di);
		ab8500_chargalg_state_to(di, STATE_SAFETY_TIMER_EXPIRED);
		/* Intentional fallthrough */

	case STATE_SAFETY_TIMER_EXPIRED:
		/* We exit this state when charger is removed */
		break;

	case STATE_NORMAL_INIT:
		ab8500_chargalg_start_charging(di,
			di->bat->bat_type[di->bat->batt_id].normal_vol_lvl,
			(di->events.batt_unknown) ?
			di->bat->bat_type[di->bat->batt_id].normal_cur_lvl :
			di->pdata->ddata->normal_cur_lvl);
		ab8500_chargalg_state_to(di, STATE_NORMAL);
		ab8500_chargalg_stop_maintenance_timer(di);
		init_maxim_chg_curr(di);
		di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		di->eoc_cnt = 0;
		di->maintenance_chg = false;
		power_supply_changed(&di->chargalg_psy);

		break;

	case STATE_NORMAL:
		handle_maxim_chg_curr(di);
		if (di->charge_status == POWER_SUPPLY_STATUS_FULL &&
			di->maintenance_chg) {
			if (di->bat->no_maintenance)
				ab8500_chargalg_state_to(di,
					STATE_WAIT_FOR_RECHARGE_INIT);
			else
				ab8500_chargalg_state_to(di,
					STATE_MAINTENANCE_A_INIT);
		}
		/* Check whether we should start the safety timer or not */
		ab8500_chargalg_check_safety_timer(di);
		break;

	/* This state will be used when the maintenance state is disabled */
	case STATE_WAIT_FOR_RECHARGE_INIT:
		ab8500_chargalg_hold_charging(di);
		ab8500_chargalg_state_to(di, STATE_WAIT_FOR_RECHARGE);
		/* Intentional fallthrough */

	case STATE_WAIT_FOR_RECHARGE:
		if (di->batt_data.percent <=
		    di->bat->bat_type[di->bat->batt_id].recharge_cap)
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		break;

	case STATE_MAINTENANCE_A_INIT:
		ab8500_chargalg_stop_safety_timer(di);
		ab8500_chargalg_start_maintenance_timer(di,
			di->bat->bat_type[
				di->bat->batt_id].maint_a_chg_timer_h);
		ab8500_chargalg_start_charging(di,
			di->bat->bat_type[
				di->bat->batt_id].maint_a_vol_lvl,
			di->bat->bat_type[
				di->bat->batt_id].maint_a_cur_lvl);
		ab8500_chargalg_state_to(di, STATE_MAINTENANCE_A);
		power_supply_changed(&di->chargalg_psy);
		/* Intentional fallthrough*/

	case STATE_MAINTENANCE_A:
		if (di->events.maintenance_timer_expired) {
			ab8500_chargalg_stop_maintenance_timer(di);
			ab8500_chargalg_state_to(di, STATE_MAINTENANCE_B_INIT);
		}
		break;

	case STATE_MAINTENANCE_B_INIT:
		ab8500_chargalg_start_maintenance_timer(di,
			di->bat->bat_type[
				di->bat->batt_id].maint_b_chg_timer_h);
		ab8500_chargalg_start_charging(di,
			di->bat->bat_type[
				di->bat->batt_id].maint_b_vol_lvl,
			di->bat->bat_type[
				di->bat->batt_id].maint_b_cur_lvl);
		ab8500_chargalg_state_to(di, STATE_MAINTENANCE_B);
		power_supply_changed(&di->chargalg_psy);
		/* Intentional fallthrough*/

	case STATE_MAINTENANCE_B:
		if (di->events.maintenance_timer_expired) {
			ab8500_chargalg_stop_maintenance_timer(di);
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		}
		break;

	case STATE_TEMP_LOWHIGH_INIT:
		ab8500_chargalg_start_charging(di,
			di->bat->bat_type[
				di->bat->batt_id].low_high_vol_lvl,
			di->bat->bat_type[
				di->bat->batt_id].low_high_cur_lvl);
		ab8500_chargalg_stop_maintenance_timer(di);
		di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		ab8500_chargalg_state_to(di, STATE_TEMP_LOWHIGH);
		power_supply_changed(&di->chargalg_psy);
		/* Intentional fallthrough */

	case STATE_TEMP_LOWHIGH:
		if (!di->events.btemp_lowhigh)
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		break;

	case STATE_WD_EXPIRED_INIT:
		ab8500_chargalg_stop_charging(di);
		ab8500_chargalg_state_to(di, STATE_WD_EXPIRED);
		/* Intentional fallthrough */

	case STATE_WD_EXPIRED:
		if (!di->events.ac_wd_expired &&
				!di->events.usb_wd_expired)
			ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		break;

	case STATE_TEMP_UNDEROVER_INIT:
		ab8500_chargalg_stop_charging(di);
		ab8500_chargalg_state_to(di, STATE_TEMP_UNDEROVER);
		/* Intentional fallthrough */

	case STATE_TEMP_UNDEROVER:
		if (!di->events.btemp_underover) {
			if (di->exceed_temp_cnt < MAX_TUO_CNT)
				ab8500_chargalg_state_to(di, STATE_NORMAL_INIT);
		}
		break;
	}

	/* Start charging directly if the new state is a charge state */
	if (di->charge_state == STATE_NORMAL_INIT ||
			di->charge_state == STATE_MAINTENANCE_A_INIT ||
			di->charge_state == STATE_MAINTENANCE_B_INIT)
		queue_work(di->chargalg_wq, &di->chargalg_work);
}

/**
 * ab8500_chargalg_periodic_work() - Periodic work for the algorithm
 * @work:	pointer to the work_struct structure
 *
 * Work queue function for the charging algorithm
 */
static void ab8500_chargalg_periodic_work(struct work_struct *work)
{
	struct ab8500_chargalg *di = container_of(work,
		struct ab8500_chargalg, chargalg_periodic_work.work);

	ab8500_chargalg_algorithm(di);

	/*
	 * If a charger is connected then the battery has to be monitored
	 * frequently, else the work can be delayed.
	 */
	if (di->chg_info.conn_chg)
		queue_delayed_work(di->chargalg_wq,
			&di->chargalg_periodic_work,
			di->bat->interval_charging * HZ);
	else
		queue_delayed_work(di->chargalg_wq,
			&di->chargalg_periodic_work,
			di->bat->interval_not_charging * HZ);
}

/**
 * ab8500_chargalg_wd_work() - periodic work to kick the charger watchdog
 * @work:	pointer to the work_struct structure
 *
 * Work queue function for kicking the charger watchdog
 */
static void ab8500_chargalg_wd_work(struct work_struct *work)
{
	int ret;
	struct ab8500_chargalg *di = container_of(work,
		struct ab8500_chargalg, chargalg_wd_work.work);

	dev_dbg(di->dev, "ab8500_chargalg_wd_work\n");

	ret = ab8500_chargalg_kick_watchdog(di);
	if (ret < 0)
		dev_err(di->dev, "failed to kick watchdog\n");

	queue_delayed_work(di->chargalg_wq,
		&di->chargalg_wd_work, CHG_WD_INTERVAL);
}

/**
 * ab8500_chargalg_work() - Work to run the charging algorithm instantly
 * @work:	pointer to the work_struct structure
 *
 * Work queue function for calling the charging algorithm
 */
static void ab8500_chargalg_work(struct work_struct *work)
{
	struct ab8500_chargalg *di = container_of(work,
		struct ab8500_chargalg, chargalg_work);

	ab8500_chargalg_algorithm(di);
}

/**
 * ab8500_chargalg_get_property() - get the chargalg properties
 * @psy:	pointer to the power_supply structure
 * @psp:	pointer to the power_supply_property structure
 * @val:	pointer to the power_supply_propval union
 *
 * This function gets called when an application tries to get the
 * chargalg properties by reading the sysfs files.
 * status:     charging/discharging/full/unknown
 * health:     health of the battery
 * Returns error code in case of failure else 0 on success
 */
static int ab8500_chargalg_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	struct ab8500_chargalg *di;

	di = to_ab8500_chargalg_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->charge_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (di->events.batt_ovv) {
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		} else if (di->events.btemp_underover) {
			/* Do not change health if it has been on COLD or
			 * OVERHEAT. This is useful to indicate where
			 * problem occured in the beginning.
			 */
			if (di->prev_health == POWER_SUPPLY_HEALTH_COLD ||
			    di->prev_health == POWER_SUPPLY_HEALTH_OVERHEAT)
				val->intval = di->prev_health;
			else if (di->batt_data.temp <= di->bat->temp_under)
				val->intval = POWER_SUPPLY_HEALTH_COLD;
			else
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else if (di->charge_state == STATE_SAFETY_TIMER_EXPIRED ||
				di->charge_state ==
				STATE_SAFETY_TIMER_EXPIRED_INIT) {
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		} else {
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		di->prev_health = val->intval;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* Exposure to the sysfs interface */

/**
 * ab8500_chargalg_sysfs_charger() - sysfs store operations
 * @kobj:      pointer to the struct kobject
 * @attr:      pointer to the struct attribute
 * @buf:       buffer that holds the parameter passed from userspace
 * @length:    length of the parameter passed
 *
 * Returns length of the buffer(input taken from user space) on success
 * else error code on failure
 * The operation to be performed on passing the parameters from the user space.
 */
static ssize_t ab8500_chargalg_sysfs_charger(struct kobject *kobj,
	struct attribute *attr, const char *buf, size_t length)
{
	struct ab8500_chargalg *di = container_of(kobj,
		struct ab8500_chargalg, chargalg_kobject);
	long int param;
	int ac_usb;
	int ret;
	char entry = *attr->name;

	switch (entry) {
	case 'c':
		ret = strict_strtol(buf, 10, &param);
		if (ret < 0)
			return ret;

		ac_usb = param;
		switch (ac_usb) {
		case 0:
			/* Disable charging */
			di->susp_status.ac_suspended = true;
			di->susp_status.usb_suspended = true;
			di->susp_status.suspended_change = true;
			/* Trigger a state change */
			queue_work(di->chargalg_wq,
				&di->chargalg_work);
			break;
		case 1:
			/* Enable AC Charging */
			di->susp_status.ac_suspended = false;
			di->susp_status.suspended_change = true;
			/* Trigger a state change */
			queue_work(di->chargalg_wq,
				&di->chargalg_work);
			break;
		case 2:
			/* Enable USB charging */
			di->susp_status.usb_suspended = false;
			di->susp_status.suspended_change = true;
			/* Trigger a state change */
			queue_work(di->chargalg_wq,
				&di->chargalg_work);
			break;
		default:
			dev_info(di->dev, "Wrong input\n"
				"Enter 0. Disable AC/USB Charging\n"
				"1. Enable AC charging\n"
				"2. Enable USB Charging\n");
		};
		break;
	};
	return strlen(buf);
}

static struct attribute ab8500_chargalg_en_charger = \
{
	.name = "chargalg",
	.mode = S_IWUGO,
};

static struct attribute *ab8500_chargalg_chg[] = {
	&ab8500_chargalg_en_charger,
	NULL
};

const struct sysfs_ops ab8500_chargalg_sysfs_ops = {
	.store = ab8500_chargalg_sysfs_charger,
};

static struct kobj_type ab8500_chargalg_ktype = {
	.sysfs_ops = &ab8500_chargalg_sysfs_ops,
	.default_attrs = ab8500_chargalg_chg,
};

/**
 * ab8500_chargalg_sysfs_exit() - de-init of sysfs entry
 * @di:                pointer to the struct ab8500_chargalg
 *
 * This function removes the entry in sysfs.
 */
static void ab8500_chargalg_sysfs_exit(struct ab8500_chargalg *di)
{
	kobject_del(&di->chargalg_kobject);
}

/**
 * ab8500_chargalg_sysfs_init() - init of sysfs entry
 * @di:                pointer to the struct ab8500_chargalg
 *
 * This function adds an entry in sysfs.
 * Returns error code in case of failure else 0(on success)
 */
static int ab8500_chargalg_sysfs_init(struct ab8500_chargalg *di)
{
	int ret = 0;

	ret = kobject_init_and_add(&di->chargalg_kobject,
		&ab8500_chargalg_ktype,
		NULL, "ab8500_chargalg");
	if (ret < 0)
		dev_err(di->dev, "failed to create sysfs entry\n");

	return ret;
}
/* Exposure to the sysfs interface <<END>> */

#if defined(CONFIG_PM)
static int ab8500_chargalg_resume(struct platform_device *pdev)
{
	struct ab8500_chargalg *di = platform_get_drvdata(pdev);

	/* Kick charger watchdog if charging (any charger online) */
	if (di->chg_info.online_chg)
		queue_delayed_work(di->chargalg_wq, &di->chargalg_wd_work, 0);

	/*
	 * Run the charging algorithm directly to be sure we don't
	 * do it too seldom
	 */
	queue_delayed_work(di->chargalg_wq, &di->chargalg_periodic_work, 0);

	return 0;
}

static int ab8500_chargalg_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct ab8500_chargalg *di = platform_get_drvdata(pdev);

	if (di->chg_info.online_chg)
		cancel_delayed_work_sync(&di->chargalg_wd_work);

	cancel_delayed_work_sync(&di->chargalg_periodic_work);

	return 0;
}
#else
#define ab8500_chargalg_suspend      NULL
#define ab8500_chargalg_resume       NULL
#endif

static int __devexit ab8500_chargalg_remove(struct platform_device *pdev)
{
	struct ab8500_chargalg *di = platform_get_drvdata(pdev);

	/* sysfs interface to enable/disbale charging from user space */
	ab8500_chargalg_sysfs_exit(di);

	hrtimer_cancel(&di->safety_timer);
	hrtimer_cancel(&di->maintenance_timer);

	cancel_delayed_work_sync(&di->chargalg_periodic_work);
	cancel_delayed_work_sync(&di->chargalg_wd_work);
	cancel_work_sync(&di->chargalg_work);

	/* Delete the work queue */
	destroy_workqueue(di->chargalg_wq);

	power_supply_unregister(&di->chargalg_psy);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

static int __devinit ab8500_chargalg_probe(struct platform_device *pdev)
{
	struct ab8500_platform_data *plat;
	int ret = 0;

	struct ab8500_chargalg *di =
		kzalloc(sizeof(struct ab8500_chargalg), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	/* get parent data */
	di->dev = &pdev->dev;
	di->parent = dev_get_drvdata(pdev->dev.parent);

	plat = dev_get_platdata(di->parent->dev);

	/* get chargalg specific platform data */
	if (!plat->chargalg) {
		dev_err(di->dev, "no chargalg platform data supplied\n");
		ret = -EINVAL;
		goto free_device_info;
	}
	di->pdata = plat->chargalg;

	/* get battery specific platform data */
	if (!plat->battery) {
		dev_err(di->dev, "no battery platform data supplied\n");
		ret = -EINVAL;
		goto free_device_info;
	}
	di->bat = plat->battery;

	/* chargalg supply */
	di->chargalg_psy.name = "ab8500_chargalg";
	di->chargalg_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	di->chargalg_psy.properties = ab8500_chargalg_props;
	di->chargalg_psy.num_properties = ARRAY_SIZE(ab8500_chargalg_props);
	di->chargalg_psy.get_property = ab8500_chargalg_get_property;
	di->chargalg_psy.supplied_to = di->pdata->supplied_to;
	di->chargalg_psy.num_supplicants = di->pdata->num_supplicants;
	di->chargalg_psy.external_power_changed =
		ab8500_chargalg_external_power_changed;

	/* Initilialize safety timer */
	hrtimer_init(&di->safety_timer, CLOCK_REALTIME, HRTIMER_MODE_ABS);
	di->safety_timer.function = ab8500_chargalg_safety_timer_expired;

	/* Initilialize maintenance timer */
	hrtimer_init(&di->maintenance_timer, CLOCK_REALTIME, HRTIMER_MODE_ABS);
	di->maintenance_timer.function =
		ab8500_chargalg_maintenance_timer_expired;

	/* Create a work queue for the chargalg */
	di->chargalg_wq =
		create_singlethread_workqueue("ab8500_chargalg_wq");
	if (di->chargalg_wq == NULL) {
		dev_err(di->dev, "failed to create work queue\n");
		goto free_device_info;
	}

	/* Init work for chargalg */
	INIT_DELAYED_WORK_DEFERRABLE(&di->chargalg_periodic_work,
		ab8500_chargalg_periodic_work);
	INIT_DELAYED_WORK_DEFERRABLE(&di->chargalg_wd_work,
		ab8500_chargalg_wd_work);

	/* Init work for chargalg */
	INIT_WORK(&di->chargalg_work, ab8500_chargalg_work);

	/* To detect charger at startup */
	di->chg_info.prev_conn_chg = -1;

	/* Register chargalg power supply class */
	ret = power_supply_register(di->dev, &di->chargalg_psy);
	if (ret) {
		dev_err(di->dev, "failed to register chargalg psy\n");
		goto free_chargalg_wq;
	}

	platform_set_drvdata(pdev, di);

	/* sysfs interface to enable/disable charging from user space */
	ret = ab8500_chargalg_sysfs_init(di);
	if (ret) {
		dev_err(di->dev, "failed to create sysfs entry\n");
		goto free_psy;
	}

	di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	ab8500_chargalg_state_to(di, STATE_HANDHELD);

	/* Run the charging algorithm */
	queue_delayed_work(di->chargalg_wq, &di->chargalg_periodic_work, 0);
	return ret;

free_psy:
	power_supply_unregister(&di->chargalg_psy);
free_chargalg_wq:
	destroy_workqueue(di->chargalg_wq);
free_device_info:
	kfree(di);

	return ret;
}

static struct platform_driver ab8500_chargalg_driver = {
	.probe = ab8500_chargalg_probe,
	.remove = __devexit_p(ab8500_chargalg_remove),
	.suspend = ab8500_chargalg_suspend,
	.resume = ab8500_chargalg_resume,
	.driver = {
		.name = "ab8500-chargalg",
		.owner = THIS_MODULE,
	},
};

static int __init ab8500_chargalg_init(void)
{
	return platform_driver_register(&ab8500_chargalg_driver);
}

static void __exit ab8500_chargalg_exit(void)
{
	platform_driver_unregister(&ab8500_chargalg_driver);
}

module_init(ab8500_chargalg_init);
module_exit(ab8500_chargalg_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Johan Palsson, Karl Komierowski");
MODULE_ALIAS("platform:ab8500-chargalg");
MODULE_DESCRIPTION("AB8500 battery temperature driver");

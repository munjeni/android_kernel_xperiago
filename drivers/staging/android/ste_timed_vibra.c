/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Marcin Mielczarczyk <marcin.mielczarczyk@tieto.com>
 *	   for ST-Ericsson
 * License Terms: GNU General Public License v2
 */

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/ste_timed_vibra.h>
#include <linux/delay.h>
#include "timed_output.h"

/**
 * struct vibra_info - Vibrator information structure
 * @tdev:		Pointer to timed output device structure
 * @linear_workqueue:   Pointer to linear vibrator workqueue structure
 * @linear_work:	Linear Vibrator work
 * @linear_tick:	Linear Vibrator high resolution timer
 * @vibra_workqueue:    Pointer to vibrator workqueue structure
 * @vibra_work:		Vibrator work
 * @vibra_workqueue:    Pointer to enable workqueue structure
 * @vibra_work:		Enable work
 * @vibra_timer:	Vibrator high resolution timer
 * @vibra_lock:		Vibrator lock
 * @timer_cancelled:	Cancels run of timer code
 * @vibra_state:	Actual vibrator state
 * @timeout:		Indicates how long time the vibrator will be enabled
 * @queued_timeout:	Queued timeout
 * @next_timeout:	Next timeout to be applied
 * @pdata:		Local pointer to platform data with vibrator parameters
 *
 * Structure vibra_info holds vibrator information
 **/
struct vibra_info {
	struct timed_output_dev			tdev;
	struct workqueue_struct			*linear_workqueue;
	struct work_struct			linear_work;
	struct hrtimer				linear_tick;
	struct workqueue_struct			*vibra_workqueue;
	struct work_struct			vibra_work;
	struct workqueue_struct			*enable_workqueue;
	struct work_struct			enable_work;
	struct hrtimer				vibra_timer;
	spinlock_t				vibra_lock;
	atomic_t				timer_cancelled;
	enum ste_timed_vibra_states		vibra_state;
	unsigned int				timeout;
	int					queued_timeout;
	int					next_timeout;
	struct ste_timed_vibra_platform_data	*pdata;
};

/*
 * Linear vibrator hardware operates on a particular resonance
 * frequency. The resonance frequency (f) may also vary with h/w.
 * This define is half time period (t) in micro seconds (us).
 * For resonance frequency f = 150 Hz
 * t = T/2 = ((1/150) / 2) = 3333 usec.
 */
#define LINEAR_RESONANCE	3333

/**
 * linear_vibra_work() - Linear Vibrator work, turns on/off vibrator
 * @work:	Pointer to work structure
 *
 * This function is called from workqueue, turns on/off vibrator
 **/
static void linear_vibra_work(struct work_struct *work)
{
	struct vibra_info *vinfo =
			container_of(work, struct vibra_info, linear_work);
	unsigned char speed_pos = 0, speed_neg = 0;
	ktime_t ktime;
	static unsigned char toggle;
	unsigned long flags;

	if (toggle) {
		speed_pos = vinfo->pdata->boost_level;
		speed_neg = 0;
	} else {
		speed_neg = vinfo->pdata->boost_level;
		speed_pos = 0;
	}

	toggle = !toggle;
	vinfo->pdata->timed_vibra_control(speed_pos, speed_neg,
			speed_pos, speed_neg);

	spin_lock_irqsave(&vinfo->vibra_lock, flags);
	if ((vinfo->vibra_state != STE_VIBRA_IDLE) &&
		(vinfo->vibra_state != STE_VIBRA_OFF)) {
		spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
		ktime = ktime_set((LINEAR_RESONANCE / USEC_PER_SEC),
			(LINEAR_RESONANCE % USEC_PER_SEC) * NSEC_PER_USEC);
		hrtimer_start(&vinfo->linear_tick, ktime, HRTIMER_MODE_REL);
	} else {
		spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
	}
}

static void vibra_cancel_timer_and_flush(struct vibra_info *vinfo)
{
	atomic_set(&vinfo->timer_cancelled, 1);

	hrtimer_cancel(&vinfo->vibra_timer);
	flush_workqueue(vinfo->vibra_workqueue);
	hrtimer_cancel(&vinfo->vibra_timer);

	atomic_set(&vinfo->timer_cancelled, 0);
}

/**
 * vibra_control - Vibrator control, turns on/off vibrator
 * @vinfo:	Pointer to vibra_info structure
 *
 * This function is called from work callbacks, turns on/off vibrator
 **/
static void vibra_control(struct vibra_info *vinfo)
{
	unsigned val = 0;
	unsigned char speed_pos = 0, speed_neg = 0;
	unsigned long flags;

	spin_lock_irqsave(&vinfo->vibra_lock, flags);
	switch (vinfo->vibra_state) {
	case STE_VIBRA_BOOST:
		val = vinfo->pdata->boost_time;
		/* Turn on both vibrators with boost speed */
		if (vinfo->pdata->reverse_polarity)
			speed_neg = vinfo->pdata->boost_level;
		else
			speed_pos = vinfo->pdata->boost_level;

		break;
	case STE_VIBRA_ON:
		val = vinfo->timeout - vinfo->pdata->boost_time;
		/* Turn on both vibrators with speed */
		if (vinfo->pdata->reverse_polarity)
			speed_neg = vinfo->pdata->on_level;
		else
			speed_pos = vinfo->pdata->on_level;

		break;
	case STE_VIBRA_OFF:
		val = vinfo->pdata->off_time;
		/* Turn on both vibrators with reversed speed */
		if (vinfo->pdata->reverse_polarity)
			speed_pos = vinfo->pdata->off_level;
		else
			speed_neg = vinfo->pdata->off_level;

		break;
	default:
		break;
	}

	if (vinfo->pdata->is_linear_vibra
		&& vinfo->vibra_state == STE_VIBRA_IDLE) {
		spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
		/* Cancel work and timers of linear vibrator in IDLE state */
		hrtimer_cancel(&vinfo->linear_tick);
		flush_workqueue(vinfo->linear_workqueue);
		hrtimer_cancel(&vinfo->linear_tick);
		vinfo->pdata->timed_vibra_control(0, 0, 0, 0);
	} else {
		spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
	}

	/* Send new settings (only for rotary vibrators) */
	if (!vinfo->pdata->is_linear_vibra)
		vinfo->pdata->timed_vibra_control(speed_pos, speed_neg,
				speed_pos, speed_neg);

	spin_lock_irqsave(&vinfo->vibra_lock, flags);
	if (vinfo->vibra_state != STE_VIBRA_IDLE) {
		/* Start timer if it's not in IDLE state */
		ktime_t ktime;
		ktime = ktime_set((val / MSEC_PER_SEC),
				(val % MSEC_PER_SEC) * NSEC_PER_MSEC),
		hrtimer_start(&vinfo->vibra_timer, ktime, HRTIMER_MODE_REL);
	}
	vinfo->next_timeout = -1;
	spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
}

/**
 * vibra_control_work() - Vibrator work, calls vibra_control
 * @work:	Pointer to work structure
 *
 * This function is called from workqueue, calls vibra_control
 **/
static void vibra_control_work(struct work_struct *work)
{
	struct vibra_info *vinfo =
			container_of(work, struct vibra_info, vibra_work);
	vibra_control(vinfo);
}

/**
 * vibra_enable_work() - Executes an enable command
 * @work:	Pointer to work structure
 *
 * This executes an enable command
 **/
static void vibra_enable_work(struct work_struct *work)
{
	struct vibra_info *vinfo =
			container_of(work, struct vibra_info, enable_work);
	unsigned long flags;
	ktime_t ktime;

	vibra_cancel_timer_and_flush(vinfo);

	spin_lock_irqsave(&vinfo->vibra_lock, flags);
	if (vinfo->queued_timeout == -1) {
		dev_err(vinfo->tdev.dev,
			"Nothing queued when doing enable work\n");
		vinfo->next_timeout = 0;
	} else {
		vinfo->next_timeout = vinfo->queued_timeout;
		vinfo->queued_timeout = -1;
	}

	switch (vinfo->vibra_state) {
	case STE_VIBRA_IDLE:
		if (vinfo->next_timeout)
			vinfo->vibra_state = STE_VIBRA_BOOST;
		else    /* Already disabled */
			break;

		/* Trim timeout */
		vinfo->timeout = vinfo->next_timeout <
				vinfo->pdata->boost_time ?
				vinfo->pdata->boost_time :
				vinfo->next_timeout;

		if (vinfo->pdata->is_linear_vibra) {
			queue_work(vinfo->linear_workqueue,
					&vinfo->linear_work);
		}
		spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
		vibra_control(vinfo);
		return;
	case STE_VIBRA_BOOST:
		vinfo->timeout = vinfo->next_timeout <
			vinfo->pdata->boost_time ?
			vinfo->pdata->boost_time :
			vinfo->next_timeout;
		hrtimer_restart(&vinfo->vibra_timer);
		vinfo->next_timeout = -1;
		break;
	case STE_VIBRA_ON:
		if (!vinfo->next_timeout) {
			vinfo->vibra_state = STE_VIBRA_OFF;
			spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
			vibra_control(vinfo);
			return;
		} else {
			ktime = ktime_set((vinfo->next_timeout / MSEC_PER_SEC),
				(vinfo->next_timeout % MSEC_PER_SEC)
				* NSEC_PER_MSEC);
			hrtimer_start(&vinfo->vibra_timer, ktime,
						HRTIMER_MODE_REL);
			vinfo->next_timeout = -1;
		}
		break;
	case STE_VIBRA_OFF:
		if (vinfo->next_timeout) {
			vinfo->vibra_state = STE_VIBRA_ON;
			vinfo->timeout = vinfo->next_timeout +
				vinfo->pdata->boost_time;
			spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
			vibra_control(vinfo);
			return;
		} else {
			hrtimer_restart(&vinfo->vibra_timer);
		}
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
}

/**
 * vibra_enable() - Enables vibrator
 * @tdev:      Pointer to timed output device structure
 * @timeout:	Time indicating how long vibrator will be enabled
 *
 * This function enables vibrator
 **/
static void vibra_enable(struct timed_output_dev *tdev, int timeout)
{
	struct vibra_info *vinfo = dev_get_drvdata(tdev->dev);
	unsigned long flags;

	if (timeout < 0)
		return;

	spin_lock_irqsave(&vinfo->vibra_lock, flags);
	if (vinfo->queued_timeout == -1)
		queue_work(vinfo->enable_workqueue, &vinfo->enable_work);

	vinfo->queued_timeout = timeout;
	spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
}

/**
 * linear_vibra_tick() - Generate resonance frequency waveform
 * @hrtimer: Pointer to high resolution timer structure
 *
 * This function helps in generating the resonance frequency
 * waveform required for linear vibrators
 *
 * Returns:
 *	Returns value which indicates whether hrtimer should be restarted
 **/
static enum hrtimer_restart linear_vibra_tick(struct hrtimer *hrtimer)
{
	struct vibra_info *vinfo =
			container_of(hrtimer, struct vibra_info, linear_tick);
	unsigned long flags;

	spin_lock_irqsave(&vinfo->vibra_lock, flags);
	if ((vinfo->vibra_state != STE_VIBRA_IDLE) &&
		(vinfo->vibra_state != STE_VIBRA_OFF)) {
		spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
		queue_work(vinfo->linear_workqueue, &vinfo->linear_work);
	} else {
		spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
	}

	return HRTIMER_NORESTART;
}

/**
 * vibra_timer_expired() - Handles vibrator machine state
 * @hrtimer:      Pointer to high resolution timer structure
 *
 * This function handles vibrator machine state
 *
 * Returns:
 *	Returns value which indicates wether hrtimer should be restarted
 **/
static enum hrtimer_restart vibra_timer_expired(struct hrtimer *hrtimer)
{
	struct vibra_info *vinfo =
			container_of(hrtimer, struct vibra_info, vibra_timer);
	unsigned long flags;

	if (atomic_read(&vinfo->timer_cancelled))
		return HRTIMER_NORESTART;

	spin_lock_irqsave(&vinfo->vibra_lock, flags);
	switch (vinfo->vibra_state) {
	case STE_VIBRA_BOOST:
		vinfo->vibra_state = STE_VIBRA_ON;
		vinfo->next_timeout = vinfo->timeout - vinfo->pdata->boost_time;
		break;
	case STE_VIBRA_ON:
		vinfo->vibra_state = STE_VIBRA_OFF;
		vinfo->next_timeout = 0;
		break;
	case STE_VIBRA_OFF:
		vinfo->vibra_state = STE_VIBRA_IDLE;
		break;
	case STE_VIBRA_IDLE:
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
	queue_work(vinfo->vibra_workqueue, &vinfo->vibra_work);

	return HRTIMER_NORESTART;
}

/**
 * vibra_get_time() - Returns remaining time to disabling vibration
 * @tdev:      Pointer to timed output device structure
 *
 * This function returns time remaining to disabling vibration
 *
 * Returns:
 *	Returns remaining time to disabling vibration
 **/
static int vibra_get_time(struct timed_output_dev *tdev)
{
	struct vibra_info *vinfo = dev_get_drvdata(tdev->dev);
	int rc;
	ktime_t remain;
	unsigned long flags;
	spin_lock_irqsave(&vinfo->vibra_lock, flags);
	if (vinfo->queued_timeout != -1)
		rc = vinfo->queued_timeout;
	else if (vinfo->next_timeout != -1)
		rc = vinfo->next_timeout;
	else {
		switch (vinfo->vibra_state) {
		case STE_VIBRA_ON:
			remain = hrtimer_get_remaining(&vinfo->vibra_timer);
			if (ktime_to_ns(remain) > 0)
				rc = (u32) ktime_to_ms(remain);
			else
				rc = 0;
		break;
		case STE_VIBRA_BOOST:
			remain = hrtimer_get_remaining(&vinfo->vibra_timer);
			rc = vinfo->timeout - vinfo->pdata->boost_time;
			if (ktime_to_ns(remain) > 0)
				rc += ktime_to_ms(remain);
			break;
		default:
			rc = 0;
			break;
		}
	}

	spin_unlock_irqrestore(&vinfo->vibra_lock, flags);
	return rc;
}

static int __devinit ste_timed_vibra_probe(struct platform_device *pdev)
{
	int ret;
	struct vibra_info *vinfo;

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -ENODEV;
	}

	vinfo = kmalloc(sizeof *vinfo, GFP_KERNEL);
	if (!vinfo) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	vinfo->tdev.name = "vibrator";
	vinfo->tdev.enable = vibra_enable;
	vinfo->tdev.get_time = vibra_get_time;
	vinfo->vibra_state = STE_VIBRA_IDLE;
	atomic_set(&vinfo->timer_cancelled, 0);
	vinfo->queued_timeout = -1;
	vinfo->next_timeout = -1;
	vinfo->pdata = pdev->dev.platform_data;

	if (vinfo->pdata->is_linear_vibra)
		dev_info(&pdev->dev, "Linear Type Vibrators\n");
	else
		dev_info(&pdev->dev, "Rotary Type Vibrators\n");

	ret = timed_output_dev_register(&vinfo->tdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register timed output device\n");
		goto exit_free_vinfo;
	}

	dev_set_drvdata(vinfo->tdev.dev, vinfo);

	vinfo->linear_workqueue =
		create_singlethread_workqueue("ste-timed-linear-vibra");
	if (!vinfo->linear_workqueue) {
		dev_err(&pdev->dev, "failed to allocate workqueue\n");
		ret = -ENOMEM;
		goto exit_timed_output_unregister;
	}

	/* Create workqueue just for timed output vibrator */
	vinfo->vibra_workqueue =
		create_singlethread_workqueue("ste-timed-output-vibra");
	if (!vinfo->vibra_workqueue) {
		dev_err(&pdev->dev, "failed to allocate workqueue\n");
		ret = -ENOMEM;
		goto exit_destroy_linear_workqueue;
	}

	vinfo->enable_workqueue =
		create_singlethread_workqueue("ste-timed-enable-vibra");
	if (!vinfo->enable_workqueue) {
		dev_err(&pdev->dev, "failed to allocate workqueue\n");
		ret = -ENOMEM;
		goto exit_destroy_vibra_workqueue;
	}

	INIT_WORK(&vinfo->linear_work, linear_vibra_work);
	INIT_WORK(&vinfo->vibra_work, vibra_control_work);
	INIT_WORK(&vinfo->enable_work, vibra_enable_work);
	spin_lock_init(&vinfo->vibra_lock);
	hrtimer_init(&vinfo->linear_tick, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_init(&vinfo->vibra_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vinfo->linear_tick.function = linear_vibra_tick;
	vinfo->vibra_timer.function = vibra_timer_expired;

	platform_set_drvdata(pdev, vinfo);
	return 0;

exit_destroy_vibra_workqueue:
	destroy_workqueue(vinfo->vibra_workqueue);
exit_destroy_linear_workqueue:
	destroy_workqueue(vinfo->linear_workqueue);
exit_timed_output_unregister:
	timed_output_dev_unregister(&vinfo->tdev);
exit_free_vinfo:
	kfree(vinfo);
	return ret;
}

static int __devexit ste_timed_vibra_remove(struct platform_device *pdev)
{
	struct vibra_info *vinfo = platform_get_drvdata(pdev);

	timed_output_dev_unregister(&vinfo->tdev);
	destroy_workqueue(vinfo->linear_workqueue);
	destroy_workqueue(vinfo->vibra_workqueue);
	destroy_workqueue(vinfo->enable_workqueue);
	kfree(vinfo);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver ste_timed_vibra_driver = {
	.driver = {
		.name = "ste_timed_output_vibra",
		.owner = THIS_MODULE,
	},
	.probe  = ste_timed_vibra_probe,
	.remove = __devexit_p(ste_timed_vibra_remove)
};

static int __init ste_timed_vibra_init(void)
{
	return platform_driver_register(&ste_timed_vibra_driver);
}
module_init(ste_timed_vibra_init);

static void __exit ste_timed_vibra_exit(void)
{
	platform_driver_unregister(&ste_timed_vibra_driver);
}
module_exit(ste_timed_vibra_exit);

MODULE_AUTHOR("Marcin Mielczarczyk <marcin.mielczarczyk@tieto.com>");
MODULE_DESCRIPTION("STE Timed Output Vibrator");
MODULE_LICENSE("GPL v2");

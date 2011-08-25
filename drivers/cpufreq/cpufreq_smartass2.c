/*
 * drivers/cpufreq/cpufreq_smartass2.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Erasmux
 *
 * Based on the interactive governor By Mike Chan (mike@android.com)
 * which was adaptated to 2.6.29 kernel by Nadlabak (pavel@doshaska.net)
 *
 * SMP support based on mod by faux123
 *
 * requires to add
 * EXPORT_SYMBOL_GPL(nr_running);
 * at the end of kernel/sched.c
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <asm/cputime.h>
#include <linux/earlysuspend.h>


/******************** Tunable parameters: ********************/

/*
 * The "ideal" frequency to use when awake. The governor will ramp up faster
 * twards the ideal frequency and slower after it has passed it. Similarly,
 * lowering the frequency twards the ideal frequency is faster than below it.
 */
#define DEFAULT_AWAKE_IDEAL_FREQ 518400
static unsigned int awake_ideal_freq;

/*
 * The "ideal" frequency to use when suspended.
 * When set to 0, the governor will not track the suspended state (meaning
 * that practically when sleep_ideal_freq==0 the awake_ideal_freq is used
 * also when suspended).
 */
#define DEFAULT_SLEEP_IDEAL_FREQ 1
static unsigned int sleep_ideal_freq;

/*
 * Freqeuncy delta when ramping up.
 * zero disables and causes to always jump straight to max frequency.
 */
#define DEFAULT_RAMP_UP_STEP 128000
static unsigned int ramp_up_step;

/*
 * Freqeuncy delta when ramping down.
 * zero disables and will calculate ramp down according to load heuristic.
 */
#define DEFAULT_RAMP_DOWN_STEP 256000
static unsigned int ramp_down_step;

/*
 * CPU freq will be increased if measured load > max_cpu_load;
 */
#define DEFAULT_MAX_CPU_LOAD 50
static unsigned long max_cpu_load;

/*
 * CPU freq will be decreased if measured load < min_cpu_load;
 */
#define DEFAULT_MIN_CPU_LOAD 25
static unsigned long min_cpu_load;

/*
 * The minimum amount of time to spend at a frequency before we can ramp up.
 */
#define DEFAULT_UP_RATE_US 48000;
static unsigned long up_rate_us;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 */
#define DEFAULT_DOWN_RATE_US 99000;
static unsigned long down_rate_us;

/*
 * The frequency to set when waking up from sleep.
 * When sleep_ideal_freq=0 this will have no effect.
 */
#define DEFAULT_SLEEP_WAKEUP_FREQ 99999999
static unsigned int sleep_wakeup_freq;

/*
 * Sampling rate, I highly recommend to leave it at 2.
 */
#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;


/*************** End of tunables ***************/


static void (*pm_idle_old)(void);
static atomic_t active_count = ATOMIC_INIT(0);

struct smartass_info_s {
	struct cpufreq_policy *cur_policy;
	struct timer_list timer;
	u64 time_in_idle;
	u64 idle_exit_time;
	u64 freq_change_time;
	u64 freq_change_time_in_idle;
	int cur_cpu_load;
	unsigned int force_ramp_up;
	unsigned int enable;
	int ideal_speed;
};
static DEFINE_PER_CPU(struct smartass_info_s, smartass_info);

/* Workqueues handle frequency scaling */
static struct workqueue_struct *up_wq;
static struct workqueue_struct *down_wq;
static struct work_struct freq_scale_work;

static cpumask_t work_cpumask;
static spinlock_t cpumask_lock;

static unsigned int suspended;

enum {
	SMARTASS_DEBUG_JUMPS=1,
	SMARTASS_DEBUG_LOAD=2
};

/*
 * Combination of the above debug flags.
 */
static unsigned long debug_mask;

static int cpufreq_governor_smartass(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SMARTASS2
static
#endif
struct cpufreq_governor cpufreq_gov_smartass2 = {
	.name = "smartassV2",
	.governor = cpufreq_governor_smartass,
	.max_transition_latency = 9000000,
	.owner = THIS_MODULE,
};

inline static void smartass_update_min_max(struct smartass_info_s *this_smartass, struct cpufreq_policy *policy, int suspend) {
	if (suspend) {
		this_smartass->ideal_speed = // sleep_ideal_freq; but make sure it obeys the policy min/max
			policy->max > sleep_ideal_freq ?
			(sleep_ideal_freq > policy->min ? sleep_ideal_freq : policy->min) : policy->max;
	} else {
		this_smartass->ideal_speed = // awake_ideal_freq; but make sure it obeys the policy min/max
			policy->min < awake_ideal_freq ?
			(awake_ideal_freq < policy->max ? awake_ideal_freq : policy->max) : policy->min;
	}
}

inline static void smartass_update_min_max_allcpus() {
	unsigned int i;
	for_each_online_cpu(i) {
		struct smartass_info_s *this_smartass = &per_cpu(smartass_info, i);
		if (this_smartass->enable)
			smartass_update_min_max(this_smartass,this_smartass->cur_policy,suspended);
	}
}

inline static unsigned int validate_freq(struct cpufreq_policy *policy, int freq) {
	if (freq > policy->max)
		return policy->max;
	if (freq < policy->min)
		return policy->min;
	return freq;
}

inline static void reset_timer(unsigned long cpu, struct smartass_info_s *this_smartass) {
	this_smartass->time_in_idle = get_cpu_idle_time_us(cpu, &this_smartass->idle_exit_time);
	mod_timer(&this_smartass->timer, jiffies + sample_rate_jiffies);
}

inline static void work_cpumask_set(unsigned long cpu) {
	unsigned long flags;
	spin_lock_irqsave(&cpumask_lock, flags);
	cpumask_set_cpu(cpu, &work_cpumask);
	spin_unlock_irqrestore(&cpumask_lock, flags);
}

inline static int work_cpumask_test_and_clear(unsigned long cpu) {
	unsigned long flags;
	int res = 0;
	spin_lock_irqsave(&cpumask_lock, flags);
	res = cpumask_test_and_clear_cpu(cpu, &work_cpumask);
	spin_unlock_irqrestore(&cpumask_lock, flags);
	return res;
}

static void cpufreq_smartass_timer(unsigned long cpu)
{
	u64 delta_idle;
	u64 delta_time;
	int cpu_load;
	u64 update_time;
	u64 now_idle;
	struct smartass_info_s *this_smartass = &per_cpu(smartass_info, cpu);
	struct cpufreq_policy *policy = this_smartass->cur_policy;

	now_idle = get_cpu_idle_time_us(cpu, &update_time);

	if (this_smartass->idle_exit_time == 0 || update_time == this_smartass->idle_exit_time)
		return;

	delta_idle = cputime64_sub(now_idle, this_smartass->time_in_idle);
	delta_time = cputime64_sub(update_time, this_smartass->idle_exit_time);

	// If timer ran less than 1ms after short-term sample started, retry.
	if (delta_time < 1000) {
		if (!timer_pending(&this_smartass->timer))
			reset_timer(cpu,this_smartass);
		return;
	}

	if (delta_idle > delta_time)
		cpu_load = 0;
	else
		cpu_load = 100 * (unsigned int)(delta_time - delta_idle) / (unsigned int)delta_time;

	if (debug_mask & SMARTASS_DEBUG_LOAD)
		printk(KERN_DEBUG "smartassT @ %d: load %d (delta_time %llu)\n",policy->cur,cpu_load,delta_time);

	this_smartass->cur_cpu_load = cpu_load;

	// Scale up if load is above max or if there where no idle cycles since coming out of idle,
	// or when we are above our max speed for a very long time (should only happend if entering sleep
	// at high loads)
	if (cpu_load > max_cpu_load || delta_idle == 0)
	{
		if (policy->cur < policy->max && nr_running() > 0 &&
			 (policy->cur < this_smartass->ideal_speed || (!suspended && delta_idle == 0) ||
			  cputime64_sub(update_time, this_smartass->freq_change_time) >= up_rate_us))
		{
			this_smartass->force_ramp_up = 1;
			work_cpumask_set(cpu);
			queue_work(up_wq, &freq_scale_work);
		}
	}
	/*
	 * Do not scale down unless we have been at this frequency for the
	 * minimum sample time.
	 */
	else if (policy->cur > this_smartass->ideal_speed ||
		 (policy->cur > policy->min &&
		  cputime64_sub(update_time, this_smartass->freq_change_time) >= down_rate_us))
	{
		work_cpumask_set(cpu);
		queue_work(down_wq, &freq_scale_work);
	}

	if (policy->cur < policy->max && !timer_pending(&this_smartass->timer))
		reset_timer(cpu,this_smartass);
}

static void cpufreq_idle(void)
{
	struct smartass_info_s *this_smartass = &per_cpu(smartass_info, smp_processor_id());
	struct cpufreq_policy *policy = this_smartass->cur_policy;

	if (!this_smartass->enable) {
		pm_idle_old();
		return;
	}

	if (policy->cur == policy->min && timer_pending(&this_smartass->timer))
		del_timer(&this_smartass->timer);

	pm_idle_old();

	if (!timer_pending(&this_smartass->timer))
		reset_timer(smp_processor_id(), this_smartass);
}

/* We use the same work function to sale up and down */
static void cpufreq_smartass_freq_change_time_work(struct work_struct *work)
{
	unsigned int cpu;
	int new_freq;
	int old_freq;
	unsigned int force_ramp_up;
	int cpu_load;
	struct smartass_info_s *this_smartass;
	struct cpufreq_policy *policy;
	unsigned int relation = CPUFREQ_RELATION_L;
	for_each_possible_cpu(cpu) {
		if (!work_cpumask_test_and_clear(cpu))
			continue;

		this_smartass = &per_cpu(smartass_info, cpu);
		policy = this_smartass->cur_policy;
		old_freq = policy->cur;
		cpu_load = this_smartass->cur_cpu_load;
		force_ramp_up = this_smartass->force_ramp_up && nr_running() > 1;
		this_smartass->force_ramp_up = 0;

		if (force_ramp_up || cpu_load > max_cpu_load) {
			if (old_freq < this_smartass->ideal_speed)
				new_freq = this_smartass->ideal_speed;
			else if (ramp_up_step) {
				new_freq = old_freq + ramp_up_step;
				relation = CPUFREQ_RELATION_H;
			}
			else {
				new_freq = policy->max;
				relation = CPUFREQ_RELATION_H;
			}
		}
		else if (cpu_load < min_cpu_load) {
			if (old_freq > this_smartass->ideal_speed) {
				new_freq = this_smartass->ideal_speed;
				relation = CPUFREQ_RELATION_H;
			}
			else if (ramp_down_step)
				new_freq = old_freq - ramp_down_step;
			else {
				cpu_load += 100 - max_cpu_load; // dummy load.
				new_freq = old_freq * cpu_load / 100;
			}
		}
		else new_freq = old_freq;

		new_freq = validate_freq(policy,new_freq);

		if (new_freq != old_freq) {
			__cpufreq_driver_target(policy, new_freq, relation);
			if (policy->cur == old_freq) {
				if (new_freq > old_freq && relation==CPUFREQ_RELATION_H)
					__cpufreq_driver_target(policy, new_freq, CPUFREQ_RELATION_L);
				else if (new_freq < old_freq && relation==CPUFREQ_RELATION_L)
					__cpufreq_driver_target(policy, new_freq, CPUFREQ_RELATION_H);
			}

			if (debug_mask & SMARTASS_DEBUG_JUMPS)
				printk(KERN_DEBUG "SmartassQ: jumping from %d to %d => %d\n",
				       old_freq,new_freq,policy->cur);

			if (policy->cur == old_freq)
				printk(KERN_WARNING "Smartass: frequency change failed: %d to %d => %d\n",
				       old_freq,new_freq,policy->cur);
			else
				this_smartass->freq_change_time_in_idle =
					get_cpu_idle_time_us(cpu,&this_smartass->freq_change_time);
		}
	}
}

static ssize_t show_debug_mask(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", debug_mask);
}

static ssize_t store_debug_mask(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0)
		debug_mask = input;
	return res;
}

static ssize_t show_up_rate_us(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", up_rate_us);
}

static ssize_t store_up_rate_us(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input <= 100000000)
		up_rate_us = input;
	return res;
}

static ssize_t show_down_rate_us(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", down_rate_us);
}

static ssize_t store_down_rate_us(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0 && input <= 100000000)
		down_rate_us = input;
	return res;
}

static ssize_t show_sleep_ideal_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sleep_ideal_freq);
}

static ssize_t store_sleep_ideal_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		sleep_ideal_freq = input;
		if (suspended)
			smartass_update_min_max_allcpus();
	}
	return res;
}

static ssize_t show_sleep_wakeup_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sleep_wakeup_freq);
}

static ssize_t store_sleep_wakeup_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0)
		sleep_wakeup_freq = input;
	return res;
}

static ssize_t show_awake_ideal_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", awake_ideal_freq);
}

static ssize_t store_awake_ideal_freq(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0) {
		awake_ideal_freq = input;
		if (!suspended)
			smartass_update_min_max_allcpus();
	}
	return res;
}

static ssize_t show_sample_rate_jiffies(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sample_rate_jiffies);
}

static ssize_t store_sample_rate_jiffies(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 0 && input <= 1000)
		sample_rate_jiffies = input;
	return res;
}

static ssize_t show_ramp_up_step(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ramp_up_step);
}

static ssize_t store_ramp_up_step(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0)
		ramp_up_step = input;
	return res;
}

static ssize_t show_ramp_down_step(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ramp_down_step);
}

static ssize_t store_ramp_down_step(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input >= 0)
		ramp_down_step = input;
	return res;
}

static ssize_t show_max_cpu_load(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", max_cpu_load);
}

static ssize_t store_max_cpu_load(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 0 && input <= 100)
		max_cpu_load = input;
	return res;
}

static ssize_t show_min_cpu_load(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", min_cpu_load);
}

static ssize_t store_min_cpu_load(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res >= 0 && input > 0 && input < 100)
		min_cpu_load = input;
	return res;
}

#define define_global_rw_attr(_name)		\
static struct global_attr _name##_attr =	\
	__ATTR(_name, 0644, show_##_name, store_##_name)

define_global_rw_attr(debug_mask);
define_global_rw_attr(up_rate_us);
define_global_rw_attr(down_rate_us);
define_global_rw_attr(sleep_ideal_freq);
define_global_rw_attr(sleep_wakeup_freq);
define_global_rw_attr(awake_ideal_freq);
define_global_rw_attr(sample_rate_jiffies);
define_global_rw_attr(ramp_up_step);
define_global_rw_attr(ramp_down_step);
define_global_rw_attr(max_cpu_load);
define_global_rw_attr(min_cpu_load);

static struct attribute * smartass_attributes[] = {
	&debug_mask_attr.attr,
	&up_rate_us_attr.attr,
	&down_rate_us_attr.attr,
	&sleep_ideal_freq_attr.attr,
	&sleep_wakeup_freq_attr.attr,
	&awake_ideal_freq_attr.attr,
	&sample_rate_jiffies_attr.attr,
	&ramp_up_step_attr.attr,
	&ramp_down_step_attr.attr,
	&max_cpu_load_attr.attr,
	&min_cpu_load_attr.attr,
	NULL,
};

static struct attribute_group smartass_attr_group = {
	.attrs = smartass_attributes,
	.name = "smartass",
};

static int cpufreq_governor_smartass(struct cpufreq_policy *new_policy,
		unsigned int event)
{
	unsigned int cpu = new_policy->cpu;
	int rc;
	struct smartass_info_s *this_smartass = &per_cpu(smartass_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!new_policy->cur))
			return -EINVAL;

		this_smartass->cur_policy = new_policy;

		this_smartass->enable = 1;

		smartass_update_min_max(this_smartass,new_policy,suspended);

		smp_wmb();

		// Do not register the idle hook and create sysfs
		// entries if we have already done so.
		if (atomic_inc_return(&active_count) <= 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&smartass_attr_group);
			if (rc)
				return rc;

			pm_idle_old = pm_idle;
			pm_idle = cpufreq_idle;
		}

		break;

	case CPUFREQ_GOV_LIMITS:
		smartass_update_min_max(this_smartass,new_policy,suspended);

		if (this_smartass->cur_policy->cur > new_policy->max) {
			//if (debug_mask & SMARTASS_DEBUG_JUMPS)
				printk(KERN_DEBUG "SmartassI: jumping to new max freq: %d\n",new_policy->max);
			__cpufreq_driver_target(this_smartass->cur_policy,
						new_policy->max, CPUFREQ_RELATION_H);
		}
		else if (this_smartass->cur_policy->cur < new_policy->min) {
			//if (debug_mask & SMARTASS_DEBUG_JUMPS)
				printk(KERN_DEBUG "SmartassI: jumping to new min freq: %d\n",new_policy->min);
			__cpufreq_driver_target(this_smartass->cur_policy,
						new_policy->min, CPUFREQ_RELATION_L);
		}
		break;

	case CPUFREQ_GOV_STOP:
		this_smartass->enable = 0;
		smp_wmb();
		del_timer(&this_smartass->timer);
		flush_work(&freq_scale_work);
		this_smartass->idle_exit_time = 0;

		if (atomic_dec_return(&active_count) <= 1) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &smartass_attr_group);
			pm_idle = pm_idle_old;
		}
		break;
	}

	return 0;
}

static void smartass_suspend(int cpu, int suspend)
{
	struct smartass_info_s *this_smartass = &per_cpu(smartass_info, smp_processor_id());
	struct cpufreq_policy *policy = this_smartass->cur_policy;
	unsigned int new_freq;

	if (!this_smartass->enable || sleep_ideal_freq==0) // disable behavior for sleep_ideal_freq==0
		return;

	smartass_update_min_max(this_smartass,policy,suspend);
	if (!suspend) { // resume at max speed:
		new_freq = validate_freq(policy,sleep_wakeup_freq);

		if (debug_mask & SMARTASS_DEBUG_JUMPS)
			printk(KERN_DEBUG "SmartassS: awaking at %d\n",new_freq);

		__cpufreq_driver_target(policy, new_freq,
					CPUFREQ_RELATION_L);
	} else {
		// to avoid wakeup issues with quick sleep/wakeup don't change actual frequency when entering sleep
		// to allow some time to settle down. Instead we just reset our statistics (and reset the timer).
		// Eventually, even at full load the timer will lower the freqeuncy.

		this_smartass->freq_change_time_in_idle =
			get_cpu_idle_time_us(cpu,&this_smartass->freq_change_time);

		if (debug_mask & SMARTASS_DEBUG_JUMPS)
			printk(KERN_DEBUG "SmartassS: suspending at %d\n",policy->cur);
	}

	reset_timer(smp_processor_id(),this_smartass);
}

static void smartass_early_suspend(struct early_suspend *handler) {
	int i;
	suspended = 1;
	for_each_online_cpu(i)
		smartass_suspend(i,1);
}

static void smartass_late_resume(struct early_suspend *handler) {
	int i;
	suspended = 0;
	for_each_online_cpu(i)
		smartass_suspend(i,0);
}

static struct early_suspend smartass_power_suspend = {
	.suspend = smartass_early_suspend,
	.resume = smartass_late_resume,
#ifdef CONFIG_MACH_HERO
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
#endif
};

static int __init cpufreq_smartass_init(void)
{
	unsigned int i;
	struct smartass_info_s *this_smartass;
	debug_mask = 0;
	up_rate_us = DEFAULT_UP_RATE_US;
	down_rate_us = DEFAULT_DOWN_RATE_US;
	sleep_ideal_freq = DEFAULT_SLEEP_IDEAL_FREQ;
	sleep_wakeup_freq = DEFAULT_SLEEP_WAKEUP_FREQ;
	awake_ideal_freq = DEFAULT_AWAKE_IDEAL_FREQ;
	sample_rate_jiffies = DEFAULT_SAMPLE_RATE_JIFFIES;
	ramp_up_step = DEFAULT_RAMP_UP_STEP;
	ramp_down_step = DEFAULT_RAMP_DOWN_STEP;
	max_cpu_load = DEFAULT_MAX_CPU_LOAD;
	min_cpu_load = DEFAULT_MIN_CPU_LOAD;

	spin_lock_init(&cpumask_lock);

	suspended = 0;

	/* Initalize per-cpu data: */
	for_each_possible_cpu(i) {
		this_smartass = &per_cpu(smartass_info, i);
		this_smartass->enable = 0;
		this_smartass->cur_policy = 0;
		this_smartass->force_ramp_up = 0;
		this_smartass->time_in_idle = 0;
		this_smartass->idle_exit_time = 0;
		this_smartass->freq_change_time = 0;
		this_smartass->freq_change_time_in_idle = 0;
		this_smartass->cur_cpu_load = 0;
		// intialize timer:
		init_timer_deferrable(&this_smartass->timer);
		this_smartass->timer.function = cpufreq_smartass_timer;
		this_smartass->timer.data = i;
		work_cpumask_test_and_clear(i);
	}

	// Scale up is high priority
	up_wq = create_rt_workqueue("ksmartass_up");
	down_wq = create_workqueue("ksmartass_down");
	if (!up_wq || !down_wq)
		return -ENOMEM;

	INIT_WORK(&freq_scale_work, cpufreq_smartass_freq_change_time_work);

	register_early_suspend(&smartass_power_suspend);

	return cpufreq_register_governor(&cpufreq_gov_smartass2);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SMARTASS2
fs_initcall(cpufreq_smartass_init);
#else
module_init(cpufreq_smartass_init);
#endif

static void __exit cpufreq_smartass_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_smartass2);
	destroy_workqueue(up_wq);
	destroy_workqueue(down_wq);
}

module_exit(cpufreq_smartass_exit);

MODULE_AUTHOR ("Erasmux");
MODULE_DESCRIPTION ("'cpufreq_smartass2' - A smart cpufreq governor");
MODULE_LICENSE ("GPL");

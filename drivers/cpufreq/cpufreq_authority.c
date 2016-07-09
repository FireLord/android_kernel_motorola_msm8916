/*
 *  drivers/cpufreq/cpufreq_authority.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Created by @Aayushrd7
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/new_exp_suspend.h>

/*
 * authority is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(20)
#define DEF_FREQUENCY_UP_THRESHOLD		(60)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(60)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define MIN_FREQUENCY_DOWN_DIFFERENTIAL		(5)

#define cputime64_sub(__a, __b)         ((__a) - (__b))

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(2)

static unsigned int min_sampling_rate;
static unsigned long stored_sampling_rate;


#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

#define POWERSAVE_BIAS_MAXLEVEL			(1000)
#define POWERSAVE_BIAS_MINLEVEL			(-1000)

static void do_authority_timer(struct work_struct *work);
static int cpufreq_governor_authority(struct cpufreq_policy *policy,
				unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_authority
static
#endif
struct cpufreq_governor cpufreq_gov_authority = {
       .name                   = "authority",
       .governor               = cpufreq_governor_authority,
       .owner                  = THIS_MODULE,
};

/* Sampling types */
enum {authority_NORMAL_SAMPLE, authority_SUB_SAMPLE};

struct cpu_authority_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;
	unsigned int rate_mult;
	int cpu;
	unsigned int sample_type:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_authority_timer invocation. We do not want do_authority_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_authority_info_s, od_cpu_authority_info);

static inline void authority_timer_init(struct cpu_authority_info_s *authority_info);
static inline void authority_timer_exit(struct cpu_authority_info_s *authority_info);

static unsigned int authority_enable;	/* number of CPUs using this policy */

/*
 * authority_mutex protects authority_enable in governor start/stop.
 */
static DEFINE_MUTEX(authority_mutex);

static struct workqueue_struct *input_wq;

static DEFINE_PER_CPU(struct work_struct, authority_refresh_work);

static struct authority_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int down_differential;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	int          powersave_bias;
	unsigned int io_is_busy;
	//static bool io_busy_true;
} authority_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.ignore_nice = 0,
	.io_is_busy = 0,
	.powersave_bias = 0,
};

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_jiffies,
 * freq_lo, and freq_lo_jiffies in percpu area for averaging freqs.
 */
static unsigned int powersave_bias_target(struct cpufreq_policy *policy,
					  unsigned int freq_next,
					  unsigned int relation)
{
	unsigned int freq_req, freq_avg;
	unsigned int freq_hi, freq_lo;
	unsigned int index = 0;
	unsigned int jiffies_total, jiffies_hi, jiffies_lo;
	int freq_reduc;
	struct cpu_authority_info_s *authority_info = &per_cpu(od_cpu_authority_info,
						   policy->cpu);

	if (!authority_info->freq_table) {
		authority_info->freq_lo = 0;
		authority_info->freq_lo_jiffies = 0;
		return freq_next;
	}

	cpufreq_frequency_table_target(policy, authority_info->freq_table, freq_next,
			relation, &index);
	freq_req = authority_info->freq_table[index].frequency;
	freq_reduc = freq_req * authority_tuners_ins.powersave_bias / 1000;
	freq_avg = freq_req - freq_reduc;

	/* Find freq bounds for freq_avg in freq_table */
	index = 0;
	cpufreq_frequency_table_target(policy, authority_info->freq_table, freq_avg,
			CPUFREQ_RELATION_H, &index);
	freq_lo = authority_info->freq_table[index].frequency;
	index = 0;
	cpufreq_frequency_table_target(policy, authority_info->freq_table, freq_avg,
			CPUFREQ_RELATION_L, &index);
	freq_hi = authority_info->freq_table[index].frequency;

	/* Find out how long we have to be in hi and lo freqs */
	if (freq_hi == freq_lo) {
		authority_info->freq_lo = 0;
		authority_info->freq_lo_jiffies = 0;
		return freq_lo;
	}
	jiffies_total = usecs_to_jiffies(authority_tuners_ins.sampling_rate);
	jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
	jiffies_hi += ((freq_hi - freq_lo) / 2);
	jiffies_hi /= (freq_hi - freq_lo);
	jiffies_lo = jiffies_total - jiffies_hi;
	authority_info->freq_lo = freq_lo;
	authority_info->freq_lo_jiffies = jiffies_lo;
	authority_info->freq_hi_jiffies = jiffies_hi;
	return freq_hi;
}

static int authority_powersave_bias_setspeed(struct cpufreq_policy *policy,
					    struct cpufreq_policy *altpolicy,
					    int level)
{
	if (level == POWERSAVE_BIAS_MAXLEVEL) {
		/* maximum powersave; set to lowest frequency */
		__cpufreq_driver_target(policy,
			(altpolicy) ? altpolicy->min : policy->min,
			CPUFREQ_RELATION_L);
		return 1;
	} else if (level == POWERSAVE_BIAS_MINLEVEL) {
		/* minimum powersave; set to highest frequency */
		__cpufreq_driver_target(policy,
			(altpolicy) ? altpolicy->max : policy->max,
			CPUFREQ_RELATION_H);
		return 1;
	}
	return 0;
}

static void authority_powersave_bias_init_cpu(int cpu)
{
	struct cpu_authority_info_s *authority_info = &per_cpu(od_cpu_authority_info, cpu);
	authority_info->freq_table = cpufreq_frequency_get_table(cpu);
	authority_info->freq_lo = 0;
}

static void authority_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		authority_powersave_bias_init_cpu(i);
	}
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_min);

/* cpufreq_authority Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", authority_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(io_is_busy, io_is_busy);
show_one(up_threshold, up_threshold);
show_one(down_differential, down_differential);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);

static ssize_t show_powersave_bias
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", authority_tuners_ins.powersave_bias);
}

/**
 * update_sampling_rate - update sampling rate effective immediately if needed.
 * @new_rate: new sampling rate
 *
 * If new rate is smaller than the old, simply updaing
 * authority_tuners_int.sampling_rate might not be appropriate. For example,
 * if the original sampling_rate was 1 second and the requested new sampling
 * rate is 10 ms because the user needs immediate reaction from ondemand
 * governor, but not sure if higher frequency will be required or not,
 * then, the governor may change the sampling rate too late; up to 1 second
 * later. Thus, if we are reducing the sampling rate, we need to make the
 * new value effective immediately.
 */
static void update_sampling_rate(unsigned int new_rate)
{
	int cpu;

	authority_tuners_ins.sampling_rate = new_rate
				     = max(new_rate, min_sampling_rate);

	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct cpu_authority_info_s *authority_info;
		unsigned long next_sampling, appointed_at;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		authority_info = &per_cpu(od_cpu_authority_info, policy->cpu);
		cpufreq_cpu_put(policy);

		mutex_lock(&authority_info->timer_mutex);

		if (!delayed_work_pending(&authority_info->work)) {
			mutex_unlock(&authority_info->timer_mutex);
			continue;
		}

		next_sampling  = jiffies + usecs_to_jiffies(new_rate);
		appointed_at = authority_info->work.timer.expires;


		if (time_before(next_sampling, appointed_at)) {

			mutex_unlock(&authority_info->timer_mutex);
			cancel_delayed_work_sync(&authority_info->work);
			mutex_lock(&authority_info->timer_mutex);

			schedule_delayed_work_on(authority_info->cpu, &authority_info->work,
						 usecs_to_jiffies(new_rate));

		}
		mutex_unlock(&authority_info->timer_mutex);
	}
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	update_sampling_rate(input);
	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	authority_tuners_ins.io_is_busy = !!input;
	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	authority_tuners_ins.up_threshold = input;
	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input >= authority_tuners_ins.up_threshold ||
			input < MIN_FREQUENCY_DOWN_DIFFERENTIAL) {
		return -EINVAL;
	}

	authority_tuners_ins.down_differential = input;

	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	authority_tuners_ins.sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct cpu_authority_info_s *authority_info;
		authority_info = &per_cpu(od_cpu_authority_info, j);
		authority_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == authority_tuners_ins.ignore_nice) { /* nothing to do */
		return count;
	}
	authority_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_authority_info_s *authority_info;
		authority_info = &per_cpu(od_cpu_authority_info, j);
		authority_info->prev_cpu_idle = get_cpu_idle_time(j,
						&authority_info->prev_cpu_wall, authority_tuners_ins.io_is_busy);
		if (authority_tuners_ins.ignore_nice)
			authority_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

static ssize_t store_powersave_bias(struct kobject *a, struct attribute *b,
                                    const char *buf, size_t count)
{
        unsigned int input;
        int ret;
        ret = sscanf(buf, "%u", &input);

        if (ret != 1)
                return -EINVAL;

        if (input > 1000)
                input = 1000;

        authority_tuners_ins.powersave_bias = input;
        authority_powersave_bias_init();
        return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(io_is_busy);
define_one_global_rw(up_threshold);
define_one_global_rw(down_differential);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(powersave_bias);

static struct attribute *authority_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_differential.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	NULL
};

static struct attribute_group authority_attr_group = {
	.attrs = authority_attributes,
	.name = "authority",
};

/************************** sysfs end ************************/

static void authority_freq_increase(struct cpufreq_policy *p, unsigned int freq)
{
	if (authority_tuners_ins.powersave_bias)
		freq = powersave_bias_target(p, freq, CPUFREQ_RELATION_H);
	else if (p->cur == p->max)
		return;

	__cpufreq_driver_target(p, freq, authority_tuners_ins.powersave_bias ?
			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);
}

static void authority_check_cpu(struct cpu_authority_info_s *this_authority_info)
{
	unsigned int max_load_freq;

	struct cpufreq_policy *policy;
	unsigned int j;

	this_authority_info->freq_lo = 0;
	policy = this_authority_info->cur_policy;

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate, we look for a the lowest
	 * frequency which can sustain the load while keeping idle time over
	 * 30%. If such a frequency exist, we try to decrease to this frequency.
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of current frequency
	 */

	/* Get Absolute Load - in terms of freq */
	max_load_freq = 0;

	for_each_cpu(j, policy->cpus) {
		struct cpu_authority_info_s *j_authority_info;
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		unsigned int load, load_freq;
		int freq_avg;

		j_authority_info = &per_cpu(od_cpu_authority_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time, authority_tuners_ins.io_is_busy);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
				j_authority_info->prev_cpu_wall);
		j_authority_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
				j_authority_info->prev_cpu_idle);
		j_authority_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int) cputime64_sub(cur_iowait_time,
				j_authority_info->prev_cpu_iowait);
		j_authority_info->prev_cpu_iowait = cur_iowait_time;

		if (authority_tuners_ins.ignore_nice) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
				j_authority_info->prev_cpu_nice;
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_authority_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];	
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		/*
		 * For the purpose of authority, waiting for disk IO is an
		 * indication that you're performance critical, and not that
		 * the system is actually idle. So subtract the iowait time
		 * from the cpu idle time.
		 */

		if (authority_tuners_ins.io_is_busy && idle_time >= iowait_time)
			idle_time -= iowait_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;

		freq_avg = __cpufreq_driver_getavg(policy, j);
		if (freq_avg <= 0)
			freq_avg = policy->cur;

		load_freq = load * freq_avg;
		if (load_freq > max_load_freq)
			max_load_freq = load_freq;
	}

	/* Check for frequency increase */
	if (max_load_freq > authority_tuners_ins.up_threshold * policy->cur) {
		/* If switching to max speed, apply sampling_down_factor */
		if (policy->cur < policy->max)
			this_authority_info->rate_mult =
				authority_tuners_ins.sampling_down_factor;
		authority_freq_increase(policy, policy->max);

		return;
	}
	/* Check for frequency decrease */
	/* if we cannot reduce the frequency anymore, break out early */
	if (policy->cur == policy->min)
		return;

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus 10 points under the threshold.
	 */
	if (max_load_freq <
	    (authority_tuners_ins.up_threshold - authority_tuners_ins.down_differential) *
	     policy->cur) {
		unsigned int freq_next;
		freq_next = max_load_freq /
				(authority_tuners_ins.up_threshold -
				 authority_tuners_ins.down_differential);

		/* No longer fully busy, reset rate_mult */
		this_authority_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (!authority_tuners_ins.powersave_bias) {
			__cpufreq_driver_target(policy, freq_next,
					CPUFREQ_RELATION_L);
		} else {
			int freq = powersave_bias_target(policy, freq_next,
					CPUFREQ_RELATION_L);
			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
	}
}

static void do_authority_timer(struct work_struct *work)
{
	struct cpu_authority_info_s *authority_info =
		container_of(work, struct cpu_authority_info_s, work.work);
	unsigned int cpu = authority_info->cpu;
	int sample_type = authority_info->sample_type;

	int delay;

	mutex_lock(&authority_info->timer_mutex);

	/* Common NORMAL_SAMPLE setup */
	authority_info->sample_type = authority_NORMAL_SAMPLE;
	if (!authority_tuners_ins.powersave_bias ||
	    sample_type == authority_NORMAL_SAMPLE) {
		authority_check_cpu(authority_info);
		if (authority_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			authority_info->sample_type = authority_SUB_SAMPLE;
			delay = authority_info->freq_hi_jiffies;
		} else {
			/* We want all CPUs to do sampling nearly on
			 * same jiffy
			 */
			delay = usecs_to_jiffies(authority_tuners_ins.sampling_rate
				* authority_info->rate_mult);

			if (num_online_cpus() > 1)
				delay -= jiffies % delay;
		}
	} else {
		__cpufreq_driver_target(authority_info->cur_policy,
			authority_info->freq_lo, CPUFREQ_RELATION_H);
		delay = authority_info->freq_lo_jiffies;
	}
	schedule_delayed_work_on(cpu, &authority_info->work, delay);
	mutex_unlock(&authority_info->timer_mutex);
}

static inline void authority_timer_init(struct cpu_authority_info_s *authority_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(authority_tuners_ins.sampling_rate);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	authority_info->sample_type = authority_NORMAL_SAMPLE;
	INIT_DEFERRABLE_WORK(&authority_info->work, do_authority_timer);
	schedule_delayed_work_on(authority_info->cpu, &authority_info->work, delay);
}

static inline void authority_timer_exit(struct cpu_authority_info_s *authority_info)
{
	cancel_delayed_work_sync(&authority_info->work);
}

/*
 * Not all CPUs want IO time to be accounted as busy; this dependson how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (androidlcom) calis this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
	/*
	 * For Intel, Core 2 (model 15) andl later have an efficient idle.
	 */
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
	    boot_cpu_data.x86 == 6 &&
	    boot_cpu_data.x86_model >= 15)
		return 1;
#endif
	return 0;
}

static unsigned int enable_authority_input_event;
static void authority_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	int i;

	if (enable_authority_input_event) {

		if ((authority_tuners_ins.powersave_bias == POWERSAVE_BIAS_MAXLEVEL) ||
			(authority_tuners_ins.powersave_bias == POWERSAVE_BIAS_MINLEVEL)) {
			/* nothing to do */
			return;
		}

		for_each_online_cpu(i) {
			queue_work_on(i, input_wq, &per_cpu(authority_refresh_work, i));
		}
	}
}

static int authority_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void authority_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id authority_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler authority_input_handler = {
	.event		= authority_input_event,
	.connect	= authority_input_connect,
	.disconnect	= authority_input_disconnect,
	.name		= "cpufreq_ond",
	.id_table	= authority_ids,
};

static int cpufreq_governor_authority(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_authority_info_s *this_authority_info;
	unsigned int j;
	int rc;

	this_authority_info = &per_cpu(od_cpu_authority_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&authority_mutex);

		authority_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpu_authority_info_s *j_authority_info;
			j_authority_info = &per_cpu(od_cpu_authority_info, j);
			j_authority_info->cur_policy = policy;

			j_authority_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_authority_info->prev_cpu_wall, authority_tuners_ins.io_is_busy);
			if (authority_tuners_ins.ignore_nice)
				j_authority_info->prev_cpu_nice =
					kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		}
		this_authority_info->cpu = cpu;
		this_authority_info->rate_mult = 1;
		authority_powersave_bias_init_cpu(cpu);
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (authority_enable == 1) {
			unsigned int latency;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&authority_attr_group);
			if (rc) {
				mutex_unlock(&authority_mutex);
				return rc;
			}

			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;
			/* Bring kernel and HW constraints together */
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			authority_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * LATENCY_MULTIPLIER);
			authority_tuners_ins.io_is_busy = should_io_be_busy();
		}
		if (!cpu)
			rc = input_register_handler(&authority_input_handler);
		mutex_unlock(&authority_mutex);

		mutex_init(&this_authority_info->timer_mutex);

		if (!authority_powersave_bias_setspeed(
					this_authority_info->cur_policy,
					NULL,
					authority_tuners_ins.powersave_bias))
			authority_timer_init(this_authority_info);
		break;

	case CPUFREQ_GOV_STOP:
		authority_timer_exit(this_authority_info);

		mutex_lock(&authority_mutex);
		mutex_destroy(&this_authority_info->timer_mutex);
		authority_enable--;
		/* If device is being removed, policy is no longer
		 * valid. */
		this_authority_info->cur_policy = NULL;
		if (!cpu)
			input_unregister_handler(&authority_input_handler);
		mutex_unlock(&authority_mutex);
		if (!authority_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &authority_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_authority_info->timer_mutex);
		if (policy->max < this_authority_info->cur_policy->cur)
			__cpufreq_driver_target(this_authority_info->cur_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_authority_info->cur_policy->cur)
			__cpufreq_driver_target(this_authority_info->cur_policy,
				policy->min, CPUFREQ_RELATION_L);
		else if (authority_tuners_ins.powersave_bias != 0)
			authority_powersave_bias_setspeed(
				this_authority_info->cur_policy,
				policy,
				authority_tuners_ins.powersave_bias);
		mutex_unlock(&this_authority_info->timer_mutex);
		break;
	}
	return 0;
}


static void cpufreq_authority_early_suspend(struct power_suspend *h)
{
	mutex_lock(&authority_mutex);
	stored_sampling_rate = min_sampling_rate;
	min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE * 2;
	mutex_unlock(&authority_mutex);
}

static void cpufreq_authority_late_resume(struct power_suspend *h)
{
	mutex_lock(&authority_mutex);
	min_sampling_rate = stored_sampling_rate;
	mutex_unlock(&authority_mutex);
}

static struct power_suspend cpufreq_authority_early_suspend_info = {
	.suspend = cpufreq_authority_early_suspend,
	.resume = cpufreq_authority_late_resume,
	.level = POWER_SUSPEND_LEVEL_DISABLE_FB+1,
};


static int __init cpufreq_gov_authority_init(void)
{
	u64 idle_time;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		authority_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		authority_tuners_ins.down_differential =
					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		/*
		 * In no_hz/micro accounting case we set the minimum frequency
		 * not depending on HZ, but fixed (very low). The deferred
		 * timer might skip some samples if idle/sleeping as needed.
		*/
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		/* For correct statistics, we need 10 ticks for each measure */
		min_sampling_rate =
			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(1);
	}


	register_power_suspend(&cpufreq_authority_early_suspend_info);

	return cpufreq_register_governor(&cpufreq_gov_authority);
}

static void __exit cpufreq_gov_authority_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_authority);

	unregister_power_suspend(&cpufreq_authority_early_suspend_info);

	destroy_workqueue(input_wq);
}

static int set_enable_authority_input_event_param(const char *val, struct kernel_param *kp)
{
	int ret = 0;

	ret = param_set_uint(val, kp);
	if (ret)
		pr_err("%s: error setting value %d\n", __func__, ret);

	return ret;
}
module_param_call(enable_authority_input_event, set_enable_authority_input_event_param, param_get_uint,
		&enable_authority_input_event, S_IWUSR | S_IRUGO);

MODULE_AUTHOR("Aayushrd7 <aayush835@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_authority' - An intelligent dynamic cpufreq governor for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_authority
fs_initcall(cpufreq_gov_authority_init);
#else
module_init(cpufreq_gov_authority_init);
#endif
module_exit(cpufreq_gov_authority_exit);

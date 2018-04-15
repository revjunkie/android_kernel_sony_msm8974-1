/*
 * Rev Hotplug Driver
 *
 * Copyright (c) 2015, Raj Ibrahim <rajibrahim@rocketmail.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/tick.h>

static struct rev_tune
{
	unsigned int active;
	unsigned int sample_time;
	unsigned int min_cpu;
	unsigned int max_cpu;
	unsigned int unplug_rate;
	unsigned int debug;
	unsigned int cpu_load[];
} rev = {
	.active = 1,
	.sample_time = 100,
	.min_cpu = 1,
	.max_cpu = CONFIG_NR_CPUS,
	.unplug_rate = 2000,
	.debug = 0,
	.cpu_load = {90, 45, 120, 180, 150},
};

static struct rev_data
{
	unsigned int buffer_size;
	unsigned int buffer;
	unsigned int avg_load;
	cputime64_t now;
	unsigned int buffer_load[];
} data = {
	.buffer_size = 4,
};

struct cpu_info
{
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
};

static struct delayed_work hotplug_work;
static DEFINE_PER_CPU(struct cpu_info, rev_info);
static DEFINE_MUTEX(hotplug_lock);

#define REV_INFO(msg...)	\
do { 				\
	if (rev.debug)		\
		pr_info(msg);	\
} while (0)

static void __ref plug_cpu(int max_cpu)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		if (num_online_cpus() == max_cpu)
			break;
		if (cpu_online(cpu))
			continue; 
			cpu_up(cpu);
			REV_INFO("CPU %u online\n", cpu);
	}
}

static void unplug_cpu(void)
{
	unsigned int cpu, idle;

	if (ktime_to_ms(ktime_get()) < data.now + rev.unplug_rate)
		return;
	for (cpu = CONFIG_NR_CPUS - 1; cpu > rev.min_cpu - 1; cpu--) {
		if (!(cpu_online(cpu)))
			continue;
			idle = idle_cpu(cpu);
			REV_INFO("CPU %u idle state %d\n", cpu, idle);
			if (idle > 0) {
				cpu_down(cpu);
				REV_INFO("Offline cpu %d\n", cpu);
			}
	}
	data.now = ktime_to_ms(ktime_get());
}
#ifdef CONFIG_MSM_RUN_QUEUE_STATS
extern cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall);
#endif
static unsigned int get_load_cpu(unsigned int cpu)
{
	unsigned int wall_time, idle_time, load, load_at_freq;
	cputime64_t cur_wall_time, cur_idle_time;
	struct cpu_info *pcpu = &per_cpu(rev_info, cpu);
	struct cpufreq_policy policy;
	int ret;
	
	ret = cpufreq_get_policy(&policy, cpu);
	if (ret)
		return -EINVAL;
#ifdef CONFIG_MSM_RUN_QUEUE_STATS
	cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time);
#else
	cur_idle_time = get_cpu_idle_time_us(cpu, &cur_wall_time);
#endif
	wall_time = (unsigned int) (cur_wall_time - pcpu->prev_cpu_wall);
	pcpu->prev_cpu_wall = cur_wall_time;
	idle_time = (unsigned int) (cur_idle_time - pcpu->prev_cpu_idle);
	pcpu->prev_cpu_idle = cur_idle_time;
	
	if (unlikely(wall_time < 0 || wall_time < idle_time))
		return 0;
	load = 100 * (wall_time - idle_time) / wall_time;
	load_at_freq = (load * policy.cur) / policy.user_policy.max;
		REV_INFO("CPU%u: usage: %d cur: %d max: %d\n", cpu, load,
				policy.cur, policy.user_policy.max);

	return load_at_freq;
}

static void get_avg_load(void)
{
	unsigned int cpu, i, j;
	unsigned int total_load = 0, load = 0;
	
	mutex_lock(&hotplug_lock);
	for_each_online_cpu(cpu)
		total_load += get_load_cpu(cpu);
		
	data.buffer_load[data.buffer] = total_load;
	for (i = 0, j = data.buffer; i < data.buffer_size; i++, j--) {
		load += data.buffer_load[j];
		REV_INFO("load sample for %u = %u\n", i, data.buffer_load[j]);
		if (j == 0)
			j = data.buffer_size;
	}
	if (++data.buffer == data.buffer_size)
		data.buffer = 0;

	data.avg_load = load / data.buffer_size;
	mutex_unlock(&hotplug_lock);
}
		
static void __ref hotplug_decision_work(struct work_struct *work)
{
	unsigned int online_cpus = num_online_cpus();
	
	get_avg_load();
	REV_INFO("rev_hotplug - Load: %d Online CPUs: %d\n",
			data.avg_load, online_cpus);

	if (data.avg_load >= rev.cpu_load[online_cpus] &&
					online_cpus < rev.max_cpu) {
		data.now = ktime_to_ms(ktime_get());
		if (data.avg_load >= rev.cpu_load[4])
			plug_cpu(rev.max_cpu);
		else
			plug_cpu(++online_cpus);
			
	} else if (data.avg_load <= rev.cpu_load[0] &&
					online_cpus > rev.min_cpu)
		unplug_cpu();
	
    if (rev.active)
	schedule_delayed_work_on(0, &hotplug_work,
				msecs_to_jiffies(rev.sample_time));
}

/**************SYSFS*******************/

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct device * dev, struct device_attribute * attr, char * buf)		\
{									\
	return sprintf(buf, "%u\n", rev.object);			\
}
show_one(active, active);
show_one(sample_time, sample_time);
show_one(min_cpu, min_cpu);
show_one(max_cpu, max_cpu);
show_one(unplug_rate, unplug_rate);
show_one(debug, debug);

static ssize_t __ref store_active(struct device * dev,
                        struct device_attribute * attr,
                        const char * buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	rev.active = input > 1 ? 1 : input;
	if (rev.active) {
		schedule_delayed_work_on(0, &hotplug_work,
				msecs_to_jiffies(rev.sample_time));
	} else {
		plug_cpu(CONFIG_NR_CPUS);
		cancel_delayed_work(&hotplug_work);
	}
	return count;
}

static ssize_t show_load_threshold(struct device * dev,
                            struct device_attribute * attr, char * buf)
{					
	return sprintf(buf, "%u %u %u %u %u\n", rev.cpu_load[0], 
		rev.cpu_load[1], rev.cpu_load[2], rev.cpu_load[3], 
			rev.cpu_load[4]);	
}

static ssize_t store_load_threshold(struct device * dev,
                        struct device_attribute * attr,
                        const char * buf, size_t count)
{
	unsigned int input, input1, input2, input3, input4;
	int ret;

	ret = sscanf(buf, "%u %u %u %u %u", &input, &input1,
			&input2, &input3, &input4);
	if (ret != 5 || input > 200 || input1 > 100 || input2 > 200 || 
				input3 > 290 || input4 > 400)
		return -EINVAL;  
	
	rev.cpu_load[0] = input;
	rev.cpu_load[1] = input1;
	rev.cpu_load[2] = input2;
	rev.cpu_load[3] = input3;
	rev.cpu_load[4] = input4;
	
	return count;
}

#define store_one(file_name, object)					\
static ssize_t store_##file_name					\
(struct device * dev, struct device_attribute * attr, \
                        const char * buf, size_t count)	\
{									\
	unsigned int input;						\
	int ret;							\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	rev.object = input;						\
	return count;							\
}									
store_one(sample_time, sample_time);
store_one(min_cpu, min_cpu);
store_one(max_cpu, max_cpu);
store_one(unplug_rate, unplug_rate);
store_one(debug, debug);

static DEVICE_ATTR(active, 0644, show_active, store_active);
static DEVICE_ATTR(sample_time, 0644, show_sample_time, store_sample_time);
static DEVICE_ATTR(min_cpu, 0644, show_min_cpu, store_min_cpu);
static DEVICE_ATTR(max_cpu, 0644, show_max_cpu, store_max_cpu);
static DEVICE_ATTR(unplug_rate, 0644, show_unplug_rate, store_unplug_rate);
static DEVICE_ATTR(debug, 0644, show_debug, store_debug);
static DEVICE_ATTR(load_threshold, 0644, show_load_threshold, store_load_threshold);

static struct attribute *rev_hotplug_attributes[] =
{
	&dev_attr_active.attr,
	&dev_attr_sample_time.attr,
	&dev_attr_min_cpu.attr,
	&dev_attr_max_cpu.attr,
	&dev_attr_unplug_rate.attr,
	&dev_attr_debug.attr,
	&dev_attr_load_threshold.attr,
	NULL
};

static struct attribute_group rev_hotplug_group =
{
	.attrs  = rev_hotplug_attributes,
};

static int __init rev_hotplug_init(void)
{
    struct kobject *rev_kobj;
	int ret;

	INIT_DELAYED_WORK(&hotplug_work, hotplug_decision_work);
    schedule_delayed_work_on(0, &hotplug_work, HZ * 30);

	rev_kobj = kobject_create_and_add("rev_hotplug", kernel_kobj);
	if (!rev_kobj) {
        pr_err("Failed to create kobject\n");
		return -ENOMEM;
	}
    ret = sysfs_create_group(rev_kobj, &rev_hotplug_group);
	if (ret) {
		pr_err("Failed to create sysfs interface\n");
		kobject_put(rev_kobj);
        goto err;
	}
	return 0;
err:
    return ret;
}

late_initcall(rev_hotplug_init);

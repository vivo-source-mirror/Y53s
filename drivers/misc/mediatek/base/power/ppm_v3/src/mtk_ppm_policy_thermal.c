/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/topology.h>

#include "mtk_ppm_internal.h"


static void ppm_thermal_update_limit_cb(void);
static void ppm_thermal_status_change_cb(bool enable);
#ifdef PPM_THERMAL_USER_LIMIT
static bool ppm_thermal_is_policy_active(void);
#endif

static struct ppm_cluster_status *cluster_status;

/* other members will init by ppm_main */
static struct ppm_policy_data thermal_policy = {
	.name			= __stringify(PPM_POLICY_THERMAL),
	.lock			= __MUTEX_INITIALIZER(thermal_policy.lock),
	.policy			= PPM_POLICY_THERMAL,
	.priority		= PPM_POLICY_PRIO_POWER_BUDGET_BASE,
	.update_limit_cb	= ppm_thermal_update_limit_cb,
	.status_change_cb	= ppm_thermal_status_change_cb,
};

#ifdef PPM_THERMAL_USER_LIMIT
static struct ppm_thermal_data {
	struct ppm_user_limit limit[NR_PPM_CLUSTERS];
} thermal_limit_data;
#endif

void mt_ppm_cpu_thermal_protect(unsigned int limited_power)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("Get budget from thermal => limited_power = %d\n",
		limited_power);

	ppm_lock(&thermal_policy.lock);

	if (!thermal_policy.is_enabled) {
		ppm_warn("@%s: thermal policy is not enabled!\n", __func__);
		ppm_unlock(&thermal_policy.lock);
		goto end;
	}

	thermal_policy.req.power_budget = limited_power;
#ifdef PPM_THERMAL_USER_LIMIT
	thermal_policy.is_activated = ppm_thermal_is_policy_active();
#else	
	thermal_policy.is_activated = (limited_power) ? true : false;
#endif
	ppm_unlock(&thermal_policy.lock);

#ifdef PPM_THERMAL_ENHANCEMENT
	if (mutex_is_locked(&ppm_main_info.lock))
		goto end;
#endif

	mt_ppm_main();

end:
	FUNC_EXIT(FUNC_LV_POLICY);
}

#ifdef PPM_THERMAL_USER_LIMIT
static bool ppm_thermal_is_policy_active(void)
{
	int i;

	for_each_ppm_clusters(i) {
		if (thermal_limit_data.limit[i].min_freq_idx != -1 ||
		    thermal_limit_data.limit[i].max_freq_idx != -1 ||
		    thermal_limit_data.limit[i].min_core_num != -1 ||
		    thermal_limit_data.limit[i].max_core_num != -1) {
			return true;
		}
	}
	if (thermal_policy.req.power_budget != 0)
		return true;
	return false;
}
void mt_ppm_thermal_set_freq_limit(unsigned int cluster, int min_freq_idx, int max_freq_idx)
{
	if (cluster >= NR_PPM_CLUSTERS) {
		ppm_err("Invalid input: cluster = %d\n", cluster);
		return;
	}

	if ((max_freq_idx != -1 &&
		(max_freq_idx < get_cluster_max_cpufreq_idx(cluster) ||
		max_freq_idx > get_cluster_min_cpufreq_idx(cluster)))
		|| (min_freq_idx != -1
		&& (min_freq_idx > get_cluster_min_cpufreq_idx(cluster) ||
		min_freq_idx < get_cluster_max_cpufreq_idx(cluster)))) {
		ppm_err("Invalid input: cl=%d, min/max freqidx=%d/%d\n",
			cluster, min_freq_idx, max_freq_idx);
		return;
	}

	ppm_info("set cpu freq boundry in thermal policy: cluster %d min/max freqidx = %d/%d\n",
		cluster, min_freq_idx, max_freq_idx);

	ppm_lock(&thermal_policy.lock);

	if (!thermal_policy.is_enabled) {
		ppm_warn("@%s: thermal policy is not enabled!\n", __func__);
		ppm_unlock(&thermal_policy.lock);
		return;
	}

	if (min_freq_idx < max_freq_idx && max_freq_idx != -1 && min_freq_idx != -1)
		min_freq_idx = max_freq_idx;

	/* update cpu freq setting */
	thermal_limit_data.limit[cluster].min_freq_idx = min_freq_idx;
	thermal_limit_data.limit[cluster].max_freq_idx = max_freq_idx;

	/* error check */
	if (thermal_limit_data.limit[cluster].min_freq_idx != -1
		&& thermal_limit_data.limit[cluster].min_freq_idx
		< thermal_limit_data.limit[cluster].max_freq_idx)
		thermal_limit_data.limit[cluster].min_freq_idx =
			thermal_limit_data.limit[cluster].max_freq_idx;

	thermal_policy.is_activated = ppm_thermal_is_policy_active();

	if (thermal_policy.is_activated == false)
		ppm_clear_policy_limit(&thermal_policy);

	ppm_unlock(&thermal_policy.lock);
	mt_ppm_main();
}

void mt_ppm_thermal_set_core_limit(unsigned int cluster, int min_core, int max_core)
{
	if (cluster >= NR_PPM_CLUSTERS) {
		ppm_err("Invalid input: cluster = %d\n", cluster);
		return;
	}

	if ((max_core != -1 && (max_core > get_cluster_max_cpu_core(cluster)
		|| max_core < get_cluster_min_cpu_core(cluster)))
		|| (min_core != -1
		&& (min_core < get_cluster_min_cpu_core(cluster) ||
		min_core > get_cluster_max_cpu_core(cluster)))) {
		ppm_err("Invalid input: cl=%d/%d, min/max core=%d/%d\n",
			cluster, min_core, max_core);
		return;
	}

	ppm_info("set cpu core boundry in thermal policy: cluster %d min/max core = %d/%d\n",
					cluster, min_core, max_core);

	if (min_core > max_core && max_core != -1)
		min_core = max_core;

	ppm_lock(&thermal_policy.lock);

	if (!thermal_policy.is_enabled) {
		ppm_warn("@%s: thermal policy is not enabled!\n", __func__);
		ppm_unlock(&thermal_policy.lock);
		return;
	}

	/* update cpu core setting */
	thermal_limit_data.limit[cluster].min_core_num = min_core;
	thermal_limit_data.limit[cluster].max_core_num = max_core;

	/* error check */
	if (thermal_limit_data.limit[cluster].max_core_num != -1
		&& thermal_limit_data.limit[cluster].max_core_num
		< thermal_limit_data.limit[cluster].min_core_num)
		thermal_limit_data.limit[cluster].max_core_num =
			thermal_limit_data.limit[cluster].min_core_num;

	thermal_policy.is_activated = ppm_thermal_is_policy_active();

	if (thermal_policy.is_activated == false)
		ppm_clear_policy_limit(&thermal_policy);

	ppm_unlock(&thermal_policy.lock);
	mt_ppm_main();
}
#endif


unsigned int mt_ppm_thermal_get_min_power(void)
{
	return (unsigned int)ppm_get_min_pwr_idx();
}

unsigned int mt_ppm_thermal_get_max_power(void)
{
	return (unsigned int)ppm_get_max_pwr_idx();
}

unsigned int mt_ppm_thermal_get_cur_power(void)
{
#ifndef NO_SCHEDULE_API
	struct cpumask cluster_cpu, online_cpu;
#endif
	int i;
	int power;

	/* skip if DVFS is not ready (we cannot get current freq...) */
	if (!ppm_main_info.client_info[PPM_CLIENT_DVFS].limit_cb)
		return 0;

	if (!cluster_status)
		return mt_ppm_thermal_get_max_power();

	for_each_ppm_clusters(i) {
#ifndef NO_SCHEDULE_API
		arch_get_cluster_cpus(&cluster_cpu, i);
		cpumask_and(&online_cpu, &cluster_cpu, cpu_online_mask);

		cluster_status[i].core_num = cpumask_weight(&online_cpu);
#else
		cluster_status[i].core_num = get_cluster_max_cpu_core(i);
#endif
		cluster_status[i].volt = 0;	/* don't care */
		if (!cluster_status[i].core_num)
			cluster_status[i].freq_idx = -1;
		else
			cluster_status[i].freq_idx = ppm_main_freq_to_idx(
					i,
					mt_cpufreq_get_cur_phy_freq_no_lock(i),
					CPUFREQ_RELATION_L);

		ppm_ver("[%d] core = %d, freq_idx = %d\n",
			i, cluster_status[i].core_num,
			cluster_status[i].freq_idx);
	}

	power = ppm_find_pwr_idx(cluster_status);

	return (power == -1)
		? mt_ppm_thermal_get_max_power() : (unsigned int)power;
}

static void ppm_thermal_update_limit_cb(void)
{
#ifdef PPM_THERMAL_USER_LIMIT
	int i;
#endif	
	
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_clear_policy_limit(&thermal_policy);

	/* update limit according to power budget */
	if (thermal_policy.req.power_budget != 0)
		ppm_update_req_by_pwr(&thermal_policy.req);
#ifdef PPM_THERMAL_USER_LIMIT
	for_each_ppm_clusters(i) {
		if (thermal_limit_data.limit[i].min_freq_idx != -1 && 
			thermal_policy.req.limit[i].min_cpufreq_idx >
			thermal_limit_data.limit[i].min_freq_idx)
			thermal_policy.req.limit[i].min_cpufreq_idx =
				thermal_limit_data.limit[i].min_freq_idx;
		if (thermal_limit_data.limit[i].max_freq_idx != -1 &&
			thermal_policy.req.limit[i].max_cpufreq_idx <
			thermal_limit_data.limit[i].max_freq_idx)
			thermal_policy.req.limit[i].max_cpufreq_idx =
				thermal_limit_data.limit[i].max_freq_idx;
		if (thermal_limit_data.limit[i].min_core_num != -1 &&
			thermal_policy.req.limit[i].min_cpu_core <
			thermal_limit_data.limit[i].min_core_num)
			thermal_policy.req.limit[i].min_cpu_core =
				thermal_limit_data.limit[i].min_core_num;
		if (thermal_limit_data.limit[i].max_core_num != -1 &&
			thermal_policy.req.limit[i].max_cpu_core >
			thermal_limit_data.limit[i].max_core_num)
			thermal_policy.req.limit[i].max_cpu_core =
				thermal_limit_data.limit[i].max_core_num;
		if (thermal_policy.req.limit[i].min_cpu_core 
			> thermal_policy.req.limit[i].max_cpu_core)
			thermal_policy.req.limit[i].min_cpu_core =
				thermal_policy.req.limit[i].max_cpu_core;
	}
#endif

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_thermal_status_change_cb(bool enable)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("thermal policy status changed to %d\n", enable);

	FUNC_EXIT(FUNC_LV_POLICY);
}

static int ppm_thermal_limit_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "limited power = %d\n",
		thermal_policy.req.power_budget);
	seq_printf(m, "PPM thermal activate = %d\n",
		thermal_policy.is_activated);

	return 0;
}

static ssize_t ppm_thermal_limit_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int limited_power = 0;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &limited_power))
#ifdef PPM_SSPM_SUPPORT
		ppm_ipi_thermal_limit_test(limited_power);
#else
		mt_ppm_cpu_thermal_protect(limited_power);
#endif
	else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

static int ppm_thermal_cur_power_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "current power = %d\n", mt_ppm_thermal_get_cur_power());
	seq_printf(m, "min power = %d\n", mt_ppm_thermal_get_min_power());
	seq_printf(m, "max power = %d\n", mt_ppm_thermal_get_max_power());

	return 0;
}

PROC_FOPS_RW(thermal_limit);
PROC_FOPS_RO(thermal_cur_power);

static int __init ppm_thermal_policy_init(void)
{
	int i, ret = 0;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(thermal_limit),
		PROC_ENTRY(thermal_cur_power),
	};

	FUNC_ENTER(FUNC_LV_POLICY);

	/* create procfs */
	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, 0644,
			policy_dir, entries[i].fops)) {
			ppm_err("%s(), create /proc/ppm/policy/%s failed\n",
				__func__, entries[i].name);
			ret = -EINVAL;
			goto out;
		}
	}

	cluster_status = kcalloc(ppm_main_info.cluster_num,
		sizeof(*cluster_status), GFP_KERNEL);
	if (!cluster_status) {
		ret = -ENOMEM;
		goto out;
	}

	if (ppm_main_register_policy(&thermal_policy)) {
		ppm_err("@%s: thermal policy register failed\n", __func__);
		kfree(cluster_status);
		ret = -EINVAL;
		goto out;
	}

#ifdef PPM_THERMAL_USER_LIMIT
	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		thermal_limit_data.limit[i].min_freq_idx = -1;
		thermal_limit_data.limit[i].max_freq_idx = -1;
		thermal_limit_data.limit[i].min_core_num = -1;
		thermal_limit_data.limit[i].max_core_num = -1;
	}
#endif

	ppm_info("@%s: register %s done!\n", __func__, thermal_policy.name);

out:
	FUNC_EXIT(FUNC_LV_POLICY);

	return ret;
}

static void __exit ppm_thermal_policy_exit(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	kfree(cluster_status);

	ppm_main_unregister_policy(&thermal_policy);

	FUNC_EXIT(FUNC_LV_POLICY);
}

module_init(ppm_thermal_policy_init);
module_exit(ppm_thermal_policy_exit);


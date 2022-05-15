/*
 * Copyright (C) 2019 MediaTek Inc.
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

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>

#include "mt-plat/mtk_thermal_monitor.h"
#include <tscpu_settings.h>
#include <mtk_ppm_api.h>
#include <mtk_ppm_platform.h>
#include "mtk_ppm_internal.h"
#include <mtk_gpu_utility.h>
#include <mtk_gpufreq_plat.h>

/* init config */
#define CPU_FCT_UNLIMIT (-1)
#define GPU_FCT_UNLIMIT (-1)


#define CGPU_FCT_COOLER_NR 10

#define MTK_CL_CPU_FCT_GET_CURR_STATE(curr_state, state) \
{ curr_state = (((unsigned long) (state))&0xFFFF); }

#define MTK_CL_CPU_FCT_SET_CURR_STATE(curr_state, state) \
do { \
	if (curr_state == 0) \
		state &= ~0x1; \
	else \
		state |= 0x1; \
} while (0)

#define mtk_cooler_cpu_fct_dprintk(fmt, args...) \
	do { \
		if (cl_cpu_fct_klog_on == 1) \
			pr_notice("[Thermal/TC/fps]" fmt, ##args); \
	} while (0)


struct thermal_gpu_fct_limit {
	int max_opp;
	int min_opp;
};

struct thermal_cpu_fct_limit {
	int core_num[NR_PPM_CLUSTERS];
	int max_opp[NR_PPM_CLUSTERS];
	int min_opp[NR_PPM_CLUSTERS];
};

char *cpu_fct_cooler_name = "mtk-cl-cgpufct";

static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);

static int cl_cpu_fct_klog_on;
static struct thermal_cooling_device
*cl_cpu_fct_dev[CGPU_FCT_COOLER_NR] = { 0 };
static unsigned long cl_cpu_state[CGPU_FCT_COOLER_NR] = { 0 };
static struct thermal_cpu_fct_limit cl_cpu_limit[CGPU_FCT_COOLER_NR];
static struct thermal_cpu_fct_limit curr_cpu_limit;
static struct thermal_gpu_fct_limit cl_gpu_limit[CGPU_FCT_COOLER_NR];
static struct thermal_gpu_fct_limit curr_gpu_limit;

static int mtk_cl_cpu_fct_get_max_state
(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = 1;
	return 0;
}

static int mtk_cl_cpu_fct_get_cur_state
(struct thermal_cooling_device *cdev, unsigned long *state)
{
	MTK_CL_CPU_FCT_GET_CURR_STATE(*state, *((unsigned long *) cdev->devdata));
	mtk_cooler_cpu_fct_dprintk("[%s] %s %lu\n", __func__, cdev->type, *state);
	*state = *((unsigned long *)cdev->devdata);
	return 0;
}

static void mtk_cl_set_cpu_fct_limit(void)
{
	unsigned long curr_state;
	int i, j;

	curr_gpu_limit.max_opp = GPU_FCT_UNLIMIT;
	curr_gpu_limit.min_opp = GPU_FCT_UNLIMIT;
	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		curr_cpu_limit.core_num[i] = CPU_FCT_UNLIMIT;
		curr_cpu_limit.max_opp[i] = CPU_FCT_UNLIMIT;
		curr_cpu_limit.min_opp[i] = CPU_FCT_UNLIMIT;
	}

	for (i = 0; i < CGPU_FCT_COOLER_NR; i++) {		
		MTK_CL_CPU_FCT_GET_CURR_STATE(curr_state, cl_cpu_state[i]);
		if (curr_state == 1) {
			for (j = 0; j < NR_PPM_CLUSTERS; j++) {

				if (cl_cpu_limit[i].core_num[j] != CPU_FCT_UNLIMIT &&
					curr_cpu_limit.core_num[j] != CPU_FCT_UNLIMIT)
					curr_cpu_limit.core_num[j] =
						MIN(curr_cpu_limit.core_num[j],
						cl_cpu_limit[i].core_num[j]);
				else if (cl_cpu_limit[i].core_num[j] != CPU_FCT_UNLIMIT)
					curr_cpu_limit.core_num[j] = cl_cpu_limit[i].core_num[j];

				if (cl_cpu_limit[i].max_opp[j] != CPU_FCT_UNLIMIT &&
					curr_cpu_limit.max_opp[j] != CPU_FCT_UNLIMIT)
					curr_cpu_limit.max_opp[j] =
						MAX(curr_cpu_limit.max_opp[j],
						cl_cpu_limit[i].max_opp[j]);
				else if (cl_cpu_limit[i].max_opp[j] != CPU_FCT_UNLIMIT)
					curr_cpu_limit.max_opp[j] = cl_cpu_limit[i].max_opp[j];

				if (cl_cpu_limit[i].min_opp[j] != CPU_FCT_UNLIMIT &&
					curr_cpu_limit.min_opp[j] != CPU_FCT_UNLIMIT)
					curr_cpu_limit.min_opp[j] =
						MIN(curr_cpu_limit.min_opp[j],
						cl_cpu_limit[i].min_opp[j]);
				else if (cl_cpu_limit[i].min_opp[j] != CPU_FCT_UNLIMIT)
					curr_cpu_limit.min_opp[j] = cl_cpu_limit[i].min_opp[j];
			}

		if (cl_gpu_limit[i].max_opp != GPU_FCT_UNLIMIT &&
			curr_gpu_limit.max_opp != GPU_FCT_UNLIMIT)
				curr_gpu_limit.max_opp =
					MAX(curr_gpu_limit.max_opp,
					cl_gpu_limit[i].max_opp);
		else if (cl_gpu_limit[i].max_opp != GPU_FCT_UNLIMIT)
			curr_gpu_limit.max_opp = cl_gpu_limit[i].max_opp;

		if (cl_gpu_limit[i].min_opp != GPU_FCT_UNLIMIT &&
			curr_gpu_limit.min_opp != GPU_FCT_UNLIMIT)
			curr_gpu_limit.min_opp =
				MIN(curr_gpu_limit.min_opp,
					cl_gpu_limit[i].min_opp);
		else if (cl_gpu_limit[i].min_opp != GPU_FCT_UNLIMIT)
			curr_gpu_limit.min_opp = cl_gpu_limit[i].min_opp;
		}
	}

	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		mt_ppm_thermal_set_freq_limit(i,
			curr_cpu_limit.min_opp[i], curr_cpu_limit.max_opp[i]);
		mt_ppm_thermal_set_core_limit(i,
			CPU_FCT_UNLIMIT, curr_cpu_limit.core_num[i]);
	}

	if (curr_gpu_limit.max_opp == GPU_FCT_UNLIMIT)
		curr_gpu_limit.max_opp = 0;
	if (curr_gpu_limit.min_opp == GPU_FCT_UNLIMIT)
		curr_gpu_limit.min_opp = (int)(mt_gpufreq_get_dvfs_table_num() - 1);

	mtk_custom_boost_gpu_freq(curr_gpu_limit.min_opp);
	mtk_custom_upbound_gpu_freq(curr_gpu_limit.max_opp);

	return;
}

static int mtk_cl_cpu_fct_set_cur_state
(struct thermal_cooling_device *cdev, unsigned long state)
{
	mtk_cooler_cpu_fct_dprintk("[%s] %s %lu\n", __func__, cdev->type, state);
	if (state == *(unsigned long *)cdev->devdata)
		return 0;

	MTK_CL_CPU_FCT_SET_CURR_STATE(state, *((unsigned long *) cdev->devdata));
	mtk_cl_set_cpu_fct_limit();

	return 0;
}

/* bind fan callbacks to fan device */
static struct thermal_cooling_device_ops mtk_cl_cpu_fct_ops = {
	.get_max_state = mtk_cl_cpu_fct_get_max_state,
	.get_cur_state = mtk_cl_cpu_fct_get_cur_state,
	.set_cur_state = mtk_cl_cpu_fct_set_cur_state,
};

static int cl_cgpu_fct_setting_read(struct seq_file *m, void *v)
{
	int i, j;

	seq_printf(m, "current limit:\n");
	seq_printf(m, "gpu_min/gpu_max opp idx=%d/%d\n",
			curr_gpu_limit.min_opp,
			curr_gpu_limit.max_opp);

	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		seq_printf(m, "cluster%02d: core num=%d, min/max opp idx=%d/%d\n",
			i, curr_cpu_limit.core_num[i],
			curr_cpu_limit.min_opp[i],
			curr_cpu_limit.max_opp[i]);
	}
	seq_puts(m, "\n");

	for (i = 0; i < CGPU_FCT_COOLER_NR; i++) {
		seq_printf(m, "%s%02d: state=%d\n", cpu_fct_cooler_name, i, cl_cpu_state[i]);
		seq_printf(m, "min/max opp idx=%d/%d\n", 
			cl_gpu_limit[i].min_opp, cl_gpu_limit[i].max_opp);
		for (j = 0; j < NR_PPM_CLUSTERS; j++) {
			seq_printf(m, "cluster%02d: core num=%d,min/max opp idx=%d/%d\n",
				j, cl_cpu_limit[i].core_num[j],
				cl_cpu_limit[i].min_opp[j],
				cl_cpu_limit[i].max_opp[j]);
		}
		seq_puts(m, "\n");
	}
	return 0;
}

static ssize_t cl_cgpu_fct_setting_write
(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[128];
	int i, len = 0;
	int limit_id, num_cluster, core_num[3], max_opp[3], min_opp[3];
	int gpu_max_opp, gpu_min_opp;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';
	if (sscanf(desc, "%d %d %d %d %d %d %d %d %d %d %d %d %d",
		&limit_id, &num_cluster,
		&core_num[0], &min_opp[0], &max_opp[0],
		&core_num[1], &min_opp[1], &max_opp[1],
		&core_num[2], &min_opp[2], &max_opp[2],
		&gpu_min_opp, &gpu_max_opp) == 13) {
		if (limit_id < 0 || limit_id >= CGPU_FCT_COOLER_NR) {
			tscpu_printk("Bad arg: limit_id error\n");
			goto BAD_ARG;
		}

		if (num_cluster != NR_PPM_CLUSTERS) {
			tscpu_printk("Bad arg: Total number of clusters doesn't match\n");
			goto BAD_ARG;
		}

		for (i = 0; i < NR_PPM_CLUSTERS; i++) {
			cl_cpu_limit[limit_id].core_num[i] = core_num[i];
			if (max_opp[i] > 50 || min_opp[i] > 50) {
				cl_cpu_limit[limit_id].max_opp[i] = ppm_main_freq_to_idx(i, max_opp[i], CPUFREQ_RELATION_C);
				cl_cpu_limit[limit_id].min_opp[i] = ppm_main_freq_to_idx(i, min_opp[i], CPUFREQ_RELATION_C);				
			} else {
				cl_cpu_limit[limit_id].max_opp[i] = max_opp[i];
				cl_cpu_limit[limit_id].min_opp[i] = min_opp[i];
			}
		}

		
		cl_gpu_limit[limit_id].max_opp = gpu_max_opp;
		cl_gpu_limit[limit_id].min_opp = gpu_min_opp;
		
		if (cl_gpu_limit[limit_id].max_opp == GPU_FCT_UNLIMIT)
			cl_gpu_limit[limit_id].max_opp = 0;
		if (cl_gpu_limit[limit_id].min_opp == GPU_FCT_UNLIMIT)
			cl_gpu_limit[limit_id].min_opp == (int)(mt_gpufreq_get_dvfs_table_num() - 1);

		if (cl_gpu_limit[limit_id].max_opp >= cl_gpu_limit[limit_id].min_opp)
			cl_gpu_limit[limit_id].max_opp = cl_gpu_limit[limit_id].min_opp;

		return count;
	}
BAD_ARG:
	tscpu_printk("%s,%d: bad argument, %s\n", __func__, __LINE__, desc);
	return -EINVAL;
}

static int cl_cgpu_fct_setting_open(struct inode *inode, struct file *file)
{
	return single_open(file, cl_cgpu_fct_setting_read, PDE_DATA(inode));
}

static const struct file_operations cl_cgpu_fct_fops = {
	.owner = THIS_MODULE,
	.open = cl_cgpu_fct_setting_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = cl_cgpu_fct_setting_write,
	.release = single_release,
};

static int mtk_cooler_cgpu_fct_register_ltf(void)
{
	int i;

	mtk_cooler_cpu_fct_dprintk("%s\n", __func__);

	for (i = 0; i < CGPU_FCT_COOLER_NR; i++) {
		char temp[20] = { 0 };

		snprintf(temp, sizeof(temp), "mtk-cl-cgpufct%02d", i);
		/* /< Cooler Name: mtk-cl-cpu_fc01 */
		cl_cpu_fct_dev[i] = mtk_thermal_cooling_device_register
			(temp, (void *)&cl_cpu_state[i],
			 &mtk_cl_cpu_fct_ops);
	}

	return 0;
}

static void mtk_cooler_cgpu_fct_unregister_ltf(void)
{
	int i;

	mtk_cooler_cpu_fct_dprintk("unregister ltf\n");

	for (i = 0; i < CGPU_FCT_COOLER_NR; i++) {
		if (cl_cpu_fct_dev[i]) {
			mtk_thermal_cooling_device_unregister
				(cl_cpu_fct_dev[i]);
			cl_cpu_fct_dev[i] = NULL;
			cl_cpu_state[i] = 0;
		}
	}
}

static int __init mtk_cooler_cgpu_fct_init(void)
{
	int err = 0;
	int i, j;

	for (i = 0; i < CGPU_FCT_COOLER_NR; i++) {
		cl_gpu_limit[i].max_opp = GPU_FCT_UNLIMIT;
		cl_gpu_limit[i].min_opp = GPU_FCT_UNLIMIT;
		
		for (j = 0; j < NR_PPM_CLUSTERS; j++) {
			cl_cpu_limit[i].core_num[j] = CPU_FCT_UNLIMIT;
			cl_cpu_limit[i].max_opp[j] = CPU_FCT_UNLIMIT;
			cl_cpu_limit[i].min_opp[j] = CPU_FCT_UNLIMIT;
		}
	}

	curr_gpu_limit.max_opp = GPU_FCT_UNLIMIT;
	curr_gpu_limit.min_opp = GPU_FCT_UNLIMIT;
	
	for (i = 0; i < NR_PPM_CLUSTERS; i++) {
		curr_cpu_limit.core_num[i] = CPU_FCT_UNLIMIT;
		curr_cpu_limit.max_opp[i] = CPU_FCT_UNLIMIT;
		curr_cpu_limit.min_opp[i] = CPU_FCT_UNLIMIT;
	}

	err = mtk_cooler_cgpu_fct_register_ltf();
	if (err)
		goto err_unreg;

	{
		struct proc_dir_entry *entry = NULL;
		struct proc_dir_entry *mtktscgpu_fct_dir = NULL;
	
		mtktscgpu_fct_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
		if (!mtktscgpu_fct_dir) {
			tscpu_printk("[%s]: mkdir /proc/driver/thermal failed\n",
									__func__);
	
		} else {
			entry = proc_create("clcgpufct_setting",
					0664,
					mtktscgpu_fct_dir, &cl_cgpu_fct_fops);
	
			if (entry)
				proc_set_user(entry, uid, gid);
		}
	}

	return 0;

err_unreg:
	mtk_cooler_cgpu_fct_unregister_ltf();
	return err;
}

static void __exit mtk_cooler_cgpu_fct_exit(void)
{
	mtk_cooler_cgpu_fct_unregister_ltf();
}
module_init(mtk_cooler_cgpu_fct_init);
module_exit(mtk_cooler_cgpu_fct_exit);

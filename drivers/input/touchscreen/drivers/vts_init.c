#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include "vts_core.h"
#include "vts_init.h"

static unsigned int init_flags;

#define vts_module_init(name, FLAGS) do { \
		int ret; \
		int load; \
		ret = vts_driver_##name##_init(&load); \
		\
		if (load && !ret) { \
			init_flags |= FLAGS; \
			VTI("%s, init_flags:0x%x\n", #name, init_flags); \
		} else if (load && ret) { \
			return ret; \
		} \
	} while (0)

#define vts_module_exit(name, FLAGS) do { \
		if (init_flags & FLAGS) { \
			VTI("%s, init_flags:0x%x\n", #name, init_flags); \
			vts_driver_##name##_exit(); \
		} \
	} while (0)

#define vts_log_init(FLAGS) do { \
		int ret = 0; \
		ret = vts_log_switch_init(); \
		if (ret) \
			return ret; \
		\
		init_flags |= FLAGS; \
	} while (0)

#define vts_log_exit(FLAGS) do { \
		if (init_flags & FLAGS) \
			vts_log_switch_exit(); \
	} while (0)

enum vts_init_flags {
	LOG_MODULE = BIT(0),
	FTM4_MODULE = BIT(1),
	ST80Y_MODULE = BIT(2),
	NT_I2C_MODULE = BIT(3),
	NT_NO_FLASH_MODULE = BIT(4),
	SEC_Y761_MODULE = BIT(5),
	GT9885_MODULE = BIT(6),
	GT9886_MODULE = BIT(7),
	FT8719_FLASH_MODULE = BIT(8),
	FT8756_NO_FALSH_MODULE = BIT(9),
	HVT_NO_FLASH_MODULE = BIT(10),
	ILI_TEK_MODULE = BIT(11),
	ILI_9882N_MODULE = BIT(12),
	S3908_MODULE = BIT(13),
	GT9897_MODULE = BIT(14),
	GT9897_I2C_MODULE = BIT(15),
	CHIPONE_NC9911C_MODULE = BIT(16),
	FT3518U_MODULE = BIT(17)
};

static int __init vts_core_init(void)
{
	vts_log_init(LOG_MODULE);
	vts_module_init(ftm4, FTM4_MODULE);
	vts_module_init(st80y, ST80Y_MODULE);
	vts_module_init(nt_i2c, NT_I2C_MODULE);
	vts_module_init(nt_no_flash, NT_NO_FLASH_MODULE);
	vts_module_init(sec_y761, SEC_Y761_MODULE);
	vts_module_init(goodix_g9885, GT9885_MODULE);
	vts_module_init(goodix_gt9886, GT9886_MODULE);
	vts_module_init(ft8719_no_flash, FT8719_FLASH_MODULE);
	vts_module_init(ft8756_no_flash, FT8756_NO_FALSH_MODULE);
	vts_module_init(hvt_no_flash, HVT_NO_FLASH_MODULE);
	vts_module_init(ili_tek, ILI_TEK_MODULE);
	vts_module_init(ili_9882n, ILI_9882N_MODULE);
	vts_module_init(synaptics_S3908, S3908_MODULE);
	vts_module_init(goodix_gt9897, GT9897_MODULE);
	vts_module_init(goodix_gt9897_i2c, GT9897_I2C_MODULE);
	vts_module_init(chipone_nl9911c, CHIPONE_NC9911C_MODULE);
	vts_module_init(ft3518u, FT3518U_MODULE);
	return 0;
}

static void __exit vts_core_exit(void)
{
	vts_module_exit(ftm4, FTM4_MODULE);
	vts_module_exit(st80y, ST80Y_MODULE);
	vts_module_exit(nt_i2c, NT_I2C_MODULE);
	vts_module_exit(nt_no_flash, NT_NO_FLASH_MODULE);
	vts_module_exit(sec_y761, SEC_Y761_MODULE);
	vts_module_exit(goodix_g9885, GT9885_MODULE);
	vts_module_exit(goodix_gt9886, GT9886_MODULE);
	vts_module_exit(ft8719_no_flash, FT8719_FLASH_MODULE);
	vts_module_exit(ft8756_no_flash, FT8756_NO_FALSH_MODULE);
	vts_module_exit(hvt_no_flash, HVT_NO_FLASH_MODULE);
	vts_module_exit(ili_tek, ILI_TEK_MODULE);
	vts_module_exit(ili_9882n, ILI_9882N_MODULE);
	vts_module_exit(synaptics_S3908, S3908_MODULE);
	vts_module_exit(goodix_gt9897, GT9897_MODULE);
	vts_module_exit(goodix_gt9897_i2c, GT9897_I2C_MODULE);
	vts_module_exit(chipone_nl9911c, CHIPONE_NC9911C_MODULE);
	vts_module_exit(ft3518u, FT3518U_MODULE);
	vts_log_exit(LOG_MODULE);
}

#ifdef MODULE
module_init(vts_core_init);
module_exit(vts_core_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("VTS");
MODULE_DESCRIPTION("VTS MODULE");
MODULE_VERSION("0.01");
#else
late_initcall(vts_core_init);
module_exit(vts_core_exit);
#endif

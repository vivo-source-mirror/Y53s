#ifndef _UFS_VEN_H
#define _UFS_VEN_H

#include <linux/ktime.h>
#include <linux/timer.h>
#include <linux/devfreq.h>
#include <linux/kfifo.h>
#include <linux/kernel.h>
#include "../sd.h"

#define		UFS_VIVO_VERSION		0x0101 /* bit[0-15] */
#define		UFS_NAND_VERSION		0x0100 /* bit[31-16] */
#define		UFS_VERSION_SHIFT		32     /* bit[47-32] for UFS version */
#define		UFS_NAND_VERSION_FLAG		(1ULL << 63)	/* bit[63] */
#define		UFS_VENDOR_VERSION		((UFS_NAND_VERSION << 16) | UFS_VIVO_VERSION)

#define KB (1024)
#define MB (1024 * 1024)

#ifdef CONFIG_UFS_IO_STATISTICS
typedef enum{
	CMD_READ	= 0,
	CMD_WRITE	= 1,
	CMD_DISCARD	= 2,
	CMD_FLUSH	= 3,
	CMD_TYPE_MAX	= 4,
} cmd_types;

typedef enum{
	UFS_STAT_SIZE_4KB	= 0,
	UFS_STAT_SIZE_512KB	= 1,
	UFS_STAT_SIZE_OTHER	= 2,
	UFS_STAT_SIZE_MAX	= 3,
} size_types;

enum ufs_gear_map_type {
	GEAR_LOW,
	GEAR_HIGH,
	GEAR_INVALID,
};

#define		UFS_GEAR_MAX				(UFS_HS_G3 + 1)
#define		UFS_STAT_GEAR_MAX			GEAR_INVALID
#define		UFS_STAT_LATENCY_MAX_TIME_MS	        300
#define		UFS_STAT_CMD_COUNT			(4 << 10)
#define		UFS_STAT_CMD_SYS_MAX_COUNT		20
#define		UFS_VENDOR_LATTENCY_LOG_PRINT_ENABLE    1
#define		UFS_VENDOR_TRACE_LOG_PRINT_ENABLE	2

typedef struct {
	u_int64_t	io_cnt; /* total access counter */
	u_int64_t	size_cnt; /* total access size */
	ktime_t		lat;
	ktime_t		lat_max;
	ktime_t		lat_min;
} io_stats_info_t;

typedef struct __io_ele {
	int		cmd_type;
	char		op;
	char		lun;
	sector_t	lba;
	unsigned int  size; /* access size */
	ktime_t       tstamp; /* issue time */
	ktime_t       time; /* latency */
} io_ele_t;

typedef struct {
	io_ele_t cmd[UFS_STAT_CMD_COUNT];
	unsigned int pos;
} io_stats_cmd_t;

struct io_stats {
	unsigned int			log_enable;
	unsigned int			stat_cmd_enable;
	spinlock_t			lock;
	unsigned int			load;
	unsigned int			load_thre;
	struct ratelimit_state	ratelimit;
	ktime_t				lat_max;
	io_stats_info_t			stat[UFS_STAT_GEAR_MAX][UFS_STAT_SIZE_MAX][CMD_TYPE_MAX];
	u_int64_t				stat_io_cnt;		/* total access counter */
	u_int64_t				stat_size_cnt;		/* total IO size */
	u_int64_t			stat_warn_io_cnt; /* the count of access time over 300ms */
	io_stats_cmd_t			stat_cmd;
	struct nand_info		*nif;
};

#define		GET_BYTE(x, y) (((x) >> (y*8)) & 0xff)

#define		UFS_VENDOR_COMMAND				0xC0
#define		UFS_VENDOR_OP_CODE_ENTER_MODE			0x00
#define		UFS_VENDOR_OP_CODE_EXIT_MODE			0x01
#define		UFS_VENDOR_OP_CODE_SET_PSW			0x03
#define		UFS_VENDOR_OP_CODE_NAND_INFO_REPORT		0x40

#define		UFS_VENDOR_PSW					0x5649564F
#define		UFS_VENDOR_SIGNATURE				0x5C3823AE
#define		UFS_VENDOR_MAX_RETRIES				1
#define		UFS_VENDOR_NAND_INFO_SIZE			0x58

#define		UFS_CAPACITY_32GB				8388608  /* unit: 4KB */

struct nand_info_ss {
	uint32_t   length;
	uint8_t    desc_type;
	uint8_t    reserved_1[3];
	uint32_t   max_slc_erase_cycle;
	uint32_t   min_slc_erase_cycle;
	uint32_t   avg_slc_erase_cycle;
	uint32_t   max_mlc_erase_cycle;
	uint32_t   min_mlc_erase_cycle;
	uint32_t   avg_mlc_erase_cycle;
	uint32_t   read_reclaim_cnt;
	uint32_t   init_bad_blk;
	uint32_t   runtime_bad_blk;
	uint32_t   remain_reserved_blk;
	uint8_t    reserved_2[4];
	uint32_t   required_recovery_level;
	uint32_t   write_size;
	uint32_t   initialize_cnt;
	uint32_t   ffu_success_cnt;
	uint8_t    reserved_3[8];
	uint32_t   pon_cnt;
	uint32_t   spo_cnt;
	uint8_t    reserved_4[4];
} __attribute__((packed));

/* SKHynix */
struct nand_info_hynix {
	uint8_t    length;
	uint8_t    desc_type;
	uint8_t    life_eof;
	uint8_t    life_a;
	uint8_t    life_b;
	uint16_t   init_bad_blk;
	uint16_t   runtime_bad_blk;
	uint16_t   max_erase;
	uint16_t   min_erase;
	uint16_t   avg_erase;
	uint16_t   read_reclaim_cnt;
	uint32_t   spo_cnt;
	uint32_t   write_size;
	uint32_t   lvd_cnt;
	uint16_t   ffu_success_cnt;
	uint8_t    reserved[6];
} __attribute__((packed));

struct nand_info {
	uint64_t  version;
	uint8_t   *deviceid;
	const char *model;
	const char *fw_ver;
	uint8_t   life_a;
	uint8_t   life_b;
	uint8_t   life_eof;
	uint32_t  max_erase;
	uint32_t  min_erase;
	uint32_t  avg_erase;
	uint32_t  max_slc_erase_cycle;
	uint32_t  min_slc_erase_cycle;
	uint32_t  avg_slc_erase_cycle;
	uint32_t  max_mlc_erase_cycle;
	uint32_t  min_mlc_erase_cycle;
	uint32_t  avg_mlc_erase_cycle;
	uint32_t  read_reclaim_cnt;
	uint32_t  init_bad_blk;
	uint32_t  runtime_bad_blk;
	uint32_t  remain_reserved_blk;
	uint32_t  required_recovery_level;
	uint32_t  read_size;
	uint32_t  write_size;
	uint32_t  initialize_cnt;
	uint32_t  ffu_success_cnt;
	uint32_t  ffu_try;
	uint32_t  init_cnt;
	uint32_t  lvd_cnt;
	uint32_t  slc_rsv;
	uint32_t  mlc_rsv;
	uint32_t  pon_cnt;
	uint32_t  spo_cnt;
	uint32_t  slc_tl_er; /* slc total erase counter */
	uint32_t  mlc_tl_er; /* mlc total erase counter */
	uint32_t  capacity; /* unit: 4KB */
	uint64_t  wai;
};
#endif

#define ASCII_STD true
#define UTF16_STD false

struct ufshcd_lrb;
struct ufs_hba;

typedef struct {
	struct kobject ufs_feature_kobject;
	u8 manf_name[18];
	u8 product_name[34];
	u8 pre_end_life[128];
	u8 life_time_a[128];
	u8 life_time_b[128];
	u8 ufsid[512];
	u16 w_manufacturer_id;
	u16 wspecversion;
} ufs_ven_info_t;

int ufshcd_read_string_desc(struct ufs_hba *hba, int desc_index,
				   u8 *buf, u32 size, bool ascii);
void ufshcd_io_stats_remove(struct ufs_hba *hba);
void ufshcd_io_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp, unsigned int tag);
void ufshcd_io_stats_get_load (struct ufs_hba *hba,
		struct devfreq_dev_status *status);
void ufshcd_scsi_device0_get(struct ufs_hba *hba, struct scsi_device *sdev);
int ufs_get_ufsid (struct ufs_hba *hba);
void ufshcd_io_vinit (struct ufs_hba *hba);
int ufshcd_read_health_desc(struct ufs_hba *hba, u8 *buf, u32 size);
int ufs_feature_read_unit_desc(struct ufs_hba *hba);
int ufshcd_query_attr_retry(struct ufs_hba *hba,
	enum query_opcode opcode, enum attr_idn idn, u8 index, u8 selector,
	u32 *attr_val);

#define to_dev_attr(_attr) container_of(_attr, struct device_attribute, attr)

#endif

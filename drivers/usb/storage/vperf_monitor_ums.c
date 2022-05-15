#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <scsi/scsi.h>
#include <linux/cdrom.h>
#include <scsi/scsi_cmnd.h>

#include "scsi.h"
#include "transport.h"
#include "vperf_monitor_ums.h"

#define VPM_UMS_PERF_MONITOR_CYCLE (10*1000)

static LIST_HEAD(vpm_ums_list_head);
static DEFINE_MUTEX(vpm_ums_mutex);
static struct workqueue_struct *vpm_ums_workq;

enum vpm_ums_command {
	VPM_UMS_TEST_UNIT_READY = 0,
	VPM_UMS_REZERO_UNIT,
	VPM_UMS_REQUEST_SENSE,
	VPM_UMS_FORMAT_UNIT,
	VPM_UMS_READ_BLOCK_LIMITS,
	VPM_UMS_REASSIGN_BLOCKS,
	VPM_UMS_READ_6,
	VPM_UMS_WRITE_6,
	VPM_UMS_SEEK_6,
	VPM_UMS_READ_REVERSE,
	VPM_UMS_WRITE_FILEMARKS,
	VPM_UMS_SPACE,
	VPM_UMS_INQUIRY,
	VPM_UMS_RECOVER_BUFFERED_DATA,
	VPM_UMS_MODE_SELECT,
	VPM_UMS_RESERVE,
	VPM_UMS_RELEASE,
	VPM_UMS_COPY,
	VPM_UMS_ERASE,
	VPM_UMS_MODE_SENSE,
	VPM_UMS_START_STOP,
	VPM_UMS_RECEIVE_DIAGNOSTIC,
	VPM_UMS_SEND_DIAGNOSTIC,
	VPM_UMS_ALLOW_MEDIUM_REMOVAL,
	VPM_UMS_SET_WINDOW,
	VPM_UMS_READ_CAPACITY,
	VPM_UMS_READ_10,
	VPM_UMS_WRITE_10,
	VPM_UMS_SEEK_10,
	VPM_UMS_WRITE_VERIFY,
	VPM_UMS_VERIFY,
	VPM_UMS_SEARCH_HIGH,
	VPM_UMS_SEARCH_EQUAL,
	VPM_UMS_SEARCH_LOW,
	VPM_UMS_SET_LIMITS,
	VPM_UMS_READ_POSITION,
	VPM_UMS_SYNCHRONIZE_CACHE,
	VPM_UMS_LOCK_UNLOCK_CACHE,
	VPM_UMS_READ_DEFECT_DATA,
	VPM_UMS_MEDIUM_SCAN,
	VPM_UMS_COMPARE,
	VPM_UMS_COPY_VERIFY,
	VPM_UMS_WRITE_BUFFER,
	VPM_UMS_READ_BUFFER,
	VPM_UMS_UPDATE_BLOCK,
	VPM_UMS_READ_LONG,
	VPM_UMS_WRITE_LONG,
	VPM_UMS_CHANGE_DEFINITION,
	VPM_UMS_WRITE_SAME,
	VPM_UMS_GPCMD_READ_SUBCHANNEL,
	VPM_UMS_READ_TOC,
	VPM_UMS_GPCMD_READ_HEADER,
	VPM_UMS_GPCMD_PLAY_AUDIO_10,
	VPM_UMS_GPCMD_PLAY_AUDIO_MSF,
	VPM_UMS_GPCMD_GET_EVENT_STATUS_NOTIFICATION,
	VPM_UMS_GPCMD_PAUSE_RESUME,
	VPM_UMS_LOG_SELECT,
	VPM_UMS_LOG_SENSE,
	VPM_UMS_GPCMD_STOP_PLAY_SCAN,
	VPM_UMS_GPCMD_READ_DISC_INFO,
	VPM_UMS_GPCMD_READ_TRACK_RZONE_INFO,
	VPM_UMS_GPCMD_RESERVE_RZONE_TRACK,
	VPM_UMS_GPCMD_SEND_OPC,
	VPM_UMS_MODE_SELECT_10,
	VPM_UMS_GPCMD_REPAIR_RZONE_TRACK,
	VPM_UMS_READ_MASTER_CUE,
	VPM_UMS_MODE_SENSE_10,
	VPM_UMS_GPCMD_CLOSE_TRACK,
	VPM_UMS_READ_BUFFER_CAPACITY,
	VPM_UMS_SEND_CUE_SHEET,
	VPM_UMS_GPCMD_BLANK,
	VPM_UMS_REPORT_LUNS,
	VPM_UMS_MOVE_MEDIUM,
	VPM_UMS_READ_12,
	VPM_UMS_WRITE_12,
	VPM_UMS_WRITE_VERIFY_12,
	VPM_UMS_SEARCH_HIGH_12,
	VPM_UMS_SEARCH_EQUAL_12,
	VPM_UMS_SEARCH_LOW_12,
	VPM_UMS_SEND_VOLUME_TAG,
	VPM_UMS_READ_ELEMENT_STATUS,
	VPM_UMS_GPCMD_READ_CD_MSF,
	VPM_UMS_GPCMD_SCAN,
	VPM_UMS_GPCMD_SET_SPEED,
	VPM_UMS_GPCMD_MECHANISM_STATUS,
	VPM_UMS_GPCMD_READ_CD,
	VPM_UMS_WRITE_CONTINUE,
	VPM_UMS_WRITE_LONG_2,
	VPM_UMS_UNKNOWN_COMMAND,
};

char *vpm_ums_commands[] = {"TEST_UNIT_READY", "REZERO_UNIT", "REQUEST_SENSE", "FORMAT_UNIT",
			    "READ_BLOCK_LIMITS", "REASSIGN_BLOCKS", "READ_6", "WRITE_6",
			    "SEEK_6", "READ_REVERSE", "WRITE_FILEMARKS", "SPACE",
			    "INQUIRY", "RECOVER_BUFFERED_DATA", "MODE_SELECT", "RESERVE",
			    "RELEASE", "COPY", "ERASE", "MODE_SENSE",
			    "START_STOP", "RECEIVE_DIAGNOSTIC", "SEND_DIAGNOSTIC", "ALLOW_MEDIUM_REMOVAL",
			    "SET_WINDOW", "READ_CAPACITY", "READ_10", "WRITE_10",
			    "SEEK_10", "WRITE_VERIFY", "VERIFY", "SEARCH_HIGH",
			    "SEARCH_EQUAL", "SEARCH_LOW", "SET_LIMITS", "READ_POSITION",
			    "SYNCHRONIZE_CACHE", "LOCK_UNLOCK_CACHE", "READ_DEFECT_DATA", "MEDIUM_SCAN",
			    "COMPARE", "COPY_VERIFY", "WRITE_BUFFER", "READ_BUFFER",
			    "UPDATE_BLOCK", "READ_LONG", "WRITE_LONG", "CHANGE_DEFINITION",
			    "WRITE_SAME", "READ SUBCHANNEL", "READ_TOC", "READ HEADER",
			    "PLAY AUDIO (10)", "PLAY AUDIO MSF", "GET EVENT/STATUS NOTIFICATION", "PAUSE/RESUME",
			    "LOG_SELECT", "LOG_SENSE", "STOP PLAY/SCAN", "READ DISC INFORMATION",
			    "READ TRACK INFORMATION", "RESERVE TRACK", "SEND OPC", "MODE_SELECT_10",
			    "REPAIR TRACK", "READ MASTER CUE", "MODE_SENSE_10", "CLOSE TRACK/SESSION",
			    "READ BUFFER CAPACITY", "SEND CUE SHEET", "BLANK", "REPORT LUNS",
			    "MOVE_MEDIUM or PLAY AUDIO (12)", "READ_12", "WRITE_12", "WRITE_VERIFY_12",
			    "SEARCH_HIGH_12", "SEARCH_EQUAL_12", "SEARCH_LOW_12", "SEND_VOLUME_TAG",
			    "READ_ELEMENT_STATUS", "READ CD MSF", "SCAN", "SET CD SPEED",
			    "MECHANISM STATUS", "READ CD", "WRITE CONTINUE", "WRITE_LONG_2",
			    "(unknown command)",
};

struct vpm_ums_command_info {
	u64 count;
	u64 error;

	ktime_t start;
	ktime_t max;
	ktime_t min;
	ktime_t total;
};

struct vpm_ums_perf_info {
	u64 count;
	u64 error;

	u64 bytes;
	ktime_t start;
	ktime_t total;
};

struct vpm_ums {
	const struct us_data *us;
	struct list_head list;
	struct mutex mutex;

	struct vpm_ums_command_info command_info[VPM_UMS_UNKNOWN_COMMAND];
	struct vpm_ums_perf_info read_perf_info;
	struct vpm_ums_perf_info write_perf_info;
	struct vpm_ums_command_info csw_info;
	struct delayed_work perf_monitor_work;
};

int vpm_get_command_index(struct scsi_cmnd *srb)
{
	int index = VPM_UMS_UNKNOWN_COMMAND;

	switch (srb->cmnd[0]) {
	case TEST_UNIT_READY: index = VPM_UMS_TEST_UNIT_READY; break;
	case REZERO_UNIT: index = VPM_UMS_REZERO_UNIT; break;
	case REQUEST_SENSE: index = VPM_UMS_REQUEST_SENSE; break;
	case FORMAT_UNIT: index = VPM_UMS_FORMAT_UNIT; break;
	case READ_BLOCK_LIMITS: index = VPM_UMS_READ_BLOCK_LIMITS; break;
	case REASSIGN_BLOCKS: index = VPM_UMS_REASSIGN_BLOCKS; break;
	case READ_6: index = VPM_UMS_READ_6; break;
	case WRITE_6: index = VPM_UMS_WRITE_6; break;
	case SEEK_6: index = VPM_UMS_SEEK_6; break;
	case READ_REVERSE: index = VPM_UMS_READ_REVERSE; break;
	case WRITE_FILEMARKS: index = VPM_UMS_WRITE_FILEMARKS; break;
	case SPACE: index = VPM_UMS_SPACE; break;
	case INQUIRY: index = VPM_UMS_INQUIRY; break;
	case RECOVER_BUFFERED_DATA: index = VPM_UMS_RECOVER_BUFFERED_DATA; break;
	case MODE_SELECT: index = VPM_UMS_MODE_SELECT; break;
	case RESERVE: index = VPM_UMS_RESERVE; break;
	case RELEASE: index = VPM_UMS_RELEASE; break;
	case COPY: index = VPM_UMS_COPY; break;
	case ERASE: index = VPM_UMS_ERASE; break;
	case MODE_SENSE: index = VPM_UMS_MODE_SENSE; break;
	case START_STOP: index = VPM_UMS_START_STOP; break;
	case RECEIVE_DIAGNOSTIC: index = VPM_UMS_RECEIVE_DIAGNOSTIC; break;
	case SEND_DIAGNOSTIC: index = VPM_UMS_SEND_DIAGNOSTIC; break;
	case ALLOW_MEDIUM_REMOVAL: index = VPM_UMS_ALLOW_MEDIUM_REMOVAL; break;
	case SET_WINDOW: index = VPM_UMS_SET_WINDOW; break;
	case READ_CAPACITY: index = VPM_UMS_READ_CAPACITY; break;
	case READ_10: index = VPM_UMS_READ_10; break;
	case WRITE_10: index = VPM_UMS_WRITE_10; break;
	case SEEK_10: index = VPM_UMS_SEEK_10; break;
	case WRITE_VERIFY: index = VPM_UMS_WRITE_VERIFY; break;
	case VERIFY: index = VPM_UMS_VERIFY; break;
	case SEARCH_HIGH: index = VPM_UMS_SEARCH_HIGH; break;
	case SEARCH_EQUAL: index = VPM_UMS_SEARCH_EQUAL; break;
	case SEARCH_LOW: index = VPM_UMS_SEARCH_LOW; break;
	case SET_LIMITS: index = VPM_UMS_SET_LIMITS; break;
	case READ_POSITION: index = VPM_UMS_READ_POSITION; break;
	case SYNCHRONIZE_CACHE: index = VPM_UMS_SYNCHRONIZE_CACHE; break;
	case LOCK_UNLOCK_CACHE: index = VPM_UMS_LOCK_UNLOCK_CACHE; break;
	case READ_DEFECT_DATA: index = VPM_UMS_READ_DEFECT_DATA; break;
	case MEDIUM_SCAN: index = VPM_UMS_MEDIUM_SCAN; break;
	case COMPARE: index = VPM_UMS_COMPARE; break;
	case COPY_VERIFY: index = VPM_UMS_COPY_VERIFY; break;
	case WRITE_BUFFER: index = VPM_UMS_WRITE_BUFFER; break;
	case READ_BUFFER: index = VPM_UMS_READ_BUFFER; break;
	case UPDATE_BLOCK: index = VPM_UMS_UPDATE_BLOCK; break;
	case READ_LONG: index = VPM_UMS_READ_LONG; break;
	case WRITE_LONG: index = VPM_UMS_WRITE_LONG; break;
	case CHANGE_DEFINITION: index = VPM_UMS_CHANGE_DEFINITION; break;
	case WRITE_SAME: index = VPM_UMS_WRITE_SAME; break;
	case GPCMD_READ_SUBCHANNEL: index = VPM_UMS_GPCMD_READ_SUBCHANNEL; break;
	case READ_TOC: index = VPM_UMS_READ_TOC; break;
	case GPCMD_READ_HEADER: index = VPM_UMS_GPCMD_READ_HEADER; break;
	case GPCMD_PLAY_AUDIO_10: index = VPM_UMS_GPCMD_PLAY_AUDIO_10; break;
	case GPCMD_PLAY_AUDIO_MSF: index = VPM_UMS_GPCMD_PLAY_AUDIO_MSF; break;
	case GPCMD_GET_EVENT_STATUS_NOTIFICATION: index = VPM_UMS_GPCMD_GET_EVENT_STATUS_NOTIFICATION; break;
	case GPCMD_PAUSE_RESUME: index = VPM_UMS_GPCMD_PAUSE_RESUME; break;
	case LOG_SELECT: index = VPM_UMS_LOG_SELECT; break;
	case LOG_SENSE: index = VPM_UMS_LOG_SENSE; break;
	case GPCMD_STOP_PLAY_SCAN: index = VPM_UMS_GPCMD_STOP_PLAY_SCAN; break;
	case GPCMD_READ_DISC_INFO: index = VPM_UMS_GPCMD_READ_DISC_INFO; break;
	case GPCMD_READ_TRACK_RZONE_INFO: index = VPM_UMS_GPCMD_READ_TRACK_RZONE_INFO; break;
	case GPCMD_RESERVE_RZONE_TRACK: index = VPM_UMS_GPCMD_RESERVE_RZONE_TRACK; break;
	case GPCMD_SEND_OPC: index = VPM_UMS_GPCMD_SEND_OPC; break;
	case MODE_SELECT_10: index = VPM_UMS_MODE_SELECT_10; break;
	case GPCMD_REPAIR_RZONE_TRACK: index = VPM_UMS_GPCMD_REPAIR_RZONE_TRACK; break;
	case 0x59: index = VPM_UMS_READ_MASTER_CUE; break;
	case MODE_SENSE_10: index = VPM_UMS_MODE_SENSE_10; break;
	case GPCMD_CLOSE_TRACK: index = VPM_UMS_GPCMD_CLOSE_TRACK; break;
	case 0x5C: index = VPM_UMS_READ_BUFFER_CAPACITY; break;
	case 0x5D: index = VPM_UMS_SEND_CUE_SHEET; break;
	case GPCMD_BLANK: index = VPM_UMS_GPCMD_BLANK; break;
	case REPORT_LUNS: index = VPM_UMS_REPORT_LUNS; break;
	case MOVE_MEDIUM: index = VPM_UMS_MOVE_MEDIUM; break;
	case READ_12: index = VPM_UMS_READ_12; break;
	case WRITE_12: index = VPM_UMS_WRITE_12; break;
	case WRITE_VERIFY_12: index = VPM_UMS_WRITE_VERIFY_12; break;
	case SEARCH_HIGH_12: index = VPM_UMS_SEARCH_HIGH_12; break;
	case SEARCH_EQUAL_12: index = VPM_UMS_SEARCH_EQUAL_12; break;
	case SEARCH_LOW_12: index = VPM_UMS_SEARCH_LOW_12; break;
	case SEND_VOLUME_TAG: index = VPM_UMS_SEND_VOLUME_TAG; break;
	case READ_ELEMENT_STATUS: index = VPM_UMS_READ_ELEMENT_STATUS; break;
	case GPCMD_READ_CD_MSF: index = VPM_UMS_GPCMD_READ_CD_MSF; break;
	case GPCMD_SCAN: index = VPM_UMS_GPCMD_SCAN; break;
	case GPCMD_SET_SPEED: index = VPM_UMS_GPCMD_SET_SPEED; break;
	case GPCMD_MECHANISM_STATUS: index = VPM_UMS_GPCMD_MECHANISM_STATUS; break;
	case GPCMD_READ_CD: index = VPM_UMS_GPCMD_READ_CD; break;
	case 0xE1: index = VPM_UMS_WRITE_CONTINUE; break;
	case WRITE_LONG_2: index = VPM_UMS_WRITE_LONG_2; break;
	default: index = VPM_UMS_UNKNOWN_COMMAND; break;
	}

	return index;
}

static struct vpm_ums *vpm_ums_us_to_vpm_ums(const struct us_data *us)
{
	struct vpm_ums *vpm_ums = NULL;

	mutex_lock(&vpm_ums_mutex);
	if (list_empty(&vpm_ums_list_head))
		goto err;
	list_for_each_entry(vpm_ums, &vpm_ums_list_head, list) {
		if (us == vpm_ums->us) {
			mutex_unlock(&vpm_ums_mutex);
			return vpm_ums;
		}
	}
	vpm_ums = NULL;
err:
	mutex_unlock(&vpm_ums_mutex);

	return vpm_ums;
}

static void vpm_ums_perf_monitor(struct vpm_ums *vpm_ums)
{
	struct vpm_ums_command_info *command_info;
	struct vpm_ums_perf_info *perf_info;
	struct timeval tv;
	int i = 0;

	pr_info("%s: %d: start\n", __func__, __LINE__);
	for (; i < VPM_UMS_UNKNOWN_COMMAND; i++) {
		command_info = vpm_ums->command_info + i;
		if (command_info->count) {
			tv = ktime_to_timeval(command_info->start);
			pr_info("%s: %d: command(%s): count(err)=%d(%d), "
				"min=%lld us, max=%lld us, total=%lld us, last time=%ld.%ld\n",
				__func__, __LINE__, vpm_ums_commands[i], command_info->count,
				command_info->error, ktime_to_us(command_info->min),
				ktime_to_us(command_info->max), ktime_to_us(command_info->total),
				tv.tv_sec, tv.tv_usec);
		}
	}

	command_info = &vpm_ums->csw_info;
	tv = ktime_to_timeval(command_info->start);
	pr_info("%s: %d: csw: count(err)=%d(%d), min=%lld us, max=%lld us, total=%lld us, last time=%ld.%ld\n",
		__func__, __LINE__, command_info->count, command_info->error,
		ktime_to_us(command_info->min), ktime_to_us(command_info->max),
		ktime_to_us(command_info->total), tv.tv_sec, tv.tv_usec);

	perf_info = &vpm_ums->read_perf_info;
	tv = ktime_to_timeval(perf_info->start);
	pr_info("%s: %d: read: count(err)=%d(%d), bytes=%lu, total=%lld us, last time=%ld.%ld\n",
		__func__, __LINE__, perf_info->count, perf_info->error,
		perf_info->bytes, ktime_to_us(perf_info->total),
		tv.tv_sec, tv.tv_usec);

	perf_info = &vpm_ums->write_perf_info;
	tv = ktime_to_timeval(perf_info->start);
	pr_info("%s: %d: write: count(err)=%d(%d), bytes=%lu, total=%lld us, last time=%ld.%ld\n",
		__func__, __LINE__, perf_info->count, perf_info->error,
		perf_info->bytes, ktime_to_us(perf_info->total),
		tv.tv_sec, tv.tv_usec);
	pr_info("%s: %d: end\n", __func__, __LINE__);
}

static void vpm_ums_perf_monitor_work(struct work_struct *work)
{
	struct vpm_ums *vpm_ums = container_of(work, struct vpm_ums,
					       perf_monitor_work.work);

	mutex_lock(&vpm_ums->mutex);
	vpm_ums_perf_monitor(vpm_ums);
	mutex_unlock(&vpm_ums->mutex);

	queue_delayed_work(vpm_ums_workq, &vpm_ums->perf_monitor_work,
			   msecs_to_jiffies(VPM_UMS_PERF_MONITOR_CYCLE));
}

void vpm_ums_update_command_start(const struct us_data *us, struct scsi_cmnd *srb)
{
	struct vpm_ums *vpm_ums;
	int command_index = vpm_get_command_index(srb);
	struct vpm_ums_command_info *command_info;

	vpm_ums = vpm_ums_us_to_vpm_ums(us);
	if (!vpm_ums)
		return;

	if (command_index == VPM_UMS_UNKNOWN_COMMAND)
		return;

	mutex_lock(&vpm_ums->mutex);
	command_info = vpm_ums->command_info + command_index;

	command_info->count++;
	command_info->start = ktime_get();
	mutex_unlock(&vpm_ums->mutex);
}

void vpm_ums_update_command_finish(const struct us_data *us,
				   struct scsi_cmnd *srb, int result)
{
	struct vpm_ums *vpm_ums;
	int command_index = vpm_get_command_index(srb);
	struct vpm_ums_command_info *command_info;
	ktime_t diff;

	vpm_ums = vpm_ums_us_to_vpm_ums(us);
	if (!vpm_ums)
		return;

	if (command_index == VPM_UMS_UNKNOWN_COMMAND)
		return;

	mutex_lock(&vpm_ums->mutex);
	command_info = vpm_ums->command_info + command_index;

	diff = ktime_sub(ktime_get(), command_info->start);
	if (result != USB_STOR_XFER_GOOD) {
		command_info->error++;
		pr_err("%s: %d: command(%s): err = %d, time = %lld us\n",
		       __func__, __LINE__, vpm_ums_commands[command_index],
		       result, ktime_to_us(diff));
	} else {
		if (command_info->count == 1) {
			command_info->max = diff;
			command_info->min = diff;
		} else {
			if (ktime_compare(diff, command_info->min) < 0)
				command_info->min = diff;
			else if (ktime_compare(diff, command_info->min) > 0)
				command_info->max = diff;
		}

		command_info->total = ktime_add_unsafe(command_info->total, diff);
		if (command_info->total < 0 || command_info->total < diff ||
		    command_info->total < command_info->total) {
			command_info->count = 0;
			command_info->error = 0;
			command_info->start = ktime_set(0, 0);
			command_info->max = ktime_set(0, 0);
			command_info->min = ktime_set(0, 0);
			command_info->total = ktime_set(0, 0);
			pr_info("%s: %d: ktime overflow, discard and reset\n", __func__, __LINE__);
		}
	}
	mutex_unlock(&vpm_ums->mutex);
}

void vpm_ums_update_data_start(const struct us_data *us, bool is_read)
{
	struct vpm_ums *vpm_ums;
	struct vpm_ums_perf_info *perf_info;

	vpm_ums = vpm_ums_us_to_vpm_ums(us);
	if (!vpm_ums)
		return;

	mutex_lock(&vpm_ums->mutex);
	if (is_read)
		perf_info = &vpm_ums->read_perf_info;
	else
		perf_info = &vpm_ums->write_perf_info;

	perf_info->count++;
	perf_info->start = ktime_get();
	mutex_unlock(&vpm_ums->mutex);
}

void vpm_ums_update_data_finish(const struct us_data *us, bool is_read,
				unsigned int act_len, int result)
{
	struct vpm_ums *vpm_ums;
	struct vpm_ums_perf_info *perf_info;
	ktime_t diff;

	vpm_ums = vpm_ums_us_to_vpm_ums(us);
	if (!vpm_ums)
		return;

	mutex_lock(&vpm_ums->mutex);
	if (is_read)
		perf_info = &vpm_ums->read_perf_info;
	else
		perf_info = &vpm_ums->write_perf_info;

	diff = ktime_sub(ktime_get(), perf_info->start);

	if (result == USB_STOR_XFER_ERROR) {
		perf_info->error++;
		pr_err("%s: %d: %s err = %d, time = %lld us\n",
		       __func__, __LINE__, is_read ? "read" : "write",
		       result, ktime_to_us(diff));
	} else {
		perf_info->bytes += act_len;

		perf_info->total = ktime_add_unsafe(perf_info->total, diff);
		if (perf_info->total < 0 || perf_info->total < diff ||
		    perf_info->total < perf_info->total) {
			perf_info->count = 0;
			perf_info->error = 0;
			perf_info->bytes = 0;
			perf_info->start = ktime_set(0, 0);
			perf_info->total = ktime_set(0, 0);
			pr_info("%s: %d: ktime overflow, discard and reset\n", __func__, __LINE__);
		}
	}
end:
	mutex_unlock(&vpm_ums->mutex);
}

void vpm_ums_update_csw_start(const struct us_data *us)
{
	struct vpm_ums *vpm_ums;
	struct vpm_ums_command_info *command_info;

	vpm_ums = vpm_ums_us_to_vpm_ums(us);
	if (!vpm_ums)
		return;

	mutex_lock(&vpm_ums->mutex);
	command_info = &vpm_ums->csw_info;

	command_info->count++;
	command_info->start = ktime_get();
	mutex_unlock(&vpm_ums->mutex);
}

void vpm_ums_update_csw_finish(const struct us_data *us, int result)
{
	struct vpm_ums *vpm_ums;
	struct vpm_ums_command_info *command_info;
	ktime_t diff;

	vpm_ums = vpm_ums_us_to_vpm_ums(us);
	if (!vpm_ums)
		return;

	mutex_lock(&vpm_ums->mutex);
	command_info = &vpm_ums->csw_info;

	diff = ktime_sub(ktime_get(), command_info->start);
	if (result != USB_STOR_XFER_GOOD) {
		command_info->error++;
		pr_err("%s: %d: csw err = %d, time = %lld us\n",
		       __func__, __LINE__, result, ktime_to_us(diff));
	} else {
		if (command_info->count == 1) {
			command_info->max = diff;
			command_info->min = diff;
		} else {
			if (ktime_compare(diff, command_info->min) < 0)
				command_info->min = diff;
			else if ((ktime_compare(diff, command_info->min) > 0))
				command_info->max = diff;
		}

		command_info->total = ktime_add_unsafe(command_info->total, diff);
		if (command_info->total < 0 || command_info->total < diff ||
		    command_info->total < command_info->total) {
			command_info->count = 0;
			command_info->error = 0;
			command_info->start = ktime_set(0, 0);
			command_info->max = ktime_set(0, 0);
			command_info->min = ktime_set(0, 0);
			command_info->total = ktime_set(0, 0);
			pr_info("%s: %d: ktime overflow, discard and reset\n", __func__, __LINE__);
		}
	}
	mutex_unlock(&vpm_ums->mutex);
}

int vpm_ums_register(const struct us_data *us)
{
	struct vpm_ums *vpm_ums;
	int ret = 0;

	if (!us) {
		pr_err("%s: %d: us is null\n", __func__, __LINE__);
		return -EPERM;
	}

	mutex_lock(&vpm_ums_mutex);
	if (list_empty(&vpm_ums_list_head)) {
		vpm_ums_workq = create_singlethread_workqueue("vpm_ums_workq");
		if (!vpm_ums_workq) {
			pr_err("%s: %d: create workq fail\n", __func__, __LINE__);
			ret = -ENOMEM;
			goto err;
		}
		goto add;
	}
	list_for_each_entry(vpm_ums, &vpm_ums_list_head, list) {
		if (us == vpm_ums->us) {
			pr_err("%s: %d: us is already register\n", __func__, __LINE__);
			ret = -EPERM;
			goto err;
		}
	}
add:
	vpm_ums = kzalloc(sizeof(*vpm_ums), GFP_KERNEL);
	if (!vpm_ums) {
		pr_err("%s: %d: alloc vpm_ums fail\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto err;
	}

	vpm_ums->us = us;
	mutex_init(&vpm_ums->mutex);
	INIT_DELAYED_WORK(&vpm_ums->perf_monitor_work, vpm_ums_perf_monitor_work);
	list_add_tail(&vpm_ums->list, &vpm_ums_list_head);

	queue_delayed_work(vpm_ums_workq, &vpm_ums->perf_monitor_work,
			   msecs_to_jiffies(VPM_UMS_PERF_MONITOR_CYCLE));

	pr_info("%s: %d: vpm_ums(%p) register\n", __func__, __LINE__, us);
	mutex_unlock(&vpm_ums_mutex);

	return 0;
err:
	mutex_unlock(&vpm_ums_mutex);

	return ret;
}

void vpm_ums_unregister(const struct us_data *us)
{
	struct vpm_ums *vpm_ums, *next;

	if (!us) {
		pr_err("%s: %d: us is null\n", __func__, __LINE__);
		return;
	}

	mutex_lock(&vpm_ums_mutex);
	if (list_empty(&vpm_ums_list_head))
		goto err;
	list_for_each_entry_safe(vpm_ums, next, &vpm_ums_list_head, list) {
		if (us == vpm_ums->us) {
			list_del(&vpm_ums->list);
			goto find;
		}
	}

	pr_err("%s: %d: us is not found\n", __func__, __LINE__);
	goto err;

find:
	cancel_delayed_work_sync(&vpm_ums->perf_monitor_work);
	vpm_ums_perf_monitor(vpm_ums);
	kfree(vpm_ums);

	if (list_empty(&vpm_ums_list_head)) {
		destroy_workqueue(vpm_ums_workq);
		vpm_ums_workq = NULL;
	}

	pr_info("%s: %d: vpm_ums(%p) unregister\n", __func__, __LINE__, us);
	mutex_unlock(&vpm_ums_mutex);

	return;
err:
	mutex_unlock(&vpm_ums_mutex);
}

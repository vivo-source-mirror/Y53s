/****************************************************************************************
  wangyuanliang created @ 2013/4/17
****************************************************************************************/
#ifndef BBK_DRIVERS_LOG_SWITCH_H
#define BBK_DRIVERS_LOG_SWITCH_H

#define NAME_LENGTH	16

struct bbk_drivers_callback_handler {
	char name[16];
	void (*callback)(bool is_switch_on);
};

struct bbk_device_info {
	char device_type[NAME_LENGTH];
	char device_name[NAME_LENGTH];
};

struct debug_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count);
};

extern int bbk_drivers_log_switch_register_callback(struct bbk_drivers_callback_handler *handler);
extern void bbk_drivers_log_switch_unregister_callback(char *name);
extern int bbk_driver_register_device_info(char *device_type, char *device_name);
extern void bbk_driver_unregister_device_info(char *device_type, char *device_name);
extern char *get_bbk_board_version(void);
extern int get_bbk_hw_subtype(void);
extern int set_vivo_snd_card_info(unsigned int mask, unsigned int value);
extern int set_vivo_snd_card_info_e(unsigned int mask, unsigned int value);
extern int devs_create_sys_files(const struct attribute *attr);
#endif
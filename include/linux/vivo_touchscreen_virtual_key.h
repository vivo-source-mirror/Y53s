#ifndef	_BBK_TOUCHSCREEN_INFO_H_
#define	_BBK_TOUCHSCREEN_INFO_H_

extern void vivo_get_file_path_str(char **pp_file, char *file_path);

/*add for virtual key ic*/
struct touchscreen_info {
	atomic64_t  AA_release_time;
	atomic_t finger_on_2d;		/*real time state of finger on 2d area*/
	atomic_t key_state;			/*real time state of virtual state*/
	atomic_t finger_when_key_down;			/*has finger down between key down and up*/
	struct kobject *kobject_debug;
};

extern struct kobject *get_debug_kobject(void);
extern bool get_finger_on_2d(void);
extern void set_finger_on_2d(bool is_finger_on_2d);
extern long long  get_AA_release_time(void);
extern void set_AA_release_time(void);
extern int ts_info_register(struct kobject *kobject_debug);
extern void ts_info_unregister(void);
extern void set_virtual_key_state(int key_state);
extern int get_finger_when_key_down(void);

#define VIVO_KEY_LOG_ERR(fmt, param...) \
do {\
	char *file_name = NULL;\
	vivo_get_file_path_str(&file_name, __FILE__);\
	printk(KERN_ERR "VIVO_KEY_ERR^%d^"fmt, __LINE__, ##param);\
} while (0)

#define VIVO_KEY_LOG_INF(fmt, param...) \
do {\
	char *file_name = NULL;\
	vivo_get_file_path_str(&file_name, __FILE__);\
	printk(KERN_ERR "VIVO_KEY_INF^%d^"fmt, __LINE__, ##param);\
} while (0)

#define VIVO_KEY_LOG_DBG(fmt, param...) \
do {\
	if (get_ts_log_switch()) {\
		char *file_name = NULL;\
		vivo_get_file_path_str(&file_name, __FILE__);\
		printk(KERN_ERR "VIVO_KEY_DBG^%d^"fmt, __LINE__, ##param);\
	} \
} while (0)

#endif

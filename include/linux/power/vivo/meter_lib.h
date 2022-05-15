#ifndef __METER_LIB_H
#define __METER_LIB_H


#define SEQUENCE_CURRENT_MAX_SIZE	100
#define SEQUENCE_CURRENT_MIN_INTERVAL	10

typedef int (*METER_CALLBACK)(void *data);

struct meter_lib {
	void *private_data;
	bool initialized;
	int com_r_fg_value;
	int com_fg_meter_resistance;
	int ocv_tune;
	int current_tune_charging;
	int current_tune_discharging;
	int coulomb_tune_charging;
	int coulomb_tune_discharging;

	int (*init)(struct meter_lib *ml);
	int (*reset)(struct meter_lib *ml);
	int (*get_zcv)(struct meter_lib *ml, int *data);
	int (*get_ocv)(struct meter_lib *ml, int *data);
	int (*get_current)(struct meter_lib *ml, bool *is_charging, int *data);
	int (*get_coulomb)(struct meter_lib *ml, int *data);
	int (*get_zcv_current)(struct meter_lib *ml, int *data);
	int (*get_sequence_current)(struct meter_lib *ml, int interval_ms, int size, int *buf);
	int (*get_rtc_ui_soc)(struct meter_lib *ml, int *data);
	int (*set_rtc_ui_soc)(struct meter_lib *ml, int data);
	int (*get_battery_plug_out_status)(struct meter_lib *ml, int *plugout, int *plugout_time);
	int (*register_interrupt_callback)(struct meter_lib *ml, METER_CALLBACK callback);

	int (*dump)(struct meter_lib *ml);
	int (*get_version)(struct meter_lib *ml);

	int (*get_rm)(struct meter_lib *ml, int *data);
	int (*get_fcc)(struct meter_lib *ml, int *data);
	int (*get_soc)(struct meter_lib *ml, int *data);
	int (*get_cycle)(struct meter_lib *ml, int *data);
	int (*get_passedchg)(struct meter_lib *ml, int *data);
	int (*get_cell_temperature)(struct meter_lib *ml, int *data);
};

extern int meter_lib_register(struct meter_lib *ml);
extern struct meter_lib *get_meter_lib(void);


#endif/* #ifndef __METER_LIB_H */

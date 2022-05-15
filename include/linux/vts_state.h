#ifndef __VTS_STATE_H
#define __VTS_STATE_H

enum vts_state {
	VTS_STA_LCD = 0,
	VTS_STA_TDDI_LCD,
	VTS_STA_PROX_STATE,
	VTS_STA_VIRTUAL_PROX_STATE,
	VTS_STA_GESTURE,
	VTS_STA_FINGER_HIGHLIGHT,
	VTS_STA_FACE_HIGHLIGHT,
	VTS_STA_USB_CHARGE,
	VTS_STA_ROTATION,
	VTS_STA_SAVE_POWER,
	VTS_STA_FORCE_NORMAL,
	VTS_STA_NOTICE_UP,
	VTS_STA_FORCE_DOUBLE,
	VTS_STA_CALLING,
	VTS_STA_DEFAUTL,
	VTS_STA_OTHER_DEVICE_STATE_CHANGED,
	VTS_STA_FINGER_UNLOCK_OPEN,
	VTS_STA_GAME_MODE,
	VTS_STA_VIRTUAL_KEY,
	VTS_STA_VK_LONGPRESS,
	VTS_STA_VK_ACTIVEMODE,
	VTS_STA_BAND_STATE,
	VTS_STA_RESET,
	VTS_STA_IN_CALL,
	VTS_STA_FACTORY_SWITCH,
	VTS_STA_VIRTUAL_GAMEKEY,
	VTS_STA_SCREEN_CLOCK_OPEN,
	VTS_STA_SCREEN_CLOCK_REPORT_ABS,
	VTS_STA_FORCE_DISABLE,
	VTS_STA_FINGER_MODE,
	VTS_STA_INPUT_METHOD,
	VTS_STA_FINGER_CENTER,
	VTS_STA_FINGER_MODE_SUPPORT,
	VTS_STA_SCREEN_CLOCK_HIGHLIGHT,
	VTS_STA_SCREEN_SHOT_TRIGER,
	VTS_STA_REJECTION_ZONE_SCENE,
	VTS_STA_REJECTION_ZONE_SCENE_LYING,
	VTS_STA_FP_AOI,
	VTS_STA_FP_AOI_INT,
	VTS_STA_GAME_HIGH_RATE,
	VTS_STA_GAME_BATTLE,
	VTS_STA_MAX
};

struct vts_point {
	int x, y, wx;
	int physical_x, physical_y;		
	int touch_id;
	int nr_touches;
	bool large_press;
	u8 state;
};


struct drivers_callback_handler {
		char name[32];
		void (*point_report)(struct vts_point *point_info, int is_down);
};
extern int tp_report_point_register_callback(struct drivers_callback_handler *handler);
extern void tp_report_point_unregister_callback(char *callback_name);

#endif

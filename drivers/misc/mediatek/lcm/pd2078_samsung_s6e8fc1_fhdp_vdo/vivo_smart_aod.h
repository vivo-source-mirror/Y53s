/*
 * For ramless aod display
 * For Smart AOD setting
 */
#define AOD_MAX_AREA_NUM 6
#define AOD_MAX_DISPLAY_SIZE	1300000 /* bits */
enum color_mode {
	AOD_COLOR_WHITE = 0,
	AOD_COLOR_RED,
	AOD_COLOR_GREEN,
	AOD_COLOR_BLUE,
	AOD_COLOR_MAGENTA,
	AOD_COLOR_CYAN,
	AOD_COLOR_YELLOW,
	AOD_COLOR_BLACK,
};
enum color_depth {
	AOD_1_BIT = 0,
	AOD_2_BIT,
	AOD_4_BIT,
	AOD_8_BIT,
};
struct aod_area_data {
	bool enable;
	bool rgb_enable;
	enum color_mode color;
	enum color_depth depth;
	unsigned int depth_bit;
	unsigned char gray;
	unsigned int area_id;
	unsigned int bit_num;
	unsigned short x_start;
	unsigned short y_start;
	unsigned short width;
	unsigned short height;
};
struct aod_set_data {
	bool aod_area_support;
	struct aod_area_data area[AOD_MAX_AREA_NUM];
};


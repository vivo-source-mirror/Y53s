/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2017 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include "synaptics_tcm_core.h"
//#include <linux/vts_core.h>

#define TYPE_B_PROTOCOL
#define COMMAND_RETRY 3
/*#define USE_DEFAULT_TOUCH_REPORT_CONFIG*/

#define TOUCH_REPORT_CONFIG_SIZE 128

#define TOUCH_FINGER_PRINT_ID		254

enum touch_status {
	LIFT = 0,
	FINGER = 1,
	GLOVED_FINGER = 2,
	NOP = -1,
};

enum touch_report_code {
	TOUCH_END = 0,
	TOUCH_FOREACH_ACTIVE_OBJECT,
	TOUCH_FOREACH_OBJECT,
	TOUCH_FOREACH_END,
	TOUCH_PAD_TO_NEXT_BYTE,
	TOUCH_TIMESTAMP,
	TOUCH_OBJECT_N_INDEX,
	TOUCH_OBJECT_N_CLASSIFICATION,
	TOUCH_OBJECT_N_X_POSITION,
	TOUCH_OBJECT_N_Y_POSITION,
	TOUCH_OBJECT_N_Z,
	TOUCH_OBJECT_N_X_WIDTH,
	TOUCH_OBJECT_N_Y_WIDTH,
	TOUCH_OBJECT_N_TX_POSITION_TIXELS,
	TOUCH_OBJECT_N_RX_POSITION_TIXELS,
	TOUCH_0D_BUTTONS_STATE,
	TOUCH_GESTURE_DOUBLE_TAP,
	TOUCH_FRAME_RATE,
	TOUCH_POWER_IM,
	TOUCH_CID_IM,
	TOUCH_RAIL_IM,
	TOUCH_CID_VARIANCE_IM,
	TOUCH_NSM_FREQUENCY,
	TOUCH_NSM_STATE,
	TOUCH_NUM_OF_ACTIVE_OBJECTS,
	TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME,
	TOUCH_TUNING_GAUSSIAN_WIDTHS = 0x80,
	TOUCH_TUNING_SMALL_OBJECT_PARAMS,
	TOUCH_TUNING_0D_BUTTONS_VARIANCE,
	TOUCH_GESTURE_SWIPE = 0xC1,
	TOUCH_GESTURE_CIRCLE = 0xC2,
	TOUCH_GESTURE_UNICODE = 0xC3,
	TOUCH_GESTURE_VEE = 0xC4,
	TOUCH_GESTURE_TRIANGLE = 0xC5,
	TOUCH_GESTURE_ID = 0xC6,
	TOUCH_GESTURE_COORDINATE = 0xC7,
	TOUCH_PALM_DETECTED = 0xc8,
	TOUCH_FACE_DETECTED = 0x1A,
	TOUCH_REPORT_BROKEN_LINE_FLAG = 0xc9,
	TOUCH_FINGER_DEBUGINFO = 202,
};

struct object_data {
	unsigned char status;
	unsigned int x_pos;
	unsigned int y_pos;
	unsigned int x_width;
	unsigned int y_width;
	unsigned int z;
	unsigned int tx_pos;
	unsigned int rx_pos;
};

struct pre_data {
	unsigned int x_pos;
	unsigned int y_pos;
	unsigned int z;
};


struct input_params {
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
};

struct touch_data {
	struct object_data *object_data;
	struct pre_data pre_data[10];
	unsigned int timestamp;
	unsigned int buttons_state;
	unsigned int gesture_report;
	unsigned char gesture_info[6];
	unsigned char gesture_coordinate[24];
	unsigned char touch_debug_info[14];
	unsigned int gesture_id;
	unsigned int frame_rate;
	unsigned int power_im;
	unsigned int cid_im;
	unsigned int rail_im;
	unsigned int cid_variance_im;
	unsigned int nsm_frequency;
	unsigned int nsm_state;
	unsigned int num_of_active_objects;
	unsigned int num_of_cpu_cycles;
	unsigned int palm_detected;
	unsigned int face_detected;
	unsigned int last_face_detected;
	int x;
	int y;   //gesture point
	long long channel_data;
	long long pre_channel_data;
};

struct touch_hcd {
	bool irq_wake;
	bool report_touch;
	unsigned char *prev_status;
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
	struct mutex report_mutex;
	struct input_dev *input_dev;
	struct touch_data touch_data;
	struct input_params input_params;
	struct syna_tcm_buffer out;
	struct syna_tcm_buffer resp;
	struct syna_tcm_hcd *tcm_hcd;
};

DECLARE_COMPLETION(touch_remove_complete_v2);

static struct touch_hcd *touch_hcd;

/**
 * touch_free_objects() - Free all touch objects
 *
 * Report finger lift events to the input subsystem for all touch objects.
 */
static void touch_free_objects(void)
{
/*#ifdef TYPE_B_PROTOCOL
	unsigned int idx;
#endif

	if (touch_hcd->input_dev == NULL)
		return;

	mutex_lock(&touch_hcd->report_mutex);

#ifdef TYPE_B_PROTOCOL
	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		input_mt_slot(touch_hcd->input_dev, idx);
		input_mt_report_slot_state(touch_hcd->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(touch_hcd->input_dev,
			BTN_TOUCH, 0);
	input_report_key(touch_hcd->input_dev,
			BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(touch_hcd->input_dev);
#endif
	input_sync(touch_hcd->input_dev);

	mutex_unlock(&touch_hcd->report_mutex);*/

	return;
}

/**
 * touch_get_report_data() - Retrieve data from touch report
 *
 * Retrieve data from the touch report based on the bit offset and bit length
 * information from the touch report configuration.
 */
static int touch_get_report_data(unsigned int offset,
		unsigned int bits, unsigned int *data)
{
	unsigned char mask;
	unsigned char byte_data;
	unsigned int output_data;
	unsigned int bit_offset;
	unsigned int byte_offset;
	unsigned int data_bits;
	unsigned int available_bits;
	unsigned int remaining_bits;
	unsigned char *touch_report;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (bits == 0 || bits > 32) {
		//LOGE(tcm_hcd->pdev->dev.parent,
		//		"Invalid number of bits\n");
		//return -EINVAL;
		memcpy(data, &tcm_hcd->report.buffer.buf[offset/8], bits/8);
		return 0;
	}

	if (offset + bits > tcm_hcd->report.buffer.data_length * 8) {
		*data = 0;
		return 0;
	}

	touch_report = tcm_hcd->report.buffer.buf;

	output_data = 0;
	remaining_bits = bits;

	bit_offset = offset % 8;
	byte_offset = offset / 8;

	while (remaining_bits) {
		byte_data = touch_report[byte_offset];
		byte_data >>= bit_offset;

		available_bits = 8 - bit_offset;
		data_bits = MIN(available_bits, remaining_bits);
		mask = 0xff >> (8 - data_bits);

		byte_data &= mask;

		output_data |= byte_data << (bits - remaining_bits);

		bit_offset = 0;
		byte_offset += 1;
		remaining_bits -= data_bits;
	}

	*data = output_data;

	return 0;
}


/**
 * touch_parse_report() - Parse touch report
 *
 * Traverse through the touch report configuration and parse the touch report
 * generated by the device accordingly to retrieve the touch data.
 */
static int touch_parse_report(void)
{
	int retval;
	bool active_only = 0;
	bool num_of_active_objects;
	unsigned char code;
	int j = 0;
	int i = 0;
	unsigned int size;
	unsigned int idx;
	unsigned int obj;
	unsigned int next;
	unsigned int data;
	unsigned int bits;
	unsigned int offset;
	unsigned int objects;
	unsigned int active_objects = 0;
	unsigned int report_size;
	unsigned int config_size;
	unsigned int gesture_coor_idx;
	unsigned short gesture_x[32] = {0, };
	unsigned short gesture_y[32] = {0, };
	unsigned char *config_data;
	struct touch_data *touch_data;
	struct object_data *object_data;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	static unsigned int end_of_foreach;
	u16 finger_gesture_info[4];
	u16 o_direction = 0;
	int tx_data, rx_data;
	int tx_num = 0;
	int rx_num = 0;
	int channel_broken;
	struct vts_device *vtsdev = platform_get_drvdata(tcm_hcd->pdev);

	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;

	config_data = tcm_hcd->config.buf;
	config_size = tcm_hcd->config.data_length;

	report_size = tcm_hcd->report.buffer.data_length;

	size = sizeof(*object_data) * touch_hcd->max_objects;
	memset(touch_hcd->touch_data.object_data, 0x00, size);
	memset(gesture_x, 0xffff, 32 * sizeof(short));
	memset(gesture_y, 0xffff, 32 * sizeof(short));

	num_of_active_objects = false;
	touch_data->gesture_report = 0;
	touch_data->face_detected = 0xFF; /*default not 0 or 1*/

	idx = 0;
	offset = 0;
	objects = 0;
	while (idx < config_size) {
		code = config_data[idx++];
		switch (code) {
		case TOUCH_END:
			goto exit;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			end_of_foreach = idx;
			if (active_only) {
				if (num_of_active_objects) {
					objects++;
					if (objects < active_objects)
						idx = next;
				} else if (offset < report_size * 8) {
					idx = next;
				}
			} else {
				obj++;
				if (obj < touch_hcd->max_objects)
					idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = ceil_div(offset, 8) * 8;
			break;
		case TOUCH_TIMESTAMP:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get timestamp\n");
				return retval;
			}
			touch_data->timestamp = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_INDEX:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &obj);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object index\n");
				return retval;
			}
			offset += bits;
			break;
		case TOUCH_OBJECT_N_CLASSIFICATION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object classification\n");
				return retval;
			}
			object_data[obj].status = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_POSITION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object x position\n");
				return retval;
			}
			object_data[obj].x_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_POSITION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object y position\n");
				return retval;
			}
			object_data[obj].y_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Z:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object z\n");
				return retval;
			}
			object_data[obj].z = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_WIDTH:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object x width\n");
				return retval;
			}
			object_data[obj].x_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_WIDTH:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object y width\n");
				return retval;
			}
			object_data[obj].y_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_TX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object tx position\n");
				return retval;
			}
			object_data[obj].tx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_RX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get object rx position\n");
				return retval;
			}
			object_data[obj].rx_pos = data;
			offset += bits;
			break;
		case TOUCH_0D_BUTTONS_STATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get 0D buttons state\n");
				return retval;
			}
			touch_data->buttons_state = data;
			offset += bits;
			break;
#ifdef WAKEUP_GESTURE
		case TOUCH_GESTURE_DOUBLE_TAP:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get gesture double tap\n");
				return retval;
			}
			touch_data->gesture_report = data;
			if (data) {
				LOGI(tcm_hcd->pdev->dev.parent,
						"Get gesture double tap data=[0x%2x]\n", data);
			}
			offset += bits;
			break;
		case TOUCH_GESTURE_SWIPE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get gesture swipe\n");
				return retval;
			}
			/*touch_data->gesture_report |= data;*/
			offset += bits;
			if (data) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Get gesture swipe data=[0x%2x]\n", data);
			}
			break;
		case TOUCH_GESTURE_CIRCLE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get gesture circle\n");
				return retval;
			}
			/*touch_data->gesture_report |= data;*/
			offset += bits;
			if (data) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Get gesture circle data=[0x%2x]\n", data);
			}
			break;
		case TOUCH_GESTURE_UNICODE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get gesture unicode\n");
				return retval;
			}
			/*touch_data->gesture_report |= data;*/
			offset += bits;
			if (data) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Get gesture unicode data=[0x%2x]\n", data);
			}
			break;
		case TOUCH_GESTURE_ID:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, (unsigned int *)&touch_data->gesture_info[0]);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get gesture info\n");
				return retval;
			}
			offset += bits;

			finger_gesture_info[0] = touch_data->gesture_info[0] | touch_data->gesture_info[1] << 8;
			finger_gesture_info[1] = touch_data->gesture_info[2] | touch_data->gesture_info[3] << 8;
			touch_data->x = finger_gesture_info[0];
			touch_data->y = finger_gesture_info[1];
			/*for gesture O direction*/
			if (touch_data->gesture_report == 0x4)
				o_direction = touch_data->gesture_info[0] | touch_data->gesture_info[1] << 8;
				
			/*
			touch_data->gesture_report |= data;
			offset += bits;
			if (data) {
				LOGE(tcm_hcd->pdev->dev.parent,
					"Get gesture id = %2x\n", data);
			}*/
			break;
		case TOUCH_GESTURE_COORDINATE:
			/*
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, (unsigned int *)&touch_data->gesture_coordinate[0]);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get gesture info\n");
				return retval;
			}
			offset += bits;*/
			
			bits = config_data[idx++];
			
			retval = touch_get_report_data(offset, bits, (unsigned int *)&touch_data->gesture_coordinate[0]);
			if (touch_data->gesture_report == 0x81) {
				VTI("finger debug info[]: %d %d %d %d %d %d %d %d %d %d %d %d", 
					touch_data->gesture_coordinate[0] | (touch_data->gesture_coordinate[1] << 8),
					touch_data->gesture_coordinate[2] | (touch_data->gesture_coordinate[3] << 8),
					touch_data->gesture_coordinate[4] | (touch_data->gesture_coordinate[5] << 8),
					touch_data->gesture_coordinate[6] | (touch_data->gesture_coordinate[7] << 8),
					touch_data->gesture_coordinate[8] | (touch_data->gesture_coordinate[9] << 8),
					touch_data->gesture_coordinate[10] | (touch_data->gesture_coordinate[11] << 8),
					touch_data->gesture_coordinate[12] | (touch_data->gesture_coordinate[13] << 8),
					touch_data->gesture_coordinate[14] | (touch_data->gesture_coordinate[15] << 8),
					touch_data->gesture_coordinate[16] | (touch_data->gesture_coordinate[17] << 8),
					touch_data->gesture_coordinate[18] | (touch_data->gesture_coordinate[19] << 8),
					touch_data->gesture_coordinate[20] | (touch_data->gesture_coordinate[21] << 8),
					touch_data->gesture_coordinate[22] | (touch_data->gesture_coordinate[23] << 8));
			}
			for (j = 0; j < bits/32; j++) {
				int x = 0, y = 0;
				x = (touch_data->gesture_coordinate[j * 4] | touch_data->gesture_coordinate[j * 4 + 1] << 8);
				y = (touch_data->gesture_coordinate[j * 4+ 2] | touch_data->gesture_coordinate[j * 4 + 3] << 8);
				if (x == 0 && y == 0) {
					x = 0xffff;
					y = 0xffff;
				}
				LOGD(tcm_hcd->pdev->dev.parent,
						"x[%d]:%d y[%d]:%d\n", j, x, j, y);
				gesture_x[j] = x;
				gesture_y[j] = y;
			}

			if (touch_data->gesture_report == 0x63) {
				gesture_x[2] = gesture_x[1];
				gesture_y[2] = gesture_y[1];
				gesture_x[1] = gesture_x[3];
				gesture_y[1] = gesture_y[3];
				for (i = 3; i <= 6; i++) {
					gesture_x[i] = 0xffff;
					gesture_y[i] = 0xffff;
				}
			}

			if (touch_data->gesture_report == 0x4)
				gesture_y[9] = o_direction;
			
			vts_report_coordinates_set(vtsdev, (u16 *)gesture_x, (u16 *)gesture_y, 10);

		if (0) {
			if (touch_data->gesture_report) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Get gesture \n");

				for (gesture_coor_idx = 0; gesture_coor_idx < (bits / 32); ++gesture_coor_idx) {
					retval = touch_get_report_data((offset + gesture_coor_idx * 32), 16, &data);
					if (retval < 0) {
						LOGE(tcm_hcd->pdev->dev.parent,
								"Failed to get gesture coordinate\n");
						return retval;
					}
					gesture_x[gesture_coor_idx] = data;
					retval = touch_get_report_data((offset + gesture_coor_idx*32+16), 16, &data);
					if (retval < 0) {
						LOGE(tcm_hcd->pdev->dev.parent,
								"Failed to get gesture coordinate\n");
						return retval;
					}
					gesture_y[gesture_coor_idx] = data;
				}
				vts_report_coordinates_set(vtsdev, (u16 *)gesture_x, (u16 *)gesture_y, (int)gesture_coor_idx);
			}
		}
			offset += bits;


			break;
#endif
		case TOUCH_FRAME_RATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get frame rate\n");
				return retval;
			}
			touch_data->frame_rate = data;
			offset += bits;
			break;
		case TOUCH_POWER_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get power IM\n");
				return retval;
			}
			touch_data->power_im = data;
			offset += bits;
			break;
		case TOUCH_CID_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get CID IM\n");
				return retval;
			}
			touch_data->cid_im = data;
			offset += bits;
			break;
		case TOUCH_RAIL_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get rail IM\n");
				return retval;
			}
			touch_data->rail_im = data;
			offset += bits;
			break;
		case TOUCH_CID_VARIANCE_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get CID variance IM\n");
				return retval;
			}
			touch_data->cid_variance_im = data;
			offset += bits;
			break;
		case TOUCH_NSM_FREQUENCY:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get NSM frequency\n");
				return retval;
			}
			touch_data->nsm_frequency = data;
			offset += bits;
			break;
		case TOUCH_NSM_STATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get NSM state\n");
				return retval;
			}
			touch_data->nsm_state = data;
			offset += bits;
			break;
		case TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get number of active objects\n");
				return retval;
			}
			active_objects = data;
			num_of_active_objects = true;
			touch_data->num_of_active_objects = data;
			offset += bits;
			if (touch_data->num_of_active_objects == 0)
				idx = end_of_foreach;
			break;
		case TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get number of CPU cycles used since last frame\n");
				return retval;
			}
			touch_data->num_of_cpu_cycles = data;
			offset += bits;
			break;
		case TOUCH_TUNING_GAUSSIAN_WIDTHS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_SMALL_OBJECT_PARAMS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_0D_BUTTONS_VARIANCE:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_FACE_DETECTED:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get face detected\n");
				return retval;
			}
			if (tcm_hcd->virtual_prox_enable) {
				touch_data->face_detected = data;
			}
			offset += bits;
			break;
		case TOUCH_PALM_DETECTED:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get palm detected\n");
				return retval;
			}
			touch_data->palm_detected = data;
			offset += bits;
			break;
		case TOUCH_REPORT_BROKEN_LINE_FLAG:
			bits = config_data[idx++];
			vts_property_get(vtsdev, VTS_PROPERTY_TP_CHANNEL_COMP, &channel_broken);
			if (channel_broken == 0) {
				offset += bits;
				break;
			}
			retval = touch_get_report_data(offset, bits, (unsigned int*)&(touch_data->channel_data));
			if (touch_data->channel_data == touch_data->pre_channel_data) {
				offset += bits;
				break;
			}
			tx_data = touch_data->channel_data >> 40;
			rx_data = touch_data->channel_data & 0xffffffffff;
			for (i = 0; i < 20; i++) {
				if (tx_data & (1 << i))
					tx_num++;
			}
			for (i = 0; i < 40; i++) {
				if (rx_data & (1 << i))
					rx_num++;
			}
			VTI("channel broken! tx_num:%d   rx_num:%d   tx_data:0x%x   rx_data:0x%x", tx_num, rx_num, tx_data, rx_data);
			vts_channel_broken_collect(TOUCH_VCODE_CHN_EVENT, tx_num, rx_num, tx_data, rx_data);
			touch_data->pre_channel_data = touch_data->channel_data;
			offset += bits;
			break;
		case TOUCH_FINGER_DEBUGINFO:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, (unsigned int *)&touch_hcd->touch_data.touch_debug_info[0]);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to get palm detected\n");
				return retval;
			}
			offset += bits;
			break;
		default:
			break;
		}
	}

exit:
	return 0;
}

/**
 * touch_report() - Report touch events
 *
 * Retrieve data from the touch report generated by the device and report touch
 * events to the input subsystem.
 */
static void touch_report(void)
{
	int retval;
	int key_value = 0;
	unsigned int idx, i;
	unsigned int x, y, z;
	unsigned int temp;
	unsigned int status;
	unsigned int touch_count, left_count;
	struct touch_data *touch_data;
	struct object_data *object_data;
	struct pre_data *pre_data;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
	struct vts_device *vtsdev = platform_get_drvdata(tcm_hcd->pdev);
	int resolution = 0;
	int display_x = 1;
	int display_y = 1;
	int dimention_x = 1;
	int dimention_y = 1;

	if (!touch_hcd->report_touch)
		return;

	if (touch_hcd->input_dev == NULL)
		return;

	if (!mutex_trylock(&touch_hcd->report_mutex)) {
		vts_ng_panel_collect(TOUCH_VCODE_NG_PANEL_DETECT);
		goto exit_no_lock;
	}

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);
	if (resolution) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimention_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimention_y);
	}

	retval = touch_parse_report();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to parse touch report\n");
		goto exit;
	}

	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;
	pre_data = touch_hcd->touch_data.pre_data;
	VTD("gesture_report=%d, id = %02x, touchscreen_status=%d",
		touch_data->gesture_report, \
		touch_data->gesture_id, \
		tcm_hcd->switch_to_mode);

#ifdef WAKEUP_GESTURE
#if 0
	if (touch_data->gesture_report && tcm_hcd->in_suspend) {
#else
	if (touch_data->gesture_report && (VTS_ST_GESTURE == tcm_hcd->switch_to_mode)) {
#endif
		switch (touch_data->gesture_report) {
		case 0x01:
		case 0x08:
			key_value = VTS_EVENT_GESTURE_DOUBLE_CLICK;
			touch_data->x = touch_data->x * display_x / dimention_x;
			touch_data->y = touch_data->y * display_y / dimention_y;
			vts_update_dclick_point(vtsdev, touch_data->x, touch_data->y);
			break;
		/*case 0x48:
			key_value = KEY_UP;
			break;
		case 0x44:
			key_value = KEY_WAKEUP_SWIPE;
			break;
		case 0x42:
			key_value = KEY_LEFT;
			break;
		case 0x41:
			key_value = KEY_RIGHT;
			break;*/
		case 0x2:
			if (touch_data->gesture_info[4] == 0x1) {
				key_value = VTS_EVENT_GESTURE_PATTERN_RIGHT;
			} else if (touch_data->gesture_info[4] == 0x2) {
				key_value = VTS_EVENT_GESTURE_PATTERN_LEFT;
			} else if (touch_data->gesture_info[4] == 0x4) {
				key_value = VTS_EVENT_GESTURE_PATTERN_DOWN;
			} else if (touch_data->gesture_info[4] == 0x8) {
				key_value = VTS_EVENT_GESTURE_PATTERN_UP;
			}
			break;
		case 0x63:
			key_value = VTS_EVENT_GESTURE_PATTERN_C;
			break;
		case 0x77:
			key_value = VTS_EVENT_GESTURE_PATTERN_W;
			break;
		case 0x6d:
			key_value = VTS_EVENT_GESTURE_PATTERN_M;
			break;
		case 0x65:
			key_value = VTS_EVENT_GESTURE_PATTERN_E;
			break;
		case 0x66:
			key_value = VTS_EVENT_GESTURE_PATTERN_F;
			break;

		case 0x4:
			//if (touch_data->gesture_info[2] == 0x10)  
		//case 0x10:
		//case 0x20:
			#if 0
			if (touch_data->gesture_report == 0x10) {
				vtsGesturePointsReport(VTS_GESTURE_O_DIR, 1, -1, -1);
			} else {
				vtsGesturePointsReport(VTS_GESTURE_O_DIR, 0, -1, -1);
			}
			#endif
			key_value = VTS_EVENT_GESTURE_PATTERN_O;
			break;
		case 0x40:
			key_value = VTS_EVENT_GESTURE_PATTERN_A;
			break;
		default:
			VTI("Unrecognized gesture, touch_report=%2x", touch_data->gesture_report);
		}

		if (touch_data->gesture_report != 0x80 && touch_data->gesture_report != 0x81) {
			vts_report_event_down(vtsdev, key_value);
			vts_report_event_up(vtsdev, key_value);
		}
	}
#endif
	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		if (touch_hcd->prev_status[idx] == LIFT &&
				object_data[idx].status == LIFT)
			status = NOP;
		else
			status = object_data[idx].status;
			
		}

	if (touch_data->gesture_report == 0x80) {
		key_value = VTS_EVENT_GESTURE_FINGERPRINT_DETECT;
		vts_report_event_down(vtsdev, key_value);
	}

	if (touch_data->gesture_report == 0x81) {
		key_value = VTS_EVENT_GESTURE_FINGERPRINT_DETECT;
		vts_report_event_up(vtsdev, key_value);
	}

	//if (tcm_hcd->in_suspend)
	if (VTS_ST_NORMAL != tcm_hcd->switch_to_mode) {
		VTI("stay in other mode, no normal mode");
		goto exit;
	}

	/*face detect*/
	if (touch_data->face_detected != touch_data->last_face_detected && tcm_hcd->virtual_prox_enable) {
		touch_data->last_face_detected = touch_data->face_detected;
		if (touch_data->face_detected == 0x01) {
			VTI("report faceDetect near");
			vts_proxminity_report(vtsdev, 0, 0, 0);
		}

		if (touch_data->face_detected == 0x00) {
			VTI("report faceDetect far");
			vts_proxminity_report(vtsdev, 1, 0, 0);
		}
	}

	if (touch_data->palm_detected) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Palm detected\n");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
	}

	touch_count = 0;
	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		if (touch_hcd->prev_status[idx] == LIFT &&
				object_data[idx].status == LIFT)
			status = NOP;
		else
			status = object_data[idx].status;

		switch (status) {
		case LIFT:
#if 0
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(touch_hcd->input_dev, idx);
			input_mt_report_slot_state(touch_hcd->input_dev,
					MT_TOOL_FINGER, 0);
#endif
#else
		x = pre_data[idx].x_pos;
		y = pre_data[idx].y_pos;
		z = pre_data[idx].z;
		left_count = 0;
		for (i = 0; i < touch_hcd->max_objects; i++) {
			if (object_data[i].status == GLOVED_FINGER || object_data[i].status == FINGER)
				left_count++;
		}
		vts_report_point_up(vtsdev, idx, left_count, x, y, z, z, touch_data->palm_detected, ktime_get());
#endif
			break;
		case FINGER:
		case GLOVED_FINGER:
			x = object_data[idx].x_pos;
			y = object_data[idx].y_pos;
			if (bdata->swap_axes) {
				temp = x;
				x = y;
				y = temp;
			}
			if (bdata->x_flip)
				x = touch_hcd->input_params.max_x - x;
			if (bdata->y_flip)
				y = touch_hcd->input_params.max_y - y;
			z = (x + y) % 10 + 1;
			touch_hcd->touch_data.pre_data[idx].x_pos = x;
			touch_hcd->touch_data.pre_data[idx].y_pos = y;
			touch_hcd->touch_data.pre_data[idx].z = z;
#if 0
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(touch_hcd->input_dev, idx);
			input_mt_report_slot_state(touch_hcd->input_dev,
					MT_TOOL_FINGER, 1);
#endif
			input_report_key(touch_hcd->input_dev,
					BTN_TOUCH, 1);
			input_report_key(touch_hcd->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(touch_hcd->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(touch_hcd->input_dev,
					ABS_MT_POSITION_Y, y);
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(touch_hcd->input_dev);
#endif
#else

		touch_count++;
		vts_report_point_down(vtsdev, idx, touch_count, x, y, z, z, touch_data->palm_detected, (u8*)(touch_hcd->touch_data.touch_debug_info), 14, tcm_hcd->ktime);
#endif
			break;
		default:
			break;
		}

		touch_hcd->prev_status[idx] = object_data[idx].status;
	}

	vts_report_point_sync(vtsdev);
	
	if (touch_count == 0) {
#if 0
		input_report_key(touch_hcd->input_dev,
				BTN_TOUCH, 0);
		input_report_key(touch_hcd->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(touch_hcd->input_dev);
#endif
#endif
		vts_report_release(vtsdev);
	}
#if 0
	input_sync(touch_hcd->input_dev);
#endif

exit:
	mutex_unlock(&touch_hcd->report_mutex);
exit_no_lock:
	return;
}

/**
 * touch_set_input_params() - Set input parameters
 *
 * Set the input parameters of the input device based on the information
 * retrieved from the application information packet. In addition, set up an
 * array for tracking the status of touch objects.
 */
static int touch_set_input_params(void)
{
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_POSITION_X, 0, touch_hcd->max_x, 0, 0);
	input_set_abs_params(touch_hcd->input_dev,
			ABS_MT_POSITION_Y, 0, touch_hcd->max_y, 0, 0);

	input_mt_init_slots(touch_hcd->input_dev, touch_hcd->max_objects,
			INPUT_MT_DIRECT);

	touch_hcd->input_params.max_x = touch_hcd->max_x;
	touch_hcd->input_params.max_y = touch_hcd->max_y;
	touch_hcd->input_params.max_objects = touch_hcd->max_objects;

	if (touch_hcd->max_objects == 0)
		return 0;

	kfree(touch_hcd->prev_status);
	touch_hcd->prev_status = kzalloc(touch_hcd->max_objects, GFP_KERNEL);
	if (!touch_hcd->prev_status) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd->prev_status\n");
		return -ENOMEM;
	}

	return 0;
}

/**
 * touch_get_input_params() - Get input parameters
 *
 * Retrieve the input parameters to register with the input subsystem for
 * the input device from the application information packet. In addition,
 * the touch report configuration is retrieved and stored.
 */
static int touch_get_input_params(void)
{
	int retval;
	unsigned int temp;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
	

	app_info = &tcm_hcd->app_info;
	touch_hcd->max_x = le2_to_uint(app_info->max_x);
	touch_hcd->max_y = le2_to_uint(app_info->max_y);
	touch_hcd->max_objects = le2_to_uint(app_info->max_objects);

	if (bdata->swap_axes) {
		temp = touch_hcd->max_x;
		touch_hcd->max_x = touch_hcd->max_y;
		touch_hcd->max_y = temp;
	}

	LOCK_BUFFER(tcm_hcd->config);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_TOUCH_REPORT_CONFIG,
			NULL,
			0,
			&tcm_hcd->config.buf,
			&tcm_hcd->config.buf_size,
			&tcm_hcd->config.data_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(tcm_hcd->config);
		return retval;
	}

	UNLOCK_BUFFER(tcm_hcd->config);

	return 0;
}

/**
 * touch_set_input_dev() - Set up input device
 *
 * Allocate an input device, configure the input device based on the particular
 * input events to be reported, and register the input device with the input
 * subsystem.
 */
static int touch_set_input_dev(void)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	touch_hcd->input_dev = input_allocate_device();
	if (touch_hcd->input_dev == NULL) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate input device\n");
		return -ENODEV;
	}

	touch_hcd->input_dev->name = TOUCH_INPUT_NAME;
	touch_hcd->input_dev->phys = TOUCH_INPUT_PHYS_PATH;
	touch_hcd->input_dev->id.product = SYNAPTICS_TCM_ID_PRODUCT;
	touch_hcd->input_dev->id.version = SYNAPTICS_TCM_ID_VERSION;
	touch_hcd->input_dev->dev.parent = tcm_hcd->pdev->dev.parent;
	input_set_drvdata(touch_hcd->input_dev, tcm_hcd);

	set_bit(EV_SYN, touch_hcd->input_dev->evbit);
	set_bit(EV_KEY, touch_hcd->input_dev->evbit);
	set_bit(EV_ABS, touch_hcd->input_dev->evbit);
	set_bit(BTN_TOUCH, touch_hcd->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, touch_hcd->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, touch_hcd->input_dev->propbit);
#endif

#ifdef WAKEUP_GESTURE
	set_bit(KEY_WAKEUP, touch_hcd->input_dev->keybit);
	input_set_capability(touch_hcd->input_dev, EV_KEY, KEY_WAKEUP);
#endif

	retval = touch_set_input_params();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set input parameters\n");
		input_free_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
		return retval;
	}

	retval = input_register_device(touch_hcd->input_dev);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to register input device\n");
		input_free_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
		return retval;
	}

	return 0;
}

/**
 * touch_set_report_config() - Set touch report configuration
 *
 * Send the SET_TOUCH_REPORT_CONFIG command to configure the format and content
 * of the touch report.
 */
static int touch_set_report_config(void)
{
	int retval;
	unsigned int idx;
	unsigned int length;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

#ifdef USE_DEFAULT_TOUCH_REPORT_CONFIG
	return 0;
#endif

	VTI("***touch_set_report_config***");
	app_info = &tcm_hcd->app_info;
	length = le2_to_uint(app_info->max_touch_report_config_size);

	if (length < TOUCH_REPORT_CONFIG_SIZE) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid maximum touch report config size\n");
		return -EINVAL;
	}

	LOCK_BUFFER(touch_hcd->out);

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&touch_hcd->out,
			length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd->out.buf\n");
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	idx = 0;
#if 0 /*def WAKEUP_GESTURE*/
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_DOUBLE_TAP;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_DOUBLE_TAP_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_SWIPE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_SWIPE_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_CIRCLE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_CIRCLE_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_UNICODE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_UNICODE_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_ID;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_ID_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_COORDINATE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_COORDINATE_LEN;

#endif

	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_DOUBLE_TAP;
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_PALM_DETECTED;
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_FACE_DETECTED; /* add for Face detect */
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_REPORT_BROKEN_LINE_FLAG;
	touch_hcd->out.buf[idx++] = 64;
	touch_hcd->out.buf[idx++] = TOUCH_FINGER_DEBUGINFO;
	touch_hcd->out.buf[idx++] = 112;
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_ACTIVE_OBJECT;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_INDEX;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_CLASSIFICATION;
	touch_hcd->out.buf[idx++] = 4;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_X_POSITION;
	touch_hcd->out.buf[idx++] = 12;
	touch_hcd->out.buf[idx++] = TOUCH_OBJECT_N_Y_POSITION;
	touch_hcd->out.buf[idx++] = 12;
	touch_hcd->out.buf[idx++] = TOUCH_FOREACH_END;
	touch_hcd->out.buf[idx++] = TOUCH_END;

	LOCK_BUFFER(touch_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_SET_TOUCH_REPORT_CONFIG,
			touch_hcd->out.buf,
			length,
			&touch_hcd->resp.buf,
			&touch_hcd->resp.buf_size,
			&touch_hcd->resp.data_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_SET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(touch_hcd->resp);
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(touch_hcd->resp);
	UNLOCK_BUFFER(touch_hcd->out);

	return 0;
}

/**
 * touch_set_lpwg_report_config() - Set touch report configuration in LPWG mode
 *
 * Send the SET_TOUCH_REPORT_CONFIG command to configure the format and content
 * of the touch report.
 */
#ifdef WAKEUP_GESTURE
static int touch_set_lpwg_report_config(void)
{
	int retval;
	unsigned int idx;
	unsigned int length;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

#ifdef USE_DEFAULT_TOUCH_REPORT_CONFIG
	return 0;
#endif

	app_info = &tcm_hcd->app_info;
	length = le2_to_uint(app_info->max_touch_report_config_size);

	if (length < TOUCH_REPORT_CONFIG_SIZE) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid maximum touch report config size\n");
		return -EINVAL;
	}

	LOCK_BUFFER(touch_hcd->out);

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&touch_hcd->out,
			length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd->out.buf\n");
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	idx = 0;
/*
	touch_hcd->out.buf[idx++] = TOUCH_PALM_DETECTED;
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_DOUBLE_TAP;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_DOUBLE_TAP_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_SWIPE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_SWIPE_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_CIRCLE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_CIRCLE_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_UNICODE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_UNICODE_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_ID;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_ID_LEN;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_COORDINATE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_COORDINATE_LEN;
*/


	touch_hcd->out.buf[idx++] = 0x10;
	touch_hcd->out.buf[idx++] = 8;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_ID;
	touch_hcd->out.buf[idx++] = 48;
	touch_hcd->out.buf[idx++] = TOUCH_GESTURE_COORDINATE;
	touch_hcd->out.buf[idx++] = GESTURE_CFG_COORDINATE_LEN;
	touch_hcd->out.buf[idx++] = 0;
	

	VTI("code=[%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]",
		touch_hcd->out.buf[0],\
		touch_hcd->out.buf[1],\
		touch_hcd->out.buf[2],\
		touch_hcd->out.buf[3],\
		touch_hcd->out.buf[4],\
		touch_hcd->out.buf[5],\
		touch_hcd->out.buf[6],\
		touch_hcd->out.buf[7],\
		touch_hcd->out.buf[8],\
		touch_hcd->out.buf[9],\
		touch_hcd->out.buf[10],\
		touch_hcd->out.buf[11],\
		touch_hcd->out.buf[12]);

	LOCK_BUFFER(touch_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_SET_TOUCH_REPORT_CONFIG,
			touch_hcd->out.buf,
			length,
			&touch_hcd->resp.buf,
			&touch_hcd->resp.buf_size,
			&touch_hcd->resp.data_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_SET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(touch_hcd->resp);
		UNLOCK_BUFFER(touch_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(touch_hcd->resp);
	UNLOCK_BUFFER(touch_hcd->out);

	return 0;
}
#endif

/**
 * touch_check_input_params() - Check input parameters
 *
 * Check if any of the input parameters registered with the input subsystem for
 * the input device has changed.
 */
static int touch_check_input_params(void)
{
	unsigned int size;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (touch_hcd->max_x == 0 && touch_hcd->max_y == 0)
		return 0;

	if (touch_hcd->input_params.max_objects != touch_hcd->max_objects) {
		kfree(touch_hcd->touch_data.object_data);
		size = sizeof(*touch_hcd->touch_data.object_data);
		size *= touch_hcd->max_objects;
		touch_hcd->touch_data.object_data = kzalloc(size, GFP_KERNEL);
		if (!touch_hcd->touch_data.object_data) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for touch_hcd->touch_data.object_data\n");
			return -ENOMEM;
		}
		return 1;
	}

	if (touch_hcd->input_params.max_x != touch_hcd->max_x)
		return 1;

	if (touch_hcd->input_params.max_y != touch_hcd->max_y)
		return 1;

	return 0;
}

/**
 * touch_set_input_reporting() - Configure touch report and set up new input
 * device if necessary
 *
 * After a device reset event, configure the touch report and set up a new input
 * device if any of the input parameters has changed after the device reset.
 */
static int touch_set_input_reporting(void)
{
	int retval, attempt;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application firmware not running\n");
		return 0;
	}

	touch_hcd->report_touch = false;

	touch_free_objects();

	mutex_lock(&touch_hcd->report_mutex);

	/* retry more times if send command failed */
	for (attempt = 0; attempt < COMMAND_RETRY; ++attempt) {
		
#ifdef WAKEUP_GESTURE
		if (VTS_ST_GESTURE == tcm_hcd->switch_to_mode) {
			retval = touch_set_lpwg_report_config();
		} else {
#endif
			retval = touch_set_report_config();

#ifdef WAKEUP_GESTURE
		}
#endif

	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set report config, attempt = %d\n", attempt);
		} else {
			break;
		}
		
		msleep(20);
	}

	retval = touch_get_input_params();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to get input parameters\n");
		goto exit;
	}

	retval = touch_check_input_params();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to check input parameters\n");
		goto exit;
	} else if (retval == 0) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Input parameters unchanged\n");
		goto exit;
	}

	if (touch_hcd->input_dev != NULL) {
		input_unregister_device(touch_hcd->input_dev);
		touch_hcd->input_dev = NULL;
	}

	retval = touch_set_input_dev();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input device\n");
		goto exit;
	}

exit:
	mutex_unlock(&touch_hcd->report_mutex);

	touch_hcd->report_touch = retval < 0 ? false : true;

	return retval;
}

static int touch_init(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	VTI("touch_init start !");
	touch_hcd = kzalloc(sizeof(*touch_hcd), GFP_KERNEL);
	if (!touch_hcd) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for touch_hcd\n");
		return -ENOMEM;
	}

	touch_hcd->tcm_hcd = tcm_hcd;

	mutex_init(&touch_hcd->report_mutex);

	INIT_BUFFER(touch_hcd->out, false);
	INIT_BUFFER(touch_hcd->resp, false);

	retval = touch_set_input_reporting();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input reporting\n");
		goto err_set_input_reporting;
	}

	tcm_hcd->report_touch = touch_report;

	return 0;

err_set_input_reporting:
	kfree(touch_hcd->touch_data.object_data);
	kfree(touch_hcd->prev_status);

	RELEASE_BUFFER(touch_hcd->resp);
	RELEASE_BUFFER(touch_hcd->out);

	kfree(touch_hcd);
	touch_hcd = NULL;

	return retval;
}

static int touch_remove(struct syna_tcm_hcd *tcm_hcd)
{
	if (!touch_hcd)
		goto exit;

	tcm_hcd->report_touch = NULL;

	input_unregister_device(touch_hcd->input_dev);

	kfree(touch_hcd->touch_data.object_data);
	kfree(touch_hcd->prev_status);

	RELEASE_BUFFER(touch_hcd->resp);
	RELEASE_BUFFER(touch_hcd->out);

	kfree(touch_hcd);
	touch_hcd = NULL;

exit:
	complete(&touch_remove_complete_v2);

	return 0;
}

static int touch_syncbox(struct syna_tcm_hcd *tcm_hcd)
{	
	if (!touch_hcd)
		return 0;
	VTD("id=%d", tcm_hcd->report.id);
	switch (tcm_hcd->report.id) {
	case REPORT_IDENTIFY:
		touch_free_objects();
		break;
	case REPORT_TOUCH:
		touch_report();
		break;
	default:
		break;
	}

	return 0;
}

int touch_reinit(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;

	if (!touch_hcd) {
		retval = touch_init(tcm_hcd);
		return retval;
	}

	touch_free_objects();

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode)) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Application mode is not running (firmware mode = %d)\n",
				tcm_hcd->id_info.mode);
		return 0;
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		return retval;
	}

	retval = touch_set_input_reporting();
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set up input reporting\n");
	}

	return retval;
}

static int touch_reset(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	VTI("***start***");

	if (!touch_hcd) {
		retval = touch_init(tcm_hcd);
		return retval;
	}

	if (tcm_hcd->id_info.mode == MODE_APPLICATION) {
		retval = touch_set_input_reporting();
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set up input reporting\n");
			return retval;
		}
	}

	return 0;
}

static int touch_suspend(struct syna_tcm_hcd *tcm_hcd)
{
#ifdef WAKEUP_GESTURE
	int retval;
	int attempt;
#endif

	VTI("***start***");

	if (!touch_hcd)
		return 0;

	touch_free_objects();

#ifdef WAKEUP_GESTURE
	if (VTS_ST_GESTURE == tcm_hcd->switch_to_mode) {
		if (!touch_hcd->irq_wake) {
			enable_irq_wake(tcm_hcd->irq);
			touch_hcd->irq_wake = true;
		}

		/* retry more times if send command failed */
		for (attempt = 0; attempt < COMMAND_RETRY; ++attempt) {
			retval = tcm_hcd->set_dynamic_config(tcm_hcd,
				DC_IN_WAKEUP_GESTURE_MODE,
				1);			
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to enable wakeup gesture mode, attempt = %d\n", attempt);
			} else {
				break;
			}	
			msleep(20);
		}

		/* fingerprint open in gesture mode */
		for (attempt = 0; attempt < COMMAND_RETRY; ++attempt) {
			retval = tcm_hcd->set_dynamic_config(tcm_hcd,
				DC_FINGERPRINT_ENABLE,
				1);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to open fingerprint in gesture mode, attempt = %d\n", attempt);
			} else {
				break;
			}			
			msleep(20);
		}
		
		retval = touch_set_input_reporting();
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set up input reporting\n");
			return retval;
		}

	}
#endif

	return 0;
}

static int touch_resume(struct syna_tcm_hcd *tcm_hcd)
{
#ifdef WAKEUP_GESTURE
	int retval;
	int attempt;
#endif

	VTI("***start***");

	if (!touch_hcd)
		return 0;
	touch_hcd->touch_data.face_detected = 1;
	touch_hcd->touch_data.last_face_detected = 0;
#ifdef WAKEUP_GESTURE
	if (VTS_ST_GESTURE != tcm_hcd->switch_to_mode) {
		if (touch_hcd->irq_wake) {
			disable_irq_wake(tcm_hcd->irq);
			touch_hcd->irq_wake = false;
		}

		/* retry more times if send command failed */
		for (attempt = 0; attempt < COMMAND_RETRY; ++attempt) {
			retval = tcm_hcd->set_dynamic_config(tcm_hcd,
				DC_IN_WAKEUP_GESTURE_MODE,
				0);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to disable wakeup gesture mode, attempt = %d\n", attempt);
			} else {
				break;
			}			
			msleep(20);
		}

		retval = touch_set_input_reporting();
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set up input reporting\n");
			return retval;
		}
#endif
	}

	if (VTS_ST_NORMAL == tcm_hcd->switch_to_mode) {
		/* fingerprint open in normal mode */
		for (attempt = 0; attempt < COMMAND_RETRY; ++attempt) {
			retval = tcm_hcd->set_dynamic_config(tcm_hcd,
				DC_FINGERPRINT_ENABLE,
				3);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to open fingerprint in normal mode, attempt = %d\n", attempt);
			} else {
				break;
			}			
			msleep(20);
		}
	}
	
	return 0;
}

static struct syna_tcm_module_cb touch_module = {
	.type = TCM_TOUCH,
	.init = touch_init,
	.remove = touch_remove,
	.syncbox = touch_syncbox,
	.asyncbox = NULL,
	.reset = touch_reset,
	.suspend = touch_suspend,
	.resume = touch_resume,
};

int touch_module_init(void)
{
	return syna_tcm_add_module_v2(&touch_module, true);
}

void touch_module_exit(void)
{
	syna_tcm_add_module_v2(&touch_module, false);

	wait_for_completion(&touch_remove_complete_v2);

	return;
}


/*static int __init touch_module_init(void)
{
	return syna_tcm_add_module_v2(&touch_module, true);
}

static void __exit touch_module_exit(void)
{
	syna_tcm_add_module_v2(&touch_module, false);

	wait_for_completion(&touch_remove_complete_v2);

	return;
}

late_initcall(touch_module_init);
module_exit(touch_module_exit);*/

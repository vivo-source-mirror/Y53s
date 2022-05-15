/*
 * TUSB422 Power Delivery
 *
 * Author: Brian Quach <brian.quach@ti.com>
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "usb_pd_pal.h"
#include "tusb422_tcpm.h"
#include "tusb422_common.h"
#include "usb_pd_policy_engine.h"
//#include <huawei_platform/usb/hw_pd_dev.h>
#include "hw_pd_dev.h"
#include <linux/power/vivo/tusb_notify.h>


#define TCP_VBUS_CTRL_PD_DETECT (1 << 7)

// TODO: add port number to device tree and use that for port.

void usb_pd_pal_source_vconn(unsigned int port, bool enable)
{
	//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_SOURCE_VCONN, enable);
	return;
}

void usb_pd_pal_source_vbus(unsigned int port, bool usb_pd, uint16_t mv, uint16_t ma)
{
	#if 0
	struct pd_dpm_vbus_state vbus_state;

	PRINT("%s: %u mV, %s\n", __func__, mv, (usb_pd) ? "USB_PD" : "TYPE-C");

	vbus_state.vbus_type = (usb_pd) ? TCP_VBUS_CTRL_PD_DETECT : 0;

	vbus_state.mv = mv;
	vbus_state.ma = ma;

	//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_SOURCE_VBUS, (void *)&vbus_state);
	#else
	/* vivo add start */
	struct tusb_tcp_notify tcp_noti;
	dump_stack();
	pr_err("%s: %u mV, %s\n", __func__, mv, (usb_pd) ? "USB_PD" : "TYPE-C");
	tcp_noti.vbus_state.ma = ma;
	tcp_noti.vbus_state.mv = mv;
	//tcp_noti.vbus_state.type = type;

	tusb_notifier_call_chain(TUSB_TCP_NOTIFY_SOURCE_VBUS,	 (void *)&tcp_noti);
	/* vivo add end */
	#endif
	return;
}

void usb_pd_pal_disable_vbus(unsigned int port)
{
	//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_DIS_VBUS_CTRL, NULL);
	/* vivo add start */
	struct tusb_tcp_notify tcp_noti;

	pr_err("%s:port=%d\n", __func__, port);

	dump_stack();

	tcp_noti.vbus_state.ma = 0;
	tcp_noti.vbus_state.mv = 0;
	//tcp_noti.vbus_state.type = type;

	tusb_notifier_call_chain(TUSB_TCP_NOTIFY_SOURCE_VBUS,	 (void *)&tcp_noti);
	/* vivo add end */
	return;
}

void usb_pd_pal_sink_vbus(unsigned int port, bool usb_pd, uint16_t mv, uint16_t ma)
{
	struct pd_dpm_vbus_state vbus_state;

	PRINT("%s: %u mV, %u mA %s\n", __func__, mv, ma, (usb_pd) ? "USB_PD" : "TYPE-C");

	vbus_state.vbus_type = (usb_pd) ? TCP_VBUS_CTRL_PD_DETECT : 0;

	if (usb_pd)
	{
		vbus_state.ext_power = usb_pd_pe_is_remote_externally_powered(port);
		PRINT("%s: ext_power = %u\n", __func__, vbus_state.ext_power); 
	}

	vbus_state.mv = mv;
	vbus_state.ma = ma;

	//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_SINK_VBUS, (void *)&vbus_state);
	return;
}

/* For battery supplies */
void usb_pd_pal_sink_vbus_batt(unsigned int port, uint16_t min_mv, uint16_t max_mv, uint16_t mw)
{
	PRINT("%s: %u - %u mV, %u mW\n", __func__, min_mv, max_mv, mw);

	return;
}

/* For variable supplies */
void usb_pd_pal_sink_vbus_vari(unsigned int port, uint16_t min_mv, uint16_t max_mv, uint16_t ma)
{
	PRINT("%s: %u - %u mV, %u mA\n", __func__, min_mv, max_mv, ma);

	return;
}

void usb_pd_pal_notify_pd_state(unsigned int port, usb_pd_pe_state_t state)
{
	struct pd_dpm_pd_state pd_state;

	PRINT("%s: %s\n", __func__, 
		  (state == PE_SRC_READY) ? "PE_SRC_READY" : 
		  (state == PE_SNK_READY) ? "PE_SNK_READY" : "?");

	switch (state)
	{
		case PE_SRC_READY:
			pd_state.connected = PD_CONNECT_PE_READY_SRC;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_PD_STATE, (void *)&pd_state);
			break;

		case PE_SNK_READY:
			pd_state.connected = PD_CONNECT_PE_READY_SNK;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_PD_STATE, (void *)&pd_state);
			break;

		default:
			break;
	}

	return;
}
static int typec_old_state = TUSB_TYPEC_UNATTACHED;
static int typec_new_state = TUSB_TYPEC_UNATTACHED;
void usb_pd_pal_notify_connect_state(unsigned int port, tcpc_state_t state, bool polarity)
{
#if 0
	struct pd_dpm_typec_state tc_state;

	tc_state.polarity = polarity;

	switch (state)
	{
		case TCPC_STATE_UNATTACHED_SRC:
		case TCPC_STATE_UNATTACHED_SNK:
			PRINT("%s: TYPEC_UNATTACHED\n", __func__);
			tc_state.new_state = PD_DPM_TYPEC_UNATTACHED;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			break;

		case TCPC_STATE_ATTACHED_SRC:
			PRINT("%s: TYPEC_ATTACHED_SRC, polarity = %d\n", __func__, polarity);
			tc_state.new_state = PD_DPM_TYPEC_ATTACHED_SRC;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			tusb_notifier_call_chain(TCP_NOTIFY_SOURCE_VBUS,	 (void *)&tcp_noti);
			break;

		case TCPC_STATE_ATTACHED_SNK:
			PRINT("%s: TYPEC_ATTACHED_SNK, polarity = %d, c2a_cable = %d\n", __func__, polarity, tcpm_c2a_cable_detected(port));
			tc_state.new_state = PD_DPM_TYPEC_ATTACHED_SNK;
			//tc_state.cable_type = tcpm_c2a_cable_detected(port) ? PD_DPM_C2A_CABLE : PD_DPM_C2C_CABLE;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			// Delay for USB PHY init. (Huawei fix for no "Use USB for" menu when sink)
			tcpm_msleep(50);
			break;

		case TCPC_STATE_AUDIO_ACC:
			PRINT("%s: TYPEC_ATTACHED_AUDIO\n", __func__);
			tc_state.new_state = PD_DPM_TYPEC_ATTACHED_AUDIO;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			break;

		case TCPC_STATE_DEBUG_ACC_SNK:
			PRINT("%s: TYPEC_ATTACHED_CUSTOM_SRC\n", __func__);
			tc_state.new_state = PD_DPM_TYPEC_ATTACHED_CUSTOM_SRC;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			break;

		default:
			break;
	}
#else
	struct tusb_tcp_notify tcp_noti;
	dump_stack();
	pr_err("%s: port=%d,state=%d,polarity=%d\n", __func__, port, state, polarity);
	tcp_noti.typec_state.polarity = polarity;
	/*
	tcp_noti.typec_state.old_state = tcpc->typec_attach_old;
	tcp_noti.typec_state.new_state = tcpc->typec_attach_new;
	tcp_noti.typec_state.rp_level = tcpc->typec_remote_rp_level;
	*/
	typec_old_state = typec_new_state;
	tcp_noti.typec_state.old_state = typec_old_state;
	tcp_noti.typec_state.new_state = typec_new_state;
	switch (state)
	{
		case TCPC_STATE_UNATTACHED_SRC:
		case TCPC_STATE_UNATTACHED_SNK:
			PRINT("%s: TYPEC_UNATTACHED\n", __func__);
			typec_new_state = TUSB_TYPEC_UNATTACHED;
			tcp_noti.typec_state.new_state = typec_new_state;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			break;

		case TCPC_STATE_ATTACHED_SRC:
			PRINT("%s: TYPEC_ATTACHED_SRC, polarity = %d\n", __func__, polarity);
			typec_new_state = TUSB_TYPEC_ATTACHED_SRC;
			tcp_noti.typec_state.new_state = typec_new_state;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			break;

		case TCPC_STATE_ATTACHED_SNK:
			PRINT("%s: TYPEC_ATTACHED_SNK, polarity = %d, c2a_cable = %d\n", __func__, polarity, tcpm_c2a_cable_detected(port));
			typec_new_state = TUSB_TYPEC_ATTACHED_SNK;
			tcp_noti.typec_state.new_state = typec_new_state;
			//tc_state.cable_type = tcpm_c2a_cable_detected(port) ? PD_DPM_C2A_CABLE : PD_DPM_C2C_CABLE;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			// Delay for USB PHY init. (Huawei fix for no "Use USB for" menu when sink)
			tcpm_msleep(50);
			break;

		case TCPC_STATE_AUDIO_ACC:
			PRINT("%s: TYPEC_ATTACHED_AUDIO\n", __func__);
			typec_new_state = TUSB_TYPEC_ATTACHED_AUDIO;
			tcp_noti.typec_state.new_state = typec_new_state;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			break;

		case TCPC_STATE_DEBUG_ACC_SNK:
			PRINT("%s: TYPEC_ATTACHED_CUSTOM_SRC\n", __func__);
			typec_new_state = TUSB_TYPEC_ATTACHED_DBGACC_SNK;
			tcp_noti.typec_state.new_state = typec_new_state;
			//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
			break;

		default:
			break;
	}

	tusb_notifier_call_chain(TUSB_TCP_NOTIFY_TYPEC_STATE,  (void *)&tcp_noti);
#endif
	return;
}

void usb_pd_pal_data_role_swap(unsigned int port, uint8_t new_role)
{
	struct pd_dpm_swap_state swap_state;

	PRINT("%s: new_role = %u\n", __func__, new_role);

	swap_state.new_role = new_role;

	//pd_dpm_handle_pe_event(PD_DPM_PE_EVT_DR_SWAP, (void *)&swap_state);

	return;
}

void usb_pd_pal_power_role_swap(unsigned int port, uint8_t new_role)
{
	//pd_dpm_handle_pe_event();

	return;
}

/* Custom Type-C headphone support */
#ifdef CONFIG_HIFI_USB
void usb_pd_pal_check_hifi_usb_status(unsigned int port)
{
	//hisi_usb_check_hifi_usb_status(HIFI_USB_TCPC); 
	return;
}
#endif

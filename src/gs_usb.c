#include "gs_usb.h"

#include "can.h"
#include "config.h"
#include "flash.h"
#include "gpio.h"
#include "led.h"
#include "queue.h"
#include "timer.h"

#include "device/usbd.h"
#include "class/vendor/vendor_device.h"

#include <stdlib.h>
#include <string.h>

#define VENDOR_ITF 0

// buffer must be larger than the largest type in gs_usb_ctrl_request_size
char buffer[100];

static bool timestamps_enabled;
static bool pad_pkts_to_max_pkt_size;

static uint32_t sof_timestamp_us;

can_data_t hCAN;
led_data_t hLED;

queue_t *q_frame_pool;
queue_t *q_from_host;
queue_t *q_to_host;

can_data_t *channels[NUM_CAN_CHANNEL] =  {
	&hCAN
};

// device info
static const struct gs_device_config USBD_GS_CAN_dconf = {
	0, // reserved 1
	0, // reserved 2
	0, // reserved 3
	0, // interface count (0=1, 1=2..)
	2, // software version
	1  // hardware version
};

// bit timing constraints
static const struct gs_device_bt_const USBD_GS_CAN_btconst = {
	GS_CAN_FEATURE_LISTEN_ONLY  // supported features
	| GS_CAN_FEATURE_LOOP_BACK
	| GS_CAN_FEATURE_HW_TIMESTAMP
	| GS_CAN_FEATURE_IDENTIFY
	| GS_CAN_FEATURE_USER_ID
	| GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE
#ifdef PIN_TERM_Pin
	| GS_CAN_FEATURE_TERMINATION
#endif
	,
	CAN_CLOCK_SPEED, // can timing base clock
	1,    // tseg1 min
	16,   // tseg1 max
	1,    // tseg2 min
	8,    // tseg2 max
	4,    // sjw max
	1,    // brp min
	1024, // brp_max
	1,    // brp increment;
};

static int gs_usb_ctrl_request_size(tusb_control_request_t const * req)
{
	switch ((enum gs_usb_breq)req->bRequest) {

	case GS_USB_BREQ_HOST_FORMAT:
	case GS_USB_BREQ_IDENTIFY:
	case GS_USB_BREQ_GET_TERMINATION:
	case GS_USB_BREQ_SET_TERMINATION:
	case GS_USB_BREQ_SET_USER_ID:
		return sizeof(uint32_t);

	case GS_USB_BREQ_MODE:
		return sizeof(struct gs_device_mode);
	case GS_USB_BREQ_BT_CONST:
		return sizeof(struct gs_device_bt_const);
	case GS_USB_BREQ_BT_CONST_EXT:
		return sizeof(struct gs_device_bt_const_extended);
	case GS_USB_BREQ_DEVICE_CONFIG:
		return sizeof(struct gs_device_config);
	case GS_USB_BREQ_BITTIMING:
	case GS_USB_BREQ_DATA_BITTIMING:
		return sizeof(struct gs_device_bittiming);

		// Unused
	case GS_USB_BREQ_BERR:
	case GS_USB_BREQ_TIMESTAMP:
	case GS_USB_BREQ_GET_USER_ID:
		break;
	}

	return 0;
}

static bool gs_usb_ctrl_setup(uint8_t rhport, tusb_control_request_t const * req)
{
	if (gs_usb_ctrl_request_size(req) != req->wLength)
	{
		printf("Invalid request length: %i, %i\n", gs_usb_ctrl_request_size(req), req->wLength);
		return false;
	}

	switch (req->bRequest) {

	case GS_USB_BREQ_HOST_FORMAT:
	case GS_USB_BREQ_MODE:
	case GS_USB_BREQ_BITTIMING:
	case GS_USB_BREQ_IDENTIFY:
	case GS_USB_BREQ_SET_USER_ID:
	case GS_USB_BREQ_SET_TERMINATION:
		// get data
		break;

	case GS_USB_BREQ_GET_TERMINATION:
	{
		enum terminator_status term_state = get_term(req->wValue);

		if (term_state == term_unsupported) {
			return false; // term resistor unsupported
		}

		uint32_t d32 = (uint32_t)term_state;
		memcpy(buffer, &d32, sizeof(d32));
		break;
	}

	case GS_USB_BREQ_DEVICE_CONFIG:
		memcpy(buffer, &USBD_GS_CAN_dconf, sizeof(USBD_GS_CAN_dconf));
		break;

	case GS_USB_BREQ_BT_CONST:
		memcpy(buffer, &USBD_GS_CAN_btconst, sizeof(USBD_GS_CAN_btconst));
		break;

	case GS_USB_BREQ_TIMESTAMP:
		memcpy(buffer, &sof_timestamp_us, sizeof(sof_timestamp_us));
		break;

	case GS_USB_BREQ_GET_USER_ID:
		if (req->wValue >= NUM_CAN_CHANNEL)
			return false; // don't be so cruel

		uint32_t d32 = flash_get_user_id(req->wValue);
		memcpy(buffer, &d32, sizeof(d32));
		break;

	default:
		return false; // unknown
	}

	return tud_control_xfer(rhport, req, buffer, sizeof(buffer));
}

static bool gs_usb_ctrl_data(uint8_t rhport, tusb_control_request_t const * req)
{
	static const led_seq_step_t led_identify_seq[] = {
		{ .state = 0x01, .time_in_10ms = 10 },
		{ .state = 0x02, .time_in_10ms = 10 },
		{ .state = 0x00, .time_in_10ms = 0 }
	};

	uint32_t param_u32;

	switch (req->bRequest) {

	case GS_USB_BREQ_HOST_FORMAT:
		memcpy(&param_u32, buffer, sizeof(uint32_t));
		return param_u32 == 0x0000beef;

	case GS_USB_BREQ_IDENTIFY:
		memcpy(&param_u32, buffer, sizeof(param_u32));
		if (param_u32) {
			led_run_sequence(&hLED, led_identify_seq, -1);
		} else if (req->wValue < NUM_CAN_CHANNEL) {
			can_data_t *ch = channels[req->wValue];
			led_set_mode(&hLED, can_is_enabled(ch) ? led_mode_normal : led_mode_off);
		}
		return true;

	case GS_USB_BREQ_SET_TERMINATION:
		memcpy(&param_u32, buffer, sizeof(param_u32));
		if (set_term(req->wValue, param_u32) == term_unsupported)
			return false; //USBD_CtlError(pdev, req);
		return true;

	case GS_USB_BREQ_SET_USER_ID:
		memcpy(&param_u32, buffer, sizeof(param_u32));
		if (flash_set_user_id(req->wValue, param_u32)) {
			flash_flush();
			return true;
		}
		break;

	case GS_USB_BREQ_MODE:
		if (req->wValue < NUM_CAN_CHANNEL) {
			struct gs_device_mode *mode = (struct gs_device_mode*)buffer;
			can_data_t *ch = channels[req->wValue];

			switch (mode->mode)
			{
			case GS_CAN_MODE_RESET:
				can_disable(ch);
				led_set_mode(&hLED, led_mode_off);
				break;

			case GS_CAN_MODE_START:
				timestamps_enabled = (mode->flags & GS_CAN_MODE_HW_TIMESTAMP) != 0;
				pad_pkts_to_max_pkt_size = (mode->flags & GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE) != 0;

				can_enable(ch,
							(mode->flags & GS_CAN_MODE_LOOP_BACK) != 0,
							(mode->flags & GS_CAN_MODE_LISTEN_ONLY) != 0,
							(mode->flags & GS_CAN_MODE_ONE_SHOT) != 0
							// triple sampling not supported on bxCAN
						);

				led_set_mode(&hLED, led_mode_normal);
				break;

			default:
				return false;
			}
			return true;
		}
		break;

	case GS_USB_BREQ_BITTIMING:
		if (req->wValue < NUM_CAN_CHANNEL) {
			struct gs_device_bittiming *timing = (struct gs_device_bittiming*)buffer;
			can_data_t *ch = channels[req->wValue];

			can_set_bittiming(
						ch,
						timing->brp,
						timing->prop_seg + timing->phase_seg1,
						timing->phase_seg2,
						timing->sjw
					);

			return true;
		}
		break;

	case GS_USB_BREQ_DEVICE_CONFIG:
		memcpy(buffer, &USBD_GS_CAN_dconf, sizeof(USBD_GS_CAN_dconf));
		return tud_control_xfer(rhport, req, buffer, sizeof(USBD_GS_CAN_dconf));

	default:
		break;
	}

	return true;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * req)
{
	switch(stage)
	{
	case CONTROL_STAGE_IDLE:
		break;

	case CONTROL_STAGE_SETUP:
		return gs_usb_ctrl_setup(rhport, req);

	case CONTROL_STAGE_DATA:
		return gs_usb_ctrl_data(rhport, req);

	case CONTROL_STAGE_ACK:
		return true;
	}

	printf("How'd we get here?\n");
	// stall unknown request
	return false;
}

static void try_reading_from_host(void)
{
	const uint32_t size = sizeof(struct gs_host_frame) - 4 * ((int)!timestamps_enabled);

	bool flush = false;

	while(tud_vendor_n_available(VENDOR_ITF) >= size)
	{
		struct gs_host_frame *frame = queue_pop_front(q_frame_pool);
		if(!frame)
			return; // no frames to use

		if (size != tud_vendor_n_read(VENDOR_ITF, frame, size))
		{
			queue_push_back(q_frame_pool, frame); // frame unused, return
			return; // odd...
		}

		queue_push_back(q_from_host, frame);
		flush = true;
	}

	if (flush)
	{
		tud_vendor_n_read_flush(VENDOR_ITF);
	}
}

static bool send_to_host(struct gs_host_frame *frame)
{
	uint8_t buf[CAN_DATA_MAX_PACKET_SIZE];
	uint8_t *send_addr;

	uint32_t len = sizeof(struct gs_host_frame) - 4 * ((int)!timestamps_enabled);

	if (pad_pkts_to_max_pkt_size) {
		// When talking to WinUSB it seems to help a lot if the
		// size of packet you send equals the max packet size.
		// In this mode, fill packets out to max packet size and
		// then send.
		memcpy(buf, frame, len);

		// zero rest of buffer
		memset(buf + len, 0, sizeof(buf) - len);
		send_addr = buf;
		len = sizeof(buf);
	} else {
		send_addr = (uint8_t *)frame;
	}

	if (tud_vendor_n_write_available(VENDOR_ITF) < len)
		return false;

	uint32_t result = tud_vendor_n_write(VENDOR_ITF, send_addr, len);
	tud_vendor_n_flush(VENDOR_ITF);

	return result == len;
}

static void try_sending_to_host(void)
{
	struct gs_host_frame *frame = queue_pop_front(q_to_host);
	if(!frame)
		return;

	if (send_to_host(frame)) {
		queue_push_back(q_frame_pool, frame);
	} else {
		queue_push_front(q_to_host, frame);
	}
}

static void enqueue_to_host(struct gs_host_frame *frame)
{
	queue_push_back(q_to_host, frame);
}

static void queue_new_rx_can_msg(void)
{
	struct gs_host_frame *frame = queue_pop_front(q_frame_pool);
	if(!frame)
		return; // no frames to use

	if (can_receive(&hCAN, frame)) {
		frame->timestamp_us = timer_get();
		frame->echo_id = 0xFFFFFFFF; // not an echo frame
		frame->channel = 0;
		frame->flags = 0;
		frame->reserved = 0;

		enqueue_to_host(frame);

		led_indicate_trx(&hLED, led_rx);

	} else {
		queue_push_back(q_frame_pool, frame); // frame unused, return
	}
}

static void queue_new_error_can_msg(void)
{
	// If there are frames to receive, don't report any error frames. The
	// best we can localize the errors to is "after the last successfully
	// received frame", so wait until we get there. LEC will hold some error
	// to report even if multiple pass by.

	static uint32_t last_can_error_status = 0;

	const uint32_t can_err = can_get_error_status(&hCAN);
	struct gs_host_frame *frame = queue_pop_front(q_frame_pool);
	if(!frame)
		return; // no frames to use

	frame->timestamp_us = timer_get();
	if (can_parse_error_status(can_err, last_can_error_status, &hCAN, frame)) {
		enqueue_to_host(frame);
		last_can_error_status = can_err;

	} else {
		queue_push_back(q_frame_pool, frame); // no change to status
	}
}

static void send_queued_tx_can_msg(void)
{
	struct gs_host_frame *frame = queue_pop_front(q_from_host);
	if (!frame)
		return;

	// send can message from host
	if (can_send(&hCAN, frame)) {
		// Echo sent frame back to host
		frame->flags = 0x0;
		frame->reserved = 0x0;
		frame->timestamp_us = timer_get();
		enqueue_to_host(frame);

		led_indicate_trx(&hLED, led_tx);

	} else {
		queue_push_front(q_from_host, frame); // retry later
	}
}

void gs_usb_task(void)
{
	try_reading_from_host();

	send_queued_tx_can_msg();

	try_sending_to_host();

	if (can_is_rx_pending(&hCAN)) {
		queue_new_rx_can_msg();
	} else {
		queue_new_error_can_msg();
	}

	led_update(&hLED);

//	if (USBD_GS_CAN_DfuDetachRequested(&hUSB)) {
//		dfu_run_bootloader();
//	}
}

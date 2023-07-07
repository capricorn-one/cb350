#include "moto_can.h"
#include "mm_types.h"
#include "moto.h"
#include "hal.h"

#include <mcp_can.h>

#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
#define STATE_POLL_THREAD_STACK_SIZE 512
#define STATE_POLL_THREAD_PRIORITY 2
#define LED_MSG_ID 0x10
#define COUNTER_MSG_ID 0x12345
#define SET_LED 1
#define RESET_LED 0
#define SLEEP_TIME K_MSEC(250)


static volatile bool can_data_ready_flag = false;

MCP_CAN CAN0(PIN_CAN_CS);                              


void can_rx_callback(void)
{
	can_data_ready_flag = true;
}

void canbus::init(void) {

	if(CAN0.begin(MCP_EXT, CAN_250KBPS, MCP_16MHZ) == CAN_OK) {
		LOG_INF("CAN0 init ok");
		CAN0.setMode(MCP_NORMAL);

		attachInterrupt(PIN_CAN_INT, can_rx_callback, FALLING);
	}
	else {
		LOG_ERR("CAN0 init failed");
	}
	// const struct can_filter handlebar_switch_filter = {
	// 	.id = MM_CAN_ID_HANDLEBAR_SWITCH_STATES,
    //     // .rtr = CAN_DATAFRAME,
    //     // .id_type = CAN_STANDARD_IDENTIFIER,
    //     // .id_mask = CAN_STD_ID_MASK,
	// };

	// int filter_id;

	// filter_id = can_add_rx_filter(can_dev, can_rx_callback, NULL, &handlebar_switch_filter);
	// if (filter_id < 0) {
	// 	LOG_ERR("Unable to add rx filter [%d]", filter_id);
	// }

	// const struct can_filter vfd_telemetry_filter = {
	// 	.id = MM_CAN_ID_VFD_TELEM,
    //     // .rtr = CAN_DATAFRAME,
    //     // .id_type = CAN_STANDARD_IDENTIFIER,
    //     // .id_mask = CAN_STD_ID_MASK,
	// };

	// filter_id = can_add_rx_filter(can_dev, can_rx_callback, NULL, &vfd_telemetry_filter);
	// if (filter_id < 0) {
	// 	LOG_ERR("Unable to add rx filter [%d]", filter_id);
	// }

	// ready = true;

	LOG_INF("Finished init.");

}

void canbus::update(void) {

	if(!can_data_ready_flag)
		return;


	long unsigned int rxId;
	unsigned char len = 0;
	unsigned char rxBuf[8];


	// Read the new data if there is any... process it

	CAN0.readMsgBuf(&rxId, &len, rxBuf);

	can_data_ready_flag = false;
	// can_frame frame;

	// int ret;

	// do {
		
	// 	ret = k_msgq_get(&canbus_msgq, &frame, K_NO_WAIT);

	// 	if(ret == 0) {

	// 		switch(frame.id) {
			
	// 			case MM_CAN_ID_HANDLEBAR_SWITCH_STATES:
	// 				moto.handleBarCallbackHandler((moto_pushbutton_num_t)frame.data[0], (moto_switch_trigger_t)frame.data[1]);
	// 				LOG_DBG("HANDLEBAR CALLBACK HANDLER EXECUTED WITH STATES %u", frame.data[0]);
	// 				break;

	// 			case MM_CAN_ID_VFD_TELEM:
	// 				moto.vfdTelemetryCallbackHandler((vfd_telem_t *)frame.data);
	// 				break;
	// 		}

	// 	}

	// } while(ret == 0);

}

void canbus::send_hubOutputStates(uint16_t states) {

	if(!ready)
		return;

    // struct can_frame frame = {
    //     .id = MM_CAN_ID_HUB_OUTPUT_STATES,
	// 	// .rtr = CAN_DATAFRAME,
	// 	// .id_type = CAN_STANDARD_IDENTIFIER,
    //     .dlc = 2,
    // };

    // frame.data[0] = states&0xFF;
    // frame.data[1] = states>>8;

    // can_send(can_dev, &frame, K_MSEC(250), tx_callback, NULL);
}

canbus moto_can("canbus");

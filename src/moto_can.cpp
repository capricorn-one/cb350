#include "moto_can.h"
#include "mm_types.h"
#include "moto.h"
#include "hal.h"
#include "hal_can.h"

static void handlebar_callback(uint16_t *data) {
	moto.handleBarCallbackHandler((moto_pushbutton_num_t)data[0], (moto_switch_trigger_t)data[1]);
	// LOG_DBG("HANDLEBAR CALLBACK HANDLER EXECUTED WITH STATES %u", frame.data[0]);
}

static void vfd_callback(uint16_t *data) {
	moto.vfdTelemetryCallbackHandler((vfd_telem_t *)data);
	// LOG_DBG("VFD CALLBACK HANDLER EXECUTED WITH STATES %u", frame.data[0]);
}

void canbus::init(void) {

	hal_register_handlebar_callback(handlebar_callback);

	hal_register_vfd_callback(vfd_callback);
	
	hal_can_init();

}

void canbus::update(void) {

	
}

void canbus::send_output_state_change(uint16_t states) {

    // struct can_frame frame = {
    //     .id = MM_CAN_ID_HUB_OUTPUT_STATES,
	// 	// .rtr = CAN_DATAFRAME,
	// 	// .id_type = CAN_STANDARD_IDENTIFIER,
    //     .dlc = 2,
    // };

	hal_can_send(MM_CAN_ID_HUB_OUTPUT_STATES, (uint8_t *)&states, sizeof(states));
}

canbus moto_can("canbus");

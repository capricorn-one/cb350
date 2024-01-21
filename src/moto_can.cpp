#include "moto_can.h"
#include "mm_types.h"
#include "moto.h"
#include "hal.h"
#include "hal_can.h"

static void handlebar_callback(uint8_t *data) {

	LOG_INF("Received handlebar switch states: %d, %d", data[0], data[1]);

	moto.handleBarCallbackHandler((moto_pushbutton_num_t)data[0], (moto_switch_trigger_t)data[1]);
}

static void vfd_callback(uint8_t *data) {
	moto.vfdTelemetryCallbackHandler((vfd_telem_t *)data);
}

bool canbus::begin(void) {

	hal_register_handlebar_callback(handlebar_callback);

	hal_register_vfd_callback(vfd_callback);
	
	return hal_can_init();

}

bool canbus::update(void) {

	hal_can_update();

	return true;	
}

void canbus::send_output_state_change(uint16_t states) {
	hal_can_send(MM_CAN_ID_HUB_OUTPUT_STATES, (uint8_t *)&states, sizeof(states));
}

canbus moto_can("canbus");

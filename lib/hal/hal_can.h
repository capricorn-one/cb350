#ifndef _HAL_CAN_H
#define _HAL_CAN_H

#include <stdint.h>
#include <stdbool.h>

    bool hal_can_init(void);

    void hal_can_send(uint32_t id, uint8_t *data, uint8_t len);

    void hal_register_handlebar_callback(void (*callback)(uint16_t *data));

    void hal_register_vfd_callback(void (*callback)(uint16_t *data));

#endif
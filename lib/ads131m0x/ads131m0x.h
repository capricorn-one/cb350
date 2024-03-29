/* 
 * File:   ads131m0x.h
 * Author: colin
 *
 * Created on February 19, 2023, 1:00 PM
 */

#ifndef ADS131M0X_H
#define	ADS131M0X_H

// Must call this before including ads131m0x_def.h
#define ADS131M0X_CHANNEL_COUNT 8   	// ADS131M08 -> 8 Channels

#if ((ADS131M0X_CHANNEL_COUNT < 1) || (ADS131M0X_CHANNEL_COUNT > 8))
    #error Invalid channel count configured in 'ads131m0x.h'.
#endif

#include <stdint.h>
#include <stdbool.h>
#include "ads131m0x_def.h"

//****************************************************************************
//
// Select the device variant to use...
//
//****************************************************************************



#ifdef	__cplusplus
extern "C" {
#endif

	typedef union {
        int32_t raw;
        uint8_t b[4];
    } ads131m0x_conversion_t;
	typedef struct {
		void		(*delay_ms)(uint32_t milliseconds);
        void		(*delay_us)(uint32_t microseconds);
		uint32_t	(*millis)(void);
		void		(*set_syncResetPin)(bool state);
		void		(*transferFrame)(uint8_t frame_length);
		uint8_t 	*tx_buffer;
		uint8_t 	*rx_buffer;
		uint16_t 	clock;
		uint16_t 	config;
		uint8_t 	gain[ADS131M0X_CHANNEL_COUNT];		// more memory but much faster execution for conversions
		ads131m0x_conversion_t *conversion;
	} ads131m0x_hal_t;
    
    uint16_t ads131m0x_init(ads131m0x_hal_t *hal);

    void ads131m0x_reset(ads131m0x_hal_t *hal);
    
    void ads131m0x_resync(ads131m0x_hal_t *hal);

	uint16_t ads131m0x_process_new_conversion(ads131m0x_hal_t *hal);


    // Command Functions
	void ads131m0x_start_conversion(ads131m0x_hal_t *hal);

	void ads131m0x_enable_channels(ads131m0x_hal_t *hal, uint8_t enabled_channel_bitmap);

	void ads131m0x_disable_channels(ads131m0x_hal_t *hal, uint8_t disabled_channel_bitmap);

	void ads131m0x_enable_external_reference(ads131m0x_hal_t *hal);

	void ads131m0x_disable_external_reference(ads131m0x_hal_t *hal);

	void ads131m0x_channel_pga_update(ads131m0x_hal_t *hal);

	uint16_t ads131m0x_read_id(ads131m0x_hal_t *hal);

	uint16_t ads131m0x_read_status(ads131m0x_hal_t *hal);

	void ads131m0x_standby(ads131m0x_hal_t *hal);

	void ads131m0x_wakeup(ads131m0x_hal_t *hal);


#ifdef	__cplusplus
}
#endif

#endif	/* ADS131M0X_H */

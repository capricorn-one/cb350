/* 
 * File:   ads131m0x.h
 * Author: colin
 *
 * Created on February 19, 2023, 1:00 PM
 */

#ifndef ADS131M0X_H
#define	ADS131M0X_H

#include <stdint.h>
#include <stdbool.h>

//****************************************************************************
//
// Select the device variant to use...
//
//****************************************************************************

#define ADS131M0X_CHANNEL_COUNT (8)   // ADS131M04 -> 4 Channels

#define WORD_LENGTH_24BIT
//#define WORD_LENGTH_32BIT_SIGN_EXTEND
//#define WORD_LENGTH_32BIT_ZERO_PADDED


#if defined(WORD_LENGTH_16BIT_TRUNCATED)
#define WLENGTH     2
#elif defined(WORD_LENGTH_24BIT) 
#define WLENGTH     3
#elif defined(WORD_LENGTH_32BIT_SIGN_EXTEND) 
#define WLENGTH     4
#elif defined(WORD_LENGTH_32BIT_ZERO_PADDED)
#define WLENGTH     4
#endif

#define FRAME_LENGTH	((ADS131M0X_CHANNEL_COUNT + 2) * WLENGTH)		// 2 for Command and CRC


#if ((ADS131M0X_CHANNEL_COUNT < 1) || (ADS131M0X_CHANNEL_COUNT > 8))
    #error Invalid channel count configured in 'ads131m0x.h'.
#endif

#ifdef	__cplusplus
extern "C" {
#endif

	typedef struct {
		void		(*delay_ms)(uint32_t milliseconds);
        void		(*delay_us)(uint32_t microseconds);
		void		(*set_syncResetPin)(bool state);
		bool		(*dataReady)(void);
		void		(*transferFrame)(void);
		uint8_t 	transfer_buffer[FRAME_LENGTH];
		union {
			int32_t raw;
			uint8_t b[4];
		} conversion[ADS131M0X_CHANNEL_COUNT];
		float gain_ratio[ADS131M0X_CHANNEL_COUNT];
		
	} ads131m0x_hal_t;

    
    void ads131m0x_init(ads131m0x_hal_t *hal);

    void ads131m0x_reset(ads131m0x_hal_t *hal);
    
    void ads131m0x_trigger_conversion(ads131m0x_hal_t *hal);
    
    void ads131m0x_set_channel_gain_ratio(ads131m0x_hal_t *hal, uint8_t channel, float ratio);

    void ads131m0x_get_new_data(ads131m0x_hal_t *hal);

    float ads131m0x_get_channel_data(ads131m0x_hal_t *hal, uint8_t channel);

    int32_t ads131m0x_get_channel_data_raw(ads131m0x_hal_t *hal, uint8_t channel);
	

#ifdef	__cplusplus
}
#endif

#endif	/* ADS131M0X_H */
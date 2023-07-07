#include "ads131m0x.h"
#include "ads131m0x_drv.h"
#include <string.h>

// #include <stdio.h>

//****************************************************************************
//
// Function prototypes
//
//****************************************************************************

uint16_t    readSingleRegister(ads131m0x_hal_t *hal, uint8_t address);
void        writeSingleRegister(ads131m0x_hal_t *hal, uint8_t address, uint16_t data);
uint16_t    sendCommand(ads131m0x_hal_t *hal, uint16_t command);
bool        lockRegisters(ads131m0x_hal_t *hal);
bool        unlockRegisters(ads131m0x_hal_t *hal);
void        resetDevice(ads131m0x_hal_t *hal);
uint16_t    calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue);


static uint16_t transfer_data(ads131m0x_hal_t *hal, uint16_t cmd, uint16_t data);
static uint16_t read_frame_data(ads131m0x_hal_t *hal);

void ads131m0x_init(ads131m0x_hal_t *hal) {
    
    for(uint8_t i=0; i<ADS131M0X_CHANNEL_COUNT; i++) {
		hal->gain_ratio[i] = 1.00;
		hal->conversion[i].raw = 0;
	}

	memset(hal->transfer_buffer, 0, FRAME_LENGTH);

    ///* (OPTIONAL) Toggle nRESET pin to ensure default register settings. */
    ///* NOTE: This also ensures that the device registers are unlocked.	 */
    //hal->toggleRESET();
	//
	//hal->delay_ms(5);

    /* (OPTIONAL) Validate first response word when beginning SPI communication: (0xFF20 | CHANCNT) */
    // uint16_t response = sendCommand(OPCODE_NULL);
    //sendCommand(OPCODE_NULL);

	

	writeSingleRegister(hal,
						CLOCK_ADDRESS,
						CLOCK_CH0_EN_ENABLED |
						CLOCK_CH1_EN_ENABLED |
						CLOCK_CH2_EN_ENABLED |
						CLOCK_CH3_EN_ENABLED |
						CLOCK_CH4_EN_ENABLED |
						CLOCK_CH5_EN_ENABLED |
						CLOCK_CH6_EN_ENABLED |
						CLOCK_CH7_EN_ENABLED |
						CLOCK_XTAL_DISABLED |
						CLOCK_EXTREF_ENABLED |
						CLOCK_OSR_16256 |
						CLOCK_PWR_HR
	);

	writeSingleRegister(hal,
						CFG_ADDRESS,
						CFG_GC_DLY_65536 |
						CFG_GC_EN_ENABLED
	);

	writeSingleRegister(hal, MODE_ADDRESS, MODE_DEFAULT);
	
	//lockRegisters(hal);
	 
	//printf("ADS131m0x found with chip id 0x%04X\r\n", readSingleRegister(hal, ID_ADDRESS));
	 
	// uint16_t cfg_reg = readSingleRegister(hal, CFG_ADDRESS);
	 
	//printf("CFG Reg 0x%04X\r\n", cfg_reg);
	 
	// uint16_t clock_reg = readSingleRegister(hal, CLOCK_ADDRESS);
	
	//printf("Clock Reg 0x%04X\r\n", clock_reg);
	 
	//sendCommand(OPCODE_STANDBY);

}

void ads131m0x_reset(ads131m0x_hal_t *hal) {
    hal->set_syncResetPin(false);
    hal->delay_ms(3);
    hal->set_syncResetPin(true);
    hal->delay_ms(3);
}

void ads131m0x_trigger_conversion(ads131m0x_hal_t *hal) {
    hal->set_syncResetPin(false);
    hal->delay_us(3);
    hal->set_syncResetPin(true);
}

void ads131m0x_set_channel_gain_ratio(ads131m0x_hal_t *hal, uint8_t channel, float ratio) {
    if(channel >= ADS131M0X_CHANNEL_COUNT)
		return;

	hal->gain_ratio[channel] = ratio;
}

void ads131m0x_get_new_data(ads131m0x_hal_t *hal) {
	
	while(!hal->dataReady());
	
	// read new data
	sendCommand(hal, OPCODE_NULL);

}

float ads131m0x_get_channel_data(ads131m0x_hal_t *hal, uint8_t channel) {
    if(channel >= ADS131M0X_CHANNEL_COUNT)
		return -1.000;

	return (hal->conversion[channel].raw / 8388607.) * 1.200 * hal->gain_ratio[channel];
}

int32_t ads131m0x_get_channel_data_raw(ads131m0x_hal_t *hal, uint8_t channel) {
    if(channel >= ADS131M0X_CHANNEL_COUNT)
		return -1;

	return hal->conversion[channel].raw;
}

static uint16_t transfer_data(ads131m0x_hal_t *hal, uint16_t cmd, uint16_t data) {
	
	hal->transfer_buffer[0] = cmd>>8;
	hal->transfer_buffer[1] = cmd&0xFF;
	hal->transfer_buffer[2] = 0x00;
	
	hal->transfer_buffer[3] = data>>8;
	hal->transfer_buffer[4] = data&0xFF;
	hal->transfer_buffer[5] = 0x00;

	
	hal->transferFrame();
	
	return read_frame_data(hal);
}

static uint16_t read_frame_data(ads131m0x_hal_t *hal) {

	for(uint8_t i=0; i<ADS131M0X_CHANNEL_COUNT; i++) {
		
		hal->conversion[i].b[3] = hal->transfer_buffer[3*(i+1) + 0];
		hal->conversion[i].b[2] = hal->transfer_buffer[3*(i+1) + 1];
		hal->conversion[i].b[1] = hal->transfer_buffer[3*(i+1) + 2];
		
		hal->conversion[i].raw = hal->conversion[i].raw>>8;		// Right-shift of signed data maintains signed bit
	}

	//rx_buffer[27...29] = CRC

	return ((hal->transfer_buffer[0]<<8) | (hal->transfer_buffer[1]&0xFF));

}

//*****************************************************************************
//
//! Reads the contents of a single register at the specified address.
//!
//! \fn uint16_t readSingleRegister(uint8_t address)
//!
//! \param address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//
//*****************************************************************************
uint16_t readSingleRegister(ads131m0x_hal_t *hal, uint8_t address)
{
	/* Check that the register address is in range */
	assert(address < NUM_REGISTERS);

// // Build TX and RX byte array
// #ifdef ENABLE_CRC_IN
//     uint8_t dataTx[8] = { 0 };      // 2 words, up to 4 bytes each = 8 bytes maximum
//     uint8_t dataRx[8] = { 0 };
// #else
//     uint8_t dataTx[4] = { 0 };      // 1 word, up to 4 bytes long = 4 bytes maximum
//     uint8_t dataRx[4] = { 0 };
// #endif
//     uint16_t opcode = OPCODE_RREG | (((uint16_t) address) << 7);
//     uint8_t numberOfBytes = buildSPIarray(&opcode, 1, dataTx);

	// [FRAME 1] Send RREG command
	// ads131m0x_comm->spiSendReceiveArrays(dataTx, dataRx, numberOfBytes);
    sendCommand(hal, OPCODE_RREG | (((uint16_t) address) << 7));

	// [FRAME 2] Send NULL command to retrieve the register data
	return sendCommand(hal, OPCODE_NULL);
}



//*****************************************************************************
//
//! Writes data to a single register.
//!
//! \fn void writeSingleRegister(uint8_t address, uint16_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! This command will be ignored if device registers are locked.
//!
//! \return None.
//
//*****************************************************************************
void writeSingleRegister(ads131m0x_hal_t *hal, uint8_t address, uint16_t data)
{
    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    transfer_data(hal, OPCODE_WREG | (((uint16_t) address) << 7), data);
}



//*****************************************************************************
//
//! Sends the specified SPI command to the ADC (NULL, STANDBY, or WAKEUP).
//!
//! \fn uint16_t sendCommand(uint16_t opcode)
//!
//! \param opcode SPI command byte.
//!
//! NOTE: Other commands have their own dedicated functions to support
//! additional functionality.
//!
//! \return ADC response byte (typically the STATUS byte).
//
//*****************************************************************************
uint16_t sendCommand(ads131m0x_hal_t *hal, uint16_t command)
{
    /* Assert if this function is used to send any of the following opcodes */
    //assert(OPCODE_RREG != opcode);      /* Use "readSingleRegister()"   */
    //assert(OPCODE_WREG != opcode);      /* Use "writeSingleRegister()"  */
    //assert(OPCODE_LOCK != opcode);      /* Use "lockRegisters()"        */
    //assert(OPCODE_UNLOCK != opcode);    /* Use "unlockRegisters()"      */
    //assert(OPCODE_RESET != opcode);     /* Use "resetDevice()"          */

	return transfer_data(hal, command, 0);
}



//*****************************************************************************
//
//! Resets the device.
//!
//! \fn void resetDevice(void)
//!
//! NOTE: This function does not capture DOUT data, but it could be modified
//! to do so.
//!
//! \return None.
//
//*****************************************************************************
void resetDevice(ads131m0x_hal_t *hal)
{
    sendCommand(hal, OPCODE_RESET);

    // tSRLRST delay, ~1ms with 2.048 MHz fCLK
    hal->delay_ms(2);

    // Update register setting array to keep software in sync with device
    //restoreRegisterDefaults();

    // Write to MODE register to enforce mode settings
    writeSingleRegister(hal, MODE_ADDRESS, MODE_DEFAULT);
}



//*****************************************************************************
//
//! Sends the LOCK command and verifies that registers are locked.
//!
//! \fn bool lockRegisters(void)
//!
//! \return boolean to indicate if an error occurred (0 = no error; 1 = error)
//
//*****************************************************************************
bool lockRegisters(ads131m0x_hal_t *hal)
{
    sendCommand(hal, OPCODE_LOCK);

    return false;
}



//*****************************************************************************
//
//! Sends the UNLOCK command and verifies that registers are unlocked
//!
//! \fn bool unlockRegisters(void)
//!
//! \return boolean to indicate if an error occurred (0 = no error; 1 = error)
//
//*****************************************************************************
bool unlockRegisters(ads131m0x_hal_t *hal)
{
    sendCommand(hal, OPCODE_UNLOCK);

    return false;
}


//*****************************************************************************
//
//! Calculates the 16-bit CRC for the selected CRC polynomial.
//!
//! \fn uint16_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
//!
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes number of bytes to be used in CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use 0xFFFF when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and is not optimized for speed.
//!
//! \return 16-bit calculated CRC word
//
//*****************************************************************************
uint16_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
{
	/* Check that "dataBytes" is not a null pointer */
	assert(dataBytes != 0x00);

	int         bitIndex, byteIndex;
	bool        dataMSb;						/* Most significant bit of data byte */
	bool        crcMSb;						    /* Most significant bit of crc byte  */
	// uint8_t     bytesPerWord = wlength_byte_values[WLENGTH];

	/*
     * Initial value of crc register
     * NOTE: The ADS131M0x defaults to 0xFFFF,
     * but can be set at function call to continue an on-going calculation
     */
    uint16_t crc = initialValue;

    #ifdef CRC_CCITT
    /* CCITT CRC polynomial = x^16 + x^12 + x^5 + 1 */
    const uint16_t poly = 0x1021;
    #endif

    #ifdef CRC_ANSI
    /* ANSI CRC polynomial = x^16 + x^15 + x^2 + 1 */
    const uint16_t poly = 0x8005;
    #endif

    //
    // CRC algorithm
    //

    // Loop through all bytes in the dataBytes[] array
	for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
	{
	    // Point to MSb in byte
	    bitIndex = 0x80u;

	    // Loop through all bits in the current byte
	    while (bitIndex > 0)
	    {
	        // Check MSB's of data and crc
	        dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
	        crcMSb  = (bool) (crc & 0x8000u);

	        crc <<= 1;              /* Left shift CRC register */

	        // Check if XOR operation of MSBs results in additional XOR operations
	        if (dataMSb ^ crcMSb)
	        {
	            crc ^= poly;        /* XOR crc with polynomial */
	        }

	        /* Shift MSb pointer to the next data bit */
	        bitIndex >>= 1;
	    }
	}

	return crc;
}
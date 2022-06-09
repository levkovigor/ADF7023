/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "Arduino.h"
#include <stdlib.h>
#include "adf7023_config.h"
#include "adf7023.h"
#include <SPI.h>


/******************************************************************************/
/*************************** Macros Definitions *******************************/
/******************************************************************************/


adf7023::adf7023(int slaveSelectPin, int misoPin, int spiClockDivider, SPIClass *spiinterface){
	
	// set up chip select, miso pins, spi clock divider (SPI_CLOCK_DIV32 if not set) and spi interface (default SPI if not set)
	_slaveSelectPin = slaveSelectPin;
	_misoPin = misoPin;
	_lspi = spiinterface;
	_spiClockDivider = spiClockDivider;
	
}

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/


/***************************************************************************//**
 * @brief Transfers one byte of data.
 *
 * @param write_byte - Write data.
 * @param read_byte  - Read data.
 *
 * @return None.
*******************************************************************************/
void adf7023::write_read_byte(uint8_t write_byte,
			     uint8_t* read_byte)
{
	uint8_t data = 0;

	data = write_byte;
	data = _lspi->transfer(data);
	if (read_byte)
		*read_byte = data;
}

/***************************************************************************//**
 * @brief Initializes the ADF7023.
 *
 * @return ret - Result of the initialization procedure.
 *               Example: 0 - if initialization was successful;
 *                        -1 - if initialization was unsuccessful.
*******************************************************************************/
int32_t adf7023::init()
{
	uint8_t miso = 0;
	uint16_t timeout = 0;
	uint8_t status = 0;
	int32_t ret = 0;

	pinMode(_slaveSelectPin, OUTPUT);

	_lspi->setBitOrder(MSBFIRST);
	_lspi->setDataMode(SPI_MODE0);
	_lspi->setClockDivider(_spiClockDivider);
	_lspi->begin();

	adf7023_bbram_current = adf7023_bbram_default;

	digitalWrite(_slaveSelectPin, HIGH);
	delay(1000);
	digitalWrite(_slaveSelectPin, LOW);
	unsigned long timeOut = millis();
	while ((miso == 0) && (timeout < 1000)) {
		miso = digitalRead(_misoPin);
		if ((timeOut + 5000) < millis()) return -1;
	}
	
	/*if (timeout == 1000) {
		ret = -1;
		//return ret;
	}*/
	
	timeOut = millis();
	
	while(!(status & STATUS_CMD_READY)) {
		get_status(&status);
		if ((timeOut + 5000) < millis()) return -1;
	}
	set_ram(0x100, 64, (uint8_t*)&adf7023_bbram_current);
	set_command(CMD_CONFIG_DEV);

	return 0;
}

/***************************************************************************//**
 * @brief Free the resources allocated by adf7023_init().
 *
 * @return ret - Result of the procedure.
 *               Example: 0 - if successful;
 *                        -1 - if unsuccessful.
*******************************************************************************/
int32_t adf7023::remove()
{
	int32_t ret = 0;
	set_command(CMD_PHY_OFF);
	_lspi->end();

	return ret;
}

/***************************************************************************//**
 * @brief Reads the status word of the ADF7023.
 *
 * @param status - Status word.
 *
 * @return None.
*******************************************************************************/
void adf7023::get_status(uint8_t* status)
{
	digitalWrite(_slaveSelectPin, LOW);
	write_read_byte(SPI_NOP, 0);
	write_read_byte(SPI_NOP, status);
	digitalWrite(_slaveSelectPin, HIGH);
}

/***************************************************************************//**
 * @brief Initiates a command.
 *
 * @param command - Command.
 *
 * @return None.
*******************************************************************************/
void adf7023::set_command(uint8_t command)
{
	digitalWrite(_slaveSelectPin, LOW);
	write_read_byte(command, 0);
	digitalWrite(_slaveSelectPin, HIGH);
}

/***************************************************************************//**
 * @brief Sets a FW state and waits until the device enters in that state.
 *
 * @param fw_state - FW state.
 *
 * @return None.
*******************************************************************************/
void adf7023::set_fw_state(uint8_t fw_state)
{
	uint8_t status = 0;

	switch(fw_state) {
	case FW_STATE_PHY_OFF:
		set_command(CMD_PHY_OFF);
		break;
	case FW_STATE_PHY_ON:
		set_command(CMD_PHY_ON);
		break;
	case FW_STATE_PHY_RX:
		set_command(CMD_PHY_RX);
		break;
	case FW_STATE_PHY_TX:
		set_command(CMD_PHY_TX);
		break;
	case FW_STATE_HW_RESET:
		set_command(CMD_HW_RESET);
		fw_state = FW_STATE_PHY_OFF;
		break;
	case FW_STATE_GET_RSSI:
		set_command(CMD_GET_RSSI);
		fw_state = FW_STATE_PHY_ON;
		break;
	default:
		set_command(CMD_PHY_SLEEP);
	}
	while((status & STATUS_FW_STATE) != fw_state) {
		get_status(&status);	
	}	
}

/***************************************************************************//**
 * @brief Reads data from the RAM.
 *
 * @param address - Start address.
 * @param length  - Number of bytes to write.
 * @param data    - Read buffer.
 *
 * @return None.
*******************************************************************************/
void adf7023::get_ram(uint32_t address,
		     uint32_t length,
		     uint8_t* data)
{
	digitalWrite(_slaveSelectPin, LOW);
	write_read_byte(SPI_MEM_RD | ((address & 0x700) >> 8), 0);
	write_read_byte(address & 0xFF, 0);
	write_read_byte(SPI_NOP, 0);
	while(length--) {
		write_read_byte(SPI_NOP, data++);
	}
	digitalWrite(_slaveSelectPin, HIGH);
}

/***************************************************************************//**
 * @brief Writes data to RAM.
 *
 * @param address - Start address.
 * @param length  - Number of bytes to write.
 * @param data    - Write buffer.
 *
 * @return None.
*******************************************************************************/
void adf7023::set_ram(uint32_t address,
		     uint32_t length,
		     uint8_t* data)
{
	digitalWrite(_slaveSelectPin, LOW);
	write_read_byte(SPI_MEM_WR | ((address & 0x700) >> 8), 0);
	write_read_byte(address & 0xFF, 0);
	while(length--) {
		write_read_byte(*(data++), 0);
	}
	digitalWrite(_slaveSelectPin, HIGH);
}

/***************************************************************************//**
 * @brief Receives one packet.
 *
 * @param packet - Data buffer.
 * @param length - Number of received bytes.
 *
 * @return None.
*******************************************************************************/
void adf7023::receive_packet(uint8_t* packet,
			    uint8_t* length)
{
	uint8_t interrupt_reg = 0;

	set_fw_state(FW_STATE_PHY_ON);
	set_fw_state(FW_STATE_PHY_RX);
	while(!(interrupt_reg & BBRAM_INTERRUPT_MASK_0_INTERRUPT_CRC_CORRECT)) {
		get_ram(MCR_REG_INTERRUPT_SOURCE_0,
				0x1,
				&interrupt_reg);
	}
	set_ram(MCR_REG_INTERRUPT_SOURCE_0,
			0x1,
			&interrupt_reg);
	get_ram(0x10, 1, length);
	get_ram(0x12, *length - 2, packet);
}

/***************************************************************************//**
 * @brief Transmits one packet.
 *
 * @param packet - Data buffer.
 * @param length - Number of bytes to transmit.
 *
 * @return None.
*******************************************************************************/
void adf7023::transmit_packet(uint8_t* packet,
			     uint8_t length)
{
	uint8_t interrupt_reg = 0;
	uint8_t header [ 2 ]    = {0, 0};

	header[0] = 2 + length;
	header[1] = adf7023_bbram_current.address_match_offset;
	set_ram(0x10, 2, header);
	set_ram(0x12, length, packet);
	set_fw_state(FW_STATE_PHY_ON);
	set_fw_state(FW_STATE_PHY_TX);
	while(!(interrupt_reg & BBRAM_INTERRUPT_MASK_0_INTERRUPT_TX_EOF)) {
		get_ram(MCR_REG_INTERRUPT_SOURCE_0,
				0x1,
				&interrupt_reg);
	}
}

/***************************************************************************//**
 * @brief Sets the channel frequency.
 *
 * @param ch_freq - Channel frequency.
 *
 * @return None.
*******************************************************************************/
void adf7023::set_channel_frequency(uint32_t ch_freq)
{
	ch_freq = (uint32_t)(((float)ch_freq / 26000000) * 65535);
	adf7023_bbram_current.channel_freq0 = (ch_freq & 0x0000FF) >> 0;
	adf7023_bbram_current.channel_freq1 = (ch_freq & 0x00FF00) >> 8;
	adf7023_bbram_current.channel_freq2 = (ch_freq & 0xFF0000) >> 16;
	set_ram(0x100, 64, (uint8_t*)&adf7023_bbram_current);
}

/***************************************************************************//**
 * @brief Sets the data rate.
 *
 * @param data_rate - Data rate.
 *
 * @return None.
*******************************************************************************/
void adf7023::set_data_rate(uint32_t data_rate)
{
	data_rate = (uint32_t)(data_rate / 100);
	adf7023_bbram_current.radio_cfg0 =
		BBRAM_RADIO_CFG_0_DATA_RATE_7_0((data_rate & 0x00FF) >> 0);
	adf7023_bbram_current.radio_cfg1 &= ~BBRAM_RADIO_CFG_1_DATA_RATE_11_8(0xF);
	adf7023_bbram_current.radio_cfg1 |=
		BBRAM_RADIO_CFG_1_DATA_RATE_11_8((data_rate & 0x0F00) >> 8);
	set_ram(0x100, 64, (uint8_t*)&adf7023_bbram_current);
	set_fw_state(FW_STATE_PHY_OFF);
	set_command(CMD_CONFIG_DEV);
}

/***************************************************************************//**
 * @brief Sets the frequency deviation.
 *
 * @param freq_dev - Frequency deviation.
 *
 * @return None.
*******************************************************************************/
void adf7023::set_frequency_deviation(uint32_t freq_dev)
{
	freq_dev = (uint32_t)(freq_dev / 100);
	adf7023_bbram_current.radio_cfg1 &=
		~BBRAM_RADIO_CFG_1_FREQ_DEVIATION_11_8(0xF);
	adf7023_bbram_current.radio_cfg1 |=
		BBRAM_RADIO_CFG_1_FREQ_DEVIATION_11_8((freq_dev & 0x0F00) >> 8);
	adf7023_bbram_current.radio_cfg2 =
		BBRAM_RADIO_CFG_2_FREQ_DEVIATION_7_0((freq_dev & 0x00FF) >> 0);
	set_ram(0x100, 64, (uint8_t*)&adf7023_bbram_current);
	set_fw_state(FW_STATE_PHY_OFF);
	set_command(CMD_CONFIG_DEV);
}

/***************************************************************************//**
 * @brief More than 0 when Packet Received and CRC is correct.
 *
 * @return ret - More than 0 when Packet Received and CRC is correct.
 *               Example: 0 - if there is no packet;
 *                        > 0 - if packet received and CRC is correct.
*******************************************************************************/

int32_t adf7023::available()
{
	uint8_t interrupt_reg = 0;
	uint8_t status = 0;

	get_status(&status);
	if ((status & STATUS_FW_STATE) != FW_STATE_PHY_RX)
	{
		set_fw_state(FW_STATE_PHY_ON);
		set_fw_state(FW_STATE_PHY_RX);
	}
	
	get_ram(MCR_REG_INTERRUPT_SOURCE_0, 0x1, &interrupt_reg);
	
	return ((interrupt_reg & BBRAM_INTERRUPT_MASK_0_INTERRUPT_CRC_CORRECT) > 0);
}

/***************************************************************************//**
 * @brief More than 0 when Preamble Detected.
 *
 * @return ret - More than 0 when Preamble Detected.
 *               Example: 0 - if preamble is not detected;
 *                        > 0 - if preamble is detected.
*******************************************************************************/

int32_t adf7023::preambleDetected()
{
	uint8_t interrupt_reg = 0;
	uint8_t status = 0;

	get_status(&status);
	if ((status & STATUS_FW_STATE) != FW_STATE_PHY_RX)
	{
		set_fw_state(FW_STATE_PHY_ON);
		set_fw_state(FW_STATE_PHY_RX);
	}
	
	get_ram(MCR_REG_INTERRUPT_SOURCE_0, 0x1, &interrupt_reg);
	
	return ((interrupt_reg & BBRAM_INTERRUPT_MASK_0_INTERRUPT_PREMABLE_DETECT) > 0);
}

/***************************************************************************//**
 * @brief RSSI Measurement with CMD_GET_RSSI.
 *
 * @return ret - RSSI in dbm (int).
*******************************************************************************/

int adf7023::readRSSI_PHY_ON()
{
	uint8_t rssi_readback = 0;
	get_ram(MCR_REG_RSSI_READBACK, 0x1, &rssi_readback);
	return (int)rssi_readback - 107;
}

/***************************************************************************//**
 * @brief RSSI Measurement at PHY_RX state.
 *
 * @return ret - RSSI in dbm (float).
*******************************************************************************/

float adf7023::readRSSI_PHY_RX()
{
	
	uint8_t adc_readback_high = 0;
	uint8_t adc_readback_low = 0;
	uint8_t adc_gain_status = 0;
	get_ram(MCR_REG_AGC_GAIN_STATUS , 0x1, &adc_gain_status);
	
	int gain_correction = 0;
	switch(adc_gain_status) {
	case 0x00:
		gain_correction = 44;
		break;
	case 0x01:
		gain_correction = 35;
		break;
	case 0x02:
		gain_correction = 26;
		break;
	case 0x0A:
		gain_correction = 17;
		break;
	case 0x12:
		gain_correction = 10;
		break;
	case 0x16:
		gain_correction = 0;
		break;
	default:
		gain_correction = 0;
	}
	
	get_ram(MCR_REG_ADC_READBACK_HIGH , 0x1, &adc_readback_high);
	get_ram(MCR_REG_ADC_READBACK_LOW , 0x1, &adc_readback_low);
	get_ram(MCR_REG_ADC_READBACK_HIGH , 0x1, &adc_readback_high);
	adc_readback_high = adc_readback_high & 0b11111100;
	adc_readback_low = adc_readback_low & 0b000000011;
	adc_readback_high = adc_readback_high | adc_readback_low;
	return (float)adc_readback_high / 7.0 + (float)gain_correction - 109.0;
}

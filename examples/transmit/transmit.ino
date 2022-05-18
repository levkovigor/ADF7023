#include <adf7023.h>

/*
 * if not default SPI Interface - useful for STM32Duino
 *
 * SPIClass _mySPI;
 */

adf7023 ADF7023(5, 19/*, SPI_CLOCK_DIV32, &_mySPI*/);
/*
 * For STM32F405RGT6 tested with:
 * adf7023 ADF7023(PA4, PA6, SPI_CLOCK_DIV32, &_mySPI);
 * 
 * For ESP32 tested with:
 * adf7023 ADF7023(5, 19, SPI_CLOCK_DIV32);
 */
 
void setup() { 
  Serial.begin(115200);
  
  /*
   * Custom SPI Pins - useful for STM32Duino
   *
   * _mySPI.setMOSI(PA7);
   * _mySPI.setMISO(PA6);
   * _mySPI.setSCLK(PA5);
   */

   
  if (ADF7023.init() != -1) 
	{
	  ADF7023.set_fw_state(FW_STATE_HW_RESET);
	  ADF7023.set_fw_state(FW_STATE_PHY_ON);
	  ADF7023.set_data_rate(38400);
	  ADF7023.set_frequency_deviation(10000);
	  ADF7023.set_channel_frequency(868225000);
	} else {
		Serial.println("Init failed");
		while(1);
	}
}


void loop() {
  delay(1000);
  Serial.println("Send packet: qwerty");
  ADF7023.transmit_packet((uint8_t*)"qwerty", 6);
}

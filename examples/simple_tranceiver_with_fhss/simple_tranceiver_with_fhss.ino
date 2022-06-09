#include <adf7023.h>

#define CHANNELS          28
#define CHANNELS_REPEAT   5
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
 * 
 * For Arduino UNO tested with:
 * adf7023 ADF7023(10, 12, SPI_CLOCK_DIV2);
 */

long freq[CHANNELS] = {863150000, 863350000, 863550000, 863750000, 863950000,  //L
                       864150000, 864350000, 864550000, 864750000,            //L
                       865150000, 865350000, 865550000, 865750000, 865950000, //L
                       866150000, 866350000, 866550000, 866750000, 866950000, //L
                       867150000, 867350000, 867550000, 867750000,            //L
                       868150000, 868350000,                                  //M
                       868850000, 869050000,                                  //N
                       869850000                                              //Q/R        
                       };
float freq_rssi[CHANNELS];
int fhss_channel = 0;

unsigned long cmdID = 0;

uint8_t packet[300];
uint8_t len;

String last_cmd = "";

String serial_buffer = "";
 
void setup() { 
  Serial.begin(115200);
  
  /*
   * Custom SPI Pins - useful for STM32Duino
   *
   * _mySPI.setMOSI(PA7);
   * _mySPI.setMISO(PA6);
   * _mySPI.setSCLK(PA5);
   */

   
  int init = -1; 
  while (init == -1){
    init = ADF7023.init();
    if (init != -1) 
    {
      ADF7023.set_fw_state(FW_STATE_HW_RESET);
      ADF7023.set_fw_state(FW_STATE_PHY_ON);
      ADF7023.set_data_rate(38400);
      ADF7023.set_frequency_deviation(10000);
      ADF7023.set_channel_frequency(freq[0]);
    } else {
      Serial.println("Init failed");
      ADF7023.set_fw_state(FW_STATE_HW_RESET);
      ADF7023.remove();
    }
  }
  Serial.println("End setup");
}

void freq_swap(int ch){
  ADF7023.set_fw_state(FW_STATE_PHY_ON);
  ADF7023.set_channel_frequency(freq[ch]);
  ADF7023.set_fw_state(FW_STATE_PHY_RX);
}

int freqCHLimit(int fchl){
  if (fchl >= CHANNELS) fchl -= CHANNELS;
  if (fchl < 0) fchl += CHANNELS;
  return fchl;
}

float rssiMID(int midCH){
  float rssiMID = 0;
  int chRepeat = (CHANNELS_REPEAT - 1) / 2;
  int c = 0;
  for (int chi = midCH-3*chRepeat; chi <= midCH+3*chRepeat; chi++){
    rssiMID += freq_rssi[freqCHLimit(chi)];
    c++;
  }
  return rssiMID / (float)c;
}

void receive(){
  String receiveSerial = "";
  for (int i = 0; i < CHANNELS; i++){
      freq_swap(i);
      delayMicroseconds(20);
      freq_rssi[i] = ADF7023.readRSSI_PHY_RX();
  }

  bool found = true;
    while(found){
      found = false;
      int currentCH = 0;
      float rssi = -200;
      for (int i = 0; i < CHANNELS; i++){
        if (freq_rssi[i] > rssi){
          rssi = freq_rssi[i];
          currentCH = i;
        }
      }
      freq_rssi[currentCH] = -109;
      if (rssi > -100){
        found = true;
        freq_swap(currentCH);
        delayMicroseconds(20);
        if(ADF7023.readRSSI_PHY_RX() > -100){
          unsigned long timeOut = micros();
          while(ADF7023.preambleDetected() == 0){
            if (timeOut + 550 < micros()) break;
          }
          if (ADF7023.preambleDetected()){
            timeOut = millis();
             while(ADF7023.available() == 0){
                if (timeOut + 150 < millis()) break;
             }
             if (ADF7023.available())
              {
                String currenCMD = "";
                ADF7023.receive_packet(packet, &len);
                if ((len > 0))
                {
                  for(int i = 0; i < len-2; i++){
                    currenCMD += char(packet[i]);
                  }
                  if (currenCMD != last_cmd){
                    last_cmd = currenCMD;
                    receiveSerial = last_cmd + " :: FREQ : " + String(currentCH);
                    Serial.println(receiveSerial);
                  } else {
                    //receiveSerial += " " + String(currentCH);
                  }
                }
              } 
          }
        }
        
      }
    }
}

void transmit(String cmd){
  String pk =  String(cmdID) + ">" + cmd;
  cmdID++;
  String transmitSerial = "Send packet: " + pk;
  
  for (int i = 0; i < CHANNELS; i++){
    freq_swap(i);
    delayMicroseconds(20);
    freq_rssi[i] = ADF7023.readRSSI_PHY_RX();
  }

  fhss_channel += (CHANNELS_REPEAT - 1) * 2 + 1 + random(0, (CHANNELS_REPEAT - 1)/2);
  fhss_channel = freqCHLimit(fhss_channel);
  float minRSSI = 0;
  int currentCH = fhss_channel;
  for (int k = 0; k < CHANNELS; k++){
      float minCurRSSI = rssiMID(freqCHLimit(fhss_channel+k));
      if (minCurRSSI < minRSSI){
        minRSSI = minCurRSSI;
        currentCH = freqCHLimit(fhss_channel+k);
        if (minRSSI <= -100) break;
      }  
  }
  fhss_channel = currentCH;

  int chRepeat = (CHANNELS_REPEAT - 1) / 2;
  
  for(int j = chRepeat*-1; j <= chRepeat; j++){    
    ADF7023.set_fw_state(FW_STATE_PHY_ON);
    ADF7023.set_channel_frequency(freq[freqCHLimit(fhss_channel+j*2)]);
    ADF7023.set_fw_state(FW_STATE_PHY_TX);
    ADF7023.transmit_packet((uint8_t*)pk.c_str(), pk.length());
    transmitSerial += " " + String(freqCHLimit(fhss_channel+j*2)) + " ";
    unsigned long timeout = millis();
    while(timeout + 55 > millis()){
      receive();
    }
  }
  Serial.println(transmitSerial);
}

void loop() {
  if (Serial.available()){
    char CH_Serial = Serial.read();
    if (CH_Serial == '\n'){
      transmit(serial_buffer);
      serial_buffer = "";
    } else {
      serial_buffer += CH_Serial;
    }
  }
  
  receive(); 
}

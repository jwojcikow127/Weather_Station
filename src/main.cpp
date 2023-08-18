#include <Arduino.h>
#include <Wire.h>
#include <SensirionCore.h>
#include "PMS.h"
//#include "PMS.cpp"
// pinout definition -------------------------------------------------
// PMS sensor UART pins
#define RXD2 16
#define TXD2 17
// PMS sensor SET pin
#define PMS_SET 19
// PMS sensor RESET pin 
#define PMS_RESET 18
// CO2 TEMP and HUM I2C 


//  -------------------------------------------------------------------

#define uS_TO_S_FACTOR 1000000
#define TIME_BETWEEN_MEASURE 1200  // time that ESP will sleep between measures

PMS pms3003(Serial2);
PMS::DATA pms_data; 



void setup() {
  // PMS3003 config  
  Serial2.begin(9600);
  pms3003.passiveMode();


}

void loop() {

  // main  loop 
  esp_sleep_enable_timer_wakeup(TIME_BETWEEN_MEASURE * uS_TO_S_FACTOR);  // waking up the ESP




  // ******************* measure cycle ********************



  // pms3003 measure 
  pms3003.wakeUp(); // waking up the pollution sensor
  delay(30000); 
  pms3003.requestRead();
  if (pms3003.readUntil(pms_data))
  {

  }
  else
  {
    //something gone wrong ! need to implement a functionality 
  }
  pms3003.sleep(); // Pollution sensor goes to sleep mode

  // **************** end of measure cycle *****************





  // ******************* sending data to smartphone ********************
  // ******************* end of sending data ***************************




  esp_deep_sleep_start(); // ESP goes to sleep 

}





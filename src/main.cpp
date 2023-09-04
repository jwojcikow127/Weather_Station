#include <Arduino.h>
#include <Wire.h>
#include <SensirionCore.h>
#include "PMS.h"
#include "sensors.h"
#include <SensirionI2CScd4x.h>


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
#define i2C_SDA 21
#define i2C_SCL 22


//  -------------------------------------------------------------------

#define uS_TO_S_FACTOR 1000000
#define TIME_BETWEEN_MEASURE 1200  // time that ESP will sleep between measures

/*
// creation of object of PMS sensor and structure that storages PMS data
PMS pms3003(Serial2);
PMS::DATA pms_data; 
// creation of object of SCD4x sensor 
SensirionI2CScd4x scd4x;
*/
// structure that storages objects of sensors 
struct Sensors{
    PMS pms3003;
    PMS::DATA pms_data; 
    SensirionI2CScd4x scd4x;
    Sensors() : pms3003(Serial2) {}

} ; 
   
   

void setup() {
  // PMS3003 config  
  Serial2.begin(9600);
  Sensors sensors;
 







  //sensors.pms3003.passiveMode();
  // SCD4x config
  Wire.setPins(i2C_SDA, i2C_SCL);
  Wire.begin();
  sensors.scd4x.begin( Wire);

  
}

void loop() {

  // main  loop 
  esp_sleep_enable_timer_wakeup(TIME_BETWEEN_MEASURE * uS_TO_S_FACTOR);  // waking up the ESP




  // ******************* measure cycle ********************



  // all sensor measure  
  //sensorMeasure(sensors);

  // **************** end of measure cycle *****************





  // ******************* sending data to smartphone ********************
  // ******************* end of sending data ***************************




  esp_deep_sleep_start(); // ESP goes to sleep 

}





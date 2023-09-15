#include <Arduino.h>
#include <Wire.h>
#include <SensirionCore.h>
#include "PMS.h"
#include "sensors.h"
#include <SensirionI2CScd4x.h>
#include "bluetooth.h"
#include <esp_task_wdt.h>
#include "btn_led.h"

// operating modes of the device 
enum mode
{
  continous = 1, // continous measure mode with sleep between
  one_take = 2, // one take measure on demand 
};
mode current_mode = continous;
mode pervious_mode = current_mode;


// define constant values ******************************
#define WDT_TIMEOUT  60 // watchdog time to reset
#define uS_TO_S_FACTOR 1000000
#define TIME_BETWEEN_MEASURE 1200  // time that ESP will sleep between measures
// *****************************************************


// creating a structure with all sensors ***************
Sensors sensors;
SensorData sensor_data;
BluetoothSerial SerialBT;
// *****************************************************

  
void setup() {
  pinMode(BUTTON1,INPUT);
  // sensors and bluetooth connection initialization and configuration 
  allSensorsConfig(sensors);
  
  bluetoothConfig(SerialBT);
  

  // watchdog init--------------------------
  esp_task_wdt_init(WDT_TIMEOUT,true);
  esp_task_wdt_add(NULL);
  //----------------------------------------
  
  
  
}

// main  loop 
void loop() {
  //check mode (measurements with sleep or one single measure)

  

  esp_sleep_enable_timer_wakeup(TIME_BETWEEN_MEASURE * uS_TO_S_FACTOR);  // wake up the CPU




  // ******************* measure cycle ********************
  // all sensor measure  
  allSensorMeasure(sensors, sensor_data);

  // **************** end of measure cycle *****************





  // ******************* sending data to smartphone ********************
  bluetoothTransmit(SerialBT, sensor_data);
  // ******************* end of sending data ***************************



  esp_task_wdt_reset(); // watchdog reset 

  esp_deep_sleep_start(); // ESP goes to sleep 

}





#include <Arduino.h>
#include <Wire.h>
unsigned long previousMillis = 0;
#include <SensirionCore.h>
#include "PMS.h"
#include "sensors.h"
#include <SensirionI2CScd4x.h>
#include "bluetooth.h"
#include <esp_task_wdt.h>
#include "IO.h"

// operating modes of the device 
enum mode
{
  continous = 1, // continous measure mode with sleep between
  one_take = 2, // one take measure on demand 
};
mode current_mode = continous;
mode pervious_mode = current_mode;

mode modeCheckAndChange(mode mode)
{
  uint8_t button_state = btnClick();
  if(button_state == LOW && mode == continous )
  {
    mode = one_take;
  }
  if(button_state == HIGH && mode == one_take)
  {
    mode = continous;
  }
  return mode;
}




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

unsigned long currentMillis = millis();
  
void setup() {

  // configure button and led 
  btnAndLedConfig();
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

  //check current time 
  currentMillis = millis();
  

  current_mode = modeCheckAndChange(current_mode);
  //check mode (measurements with sleep or one single measure)
  if (current_mode == continous)
  {
  
    esp_sleep_enable_timer_wakeup(TIME_BETWEEN_MEASURE * uS_TO_S_FACTOR);  // wake up the CPU

    // ******************* measure cycle ********************
    allSensorMeasure(sensors, sensor_data, currentMillis);
    // **************** end of measure cycle *****************
    
    // ******************* sending data to smartphone ********************
    bluetoothTransmit(SerialBT, sensor_data);
    // ******************* end of sending data ***************************

    esp_deep_sleep_start(); // ESP goes to sleep 

  }
  else if (current_mode == one_take)
  {

    // ******************* measure cycle ********************      
    allSensorMeasure(sensors, sensor_data, currentMillis);
    // **************** end of measure cycle *****************

    // ******************* sending data to smartphone ********************
    bluetoothTransmit(SerialBT, sensor_data);
    // ******************* end of sending data ***************************

  }

  esp_task_wdt_reset(); // watchdog reset 


}





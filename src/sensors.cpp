#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "PMS.h"
#include "sensors.h"



// function for all sensor measurement 
void allSensorMeasure(Sensors& sensors, SensorData& data)
{
  PmsSensorMeasure(sensors.pms3003, data);
}

// function for PMS sensor measurement 
void PmsSensorMeasure(PMS& pms3003, SensorData& data)
{
  pms3003.wakeUp(); // waking up the pollution sensor
  delay(30000); 
  pms3003.requestRead();
  if (pms3003.readUntil(data.pms_data))
  {
    // need to implement some confirmation fuction 
  }
  else
  {
    //something gone wrong ! need to implement a functionality 
    Errors.error_PMS = 1;
  }
  pms3003.sleep(); // Pollution sensor goes to sleep mode
}

// fuction for sensors config 
void allSensorsConfig(Sensors& sensors)
{
  sensors.pms3003.passiveMode();
  Wire.setPins(i2C_SDA, i2C_SCL);
  Wire.begin();
  sensors.scd4x.begin(Wire);
  sensors.scd4x.powerDown();

}

void scd4xSensorMeasure(SensirionI2CScd4x& scd4x, scd4xData& data )
{
  Errors.error_scd4x = scd4x.wakeUp() ;
  Errors.error_scd4x = scd4x.measureSingleShot();
  Errors.error_scd4x = scd4x.readMeasurement(data.co2, data.temperature, data.humidity);
  
  if(Errors.error_scd4x)
  {
    //error handling
    
  }
  

}
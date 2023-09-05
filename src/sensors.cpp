#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "PMS.h"
#include "sensors.h"


// function for all sensor measurement 
void allSensorMeasure(Sensors& sensors)
{
  PmsSensorMeasure(sensors.pms3003, sensors.pms_data);

}



// function for PMS sensor measurement 
void PmsSensorMeasure(PMS pms3003, PMS::DATA data)
{
  pms3003.wakeUp(); // waking up the pollution sensor
  delay(30000); 
  pms3003.requestRead();
  if (pms3003.readUntil(data))
  {
    // need to implement some confirmation fuction 
  }
  else
  {
    //something gone wrong ! need to implement a functionality 
  }
  pms3003.sleep(); // Pollution sensor goes to sleep mode

}
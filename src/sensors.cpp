#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "PMS.h"
#include "sensors.h"
// function for sensor measurement 

void sensorMeasure(Sensors sensors)
{
  pms3003.wakeUp(); // waking up the pollution sensor
  delay(30000); 
  pms3003.requestRead();
  if (pms3003.readUntil(pms_data))
  {
    // need to implement some confirmation fuction 
  }
  else
  {
    //something gone wrong ! need to implement a functionality 
  }
  pms3003.sleep(); // Pollution sensor goes to sleep mode

}

void sensormm(Sensors sensor)
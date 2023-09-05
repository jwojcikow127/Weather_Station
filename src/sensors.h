#ifndef SENSORS_H
#define SENSORS_H

#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "PMS.h"
#include <Wire.h>

// structure that contains every sensor 
struct Sensors{
    // creation of object of PMS sensor and structure that storages PMS data
    PMS pms3003;
    PMS::DATA pms_data; 
    // creation of object of SCD4x sensor 
    SensirionI2CScd4x scd4x;
    Sensors() : pms3003(Serial2) {}
} ; 


void allSensorMeasure(Sensors& sensor);

void PmsSensorMeasure(PMS pms3003);








#endif
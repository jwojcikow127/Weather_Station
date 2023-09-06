#ifndef SENSORS_H
#define SENSORS_H

#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "PMS.h"
#include <Wire.h>


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

// structure for scd4x data
struct scd4xData{
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    bool isDataReady = false;

} ;

// structure that contains every sensor object 
struct Sensors{
    // creation of object of PMS sensor 
    PMS pms3003;
     
    // creation of object of SCD4x sensor 
    SensirionI2CScd4x scd4x;

    Sensors() : pms3003(Serial2) {}
} ; 

struct SensorData{

    PMS::DATA pms_data ;
    scd4xData scd4x_data;

};

// structure that contains every error from all sensors 
struct {
    //scd4x error 
    uint16_t error_scd4x = 0; 
    //PMS error
    uint16_t error_PMS = 0;

} Errors;


void allSensorMeasure(Sensors& sensor, SensorData& data);

void PmsSensorMeasure(PMS& pms3003, SensorData& data);

void allSensorsConfig(Sensors& sensors);








#endif
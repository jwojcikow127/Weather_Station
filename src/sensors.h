#ifndef SENSORS_H
#define SENSORS_H

#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "PMS.h"
#include <Wire.h>

#define NOERROR 0

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
// Light intensity analog pin
#define LIGHT_ANALOG 32
// IR analog pin
#define IR_ANALOG 34
// MQ2 analog pin
#define MQ2_ANALOG 35
// --------------------------------------------------------------------



// structure for scd4x data -------------------------------------------
struct scd4xData{
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    bool isDataReady = false;

} ;
// --------------------------------------------------------------------


// structure that contains every sensor object ------------------------
struct Sensors{
    // object of PMS sensor 
    PMS pms3003;
     
    // object of SCD4x sensor 
    SensirionI2CScd4x scd4x;

    // 
    Sensors() : pms3003(Serial2) {}
} ; 
// --------------------------------------------------------------------


// structure that storages all sensors data ---------------------------
struct SensorData{

    PMS::DATA pms_data ;
    scd4xData scd4x_data;
    uint16_t ir_sensor_data = 0; 
    uint16_t light_intensity_sensor_data = 0;
    uint16_t MQ2_sensor_data = 0;
    

};
//----------------------------------------------------------------------

// structure that contains every error from all sensors ----------------
struct {
    //scd4x error 
    uint16_t error_scd4x = 0; 
    //PMS error
    uint16_t error_PMS = 0;
    // IR error
    uint16_t error_IR = 0;
    // Light Intensity error 
    uint16_t error_Light_Sens = 0;
    // MQ2 error
    uint16_t error_MQ2 = 0;

} Errors;
// ----------------------------------------------------------------------



void allSensorMeasure(Sensors& sensor, SensorData& data);

void PmsSensorMeasure(PMS& pms3003, SensorData& data);

void allSensorsConfig(Sensors& sensors);

uint8_t scd4xSensorMeasure(SensirionI2CScd4x& scd4x, scd4xData& data );

uint8_t IRSensorMeasure(SensorData& data);

uint8_t LightIntensitySensorMeasure(SensorData& data);

uint8_t MQ2SensorMeasure(SensorData& data);





#endif
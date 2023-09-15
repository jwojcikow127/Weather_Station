#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "PMS.h"
#include "sensors.h"

// ********************** fuction for sensors config ********************************
void allSensorsConfig(Sensors& sensors)
{
  Serial2.begin(9600);
  sensors.pms3003.passiveMode();
  delay(50);
  Wire.setPins(i2C_SDA, i2C_SCL);
  Wire.begin();
  sensors.scd4x.begin(Wire);
  delay(50);
  sensors.scd4x.powerDown();
  delay(50);
}
// ***********************************************************************************








// **********************function for all sensor measurement *************************
void allSensorMeasure(Sensors& sensors, SensorData& data)
{
  PmsSensorMeasure(sensors.pms3003, data);
  scd4xSensorMeasure(sensors.scd4x, data.scd4x_data);
  IRSensorMeasure(data);
  LightIntensitySensorMeasure(data);
  MQ2SensorMeasure(data);

}
// ************************************************************************************





// **********************function for PMS sensor measurement *************************
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
  delay(500);
}
// ***********************************************************************************







// ********************function for scdx4 sensor measure *****************************
uint8_t scd4xSensorMeasure(SensirionI2CScd4x& scd4x, scd4xData& data )
{
  Errors.error_scd4x = scd4x.wakeUp() ;
  Errors.error_scd4x = scd4x.measureSingleShot();
  Errors.error_scd4x = scd4x.readMeasurement(data.co2, data.temperature, data.humidity);
  Errors.error_scd4x = scd4x.powerDown();
  delay(500);
  if( Errors.error_scd4x = 0 )
  {
    return NOERROR;
  }
  else 
  {
    return Errors.error_scd4x;
  }
}
// ***********************************************************************************







// ******************function for IR sensor measure **********************************

uint8_t IRSensorMeasure(SensorData& data)
{
  uint16_t value = analogRead(IR_ANALOG);
  delay(20);
  if (value = 0)
  {
    return Errors.error_IR = 1;

  }
  else
  {
    data.ir_sensor_data = map(value, 0, 4095, 760, 1100); // save value in wave lenght units( 760-1100 nm)
    delay(100);
    return NOERROR;
  }
}
// ************************************************************************************








// *****************function for Light Intensity measure*******************************

uint8_t LightIntensitySensorMeasure(SensorData& data)
{
  uint16_t value = analogRead(LIGHT_ANALOG);
  delay(20);
  if (value = 0)
  {
    return Errors.error_Light_Sens = 1;

  }
  else
  {
    data.light_intensity_sensor_data = map(value, 0, 4095, 0, 350); // save value of light intensity in lux's (0-350 lux)
    delay(100);
    return NOERROR;
  }
}
//**************************************************************************************







// ***********************function for gas concentration measure************************

uint8_t MQ2SensorMeasure(SensorData& data)
{
  uint16_t value = analogRead(MQ2_ANALOG);
  delay(20);
    if (value = 0)
  {
    return Errors.error_MQ2 = 1;

  }
  else
  {
    data.MQ2_sensor_data = map(value, 0, 4095, 0, 255); // save value in concentration units (0-255)
    delay(100);
    return NOERROR;
  }
}
// **************************************************************************************

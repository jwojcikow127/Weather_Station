#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"

// public headers
#include <PMS.h>
#include <DFRobot_SCD4X.h>

// private headers
#include "IO.h"


// define constant values ******************************
#define WDT_TIMEOUT  60 // watchdog time to reset
#define uS_TO_S_FACTOR 1000000
#define TIME_BETWEEN_MEASURE 1200  // time that ESP will sleep between measures
#define PMS_TIMER 30000
#define MQ2_TIMER 300000

// *****************************************************




//******************************************************
// main event group bits macro
#define RELAY_OFF (1 << 0)
#define RELAY_ON (1 << 1)
#define SCD4X_OFF (1 << 2)
#define SCD4X_ON (1 << 3)
#define PMS_OFF (1 << 4)
#define PMS_ON (1 << 5)
#define EVENT_6 (1 << 6)
#define EVENT_7 (1 << 7)
//******************************************************


// pinout definition ***********************************
// user button pin
#define BUTTON1 35 
// outside led pin
#define LED1 27
// mosfet relay pin 
#define RELAY 26   
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
#define MQ2_ANALOG 33
//**********************************************************

#define DEBUG_OUT Serial

// structure that storages all sensors data
struct SensorData{
    PMS::DATA pms_data ;
    uint16_t ir_sensor_data ; 
    uint16_t light_intensity_sensor_data;
    uint16_t MQ2_sensor_data ;
    uint16_t co2 ;
    float temperature;
    float humidity; 
    DFRobot_SCD4X::sSensorMeasurement_t SCD4X_data[6];   
} ;

// structure that storages all signal inputs 
 struct Signalss{
    uint8_t measure_request;
    uint8_t reset_request;
    
} ;

// type of led state
typedef enum {
    BLINKING_02s,
    BLINKING_05s,
    BLINKING_1s,
    BLINKING_2s,
    CONST_FLASH,
    OFF
}led_states;

//structure of system flags 
struct Flags{
    led_states led_state;
    uint8_t error;
    uint8_t relay_state; 
    uint8_t during_init;
    uint8_t pms_busy;
    uint8_t CO2_busy;
    uint8_t during_measure;
    uint8_t data_OK; 
    uint8_t data_send;

    
} ;

struct Timerss{
    unsigned long start_time;
    unsigned long interval;
};
 

void SCD4X_Task(void * parameter);
void PMS_Task(void * parameter);
void measure_Task(void * parameter);
void reset(void * parameter);
bool Timerr(Timerss & timer);
void MQ2_Sensor_Measure(void);
void Light_Intensity_Measure(void);
void IR_Sensor_Measure(void);
void Turn_off_Task(void * parameter);
void Error_Task(void * parameter);
void data_OK();



#endif 
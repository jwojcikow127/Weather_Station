#ifndef MAIN_H
#define MAIN_H

// define constant values ******************************
#define WDT_TIMEOUT  60 // watchdog time to reset
#define uS_TO_S_FACTOR 1000000
#define TIME_BETWEEN_MEASURE 1200  // time that ESP will sleep between measures
#define HEARTBEAT_PERIOD 500
#define PMS_TIMER 30000
#define MQ2_TIMER 300000

// *****************************************************


// pinout definition ***********************************
// user button pin
#define BUTTON1 20 //example value, change it !!!!
// outside led pin
#define LED1 23    //example value, change it !!!!\
// mosfet relay pin 
#define RELAY 26   //example value, change it !!!!
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
//**********************************************************

// structure that storages all sensors data
volatile struct SensorData{
    PMS::DATA pms_data ;
    uint16_t ir_sensor_data ; 
    uint16_t light_intensity_sensor_data;
    uint16_t MQ2_sensor_data ;
    uint16_t co2 ;
    float temperature;
    float humidity;    
} SensorData;

// structure that storages all signal inputs 
volatile struct Signals{
    uint8_t measure_request;

} signal;

// type of led state
typedef enum {
    BLINKING_05s,
    BLINKING_1s,
    BLINKING_2s,
    CONST_FLASH,
    OFF
}led_state;

//structure of system flags 
volatile struct Flags{
    led_state led_state;
    uint8_t error;

} flag;



#endif 
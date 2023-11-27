#include <Arduino.h>
#include <Wire.h>

#include <SensirionCore.h>
#include "PMS.h"

#include <SensirionI2CScd4x.h>
#include "bluetooth.h"
#include <esp_task_wdt.h>

#include <cstring>

// define constant values ******************************
#define WDT_TIMEOUT  60 // watchdog time to reset
#define uS_TO_S_FACTOR 1000000
#define TIME_BETWEEN_MEASURE 1200  // time that ESP will sleep between measures
#define HEARTBEAT_PERIOD 500
// *****************************************************


// pinout definition 
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





// definition of system flags
typedef struct 
{
  uint8_t mode_one_take:1;
  uint8_t mode_continous:1; 
  uint8_t MCU_reset:1; 
  uint8_t during_sensor_measure:1;
  uint8_t bluetooth_transfer:1;
  uint8_t button_was_pressed:1; 

} FLAG;
FLAG flags;





// definition of function states
typedef struct 
{
  uint8_t led_state = 0;
  uint8_t relay_state = 0;
  uint8_t pervious_button_state = 0;
  uint8_t actual_button_state = 0;
  uint8_t pms_ready = 0;
  uint8_t analog_ready = 0;
  uint8_t scd4x_ready = 0;


} STATE;
STATE states;






// definition of errors states
typedef struct 
{
  uint8_t pms = 0;
  uint8_t scd4x = 0;
  uint8_t MQ2 = 0;
  uint8_t IR = 0;
  uint8_t light_sensor = 0;


} ERRORS;
ERRORS errors;




//definition od system timers 
struct TIMER
{
  uint32_t laptime;
  uint32_t ticks;
} ;
TIMER heart_timer;




const uint16_t pms_wakeup = 30000;
const uint32_t MQ2_warmup = 300000;
const uint16_t LEDinterval = 500;





// structure that storages all sensors data
struct SensorData{

    PMS::DATA pms_data ;
    uint16_t ir_sensor_data = 0; 
    uint16_t light_intensity_sensor_data = 0;
    uint16_t MQ2_sensor_data = 0;
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    
};
SensorData sensors_data;


SensirionI2CScd4x scd4x; // SCD4x object creation
PMS pms3003(Serial1);
unsigned long previous_time = 0; // variable that stores pervious time 
unsigned long current_time = millis(); // gets current time 
unsigned long measure_start_time = 0;


// function for led state change
void led()
{

    //  led lights up steadily
    if( states.led_state == 1 )
    {
        digitalWrite(LED1,HIGH);

    }

    //  led is blinking with given period
    else if( states.led_state == 2 )
    {
        // change and save current time if interval is longer than desired
        if ( current_time - previous_time >= LEDinterval)
        {
            previous_time = current_time;
        

            //led status change 
            if ( digitalRead(LED1) == HIGH)
            {
                digitalWrite(LED1, LOW);
            } 
            else
            {
                digitalWrite(LED1,HIGH);
            }
        }

        
    }
    // led is switched off
    else if( states.led_state == 3)
    {
        digitalWrite(LED1,LOW);
    }

}


//active led flashing function
void flash_Active_LED()
{
  for (uint8_t i=0; i<10 ;i++)
  {
    digitalWrite(LED1, i & 2 == 0 ? HIGH : LOW);
    delay(100);
  }
}




// function for MQ2 sensor gas concentration 
bool MQ2_Sensor_Measure()
{
  uint16_t value = analogRead(MQ2_ANALOG);
  
  if (value = 0)
  {
    Serial.print("MQ2 sensor data not received");
    errors.MQ2 = 1;
    return false;

  }
  else
  {
    sensors_data.MQ2_sensor_data = map(value, 0, 4095, 0, 255); // save value in concentration units (0-255)
    Serial.print("MQ2 sensor data OK");
    errors.MQ2 = 0;
    return true;
  }
}








// function for Light Intensity measure 
bool Light_Intensity_Sensor_Measure()
{
  uint16_t value = analogRead(LIGHT_ANALOG);
 
  if (value = 0)
  {
    Serial.print("Light sensor data not received");
    errors.light_sensor = 1;
    return false;
  }
  else
  {
    sensors_data.light_intensity_sensor_data = map(value, 0, 4095, 0, 350); // save value of light intensity in lux's (0-350 lux)
    Serial.print("Light sensor data OK");
    errors.light_sensor = 0;
    return true;
  }
}








// function for IRSensor measure 
bool IR_Sensor_Measure()
{
  uint16_t value = analogRead(IR_ANALOG);
  if (value = 0)
  {
    Serial.print("IR sensor data not received");
    errors.IR = 1 ;
    return false;
  }
  else
  {
    sensors_data.ir_sensor_data = map(value, 0, 4095, 760, 1100); // save value in wave lenght units( 760-1100 nm)
    Serial.print("IR sensor data OK");
    errors.IR = 0;
    return true;
  }
}








// function for SCD4X sensor measure 
bool scd4x_Sensor_Measure()
{
  if(scd4x.measureSingleShot() == 0)
  {
    if(scd4x.readMeasurement(sensors_data.co2, sensors_data.temperature, sensors_data.humidity) == 0)
    {
      Serial.print("scd4x sensor readings OK");
      errors.scd4x = 0;
      return true;
    }
    else
    {
      
      errors.scd4x = 1;
      Serial.print("sxd4x error, can't get readings");
      return false;
    }


  }
  else
  {
    Serial.print("sxd4x error, can't get readings");
    errors.scd4x = 1;
    return false; 
  }
  
  
}











// function for PMS sensor read 
bool PMS_Sensor_Read()
{
  // clear buffer 
  while (Serial.available()) { Serial.read(); }

  pms3003.requestRead();
  if (pms3003.readUntil(sensors_data.pms_data))
  {
    
    return true;
  }
  else
  {
    // error flag 
    return false; 
  } 
}


//function for PMS sensor timer callback
void PMS_Timer_Callback()
{
  if(PMS_Sensor_Read())
  {
    Serial.print("PMS data received");
    pms3003.sleep();
    errors.pms = 0;
  }else
  {
    // need to implement error handling 
    Serial.print("PMS data not received");
    pms3003.sleep();
    errors.pms = 1;
  }
}


  
  









// function for reading analog inputs 
bool Read_Analog_Sensors()
{
  if(!MQ2_Sensor_Measure() || !IR_Sensor_Measure() || !Light_Intensity_Sensor_Measure())
  {
    return false; 

  }
  else 
  {
    return true; 
  }
  

}






// function for read digital inputs 
void Read_Digital_inputs()
{
  states.actual_button_state = digitalRead(BUTTON1);
  


}







// function for MCU reset 
void MCU_reset()
{

}







// function for relay state change 
void Relay_Change()
{
  if (states.relay_state == 1)
  {
    digitalWrite(RELAY, HIGH);
  } else if(states.relay_state == 0)
  {
    digitalWrite(RELAY, LOW);
  }
  
}








// function for IO configuration after system boot
void configIO()
{

  //beginning FLAGS and states set 
  flags.MCU_reset = false; 
  flags.bluetooth_transfer = false; 
  flags.button_was_pressed = false; 
  flags.during_sensor_measure = false;
  flags.mode_continous = true;
  flags.mode_one_take = false;
  states.led_state = 3; // led turned off 
  states.relay_state = 0; // relay switched off
  states.pms_ready = 1;
  states.analog_ready = 1;
  states.scd4x_ready = 1;


  //button config 
  pinMode(BUTTON1, INPUT);

  //external led config 
  pinMode(LED1, OUTPUT);

  //mosfet relay config
  pinMode(RELAY, OUTPUT);


  //diode state update
  led();

}







// function for sensor wakeup 
void Prepare_Sensors_For_Measure()
{
  states.relay_state = 1;
  pms3003.wakeUp();
  scd4x.wakeUp();
}






// function for sensor powerdown 
void Sensors_Power_down()
{
  states.relay_state = 0;
}






// digital output change
void Update_Digital_Output()
{
  


}

//flush sensors data 
void Flush_Sensors_Data()
{
  memset(&sensors_data, 0 , sizeof(sensors_data)); // function that fills structure memory area with zeros

}


// heartbeat 
void heartbeat()
{
  uint32_t now; 
  do{now = millis();} while(now - heart_timer.laptime < HEARTBEAT_PERIOD);
  heart_timer.laptime = now;
  heart_timer.ticks++;
  
}

//sleep function 
void Light_Sleep()
{
  esp_sleep_enable_timer_wakeup(uS_TO_S_FACTOR * TIME_BETWEEN_MEASURE);
  esp_light_sleep_start();
}



//kuibahuj




// ************************************************************************************

void setup() {


  delay(500); // delay for system startup
  // watchdog init--------------------------
  esp_task_wdt_init(WDT_TIMEOUT,true);
  esp_task_wdt_add(NULL);
  //----------------------------------------

  // led blinking for startup
  flash_Active_LED();



  Serial.begin(9600); // init of diagnostic uart connection
  Serial1.begin(9600); // init of uart pms connection
  
  // scd4x init
  Wire.setPins(i2C_SDA, i2C_SCL); // I2C interface pin set 
  Wire.begin(); // I2C interface init
    // SCD4x object init
  
  scd4x.begin(Wire);
  pms3003.passiveMode(); 
  pms3003.sleep();

  configIO();
  Relay_Change();
  led();

  heart_timer = {millis(),0};
  

}





// ************************************************************************************

void loop() {

  


  //check current time
  current_time = millis();

  // check if MCU need to reboot
  MCU_reset();

  // update 
  Read_Digital_inputs();

  if(!flags.MCU_reset)
  {
    esp_task_wdt_reset(); // watchdog reset 
  }



  // button was pressed check
  if(states.actual_button_state == 0)
  {
    flags.button_was_pressed = true;
  }

   
  // going into one take mode 
  if(flags.button_was_pressed == true &&  !flags.bluetooth_transfer && !flags.during_sensor_measure )
  {
    // led blinking in the beginning of measure
    flash_Active_LED();

    measure_start_time = current_time;
    flags.during_sensor_measure = true;
    flags.mode_one_take = true;
    Prepare_Sensors_For_Measure(); 

  } 


  // pms timer implementation
  if ((current_time - measure_start_time >= pms_wakeup ) && flags.mode_one_take && flags.during_sensor_measure ) //if it counts to 30 seconds it gets a read from PMS
  {
    PMS_Timer_Callback();

  }

  // analog readings timer implementation
  if((current_time - measure_start_time >= MQ2_warmup) && flags.mode_one_take && flags.during_sensor_measure) // if it counts to 5 mins it gets a read from analog sensors 
  {
    Read_Analog_Sensors();

  }

  // scd4x reading implementation
  if(flags.mode_one_take && flags.during_sensor_measure)
  {
    scd4x_Sensor_Measure();

  }

  if(errors.IR == 1 || errors.light_sensor == 1 || errors.MQ2 == 1 || errors.pms == 1 || errors.scd4x == 1)
  {
    // errors handling...




  }
  else if()
  {

  }








 


  // going into normal continous mode need to implement
  


  //mosfet relay change state
  Relay_Change();

  //external led state change 
  led();

  //system heartbeat 
  heartbeat();

  



}





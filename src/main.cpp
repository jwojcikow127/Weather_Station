
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
#include "main.h"


extern volatile Flags flag;
extern volatile Signalss signalss;
extern SensorData sensorData;
Timerss MQ2_timer;
Timerss PMS_timer{0, 30000};
Timerss Init_timer{0,3000}; 

PMS pms3003(Serial1);
DFRobot_SCD4X CO2;

SemaphoreHandle_t xSemaphore; 


TaskHandle_t LED_Task_handle = NULL;
TaskHandle_t reset_Task_handle = NULL;
TaskHandle_t measure_task_handle = NULL;
TaskHandle_t Input_Task_handle = NULL;

void MQ2_Sensor_Measure(void)
{
  uint16_t reading = analogRead(MQ2_ANALOG);
  if(reading == 0 )
  {
    DEBUG_OUT.print("MQ2 sensor data not received");

  }else{
    sensorData.MQ2_sensor_data = map(reading,0,4095,0,255); // value in ?
    DEBUG_OUT.print("MQ2 sensor data OK");
  }
}

void Light_Intensity_Measure(void)
{
  uint16_t reading = analogRead(LIGHT_ANALOG);
  if(reading == 0 )
  {
    DEBUG_OUT.print("Light sensor data not received");

  }else{
    sensorData.light_intensity_sensor_data = map(reading,0,4095,0,350); // value in lux 
    DEBUG_OUT.print("Light sensor data OK");
  }

}

void IR_Sensor_Measure(void)
{
  uint16_t reading = analogRead(IR_ANALOG);
  if(reading == 0 )
  {
    DEBUG_OUT.print("IR sensor data not received");

  }else{
    sensorData.ir_sensor_data = map(reading,0,4095,0,350); // value in lux 
    DEBUG_OUT.print("IR sensor data OK");
  }
}


void measure_Task(void * parameter)
{
  for(;;)
  {
    if(signalss.measure_request == 1 && flag.error == 0 && flag.during_init == 0)
    {
      vTaskSuspend(Input_Task_handle);
      flag.led_state = BLINKING_1s; 
      flag.relay_state = 1;
      DEBUG_OUT.println("GOING INTO MEASURE SEQUENCE");
      xTaskCreate(&PMS_Task, "PMS_Task",2048, NULL, 8, NULL);
      xTaskCreate(&SCD4X_Task, "SCD4X_Task",2048, NULL, 8, NULL);




    }
  }
}

void PMS_Task(void * parameter)
{
  for(;;)
  {
    if()
    while(Serial1.available()){Serial1.read();}
    DEBUG_OUT.println("Send read request...");
    pms3003.requestRead();
    DEBUG_OUT.println("Reading data...");
    if (pms3003.readUntil(sensorData.pms_data))
    {
      DEBUG_OUT.print("PM 1.0 (ug/m3): ");
      DEBUG_OUT.println(sensorData.pms_data.PM_AE_UG_1_0);

      DEBUG_OUT.print("PM 2.5 (ug/m3): ");
      DEBUG_OUT.println(sensorData.pms_data.PM_AE_UG_2_5);

      DEBUG_OUT.print("PM 10.0 (ug/m3): ");
      DEBUG_OUT.println(sensorData.pms_data.PM_AE_UG_10_0);
    }
  }


}

void SCD4X_Task(void * parameter)
{


}

void reset_Task(void * parameter)
{
  for(;;)
  {
  if(signalss.reset_request == 1)
  {
    DEBUG_OUT.print("ESP RESET \r\n"); 
    //vTaskSuspendAll();
    // reset 
    ESP.restart();
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}

void Init_LED_Flash()
{
  if(flag.during_init == 1)
  {
    Init_timer.start_time = millis();
    while(flag.during_init == 1  )
    {
      digitalWrite(LED1,HIGH);
      delay(100);
      digitalWrite(LED1,LOW);
      delay(100);
      if(Timerr(Init_timer) == true )
      {
        flag.during_init = 0;
      }
    }
    
  }

}

bool Timerr(Timerss &timer)
{
  
  unsigned long current_millis = millis();
  if (current_millis - timer.start_time >= timer.interval)
  {
    timer.start_time = current_millis;
    return true;
  }else
  {
    return false;
  }
  
}


void setup()
{ 

  delay(100); // for system startup
  DEBUG_OUT.print("System INIT \r\n");
  
  //memory clear 
  memset((void*)&flag,0, sizeof(struct Flags));
  memset((void*)&sensorData,0, sizeof(struct SensorData));
  memset((void*)&signalss,0, sizeof(struct Signalss));
  DEBUG_OUT.print("Memory flushed \r\n");

  //init configuration
  Init_Config();
  DEBUG_OUT.print("Config went OK \r\n");
  Init_LED_Flash();
  DEBUG_OUT.print("Creating FreeRTOS tasks \r\n");
  

  //creating a tasks
  xTaskCreate(&LED_Task, "toggleLED", 2048, NULL, 8, &LED_Task_handle);
  xTaskCreate(&Input_Task, "Input_Task",2048, NULL, 6, &Input_Task_handle);
  //xTaskCreate(&Relay_Task, "Relay_Task",2048, NULL, 8, NULL);
  //xTaskCreate(&measure_Task, "measure_Task",2048, NULL ,3, &measure_task_handle);
  xTaskCreate(&reset_Task, "reset_Task",1024, NULL ,8, &reset_Task_handle );
  // bluetooth transmit task 

} 



void loop()
{

}
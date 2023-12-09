
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "BluetoothSerial.h"

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

// init of system structures 
extern volatile Flags flag;
extern volatile Signalss signalss;
extern SensorData sensorData;

extern HardwareSerial PMS_Serial; 

// init of system timers 
Timerss MQ2_timer;
Timerss PMS_timer{0, 30000};
Timerss Init_timer{0,3000}; 

// UART and I2C sensors objects 
extern PMS pms3003;
extern DFRobot_SCD4X CO2;

// main event group init 
EventGroupHandle_t main_event_Group = xEventGroupCreate(); // create 8 event group;


TaskHandle_t LED_Task_handle = NULL;
TaskHandle_t reset_Task_handle = NULL;
TaskHandle_t measure_task_handle = NULL;
TaskHandle_t Input_Task_handle = NULL;


void MQ2_Sensor_Measure(void)
{
  uint16_t reading = analogRead(MQ2_ANALOG);
  if(reading == 0 )
  {
    DEBUG_OUT.println("MQ2 sensor data not received");

  }else{
    sensorData.MQ2_sensor_data = map(reading,0,4095,0,255); // value in ?
    DEBUG_OUT.println("MQ2 sensor data OK");
  }
}

void Light_Intensity_Measure(void)
{
  uint16_t reading = analogRead(LIGHT_ANALOG);
  if(reading == 0 )
  {
    DEBUG_OUT.println("Light sensor data not received");

  }else{
    sensorData.light_intensity_sensor_data = map(reading,0,4095,0,350); // value in lux 
    DEBUG_OUT.println("Light sensor data OK");
  }

}

void IR_Sensor_Measure(void)
{
  uint16_t reading = analogRead(IR_ANALOG);
  if(reading == 0 )
  {
    DEBUG_OUT.println("IR sensor data not received");

  }else{
    sensorData.ir_sensor_data = map(reading,0,4095,0,350); // value in lux 
    DEBUG_OUT.println("IR sensor data OK");
  }
}

void measure_Task(void * parameter) // main task 
{
  for(;;)
  {
    if(signalss.measure_request == 1 && flag.error == 0 && flag.during_init == 0 && flag.during_measure == 0 )
    {
      //vTaskSuspend(Input_Task_handle); // dont't know if i should suspend it, reset might not work
      flag.during_measure = 1;
      flag.led_state = BLINKING_1s; // fast blinking -> into measure mode 
      flag.relay_state = 1;
      DEBUG_OUT.println("MEASURE_TASK -> GOING INTO MEASURE SEQUENCE");
      xEventGroupSetBits(main_event_Group, RELAY_ON); // turning on 5V relay
      DEBUG_OUT.println("MEASURE_TASK -> RELAY ON, pms set into passive mode ");
      vTaskDelay(2000 / portTICK_PERIOD_MS); // wait till voltage stabilize
      pms3003.passiveMode(); // after sensor powerup 
      //vTaskDelay(2000 / portTICK_PERIOD_MS); // wait till voltage stabilize
      //pms3003.sleep();

      xEventGroupSetBits(main_event_Group, PMS_ON ); // setting pms_on event bit, unblock pms measure 
      DEBUG_OUT.println("MEASURE_TASK -> Start of a PMS event ");
      xEventGroupSetBits(main_event_Group, SCD4X_ON); // setting scd4x_on event bit, unblock CO2 measure 
      DEBUG_OUT.println("MEASURE_TASK -> Start of a SCD4X event ");
      // analog read functions
      
      Light_Intensity_Measure();

      IR_Sensor_Measure();

      //vTaskDelay(180000 / portTICK_PERIOD_MS); // wait for MQ2 sesnor to warm up 
      MQ2_Sensor_Measure(); 
      flag.during_measure = 0;
      signalss.measure_request = 0;
      DEBUG_OUT.println("MEASURE_TASK -> exit measure"); 


      // confirmation of measure gone OK 
      

    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void Turn_off_Task(void * parameter)
{

}

void Error_Task(void * parameter)
{


}

void PMS_Task(void * parameter)
{
  for(;;)
  {
    EventBits_t pms_on_bit = xEventGroupWaitBits(main_event_Group, PMS_ON, pdTRUE, pdFALSE, portMAX_DELAY);
    
    if((pms_on_bit & PMS_ON) && flag.relay_state)
    { 
      DEBUG_OUT.println("PMS_TASK -> INTO PMS READ");
      // need to create a timer 
      pms3003.wakeUp();
      flag.pms_busy = 1;
      
      vTaskDelay(32000 / portTICK_PERIOD_MS); // wait 30 sec after fan started running 

      //while(Serial2.available()){Serial2.read();}
      DEBUG_OUT.println("PMS_TASK -> PMS Send read request...");

      vTaskDelay(1000 / portTICK_PERIOD_MS);
      while (PMS_Serial.available()) { PMS_Serial.read(); }
      pms3003.requestRead();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      DEBUG_OUT.println("PMS_TASK -> PMS Reading data...");
      if (pms3003.readUntil(sensorData.pms_data))
      {
        DEBUG_OUT.print("PMS_TASK -> PM 1.0 (ug/m3): ");
        DEBUG_OUT.println(sensorData.pms_data.PM_AE_UG_1_0);

        DEBUG_OUT.print("PMS_TASK -> PM 2.5 (ug/m3): ");
        DEBUG_OUT.println(sensorData.pms_data.PM_AE_UG_2_5);

        DEBUG_OUT.print("PMS_TASK -> PM 10.0 (ug/m3): ");
        DEBUG_OUT.println(sensorData.pms_data.PM_AE_UG_10_0);
      }
      DEBUG_OUT.println("PMS TASK -> exit pms task, clear event bits ");
      pms3003.sleep();
      xEventGroupClearBits(main_event_Group,PMS_ON);

    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    

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
  DEBUG_OUT.print("SETUP -> System INIT \r\n");
  
  //xEventGroupSetBits(main_event_Group, )
  //memory clear 
  memset((void*)&flag,0, sizeof(struct Flags));
  memset((void*)&sensorData,0, sizeof(struct SensorData));
  memset((void*)&signalss,0, sizeof(struct Signalss));
  DEBUG_OUT.print("SETUP -> Memory flushed \r\n");

  //init configuration
  Init_Config();
  DEBUG_OUT.print("SETUP -> Config went OK \r\n");
  Init_LED_Flash();
  DEBUG_OUT.print("SETUP -> Creating FreeRTOS tasks \r\n");
  

  //creating a tasks
  xTaskCreate(&LED_Task, "toggleLED", 2048, NULL, 7, &LED_Task_handle);
  xTaskCreate(&Input_Task, "Input_Task",2048, NULL, 6, &Input_Task_handle);
  xTaskCreate(&Relay_Task, "Relay_Task",2048, NULL, 10, NULL);
  xTaskCreate(&measure_Task, "measure_Task",2048, NULL ,9, &measure_task_handle);
  xTaskCreate(&reset_Task, "reset_Task",1024, NULL ,8, &reset_Task_handle );
  xTaskCreate(&PMS_Task, "PMS_Task",2048, NULL, 10, NULL);
  //xTaskCreate(&SCD4X_Task, "SCD4X_Task",2048, NULL, 8, NULL);
  // bluetooth transmit task 

} 

void loop()
{

}
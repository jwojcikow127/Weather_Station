
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
SemaphoreHandle_t xSemaphore_PMS_ready;
SemaphoreHandle_t xSemaphore_SCD4X_ready;
SemaphoreHandle_t xSemaphore_bluetooth_ready;

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

      data_OK(); // wait till data is ready 
      DEBUG_OUT.println("MEASURE_TASK -> ALL DATA TAKEN ");

      vTaskSuspend(NULL);
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
      flag.pms_busy = 1;
      DEBUG_OUT.println("PMS_TASK -> INTO PMS READ");
      // need to create a timer 
      pms3003.wakeUp();
      
      
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
      flag.pms_busy = 0;
      xEventGroupClearBits(main_event_Group,PMS_ON);
      xSemaphoreGive(xSemaphore_PMS_ready);


    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    

  }

}

void SCD4X_Task(void * parameter)
{
  for(;;)
  {
    EventBits_t scd4x_on_bit = xEventGroupWaitBits(main_event_Group, SCD4X_ON, pdTRUE, pdFALSE, portMAX_DELAY);

    if((scd4x_on_bit & SCD4X_ON) )
    {
      flag.CO2_busy = 1;
      while( !CO2.begin() ){
        DEBUG_OUT.println("Communication with device failed, please check connection");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
      uint32_t averageCO2ppm=0;
      float averageTemp=0.0, averageHumidity=0.0;
      DEBUG_OUT.println("SCD4X_TASK -> Enable Period Measure ");
      CO2.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
      DEBUG_OUT.println("SCD4X_TASK -> Set sleep Mode ");
      CO2.setSleepMode(SCD4X_WAKE_UP);
      for(uint8_t i=0; i<6; i++) 
      {
        CO2.measureSingleShot(SCD4X_MEASURE_SINGLE_SHOT);
        DEBUG_OUT.println("SCD4X_TASK -> Number of iteration: ");
        DEBUG_OUT.print(i);
        while(!CO2.getDataReadyStatus()) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        CO2.readMeasurement(&sensorData.SCD4X_data[i]);
        if(0 != i) 
        {   // Discard the first set of data, because the chip datasheet indicates the first reading obtained after waking up is invalid
          averageCO2ppm += sensorData.SCD4X_data[i].CO2ppm;
          averageTemp += sensorData.SCD4X_data[i].temp;
          averageHumidity += sensorData.SCD4X_data[i].humidity;
        }
      }

      sensorData.co2 = averageCO2ppm / 5;
      DEBUG_OUT.println("SCD4X_TASK -> Carbon dioxide concentration : ");
      DEBUG_OUT.print(sensorData.co2);
      sensorData.humidity = averageHumidity / 5;
      DEBUG_OUT.println("SCD4X_TASK -> Relative humidity :");
      DEBUG_OUT.print(sensorData.co2);
      sensorData.temperature = averageTemp / 5;
      DEBUG_OUT.println("SCD4X_TASK -> Environment temperature : ");
      DEBUG_OUT.print(sensorData.co2);


      DEBUG_OUT.println("SCD4X_TASK -> Results got, SCD4X go to sleep ");
      CO2.setSleepMode(SCD4X_POWER_DOWN);
      xEventGroupClearBits(main_event_Group,SCD4X_OFF);
      flag.CO2_busy = 0;
      xSemaphoreGive(xSemaphore_SCD4X_ready);

    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    

  }

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

void data_OK()
{
  while(true) // we are waiting in loop for sensors task to be completed 
  { 
    if(xSemaphoreTake(xSemaphore_PMS_ready, portMAX_DELAY) ) 
    {
      if(xSemaphoreTake(xSemaphore_SCD4X_ready, portMAX_DELAY))
      {
        xSemaphoreGive(xSemaphore_bluetooth_ready);
        break;
      
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  } // wait till 

  digitalWrite(RELAY,LOW); // switching off mosfet relay 
  flag.led_state = BLINKING_2s;
  DEBUG_OUT.println("MEASURE_TASK -> exit measure"); 
  // confirmation of measure gone OK 


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
  xSemaphore_PMS_ready = xSemaphoreCreateBinary();
  xSemaphore_SCD4X_ready = xSemaphoreCreateBinary();
  xSemaphore_bluetooth_ready = xSemaphoreCreateBinary();

  

  //creating a tasks
  xTaskCreate(&LED_Task, "toggleLED", 2048, NULL, 7, &LED_Task_handle);
  xTaskCreate(&Input_Task, "Input_Task",2048, NULL, 7, &Input_Task_handle);
  xTaskCreate(&Relay_Task, "Relay_Task",2048, NULL, 10, NULL);
  xTaskCreate(&measure_Task, "measure_Task",2048, NULL ,9, &measure_task_handle);
  xTaskCreate(&reset_Task, "reset_Task",1024, NULL ,8, &reset_Task_handle );
  xTaskCreate(&PMS_Task, "PMS_Task",2048, NULL, 10, NULL);
  xTaskCreate(&SCD4X_Task, "SCD4X_Task",2048, NULL, 11, NULL);
  // bluetooth transmit task 
  

} 

void loop()
{

}
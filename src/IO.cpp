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
#include "main.h"
#include "IO.h"
#include "Wire.h"



volatile Button button1;
volatile Flags flag;
volatile Signalss signalss;
volatile SensorData sensorData;
extern Timerss MQ2_timer;
extern Timerss PMS_timer;
extern Timerss Init_timer; 
HardwareSerial PMS_Serial(2); 

PMS pms3003(PMS_Serial);
DFRobot_SCD4X CO2(&Wire, /*i2cAddr = */SCD4X_I2C_ADDR);

unsigned long current_millis;
extern EventGroupHandle_t main_event_Group;

extern TaskHandle_t LED_Task_handle;
extern TaskHandle_t reset_Task_handle;
extern TaskHandle_t measure_task_handle;
extern TaskHandle_t Input_Task_handle;


void LED_Task(void * parameter)
{
    for(;;){ // infinite loop
 
    if(flag.led_state == OFF)
    {
        // Turn the LED off
        digitalWrite(LED1, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);

    }

    if(flag.led_state == CONST_FLASH)
    {
        // Turn the LED on
        digitalWrite(LED1, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    if(flag.led_state == SENDING_DATA)
    {
        // Turn the LED on
        digitalWrite(LED1, HIGH);
        
        // Pause the task for 50ms
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Turn the LED off
        digitalWrite(LED1, LOW);

        // Pause the task again for 50ms
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }


    if(flag.led_state == MEASURE_MODE)
    {
        // Turn the LED on
        digitalWrite(LED1, HIGH);

        // Pause the task for 500ms
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // Turn the LED off
        digitalWrite(LED1, LOW);

        // Pause the task again for 500ms
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // Turn the LED on
        digitalWrite(LED1, HIGH);

        // Pause the task for 500ms
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // Turn the LED off
        digitalWrite(LED1, LOW);

        // Pause the task for 500ms
        vTaskDelay(800 / portTICK_PERIOD_MS);

    }

    if(flag.led_state == BLINKING_1s)
    {
        // Turn the LED on
        digitalWrite(LED1, HIGH);

        // Pause the task for 500ms
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Turn the LED off
        digitalWrite(LED1, LOW);

        // Pause the task again for 500ms
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

    if(flag.led_state == SLEEP_MODE)
    {
          // Turn the LED on
        digitalWrite(LED1, HIGH);
        //DEBUG_OUT.print("Led blink 2s \r\n ");
        // Pause the task for 500ms
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Turn the LED off
        digitalWrite(LED1, LOW);

        // Pause the task again for 500ms
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    
  }
}

void Init_Config(void)
{
    DEBUG_OUT.begin(115200);

    PMS_Serial.begin(9600,SERIAL_8N1, 16,17);

    Wire.setPins(i2C_SDA,i2C_SCL);
    Wire.begin();

    pinMode(LED1, OUTPUT);

    pinMode(BUTTON1, INPUT);

    pinMode(RELAY, OUTPUT);

    CO2.begin();

    // begin flags set 
    flag.error = 0;
    flag.led_state = SLEEP_MODE;
    flag.relay_state = LOW;
    flag.during_init = 1;
    flag.during_measure = 0;

    // begin signals set 
    signalss.measure_request = 0;
    signalss.reset_request = 0;
    button1.last_state = HIGH;
    button1.state = BUTTON_RELEASED; 

    // timers set 
   // Init_timer.start_time = 0;
    //Init_timer.interval = 3000;

    // begin settings of the sensor 

    

}

void Relay_Task(void * parameter)
{
    for(;;)
    {
    //if(flag.relay_state) 
    
        EventBits_t relay_on = xEventGroupWaitBits(main_event_Group, RELAY_ON, pdTRUE , pdFALSE, portMAX_DELAY );
         if(relay_on & RELAY_ON)
    {
        //if(signalss.measure_request == 1)
        
            DEBUG_OUT.println("RELAY_TASK -> Relay set HIGH");
            digitalWrite(RELAY,HIGH); 
        
        xEventGroupClearBits(main_event_Group, RELAY_ON);
    }

    //EventBits_t relay_off = xEventGroupWaitBits(main_event_Group, RELAY_OFF, pdTRUE , pdFALSE, portMAX_DELAY );

   
    /*
    if(relay_off & RELAY_OFF)  
    {
        if(signalss.measure_request == 0 && flag.relay_state == 0)
        {
            DEBUG_OUT.println("Relay set LOW");
            digitalWrite(RELAY,LOW); 
        }
        xEventGroupClearBits(main_event_Group, RELAY_OFF);
    }
    */
    

    }

    
}

void Input_Task(void * parameter)
{
    for(;;){ // infinite loop
        Input_Read();

        switch (button1.state)
        {
        case BUTTON_RELEASED:
            signalss.measure_request = 0;
            if(flag.data_send)
            {
                
            }
            //DEBUG_OUT.write("Button not pressed ");
            break;
        
        case BUTTON_PRESSED:
            if(!flag.during_measure)
            {
                signalss.measure_request = 1;
                DEBUG_OUT.println("INPUT TASK -> Measure request" );
                vTaskResume(measure_task_handle);
            }
            
            break;

        case BUTTON_HELD_2S:

            //DEBUG_OUT.print("Button held 2s ");

            break; 
        case BUTTON_HELD_4S: 

            //DEBUG_OUT.print("Button held 4s ");
            signalss.reset_request = 1;

            break;
        }

    vTaskDelay(50 / portTICK_PERIOD_MS);

    }
}

void Input_Read()
{
    current_millis = millis();
    // If the difference in time between the previous reading is larger than intervalButton
    if(current_millis - button1.previousButtonMillis > button1.intervalButton) 
    {
        // Read the digital value of the button (LOW/HIGH)
        uint8_t buttonState = digitalRead(BUTTON1);    

        // If the button has been pushed AND
        // If the button wasn't pressed before AND
        // IF there was not already a measurement running to determine how long the button has been pressed
        if(buttonState == HIGH && button1.last_state == HIGH)
        {
            button1.state = BUTTON_RELEASED;            
        }

        if (buttonState == LOW && button1.last_state == HIGH && !button1.buttonStateLongPress) 
        {
        button1.buttonLongPressMillis = current_millis;
        button1.last_state = LOW;
        //DEBUG_OUT.println("Button pressed");
        }

        // Calculate how long the button has been pressed
        button1.buttonPressDuration = current_millis - button1.buttonLongPressMillis;

        // If the button is pressed AND
        // If there is no measurement running to determine how long the button is pressed AND
        // If the time the button has been pressed is larger or equal to the time needed for a long press
        if (buttonState == LOW && !button1.buttonStateLongPress && button1.buttonPressDuration >= button1.minButtonLongPressDuration) {
        button1.buttonStateLongPress = true;
        DEBUG_OUT.println("INPUT TASK -> Button  pressed for 4 sec");
        button1.state = BUTTON_HELD_4S;

        }
        
        // If the button is released AND
        // If the button was pressed before
        if (buttonState == HIGH && button1.last_state == LOW) {
        button1.last_state = HIGH;
        button1.buttonStateLongPress = false;
        //DEBUG_OUT.println("Button released");
        button1.state = BUTTON_RELEASED;

        // If there is no measurement running to determine how long the button was pressed AND
        // If the time the button has been pressed is smaller than the minimal time needed for a long press
        // Note: The video shows:
        //       if (!buttonStateLongPress && buttonPressDuration < minButtonLongPressDuration) {
        //       since buttonStateLongPress is set to FALSE on line 75, !buttonStateLongPress is always TRUE
        //       and can be removed.
        if (button1.buttonPressDuration < button1.minButtonLongPressDuration) {
            DEBUG_OUT.println("INPUT TASK -> Button pressed shortly");
            button1.state = BUTTON_PRESSED;
        }
        }
        
        // store the current timestamp in previousButtonMillis
        button1.previousButtonMillis = current_millis;

    }

    
}
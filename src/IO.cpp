#include "IO.h"
#include "main.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"


void toggleLED(void * parameter)
{
    for(;;){ // infinite loop
    led_state led ;


    if(led == OFF)
    {
        // Turn the LED off
        digitalWrite(LED1, LOW);
    }

    if(led == CONST_FLASH)
    {
        // Turn the LED on
        digitalWrite(LED1, HIGH);


    }

    if(led == BLINKING_05s)
    {
        // Turn the LED on
        digitalWrite(LED1, HIGH);

        // Pause the task for 500ms
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // Turn the LED off
        digitalWrite(LED1, LOW);

        // Pause the task again for 500ms
        vTaskDelay(500 / portTICK_PERIOD_MS);

    }

    if(led == BLINKING_1s)
    {
        // Turn the LED on
        digitalWrite(LED1, HIGH);

        // Pause the task for 500ms
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Turn the LED off
        digitalWrite(LED1, LOW);

        // Pause the task again for 500ms
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

    if(led == BLINKING_2s)
    {


    }
    
  }
}
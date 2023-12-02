#include <Arduino.h>
#include <stdio.h>
#include "PMS.h"
#include <SensirionI2CScd4x.h>
#include "main.h"
#include "IO.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"


void setup(void)
{ 
  memset((void*)&flag,0, sizeof(struct Flags));
  memset((void*)&SensorData,0, sizeof(struct SensorData));
  
 

} 

void app_main(void)
{

}
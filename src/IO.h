#ifndef IO_H
#define IO_H

#include <Arduino.h>
#include <Wire.h>

#define BUTTON1 20 //example value, change it !!!!
#define LED1 23    //example value, change it !!!!
#define RELAY 26   //example value, change it !!!!

void btnAndLedConfig();
uint8_t btnClick();

const long LEDinterval = 1000;


/*function for led mode set, 
mode: 
1 - constant light
2 - blinking
3 - led off
*/ 
void led(uint8_t mode, unsigned long currentMillis,unsigned long previousMillis);
void RelayChange(uint8_t state);






#endif 
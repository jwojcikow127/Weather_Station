#ifndef BTN_LED_H
#define BTN_LED_H

#include <Arduino.h>
#include <Wire.h>

#define BUTTON1 20 //example value, change it !!!!
#define LED1 23    //example value, change it !!!!

void btnAndLedConfig()
{
    pinMode(BUTTON1,INPUT);
    pinMode(LED1,OUTPUT);
}

uint8_t btnClick()
{
    uint8_t btn_state = 0;
    btn_state = digitalRead(BUTTON1);
    return btn_state;
}
    



























#endif 
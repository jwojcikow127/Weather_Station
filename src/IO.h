#ifndef IO_H
#define IO_H

enum ButtonState {

    BUTTON_PRESSED,
    BUTTON_RELEASED,
    BUTTON_HELD_2S,
    BUTTON_HELD_4S,
    
};
struct Button 
{
    ButtonState state = BUTTON_RELEASED;
    uint8_t last_state = LOW;    
    unsigned long minButtonLongPressDuration = 4000;    // Time we wait before we see the press as a long press
    unsigned long buttonLongPressMillis;                // Time in ms when we the button was pressed
    bool buttonStateLongPress = false;                  // True if it is a long press
    const int intervalButton = 50;                      // Time between two readings of the button state
    unsigned long previousButtonMillis;                 // Timestamp of the latest reading
    unsigned long buttonPressDuration;                  // Time the button is pressed in ms

};


void Init_Config(void);
void Input_Task(void * parameter);
void LED_Task(void * parameter);
void Input_Read();
void Relay_Task(void * parameter);



#endif 
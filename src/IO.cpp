# include "IO.h"



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

void led(uint8_t mode, unsigned long currentMillis,unsigned long previousMillis)
{

    if( mode == 1 )
    {
        digitalWrite(LED1,HIGH);

    }
    else if( mode == 2 )
    {
        // change and save current time if interval is longer than desired
        if ( currentMillis - previousMillis >= LEDinterval)
        {
            previousMillis = currentMillis;
        

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
    else if( mode == 3)
    {
        digitalWrite(LED1,LOW);
    }

}
    


// ************************fuction for relay state change *******************************

void RelayChange(uint8_t state)
{
  if (state == 1)
  {
    digitalWrite(RELAY, HIGH);
  } else if(state == 0)
  {
    digitalWrite(RELAY, LOW);
  }
  
}

// **************************************************************************************

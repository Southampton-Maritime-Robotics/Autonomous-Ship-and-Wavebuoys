
/*************************************
         THRUST BLOCK ARDUINO
**************************************

// ... Communicates with Motor Controller
// ... Measures shaft RPM
// ... Measures motor current
// ... Measures motor voltage
// ... Measures motor casing temperature
*/

#include "arduino.h"
#include "avr/pgmspace.h"
#include "Wire.h"
#include "Timer.h"
#include "MemoryFree.h"
#include "i2c_helper.h"
#include "adc_helper.h"
#include "SoftwareSerial.h"

/*************************************
              SETTINGS
**************************************/


/*************************************
              DEFINITIONS
**************************************/
//... Pin Definitions
#define LED 9
#define PUMP 6
#define SWITCH A3

// ... Slave Address
#define BILGE_PUMP 12

/*************************************
              VARIABLES
**************************************/

Timer timer;

/*************************************
              SETUP
**************************************/
void setup()
{
  Wire.begin(BILGE_PUMP);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  pinMode(SWITCH, INPUT);
  pinMode(PUMP, OUTPUT);  
  
  Serial.begin(115200);
  
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  
  timer.every(500, check_water);
}

/*************************************
             LOOP
**************************************/
void loop()
{
  timer.update();
}

void check_water()
{
  Serial.println(analogRead(SWITCH));
  if (analogRead(SWITCH) > 75)
  {
    //Serial.println("HIGH");
    digitalWrite(PUMP, HIGH);
  }
  else
  {
    //Serial.println("LOW");
    digitalWrite(PUMP, LOW);
    analogWrite(LED, 100);
    timer.pulse(LED, 50, LOW);
  }
}

// **************************************
//       i2c RECEIVE EVENT HANDLER
// **************************************
void receiveEvent(int _)
{
  boolean heartbeat = true;
  WireReadAnything(heartbeat);
  if (!heartbeat)
  {
  }
  else //blink
  {
    analogWrite(LED, 100);
    timer.pulse(LED, 50, LOW);
  }
}

void requestEvent()
{
  
}

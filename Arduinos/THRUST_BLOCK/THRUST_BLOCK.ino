
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
#define THRUST_BLOCK_CALIBRATION_CONSTANT 1
#define THRUST_BLOCK_CALIBRATION_ZERO 0

#define NUM_AVERAGE 50
int sample_num = 0;


/*************************************
              DEFINITIONS
**************************************/
//... Pin Definitions
#define LED 9

// ... Slave Address
#define THRUST_BLOCK 11

/*************************************
              VARIABLES
**************************************/
float thrust;
Timer timer;
int adc1_samples[NUM_AVERAGE];

/*************************************
              SETUP
**************************************/
void setup()
{
  Wire.begin(THRUST_BLOCK);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  pinMode(SELPIN, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);  
  
  Serial.begin(115200);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  timer.every(2, readSensor);
}

/*************************************
             LOOP
**************************************/
void loop()
{
  timer.update();
}

void readSensor()
{
  adc1_samples[sample_num] = read_adc(1);
  float sum1 = 0;
  for (int i = 0; i < NUM_AVERAGE; i++)
    sum1 += (adc1_samples[i] - THRUST_BLOCK_CALIBRATION_ZERO) * THRUST_BLOCK_CALIBRATION_CONSTANT;
  
  thrust = (float)sum1;
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
  WireWriteAnything(thrust); // 4 bytes
}

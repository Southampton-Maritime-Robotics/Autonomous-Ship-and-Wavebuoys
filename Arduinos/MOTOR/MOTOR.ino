
/*************************************
         MOTOR ARDUINO
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
#define R_TOTAL 9.17
#define VOLTAGE_CONSTANT 1.021

/*************************************
              DEFINITIONS
**************************************/
//... Pin Definitions
#define LED 9
#define MOTOR_CONTROLLER 3
#define RPM_pin 2
#define CHECK_MOTOR_ON 4

SoftwareSerial motorSerial(5, MOTOR_CONTROLLER); //	RX(N/C), TX

// ... Slave Address
#define MOTOR 10

// ... Constants
#define cSTOP 0
#define cDUTY_CYCLE 1
#define cVOLTAGE 2
#define cRPM 3
#define cPOWER 4

#define NUM_AVERAGE 99
#define NUM_RPM_AVERAGE 6

/*************************************
              VARIABLES
**************************************/
volatile int Rev;
unsigned long time;

//int motorTargetMethod = cDUTY_CYCLE;
int motorTargetMethod = cRPM  ;
float motorTarget = 60;
float motorNow = 127;

float motorVoltage;
float motorCurrent;
float motorPower;
float shaftRPM;

int adc1_samples[NUM_AVERAGE];
int adc2_samples[NUM_AVERAGE];
int adc3_samples[NUM_AVERAGE];
float rpm_samples[NUM_RPM_AVERAGE];

int sample_num = 0;
int rpm_sample_num = 0;

Timer timer;


/*************************************
              SETUP
**************************************/
void setup()
{
  // stop motor immediately
  motorSerial.begin(9600);
  motorSerial.write(motorNow);
  
  Wire.begin(MOTOR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  pinMode(SELPIN, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);  
  
  Serial.begin(57600);

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  pinMode(RPM_pin, INPUT);

  timer.every(1,sample);
  timer.every(100, updateMotor);
  timer.every(15, updateRPM);
}

/*************************************
             LOOP
**************************************/
void loop()
{
  //Serial.println(shaftRPM);
  timer.update();
}

// **************************************
//       FUNCTIONS
// **************************************

void updateRPM()
{
  #define COUNTS 30
  #define TIMEOUT 15000
    
  Rev = 0;
  unsigned long elapsedTime = 0;
  time = micros();
  
  attachInterrupt(0,RPM_Count, FALLING);
  while(Rev < COUNTS && micros() - time < TIMEOUT)  ///
  {
  }
  detachInterrupt(0);
  elapsedTime = micros() - time;
  if (elapsedTime > TIMEOUT)
  {
    rpm_samples[rpm_sample_num] = 0;
    Serial.println(elapsedTime);
  }
  else
  {
    //rpm_samples[rpm_sample_num] = ((float)Rev / 1024.0) / ((float)elapsedTime / 1e6) * 60.0 / 6.;
    rpm_samples[rpm_sample_num] = ((float)Rev / 1024.0) / ((float)elapsedTime / 1e6) * 60.0 / 3.5;
  }
  
  float sum1 = 0;
  for (int i = 0; i < NUM_RPM_AVERAGE; i++)
  {
    sum1+=rpm_samples[i];  
  }
  
  shaftRPM = sum1/(float)NUM_RPM_AVERAGE;
  rpm_sample_num= ( rpm_sample_num + 1 ) % NUM_RPM_AVERAGE;
  
}

void updateMotor()
{
  //Serial.println(motorTargetMethod);
  //Serial.println(motorTarget);
  if (digitalRead(CHECK_MOTOR_ON) == LOW)
  {
    motorNow = 127;
    motorSerial.write(motorNow);
  } 
  
  float check; 
  switch (motorTargetMethod)
  {
    case cSTOP:
      motorTarget = 127;
      motorTargetMethod = cDUTY_CYCLE;
      break;
    case cDUTY_CYCLE:
      if (motorNow < motorTarget)
        motorSerial.write(++motorNow);
      else if (motorNow > motorTarget)
        motorSerial.write(--motorNow);
      break;
    case cVOLTAGE:
      check = motorVoltage - (float)motorTarget;
      if (check > 0.05 || check < -0.05)
      {
        if (motorVoltage < motorTarget)
          motorSerial.write(++motorNow);
        else
          motorSerial.write(--motorNow);
        //Serial.println(motorVoltage);
      }
      break; 
    case cRPM:
      check = shaftRPM - (float)motorTarget;
      if (check > 5 || check < -5)
      {
        if (shaftRPM < motorTarget)
          motorSerial.write(++motorNow);
        else
          motorSerial.write(--motorNow);
      }
      break;
    case cPOWER:
      check = motorPower - (float)motorTarget;
      if (check > 0.3 || check < -0.3)
      {
        if (motorPower < motorTarget)
          motorSerial.write(++motorNow);
        else
          motorSerial.write(--motorNow);
      }
      break;
  }
  if (motorNow > 254)
    motorNow = 254;
  if (motorNow < 1)
    motorNow = 1;
}

void RPM_Count()
{
  Rev++;
}

void sample()
{
	adc1_samples[sample_num] = read_adc(1);
	adc2_samples[sample_num] = read_adc(2);
	adc3_samples[sample_num] = read_adc(3);

	long sum1 = 0, sum2 = 0, sum3 = 0;
	for (int i = 0; i < NUM_AVERAGE; i++)
	{
		sum1+=adc1_samples[i] - 2195;
		sum2+=adc2_samples[i];	// black
		sum3+=adc3_samples[i];	// green
	}

	motorVoltage = ((float)sum2 - (float)sum3) * 5 / 4095 * 2.78 / (float)NUM_AVERAGE * VOLTAGE_CONSTANT;
	float gain = 5 + 200/(1/(1/R_TOTAL - 1/40));
	motorCurrent = ( (float)sum1 * (5. / 4095.) / gain / 0.02) / (float)NUM_AVERAGE;
	motorPower = motorCurrent * motorVoltage;
	sample_num = (sample_num + 1) % NUM_AVERAGE;

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
    WireReadAnything(motorTargetMethod);
    WireReadAnything(motorTarget); 
  }
  else //blink
  {
    analogWrite(LED, 100);
    timer.pulse(LED, 50, LOW);
  }
  //Serial.println(motorTargetMethod);
  //Serial.println(motorTarget);
}

void requestEvent()
{
  float data[6] = {(float)motorVoltage, (float)motorTarget, (float)motorCurrent, (float)motorPower, (float)shaftRPM, (float)motorNow};
  WireWriteAnything(data);
}

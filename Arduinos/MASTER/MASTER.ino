/*************************************
            MASTER ARDUINO
**************************************

 ... Reads battery voltage from ADC
 ... Reads reference voltage from ADC
 ... Reads case temperature from ADC
 ... Communicates with ROS (rosserial_arduino)
 ... Communicates with slave nodes (i2c)
*/

#include "avr/pgmspace.h"
#include "ArduinoHardware.h"
#include "ros.h"
#include "std_msgs\Float32.h"
#include "std_msgs\Float64.h"
#include "std_msgs\Int8.h"
#include "ros_helper.h"
#include "Wire.h"
#include "LiquidTWI2.h"
#include "Timer.h"
#include "MemoryFree.h"
#include "i2c_helper.h"
#include "adc_helper.h"

/*************************************
              SETTINGS
**************************************/
#define PUBLISH_PERIOD 100	                          // Time in milliseconds
#define BATTERY_VOLTAGE_CALIBRATION_CONSTANT 3.4894       // Actual voltage divider

/*************************************
              DEFINITIONS
**************************************/
// ... Pin Definitions
#define LED A2

// ... Slave Address Definitions
#define LCD 0
#define MOTOR 10
#define THRUST_BLOCK 11
#define BILGE_PUMP 12
#define RUDDER 13

// ... Constants
#define cSTOP 0
#define cDUTY_CYCLE 1
#define cVOLTAGE 2
#define cRPM 3
#define cPOWER 4

const boolean HEARTBEAT  = true;
const boolean ACTION = false;
/*************************************
              VARIABLES
**************************************/

float reportedTarget;
float motorNow;
double voltRef;
LiquidTWI2 lcd(0);
Timer timer;

/*************************************
              SETUP
**************************************/
void setup()
{
 // Serial.begin(115200);

  Wire.begin();

  lcd.begin(20,4); // Dimensions
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,0);
  lcd.print(F("INIT..."));
  lcd.setCursor(0,1);
  lcd.print(F("LCD OK"));
  
  pinMode(LED, OUTPUT);

  pinMode(SELPIN, OUTPUT); 
  pinMode(DATAOUT, OUTPUT); 
  pinMode(DATAIN, INPUT); 
  pinMode(SPICLOCK, OUTPUT);
  digitalWrite(SELPIN,HIGH); 
  digitalWrite(DATAOUT,LOW); 
  digitalWrite(SPICLOCK,LOW);	
  lcd.setCursor(0,2);
  lcd.print(F("SPI OK"));

  RosSetup();
  lcd.setCursor(0,3);
  lcd.print(F("ROS OK"));

  // ... Timer
  timer.every(PUBLISH_PERIOD,publishAll);
  timer.every(100,readLocalSensors);
  timer.every(100, readForeignSensors);
  timer.after(2500,lcdDelay);
  timer.every(2000,heartbeat);
  lcd.setCursor(10,1);
  lcd.print(F("TIMERS OK"));
 
  delay(1000); // allow everything to settle down
  lcd.clear();
  lcd.setCursor(5,1);
  lcd.print(F("INITIALIZING"));
  lcd.setCursor(7,2);
  lcd.print(F("COMPLETE"));
  lcd.setBacklight(LOW);
  delay(500);
  lcd.setBacklight(HIGH);
}

/*************************************
              LOOP
**************************************/
void loop()
{
  timer.update();
  nh.spinOnce();
  delay(5);
}

/*************************************
          LOCAL FUNCTIONS
 ie. reading ADC, updating LCD
**************************************/

void readLocalSensors()
{
  voltRef = 2.5 / (double)read_adc(3);
  caseTemperature.data = -50. + ( (double)read_adc(2) * voltRef / 0.01 );
  batteryVoltage.data	 = (double)read_adc(1) * voltRef * BATTERY_VOLTAGE_CALIBRATION_CONSTANT;
}

void readForeignSensors()
{
  Wire.requestFrom(THRUST_BLOCK, 4); //get thrust
  WireReadAnything(thrust.data);
  Wire.requestFrom(MOTOR, 24 );
  WireReadAnything((float&)motorVoltage.data);
  WireReadAnything((float&)reportedTarget);
  WireReadAnything((float&)motorCurrent.data);
  WireReadAnything((float&)motorPower.data);
  WireReadAnything((float&)motorRPM.data);
  WireReadAnything((float&)motorNow);
  Wire.requestFrom(RUDDER, 4 );
  WireReadAnything((float&)rudderAngle.data);
}

void lcdDelay()
{
 timer.every(250,updateLCD);
}

void updateLCD()
{
	lcd.clear();
        // ... Memory in the bottom right corner
	lcd.setCursor(14,3);
	lcd.print(2000-freeMemory());
	lcd.print(F("kB"));

	// ... Battery Voltage in bottom left corner
	lcd.setCursor(0,3);
	lcd.print(batteryVoltage.data);
	lcd.print(F("V"));

	// ... Case Temperature in bottom centre
	lcd.setCursor(7,3);
	lcd.print(caseTemperature.data);
	lcd.print(F("C"));

        // ... Thrust in middle-bottom left
        lcd.setCursor(0,2);
        lcd.print(thrust.data);
        lcd.print(F("N"));
        
        // ... Motor Current in middle-bottom centre
        lcd.setCursor(7,2 );
        lcd.print(motorCurrent.data);
        lcd.print(F("A"));
	// ... Motor Voltage in middle-bottom right
        lcd.setCursor(14,2);
        lcd.print(motorVoltage.data);
        lcd.print(F("V"));
	// ... Motor Power in middle-top left
        lcd.setCursor(0,1);
        lcd.print(motorPower.data);
        lcd.print(F("W"));
        // ... Motor RPM in middle-top centre
        lcd.setCursor(7,1);
        lcd.print(motorRPM.data);
        lcd.print(F("RPM"));
        
        // target in top left
        lcd.setCursor(0,0);
        lcd.print(motorTargetMethod.data);
        lcd.print(F("  "));
        lcd.print(reportedTarget);
        
        // target in top left
        lcd.setCursor(0,0);
        lcd.print(rudderAngle.data);
        lcd.print(F("'"));
}

void heartbeat()
{
	digitalWrite(LED, HIGH);
	timer.pulse(LED, 50, LOW);

        Wire.beginTransmission(MOTOR);
          WireWriteAnything(HEARTBEAT);
        Wire.endTransmission();
        
        Wire.beginTransmission(THRUST_BLOCK);
          WireWriteAnything(HEARTBEAT);
        Wire.endTransmission();
        
        Wire.beginTransmission(RUDDER);
          WireWriteAnything(HEARTBEAT);
        Wire.endTransmission();
        
        //Wire.beginTransmission(BILGE_PUMP);
        //  WireWriteAnything(HEARTBEAT);
        //Wire.endTransmission();
}

/*************************************
            SLAVE REQUESTS
 ie. reading ADC, updating LCD
**************************************/

void setRudderAngle(const std_msgs::Float32 &val)
{
  rudderTargetAngle.data = val.data;
 Wire.beginTransmission(RUDDER);
  WireWriteAnything(ACTION);
  WireWriteAnything(rudderTargetAngle.data);
  Wire.endTransmission(); 
}

void setMotorTargetMethod(const std_msgs::Int8 &val)
{
  // Update global variable
  motorTargetMethod.data = val.data;
  // Nothing else required (motor doesn't know about the change until the next motor target command
}

void setMotorTarget(const std_msgs::Float32 &val)
{
  // Update global variable
  motorTarget.data = val.data;
  // Send command to motor
  Wire.beginTransmission(MOTOR);
  WireWriteAnything(ACTION);
  WireWriteAnything((int)motorTargetMethod.data);
  WireWriteAnything(motorTarget.data);
  Wire.endTransmission();
}

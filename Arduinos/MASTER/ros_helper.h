// Subscriber Names
/*    s1 = "setRudderAngle";
      s2 = "setMotorTargetMethod";
      s3 = "setMotorTarget";
*/
// Publisher Names
/*    p1 = "Error";
      p2 = "Warning";
      p3 = "BatteryVoltage";
      p4 = "CaseTemperature";
      p5 = "RudderTargetAngle";
      p6 = "RudderAngle";
      p7 = "MotorTargetMethod";
      p8 = "MotorTarget";
      p9 = "MotorDutyCycle";
      p10 = "MotorRPM";
      p11 = "MotorVoltage";
      p12 = "MotorCurrent";
      p13 = "MotorPower";
      p14 = "Thrust";
*/
//PROGMEM String p14 = "Thrust";

ros::NodeHandle nh;

// ... Published Variables
std_msgs::Int8    error;
std_msgs::Int8    warning;
std_msgs::Float32 batteryVoltage;
std_msgs::Float32 caseTemperature;
std_msgs::Float32 rudderTargetAngle;	// NYI
std_msgs::Float32 rudderAngle;		// NYI
std_msgs::Int8	  motorTargetMethod;
std_msgs::Float32 motorTarget;
std_msgs::Float32 motorDutyCycle;
std_msgs::Float32 motorRPM;
std_msgs::Float32 motorVoltage;		// NYI
std_msgs::Float32 motorCurrent;		// NYI
std_msgs::Float32 motorPower;		// NYI
std_msgs::Float32 thrust;		// NYI

// Subscriber Functions (forward declaration)
void setMotorTarget(const std_msgs::Float32 &val);
void setMotorTargetMethod(const std_msgs::Int8 &val);
void setRudderAngle(const std_msgs::Float32 &val);

// Subscribers (RECEIVE DATA)
ros::Subscriber<std_msgs::Float32>	receiveRudderTarget("s1", &setRudderAngle);
ros::Subscriber<std_msgs::Int8>		receiveMotorTargetMethod("s2", &setMotorTargetMethod);
ros::Subscriber<std_msgs::Float32>	receiveMotorTarget("s3", &setMotorTarget);

// Publishers (TRANSMIT DATA)
ros::Publisher sendError("p1", &error);
ros::Publisher sendWarning("p2", &warning);
ros::Publisher sendBatteryVoltage("p3", &batteryVoltage);
ros::Publisher sendCaseTemperature("p4", &caseTemperature);
ros::Publisher sendRudderTargetAngle("p5", &rudderTargetAngle);
ros::Publisher sendRudderAngle("p6", &rudderAngle);
ros::Publisher sendMotorTargetMethod("p7", &motorTargetMethod);
ros::Publisher sendMotorTarget("p8", &motorTarget);
ros::Publisher sendMotorDutyCycle("p9", &motorDutyCycle);
ros::Publisher sendMotorRPM("p10", &motorRPM);
ros::Publisher sendMotorVoltage("p11", &motorVoltage);
ros::Publisher sendMotorCurrent("p12", &motorCurrent);
ros::Publisher sendMotorPower("p13", &motorPower);
ros::Publisher sendThrust("p14", &thrust);

void RosSetup()
{
	// ... ROS
	 using namespace ros;
	 // Initialize
	 nh.initNode();
	 delay(50);
	 // Advertise publishers
	 nh.advertise(sendError);
	 nh.advertise(sendWarning);
	 nh.advertise(sendBatteryVoltage);
	 nh.advertise(sendCaseTemperature);
	 nh.advertise(sendRudderTargetAngle);
	 nh.advertise(sendRudderAngle);
	 nh.advertise(sendMotorTargetMethod);
	 nh.advertise(sendMotorTarget);
	 nh.advertise(sendMotorDutyCycle);
	 nh.advertise(sendMotorRPM);
	 nh.advertise(sendMotorVoltage);
	 nh.advertise(sendMotorCurrent);
	 nh.advertise(sendMotorPower);
         nh.advertise(sendThrust);
	 // Subscriptions
	 nh.subscribe(receiveRudderTarget);
	 nh.subscribe(receiveMotorTargetMethod);
	 nh.subscribe(receiveMotorTarget);
}

void publishAll()
{
	sendError.publish(&error);                      // NYI
	sendWarning.publish(&warning);                  // NYI
	sendBatteryVoltage.publish(&batteryVoltage);    
	sendCaseTemperature.publish(&caseTemperature);  
	sendRudderTargetAngle.publish(&rudderTargetAngle);  
	sendRudderAngle.publish(&rudderAngle);              
	sendMotorTargetMethod.publish(&motorTargetMethod);  
	sendMotorTarget.publish(&motorTarget);              
	sendMotorDutyCycle.publish(&motorDutyCycle);         
	sendMotorRPM.publish(&motorRPM);
	sendMotorVoltage.publish(&motorVoltage);
	sendMotorCurrent.publish(&motorCurrent);
	sendMotorPower.publish(&motorPower);
        sendThrust.publish(&thrust);
}

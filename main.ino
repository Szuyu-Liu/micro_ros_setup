#include <Servo.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_subscription_t cmdVelSub;


std_msgs__msg__Float32MultiArray msg;		// to publish the totalThetaRight and totalThetaLeft
geometry_msgs__msg__Twist cmdVelMsg;		// to subscribe the movement command from nav2 topic cmd_vel

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

byte pinFeedbackRight = 3;
byte pinFeedbackLeft = 10;
byte pinServoRight = 5;
byte pinServoLeft = 6;

double diameter = 6.65;
double width = 11.0;
double friction = 0.0;
double slide = 0.0;

// receive the command from cml_vel
float cmdVelLinear = 0.0;
float cmdVelAngular = 0.0;
int servoSpeedR = 1500, servoSpeedL = 1500;


class Movement
{
private:
	Servo servoRight;
	Servo servoLeft;

	// robot information
	double diameter = 0; // in cm
	double circumference = 0;

	double turningDiameter = 0; // in cm
	double turningCircumference = 0;

	// Servo Pin
	byte pinFeedbackRight = 0;
	byte pinFeedbackLeft = 0;
	byte pinServoRight = 0;
	byte pinServoLeft = 0;
	unsigned long highRight = 0, highLeft = 0;
	unsigned long lowRight = 0, lowLeft = 0;

	const int circleUnits = 360;
	const float DC_MIN = 2.9;  // From Parallax spec sheet
	const float DC_MAX = 97.1; // From Parallax spec sheet

	double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;
	unsigned long cycleTimeRight, cycleTimeLeft;
	double thetaRight = 0.0, thetaLeft = 0.0;
	double thetaRightPrev = 0.0, thetaLeftPrev = 0.0;
	double turnsRight = 0, turnsLeft = 0;
	double deltaThetaRight = 0, deltaThetaLeft = 0;
	double totalDeltaThetaRight = 0, totalDeltaThetaLeft = 0;
	double totalThetaRight = 0, totalThetaLeft = 0;
	double totalThetaRightPrev = 0, totalThetaLeftPrev = 0;

	void printInfos();
	void updateThetasRight();
	void updateThetasLeft();
	void updateDeltaThetas();
	double reqTheta(double);
	double reqThetaDrehen(double);

public:
	char keyCommand;

	Movement(byte, byte, byte, byte, double, double, double);
	~Movement();

	void runAndPublish(rcl_publisher_t&, std_msgs__msg__Float32MultiArray&);
	void stopAndPublish(rcl_publisher_t&, std_msgs__msg__Float32MultiArray&);
};

Movement car(pinFeedbackRight, pinFeedbackLeft, pinServoRight, pinServoLeft, diameter, friction, width + slide);

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3


void cmdVelCallback(const void* msgin) {
    const geometry_msgs__msg__Twist* cmdVelMsg = (const geometry_msgs__msg__Twist*)msgin;
    cmdVelLinear = cmdVelMsg->linear.x;
    cmdVelAngular = cmdVelMsg->angular.z;
	
    twistToServo(cmdVelLinear, cmdVelAngular);
    
    if (abs(cmdVelLinear) >= 0.01 || abs(cmdVelAngular) >= 0.01) {
      car.runAndPublish(publisher, msg);
    }
    else {
      car.stopAndPublish(publisher, msg);
    } 
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "car_angles"));
	
	RCCHECK(rclc_subscription_init_default(
	  &cmdVelSub,
	  &node,
	  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	  "/cmd_vel"
	));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  rclc_executor_add_subscription(&executor, &cmdVelSub, &cmdVelMsg, &cmdVelCallback, ON_NEW_DATA);

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  
  free(msg.data.data);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;

  // to publish totalThetaRight and totalThetaLeft
  msg.data.capacity = 2;
  msg.data.size = 2;
  msg.data.data = (float *)malloc(msg.data.capacity * sizeof(float));
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  digitalWrite(LED_PIN, state == AGENT_CONNECTED ? 1 : 0);
}


Movement::~Movement() {}

Movement::Movement(byte pinFeedbackRight, byte pinFeedbackLeft, byte pinServoRight, byte pinServoLeft, double diameter, double friction, double width)
{
    this->pinFeedbackRight = pinFeedbackRight;
    this->pinFeedbackLeft = pinFeedbackLeft;
    this->pinServoRight = pinServoRight;
    this->pinServoLeft = pinServoLeft;
    this->diameter = diameter;
    this->turningDiameter = width;

    this->circumference = (diameter + friction) * PI;
    this->turningCircumference = turningDiameter * PI;

    servoRight.detach();
    servoLeft.detach();
};


void Movement::printInfos()
{
    Serial.print("ThetaRight: ");
    Serial.print(thetaRight);
    Serial.print(" , ThetaLeft: ");
    Serial.println(thetaLeft);
    Serial.print("deltaThetaRight: ");
    Serial.print(deltaThetaRight);
    Serial.print(" , deltaThetaLeft: ");
    Serial.println(deltaThetaLeft);
    Serial.print("totalThetaRight: ");
    Serial.print(totalThetaRight);
    Serial.print(" , totalThetaLeft: ");
    Serial.println(totalThetaLeft);
    Serial.print("totalDeltaThetaRight: ");
    Serial.print(totalDeltaThetaRight);
    Serial.print(" , totalDeltaThetaLeft: ");
    Serial.println(totalDeltaThetaLeft);

    Serial.println(" ");
};

void Movement::updateThetasRight()
{
    // Validation of the measurement
    while (1)
    {
        highRight = pulseIn(pinFeedbackRight, HIGH);
        lowRight = pulseIn(pinFeedbackRight, LOW);
        cycleTimeRight = highRight + lowRight;
        if (cycleTimeRight > 1000 && cycleTimeRight < 1200)
        {
            break;
        }
    }

    dutyCycleRight = 100 * (float)highRight / cycleTimeRight;
    thetaRight = (circleUnits - 1) * (dutyCycleRight - DC_MIN) / (DC_MAX - DC_MIN + 1);

    // calculate the counts
    if (thetaRight < 0.25 * circleUnits && thetaRightPrev > 0.75 * circleUnits)
        turnsRight += 1;
    if (thetaRight > 0.75 * circleUnits && thetaRightPrev < 0.25 * circleUnits)
        turnsRight -= 1;

    // calculate the total count of the wheels
    if (turnsRight >= 0)
        totalThetaRight = (turnsRight * circleUnits) + thetaRight;
    else
        totalThetaRight = ((turnsRight + 1) * circleUnits) - (circleUnits - thetaRight);

    // calculate delta angle
    deltaThetaRight = totalThetaRight - totalThetaRightPrev;

    // change the previous theta value
    thetaRightPrev = thetaRight;
    totalThetaRightPrev = totalThetaRight;
};

void Movement::updateThetasLeft()
{
    // Validation of the measurement
    while (1)
    {
        highLeft = pulseIn(pinFeedbackLeft, HIGH);
        lowLeft = pulseIn(pinFeedbackLeft, LOW);
        cycleTimeLeft = highLeft + lowLeft;
        if (cycleTimeLeft > 1000 && cycleTimeLeft < 1200)
        {
            break;
        }
    }
    dutyCycleLeft = 100 * (float)highLeft / cycleTimeLeft;
    thetaLeft = (circleUnits - 1) - circleUnits * (dutyCycleLeft - DC_MIN) / (DC_MAX - DC_MIN + 1);

    // calculate the counts
    if (thetaLeft < 0.25 * circleUnits && thetaLeftPrev > 0.75 * circleUnits)
        turnsLeft += 1;
    if (thetaLeft > 0.75 * circleUnits && thetaLeftPrev < 0.25 * circleUnits)
        turnsLeft -= 1;

    // calculate the total count of the wheels
    if (turnsLeft >= 0)
        totalThetaLeft = (turnsLeft * circleUnits) + thetaLeft;
    else
        totalThetaLeft = ((turnsLeft + 1) * circleUnits) - (circleUnits - thetaLeft);

    // calculate delta angle
    deltaThetaLeft = totalThetaLeft - totalThetaLeftPrev;

    // change the previous theta value
    thetaLeftPrev = thetaLeft;
    totalThetaLeftPrev = totalThetaLeft;
};

void Movement::updateDeltaThetas()
{
    updateThetasRight();
    updateThetasLeft();
};

double Movement::reqTheta(double distance)
{
    return distance / circumference * circleUnits;
};

double Movement::reqThetaDrehen(double angle)
{
    return turningDiameter / diameter * angle;
};

void Movement::runAndPublish(rcl_publisher_t &publisher, std_msgs__msg__Float32MultiArray &msg) {
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    servoRight.writeMicroseconds(servoSpeedR);
    servoLeft.writeMicroseconds(servoSpeedL);
	
	  updateDeltaThetas();
    msg.data.data[0] = totalThetaRight;
    msg.data.data[1] = totalThetaLeft;
    rcl_publish(&publisher, &msg, NULL);
}

void Movement::stopAndPublish(rcl_publisher_t &publisher, std_msgs__msg__Float32MultiArray &msg) {
    servoRight.detach();
    servoLeft.detach();
	
	// reset servo speed
	servoSpeedR = 1500;
	servoSpeedL = 1500;
    
	  updateDeltaThetas();
    msg.data.data[0] = totalThetaRight;
    msg.data.data[1] = totalThetaLeft;
    rcl_publish(&publisher, &msg, NULL);
}

void twistToServo(float v, float omega) {
	float v_r = v + (omega * width / 100.0 / 2.0);
	float v_l = v - (omega * width / 100.0 / 2.0);
	
	if (v_r > 0) {
		servoSpeedR = constrain(int(-351.37 * v_r + 1490.8), 1398, 1473);
	}
	else if (v_r <0) {
		servoSpeedR = constrain(int(-354.0 * v_r + 1520.92), 1540, 1614);
	}
	
	if(v_l > 0) {
		servoSpeedL = constrain(int(348.51 * v_l + 1522.08), 1540, 1614);
	}
	else if (v_l < 0) {
		servoSpeedL = constrain(int(357.84 * v_l + 1492.08), 1398, 1473);
	}
}
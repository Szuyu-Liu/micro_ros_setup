// #include "Movement.h"
#include <Servo.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32_multi_array.h>

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


std_msgs__msg__Float32MultiArray msg;		// to publish the totalThetaRight and totalThetaLeft
std_msgs__msg__Float32MultiArray msgMovement;		// to subscribe the movement command from raspberry pi

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
double width = 10.8;
double friction = 0.3;
double slide = -0.2;

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

	double controlDeviation;
	const int threshold = 3;

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
	int PcontrolRight = 0, PcontrolLeft = 0;

	void Pcontroller(int);
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
	
	void forward(double, int, int, rcl_publisher_t&, std_msgs__msg__Float32MultiArray&);
	void backward(double, int, int, rcl_publisher_t&, std_msgs__msg__Float32MultiArray&);
	void turnRight(double, int, int, rcl_publisher_t&, std_msgs__msg__Float32MultiArray&);
	void turnLeft(double, int, int, rcl_publisher_t&, std_msgs__msg__Float32MultiArray&);
	
};

Movement car(pinFeedbackRight, pinFeedbackLeft, pinServoRight, pinServoLeft, diameter, friction, width + slide);

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

void car_subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * msgMovement = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  switch (msgMovement->data.data[0]) {
    case 1:
      car.forward(msgMovement->data.data[1], 88, 98, publisher, msg);
      break;
    case 2:
      car.backward(msgMovement->data.data[1], 98, 88, publisher, msg);
      break;
    case 3:
      car.turnLeft(msgMovement->data.data[1], 89, 89, publisher, msg);
      break;
    case 4:
      car.turnRight(msgMovement->data.data[1], 97, 97, publisher, msg);
      break;
    default:
      break;
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "car_angles"));
	
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "movement"));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msgMovement, &car_subscription_callback, ON_NEW_DATA));

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
  free(msgMovement.data.data);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;

  // to publish totalThetaRight and totalThetaLeft
  msg.data.capacity = 2;
  msg.data.size = 2;
  msg.data.data = (float *)malloc(msg.data.capacity * sizeof(float));
  
  msgMovement.data.capacity = 2;
  msgMovement.data.size = 2;
  msgMovement.data.data = (float *)malloc(msgMovement.data.capacity * sizeof(float));
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

void Movement::Pcontroller(int dir)
{
    updateDeltaThetas();
    // Control for move forward
    if (dir == 1)
    {
        totalDeltaThetaRight += deltaThetaRight;
        totalDeltaThetaLeft += deltaThetaLeft;
        controlDeviation = totalDeltaThetaRight - totalDeltaThetaLeft;
        // Serial.println(controlDeviation);
        if (abs(controlDeviation) > threshold)
        { // Hysteresis of two points control
            if (controlDeviation > 0)
            { // right wheels run faster than left wheels
                PcontrolRight = 0;
                PcontrolLeft = 1; // clockwise
            }
            else
            {
                PcontrolRight = -1; // counter clockwise
                PcontrolLeft = 0;
            }
        }
        else
        {
            PcontrolRight = 0;
            PcontrolLeft = 0;
        }
    }

    // Control for move backward
    if (dir == -1)
    {
        totalDeltaThetaRight += deltaThetaRight;
        totalDeltaThetaLeft += deltaThetaLeft;
        controlDeviation = totalDeltaThetaRight - totalDeltaThetaLeft;
        // Serial.println(controlDeviation);
        if (abs(controlDeviation) > threshold)
        { // Hysteresis of two points control
            if (controlDeviation > 0)
            { // right wheels run faster than left wheels
                PcontrolRight = 1;
                PcontrolLeft = 0; // clockwise
            }
            else
            {
                PcontrolRight = 0; // counterclockwise
                PcontrolLeft = -1;
            }
        }
        else
        {
            PcontrolRight = 0;
            PcontrolLeft = 0;
        }
    }

    // control for right wheels
    if (dir == 2)
    {
        totalDeltaThetaRight -= deltaThetaRight;
        totalDeltaThetaLeft += deltaThetaLeft;
        controlDeviation = abs(totalDeltaThetaRight) - totalDeltaThetaLeft;
        // Serial.println(controlDeviation);
        if (abs(controlDeviation) > threshold)
        { // Hysteresis of two points control
            if (controlDeviation > 0)
            { // right wheels run faster than left wheels
                PcontrolRight = -1;
                PcontrolLeft = 1;
            }
            else
            {
                PcontrolRight = 1;
                PcontrolLeft = -1;
            }
        }
        else
        {
            PcontrolRight = 0;
            PcontrolLeft = 0;
        }
    }

    // control for left wheels
    if (dir == -2)
    {
        totalDeltaThetaRight += deltaThetaRight;
        totalDeltaThetaLeft -= deltaThetaLeft;
        controlDeviation = totalDeltaThetaRight - abs(totalDeltaThetaLeft);
        // Serial.println(controlDeviation);
        if (abs(controlDeviation) > threshold)
        { // Hysteresis of two points control
            if (controlDeviation > 0)
            { // right wheels run faster than left wheels
                PcontrolRight = 1;
                PcontrolLeft = -1;
            }
            else
            {
                PcontrolRight = -1;
                PcontrolLeft = 1;
            }
        }
        else
        {
            PcontrolRight = 0;
            PcontrolLeft = 0;
        }
    }

    // printInfos();
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
    Serial.print("PcontrolRight: ");
    Serial.print(PcontrolRight);
    Serial.print(" , PcontrolLeft: ");
    Serial.println(PcontrolLeft);
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

void Movement::forward(double distance, int rightServoSpeed, int leftServoSpeed, rcl_publisher_t &publisher, std_msgs__msg__Float32MultiArray &msg)
{
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight + reqTheta(distance);
    double reqThetaLeft = totalThetaLeft + reqTheta(distance);
    
    unsigned long previousMillis = millis();

    while ((totalThetaRight < reqThetaRight) || (totalThetaLeft < reqThetaLeft))
    {
        Pcontroller(1);
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);

        if (millis() - previousMillis >= 200)
        {
          previousMillis = millis();
          msg.data.data[0] = totalThetaRight;
          msg.data.data[1] = totalThetaLeft;
          rcl_publish(&publisher, &msg, NULL);
        }
    }
	
	msg.data.data[0] = totalThetaRight;
    msg.data.data[1] = totalThetaLeft;
    rcl_publish(&publisher, &msg, NULL);

    servoRight.detach();
    servoLeft.detach();
};

void Movement::backward(double distance, int rightServoSpeed, int leftServoSpeed, rcl_publisher_t &publisher, std_msgs__msg__Float32MultiArray &msg)
{
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight - reqTheta(distance);
    double reqThetaLeft = totalThetaLeft - reqTheta(distance);
    unsigned long previousMillis = millis();

    while ((totalThetaRight > reqThetaRight) || (totalThetaLeft > reqThetaLeft))
    {
        Pcontroller(-1);
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);

        if (millis() - previousMillis >= 200)
        {
          previousMillis = millis();
          msg.data.data[0] = totalThetaRight;
          msg.data.data[1] = totalThetaLeft;
          rcl_publish(&publisher, &msg, NULL);
        }
    }
	
	msg.data.data[0] = totalThetaRight;
    msg.data.data[1] = totalThetaLeft;
    rcl_publish(&publisher, &msg, NULL);

    servoRight.detach();
    servoLeft.detach();
};

void Movement::turnRight(double angle, int rightServoSpeed, int leftServoSpeed, rcl_publisher_t &publisher, std_msgs__msg__Float32MultiArray &msg)
{
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight - reqThetaDrehen(angle);
    double reqThetaLeft = totalThetaLeft + reqThetaDrehen(angle);
    unsigned long previousMillis = millis();

    while ((totalThetaRight > reqThetaRight) || (totalThetaLeft < reqThetaLeft))
    {
        Pcontroller(2);
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);

        if (millis() - previousMillis >= 200)
        {
          previousMillis = millis();
          msg.data.data[0] = totalThetaRight;
          msg.data.data[1] = totalThetaLeft;          
          rcl_publish(&publisher, &msg, NULL);
        }
    }
	
	msg.data.data[0] = totalThetaRight;
    msg.data.data[1] = totalThetaLeft;
    rcl_publish(&publisher, &msg, NULL);

    servoRight.detach();
    servoLeft.detach();
};

void Movement::turnLeft(double angle, int rightServoSpeed, int leftServoSpeed, rcl_publisher_t &publisher, std_msgs__msg__Float32MultiArray &msg)
{
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight + reqThetaDrehen(angle);
    double reqThetaLeft = totalThetaLeft - reqThetaDrehen(angle);
    unsigned long previousMillis = millis();

    while ((totalThetaRight < reqThetaRight) || (totalThetaLeft > reqThetaLeft))
    {
        Pcontroller(-2);
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);
        if (millis() - previousMillis >= 200)
        {
          previousMillis = millis();
          msg.data.data[0] = totalThetaRight;
          msg.data.data[1] = totalThetaLeft;
          rcl_publish(&publisher, &msg, NULL);
        }
    }
	
	msg.data.data[0] = totalThetaRight;
    msg.data.data[1] = totalThetaLeft;
    rcl_publish(&publisher, &msg, NULL);

    servoRight.detach();
    servoLeft.detach();
}

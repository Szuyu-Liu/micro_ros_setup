#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <AS5047P.h>
#include <BTS7960.h>
#include <Servo.h>
#include <FastPID.h>

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
#define DEGREES_PER_REVOLUTION 360
#define SAMPLING_FREQUENCY 250
#define PID_SAMPLING_FREQUENCY 25

#define STOP 0
#define FORWARD 1
#define BACKWARD 2

#define MAX_SERVO_POSITION 150
#define MIN_SERVO_POSITION 60
#define CENTER_SERVO_POSITION 111

#define DISTANCE_PER_REVOLUTION 0.02353

const uint8_t L_EN = 3;
const uint8_t R_EN = 4;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 5;
const uint8_t servo_signal = 9;

const float pidMin = -64;
const float pidMax = 64;
int outputSpeed = 0;

const float kp = 0.1;
const float ki = 0.5;
const float kd = 0;

const int output_bits = 8;
const bool output_signed = true;

float prevAngle = 0;
unsigned long prevTime = 0;
unsigned long pidSamplingTime = 0;
int prevMode = STOP;

FastPID pid_motor(kp, ki, kd, PID_SAMPLING_FREQUENCY, output_bits, output_signed);
AS5047P encoder(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);
Servo steering;

rcl_subscription_t subscriber;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

struct Command
{
    int mode;
    int speed;
    float distance;
    float direction;
};

Command command;

void processCommand(const void *msg_data);
void runMotor(int mode, int signal);
void stop();
void steer(float direction);
void passDistance(float currentAngle, float prevAngle);
float handleRollover(float deltaAngle);
float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime);

void setup()
{
    set_microros_transports();
    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "teensy_subscriber", "", &support);
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "car");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &processCommand, ON_NEW_DATA);

    steering.attach(servo_signal);

    while (!encoder.initSPI())
    {
        Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
        delay(5000);
    }

    command.mode = STOP;
    command.speed = 0;
    command.distance = 0;
    command.direction = 0;

    steering.write(CENTER_SERVO_POSITION);
    motorController.Enable();

    prevAngle = encoder.readAngleDegree();
    prevTime = micros();
}

void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    float currentAngle = encoder.readAngleDegree();
    unsigned long currentTime = micros();
    float deltaAngle = handleRollover(currentAngle - prevAngle);
    long deltaTime = currentTime - prevTime;
    float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);

    if (currentTime - pidSamplingTime >= 1000000 / PID_SAMPLING_FREQUENCY)
    {
        outputSpeed = abs(constrain(pid_motor.step(command.speed, currentSpeed), pidMin, pidMax));
        pidSamplingTime = currentTime;
    }

    if (command.mode == STOP)
    {
        stop();
    }
    else
    {
        if (prevMode != command.mode)
        {
            stop();
        }

        runMotor(command.mode, outputSpeed);

        if (command.distance > 0)
        {
            passDistance(currentAngle, prevAngle);
        }

        steer(command.direction);
    }
    prevAngle = currentAngle;
    prevTime = currentTime;
}

void processCommand(const void *msg_data)
{
    std_msgs__msg__String *message = (std_msgs__msg__String *)msg_data;
    String input = String(message->data.data);

    int speedIndex = input.indexOf("speed:");
    int distanceIndex = input.indexOf("distance:");
    int directionIndex = input.indexOf("direction:");

    if (speedIndex != -1)
    {
        int endIndex = input.indexOf(',', speedIndex);
        command.speed = (int)(input.substring(speedIndex + 6, endIndex).toFloat() * 63.7484);
        command.mode = (command.speed > 0) ? FORWARD : (command.speed < 0) ? BACKWARD
                                                                           : STOP;
        command.speed = constrain(command.speed, pidMin, pidMax);
    }

    if (distanceIndex != -1)
    {
        int endIndex = input.length();
        command.distance = input.substring(distanceIndex + 9, endIndex).toFloat();
    }

    if (directionIndex != -1)
    {
        int endIndex = input.length();
        command.direction = constrain(input.substring(directionIndex + 10, endIndex).toFloat(), -1, 1);
    }
}

void stop()
{
    motorController.Stop();
    pid_motor.clear();
    prevMode = command.mode;
}

void runMotor(int mode, int signal)
{
    if (mode == FORWARD)
    {
        motorController.TurnRight(signal);
    }
    else if (mode == BACKWARD)
    {
        motorController.TurnLeft(signal);
    }
}

void steer(float direction)
{
    float steerSignal;
    if (direction < 0)
    {
        steerSignal = CENTER_SERVO_POSITION + (CENTER_SERVO_POSITION - MIN_SERVO_POSITION) * direction;
    }
    else
    {
        steerSignal = CENTER_SERVO_POSITION + (MAX_SERVO_POSITION - CENTER_SERVO_POSITION) * direction;
    }
    steering.write((int)steerSignal);
}

float handleRollover(float deltaAngle)
{
    if (deltaAngle < -DEGREES_PER_REVOLUTION / 2)
    {
        deltaAngle += DEGREES_PER_REVOLUTION;
    }
    else if (deltaAngle > DEGREES_PER_REVOLUTION / 2)
    {
        deltaAngle -= DEGREES_PER_REVOLUTION;
    }
    return deltaAngle;
}

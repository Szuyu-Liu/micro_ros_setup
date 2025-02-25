#include "CarController.h"

#define AS5047P_CHIP_SELECT_PORT 10
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
#define DEGREES_PER_REVOLUTION 360
#define PID_SAMPLING_FREQUENCY 25
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define MAX_SERVO_POSITION 150
#define MIN_SERVO_POSITION 60
#define CENTER_SERVO_POSITION 111
#define DISTANCE_PER_REVOLUTION 0.02353

const uint8_t L_EN = 3, R_EN = 4, L_PWM = 6, R_PWM = 5, servo_signal = 9;
const float pidMin = -64, pidMax = 64;
const float kp = 0.1, ki = 0.5, kd = 0;

CarController::CarController() 
    : pid_motor(kp, ki, kd, PID_SAMPLING_FREQUENCY, 8, true),
      as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED),
      motorController(L_EN, R_EN, L_PWM, R_PWM) {}

void CarController::initialize() {
    analogWriteFrequency(L_PWM, 980);
    analogWriteFrequency(R_PWM, 980);
    steering.attach(servo_signal);
    while (!as5047p.initSPI()) {
        Serial.println(F("Can't connect to the AS5047P sensor!"));
        delay(5000);
    }
    
    outputSpeed = 0;
    prevAngle = 0;
    prevMode = STOP;
    pidSamplingTime = 0;

    command = {STOP, 0, 0, 0};
    steering.write(CENTER_SERVO_POSITION);
    motorController.Enable();

    prevAngle = as5047p.readAngleDegree();
    prevTime = micros();
}

void CarController::run() {
    if (Serial.available() > 0)
    {
      String input = Serial.readStringUntil('\n');
      if (input) // Prevent unintended reset to 0
      {
        parseCommand(input);
      }
    }
    
    float currentAngle = as5047p.readAngleDegree();
    unsigned long currentTime = micros();
    float deltaAngle = handleRollover(currentAngle - prevAngle);
    long deltaTime = currentTime - prevTime;
    float currentSpeed = calculateCurrentSpeed(deltaAngle, deltaTime);

    if (currentTime - pidSamplingTime >= 1000000 / PID_SAMPLING_FREQUENCY) {
      Serial.print("current speed: ");
      Serial.println(currentSpeed);
      Serial.print("output speed: ");
      Serial.println(outputSpeed);
        outputSpeed = abs(constrain(pid_motor.step(command.speed, currentSpeed), pidMin, pidMax));
        pidSamplingTime = currentTime;
    }

    if (command.mode == STOP) {
        stop();
    } else {
        if (prevMode != command.mode) stop();
        runMotor(command.mode, outputSpeed);
        if (command.distance > 0) passDistance(currentAngle, prevAngle);
        steer(command.direction);
    }
    
    prevAngle = currentAngle;
    prevTime = currentTime;
}

float CarController::handleRollover(float deltaAngle) {
    if (deltaAngle < -DEGREES_PER_REVOLUTION / 2) deltaAngle += DEGREES_PER_REVOLUTION;
    else if (deltaAngle > DEGREES_PER_REVOLUTION / 2) deltaAngle -= DEGREES_PER_REVOLUTION;
    return deltaAngle;
}

float CarController::calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime) {
    return (0.025 * ((deltaAngle / deltaTime / 6) * 1000000));
}

void CarController::stop() {
    motorController.Stop();
    pid_motor.clear();
    prevMode = command.mode;
}

void CarController::runMotor(int mode, int signal) {
    if (mode == FORWARD) motorController.TurnRight(signal);
    else if (mode == BACKWARD) motorController.TurnLeft(signal);
}

void CarController::steer(float direction) {
    float steerSignal = CENTER_SERVO_POSITION + (direction < 0 ? 
                     (CENTER_SERVO_POSITION - MIN_SERVO_POSITION) * direction :
                     (MAX_SERVO_POSITION - CENTER_SERVO_POSITION) * direction);
    steering.write((int)steerSignal);
}

void CarController::passDistance(float currentAngle, float prevAngle) {
    if ((prevAngle > 300 && currentAngle < 60) || (prevAngle < 60 && currentAngle > 300)) {
        command.distance -= DISTANCE_PER_REVOLUTION;
    }
    if (command.distance <= 0) {
        command.speed = 0;
        command.mode = STOP;
    }
}

void CarController::parseCommand(String input) {
    int speedIndex = input.indexOf("speed:");
    int distanceIndex = input.indexOf("distance:");
    int directionIndex = input.indexOf("direction:");
    
    if (speedIndex != -1) {
        int endIndex = input.indexOf(',', speedIndex);
        command.speed = (int)(input.substring(speedIndex + 6, endIndex).toFloat() * 63.7484);
        command.mode = command.speed > 0 ? FORWARD : (command.speed < 0 ? BACKWARD : STOP);
        command.speed = constrain(command.speed, pidMin, pidMax);
    }
    if (distanceIndex != -1) {
        int endIndex = input.length();
        command.distance = input.substring(distanceIndex + 9, endIndex).toFloat();
    }
    if (directionIndex != -1) {
        int endIndex = input.length();
        command.direction = constrain(input.substring(directionIndex + 10, endIndex).toFloat(), -1, 1);
    }
}

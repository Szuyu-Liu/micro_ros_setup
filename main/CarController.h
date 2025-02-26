#ifndef CARCONTROLLER_H
#define CARCONTROLLER_H

#include <Arduino.h>
#include <AS5047P.h>
#include <BTS7960.h>
#include <Servo.h>
#include <FastPID.h>

class CarController {
public:
    CarController();
    void initialize();
    void run(String input);
    
private:
    struct Command {
        int mode;
        int speed;
        float distance;
        float direction;
    };
    
    Command command;
    FastPID pid_motor;
    AS5047P as5047p;
    BTS7960 motorController;
    Servo steering;

    float prevAngle;
    unsigned long prevTime;
    unsigned long pidSamplingTime;
    int prevMode;
    int outputSpeed;

    float handleRollover(float deltaAngle);
    float calculateCurrentSpeed(float deltaAngle, unsigned long deltaTime);
    void runMotor(int mode, int signal);
    void stop();
    void steer(float direction);
    void passDistance(float currentAngle, float prevAngle);
    void parseCommand(String input);
};

#endif
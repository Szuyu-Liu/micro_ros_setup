#include "Movement.h"
#include <Servo.h>

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
        Serial.println(controlDeviation);
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
        Serial.println(controlDeviation);
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
        Serial.println(controlDeviation);
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
        Serial.println(controlDeviation);
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

    //printInfos();
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
    return correctedDiameter / diameter * angle;
};

void Movement::forward(double distance, int rightServoSpeed, int leftServoSpeed)
{
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight + reqTheta(distance);
    double reqThetaLeft = totalThetaLeft + reqTheta(distance);

    while ((totalThetaRight < reqThetaRight) || (totalThetaLeft < reqThetaLeft))
    {
        Pcontroller(1);
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);
    }

    servoRight.detach();
    servoLeft.detach();
};

void Movement::backward(double distance, int rightServoSpeed, int leftServoSpeed)
{
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight - reqTheta(distance);
    double reqThetaLeft = totalThetaLeft - reqTheta(distance);

    while ((totalThetaRight > reqThetaRight) || (totalThetaLeft > reqThetaLeft))
    {
        Pcontroller(-1);
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);
    }

    servoRight.detach();
    servoLeft.detach();
};

void Movement::turnRight(double angle, int rightServoSpeed, int leftServoSpeed)
{
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight - reqThetaDrehen(angle);
    double reqThetaLeft = totalThetaLeft + reqThetaDrehen(angle);

    while ((totalThetaRight > reqThetaRight) || (totalThetaLeft < reqThetaLeft))
    {
        Pcontroller(2);
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);
    }

    servoRight.detach();
    servoLeft.detach();
};

void Movement::turnLeft(double angle, int rightServoSpeed, int leftServoSpeed)
{
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight + reqThetaDrehen(angle);
    double reqThetaLeft = totalThetaLeft - reqThetaDrehen(angle);

    while ((totalThetaRight < reqThetaRight) || (totalThetaLeft > reqThetaLeft))
    {
        Pcontroller(-2);
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);
    }

    servoRight.detach();
    servoLeft.detach();
}

void Movement::forwardWithoutControl(double distance, int rightServoSpeed, int leftServoSpeed)
{
    PcontrolRight = 0;
    PcontrolLeft = 0;
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight + reqTheta(distance);
    double reqThetaLeft = totalThetaLeft + reqTheta(distance);

    while ((totalThetaRight < reqThetaRight) || (totalThetaLeft < reqThetaLeft))
    {
        updateDeltaThetas();
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);
    }

    servoRight.detach();
    servoLeft.detach();
};

void Movement::backwardWithoutControl(double distance, int rightServoSpeed, int leftServoSpeed)
{
    PcontrolRight = 0;
    PcontrolLeft = 0;
    servoRight.attach(pinServoRight);
    servoLeft.attach(pinServoLeft);
    updateDeltaThetas();

    servoRight.write(rightServoSpeed);
    servoLeft.write(leftServoSpeed);

    double reqThetaRight = totalThetaRight - reqTheta(distance);
    double reqThetaLeft = totalThetaLeft - reqTheta(distance);

    while ((totalThetaRight > reqThetaRight) || (totalThetaLeft > reqThetaLeft))
    {
        updateDeltaThetas();
        servoRight.write(rightServoSpeed + PcontrolRight);
        servoLeft.write(leftServoSpeed + PcontrolLeft);
    }

    servoRight.detach();
    servoLeft.detach();
};

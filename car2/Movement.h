/*
  Erstellt im Rahmen der Robotik Vorlesung von Jasper Wolff.
*/

#define Movement_h

#include "Arduino.h"
#include <Servo.h>

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
	
	void forward(double, int, int);
	void backward(double, int, int);
	void turnRight(double, int, int);
	void turnLeft(double, int, int);

	void forwardWithoutControl(double, int, int);
	void backwardWithoutControl(double, int, int);
};

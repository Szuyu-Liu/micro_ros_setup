#include "Movement.h"

byte pinFeedbackRight = 3;
byte pinFeedbackLeft = 10;
byte pinServoRight = 5;
byte pinServoLeft = 6;

double diameter = 6.65;
double width = 10.8;
double friction = 0.3;
double slide = -0.2;

Movement car(pinFeedbackRight, pinFeedbackLeft, pinServoRight, pinServoLeft, diameter, friction, width + slide);

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("-Odomotry and Control test-\n");
  Serial.println("[1] 1m forward/backward with control");
  Serial.println("[2] 1m forward/backward without control");
  Serial.println("[3] run a 1m square once");
  Serial.println("[4] run a 1m square twice");
}

void loop()
{
  if (Serial.available() > 0)
  {

    car.keyCommand = Serial.read();

    switch (car.keyCommand)
    {
    case '4':
      for (int i = 0; i < 8; i++)
      {
        car.forward(40, 88, 98);
        delay(200);
        car.turnLeft(90, 89, 89);
        delay(200);
      }
      break;

    case '3':
      // car.forward(97, 88, 98);
      // delay(200);
      // car.turnRight(180, 97, 97);
      // delay(200);
      // car.forward(20, 88, 98);
      // delay(200);
      car.turnLeft(180, 89, 89);
      delay(200);
      break;

    case '1':
      car.forward(100, 88, 98);
      delay(200);
      // car.backward(100, 98, 88);
      // delay(200);
      break;

    case '2':
      car.forwardWithoutControl(100, 88, 98);
      delay(200);
      car.backwardWithoutControl(100, 98, 88);
      delay(200);
      break;

    default:
      break;
    }
  }
}

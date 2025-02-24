#include "CarController.h"

CarController car;

void setup() {
    Serial.begin(115200);
    car.initialize();
}

void loop() {
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    if (input) // Prevent unintended reset to 0
    {
      car.run(input);
    }
  }
}

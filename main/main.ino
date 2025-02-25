#include "CarController.h"

CarController car;

void setup() {
  car.initialize();
}

void loop() {
  car.run();
}
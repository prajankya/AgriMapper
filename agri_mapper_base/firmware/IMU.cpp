#include "IMU.h"
#include <Arduino.h>

void IMU::init() {
  mag_sensor = Adafruit_HMC5883_Unified(12345);

  if (!mag_sensor.begin()) {
    while (1) {
      Serial.println("Ooops, no IMU detected ... Check your wiring!");
    }
  }
}

void IMU::loop() {
  sensors_event_t event;

  mag_sensor.getEvent(&event);

  char x[10];
  dtostrf(event.magnetic.x, 6, 2, x);

  char y[10];
  dtostrf(event.magnetic.y, 6, 2, y);

  char z[10];
  dtostrf(event.magnetic.z, 6, 2, z);

  String s = String(x) + "," + String(y) + "," + String(z);
  s.toCharArray(msg, 50);
}

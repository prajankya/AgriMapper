#include "IMU.h"
#include <Arduino.h>

void IMU::init() {
#ifdef USE_MAGNETOMETER
  mag_sensor = Adafruit_HMC5883_Unified(12345);
  mag_sensor.begin();
#endif

#ifdef USE_ACCELEROMETER
  acc_sensor = Adafruit_ADXL345_Unified(23456);
  acc_sensor.begin();
#endif
}

void IMU::loop() {
#ifdef USE_MAGNETOMETER
  sensors_event_t mag_event;

  mag_sensor.getEvent(&mag_event);

  char mag_x[10];
  dtostrf(mag_event.magnetic.x, 6, 2, mag_x);

  char mag_y[10];
  dtostrf(mag_event.magnetic.y, 6, 2, mag_y);

  char mag_z[10];
  dtostrf(mag_event.magnetic.z, 6, 2, mag_z);

  String mag_s = String(mag_x) + "," + String(mag_y) + "," + String(mag_z);
  mag_s.toCharArray(mag_msg, 50);
#endif

#ifdef USE_ACCELEROMETER
  sensors_event_t acc_event;

  acc_sensor.getEvent(&acc_event);

  char acc_x[10];
  dtostrf(acc_event.acceleration.x, 6, 2, acc_x);

  char acc_y[10];
  dtostrf(acc_event.acceleration.y, 6, 2, acc_y);

  char acc_z[10];
  dtostrf(acc_event.acceleration.z, 6, 2, acc_z);

  String acc_s = String(acc_x) + "," + String(acc_y) + "," + String(acc_z);
  acc_s.toCharArray(acc_msg, 50);
#endif
}

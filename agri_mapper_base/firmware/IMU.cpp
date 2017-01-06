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

#ifdef USE_BAROMETER
  baro_sensor = Adafruit_BMP085_Unified(10085);
  baro_sensor.begin();
#endif

#ifdef USE_GYROSCOPE

  while (!gyro_sensor.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50)) {
    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyro_sensor.calibrate();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  gyro_sensor.setThreshold(3);

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

#ifdef USE_BAROMETER
  sensors_event_t baro_event;

  baro_sensor.getEvent(&baro_event);

  char pressure[10];
  dtostrf(baro_event.pressure, 6, 2, pressure);

  float temp;
  baro_sensor.getTemperature(&temp);

  char temperature[10];
  dtostrf(temp, 6, 2, temperature);

  String baro_s = String(pressure) + "," + String(temperature);
  baro_s.toCharArray(baro_msg, 50);
#endif // ifdef USE_BAROMETER

#ifdef USE_GYROSCOPE
// Read normalized values
  Vector raw = gyro_sensor.readRaw();

  char gyro_raw_x[10];
  dtostrf(raw.XAxis, 6, 2, gyro_raw_x);
  char gyro_raw_y[10];
  dtostrf(raw.YAxis, 6, 2, gyro_raw_y);
  char gyro_raw_z[10];
  dtostrf(raw.ZAxis, 6, 2, gyro_raw_z);


  // Read normalized values in deg/sec
  Vector norm = gyro_sensor.readNormalize();
  char gyro_norm_x[10];
  dtostrf(norm.XAxis, 6, 2, gyro_norm_x);
  char gyro_norm_y[10];
  dtostrf(norm.YAxis, 6, 2, gyro_norm_y);
  char gyro_norm_z[10];
  dtostrf(norm.ZAxis, 6, 2, gyro_norm_z);

  String gyro_s = String(gyro_raw_x) + "," + String(gyro_raw_y) + "," + String(gyro_raw_z) + "," +
    String(gyro_norm_x) + "," + String(gyro_norm_y) + "," + String(gyro_norm_z);
  gyro_s.toCharArray(gyro_msg, 100);
#endif // ifdef USE_GYROSCOPE
}

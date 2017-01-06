#define USE_MAGNETOMETER  1
#define USE_ACCELEROMETER 1
#define USE_BAROMETER     1
#define USE_GYROSCOPE     1

#ifndef _IMU_H_
  #define _IMU_H_

  #include <Arduino.h>
  #include "Adafruit_Sensor.h"

  #ifdef USE_MAGNETOMETER
    #include "Adafruit_HMC5883_U.h"
  #endif

  #ifdef USE_ACCELEROMETER
    #include "Adafruit_ADXL345_U.h"
  #endif

  #ifdef USE_BAROMETER
    #include "Adafruit_BMP085_U.h"
  #endif

  #ifdef USE_GYROSCOPE
    #include "L3G4200D.h"
  #endif

class IMU {
  private:
  #ifdef USE_MAGNETOMETER
    Adafruit_HMC5883_Unified mag_sensor;
  #endif

  #ifdef USE_ACCELEROMETER
    Adafruit_ADXL345_Unified acc_sensor;
  #endif

  #ifdef USE_BAROMETER
    Adafruit_BMP085_Unified baro_sensor;
  #endif

  #ifdef USE_GYROSCOPE
    L3G4200D gyro_sensor;
  #endif

  public:
    void init();
    void loop();
    String toString();

  #ifdef USE_MAGNETOMETER
    char mag_msg[50];
  #endif

  #ifdef USE_ACCELEROMETER
    char acc_msg[50];
  #endif

  #ifdef USE_BAROMETER
    char baro_msg[50];
  #endif

  #ifdef USE_GYROSCOPE
    char gyro_msg[100];
  #endif
};

#endif // ifndef _IMU_H_

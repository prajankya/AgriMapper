#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883_U.h"
#include "Adafruit_ADXL345_U.h"
#include "L3G4200D.h"
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>

class IMU {
  private:
    Adafruit_HMC5883_Unified mag_sensor;

  public:
    void init();
    void loop();

    char msg[50];
};

#endif // ifndef _IMU_H_

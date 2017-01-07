#include <Arduino.h>

#define USE_ROS 1
#define USE_IMU 1

#define USE_MAGNETOMETER  1
#define USE_ACCELEROMETER 1
#define USE_BAROMETER     1
#define USE_GYROSCOPE     1

#ifdef USE_ROS
  #include <ros.h>
  #include <ros/time.h>
  #include <std_msgs/String.h>
#endif

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

#include "odom.h"


unsigned long previousMillis_blink = 0;
long interval_blink = 100;
bool state_blink = LOW;
void blink();


#ifdef USE_ROS
ros::NodeHandle nh;

std_msgs::String odom_msg;
ros::Publisher odom_pub("odom_pub", &odom_msg);

  #ifdef USE_IMU
std_msgs::String imu_msg;
ros::Publisher imu_pub("imu_msg", &imu_msg);
  #endif

#endif

Odom odom;
String imu_String();

#ifdef USE_IMU
  #ifdef USE_MAGNETOMETER
Adafruit_HMC5883_Unified mag_sensor;
char mag_msg[50];
  #endif

  #ifdef USE_ACCELEROMETER
Adafruit_ADXL345_Unified acc_sensor;
char acc_msg[50];
  #endif

  #ifdef USE_BAROMETER
Adafruit_BMP085_Unified baro_sensor;
char baro_msg[50];
  #endif

  #ifdef USE_GYROSCOPE
L3G4200D gyro_sensor;
char gyro_msg[100];
  #endif

#endif // ifdef USE_IMU

void setup() {
  pinMode(12, OUTPUT);

#ifndef USE_ROS
  Serial.begin(9600);
  Serial.println("Starting Node..");
#endif

#ifdef USE_ROS
  nh.initNode();
  nh.advertise(odom_pub);

  #ifdef USE_IMU
  nh.advertise(imu_pub);
  #endif
#endif // ifdef USE_ROS

  odom.init(7, 8, 4, 6);

#ifdef USE_IMU
  #ifdef USE_MAGNETOMETER
  mag_sensor = Adafruit_HMC5883_Unified(12345);

  while (!mag_sensor.begin()) {
    blink();
  }

  #endif

  #ifdef USE_ACCELEROMETER
  acc_sensor = Adafruit_ADXL345_Unified(23456);

  while (!acc_sensor.begin()) {
    blink();
  }

  #endif

  #ifdef USE_BAROMETER
  baro_sensor = Adafruit_BMP085_Unified(10085);

  while (!baro_sensor.begin()) {
    blink();
  }
  #endif

  #ifdef USE_GYROSCOPE

  while (!gyro_sensor.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50)) {
    blink();
  }

// Calibrate gyroscope. The calibration must be at rest.
// If you don't want calibrate, comment this line.
//gyro_sensor.calibrate();

// Set threshold sensivty. Default 3.
// If you don't want use threshold, comment this line or set 0.
//gyro_sensor.setThreshold(3);

  #endif // ifdef USE_GYROSCOPE
#endif // ifdef USE_IMU
}

unsigned long previousMillis = 0;

const long interval = 1;

void loop() {
  odom.loop();

#ifdef USE_IMU
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
#endif // ifdef USE_IMU

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

#ifdef USE_ROS
    odom_msg.data = odom.msg;
    odom_pub.publish(&odom_msg);

  #ifdef USE_IMU
    String st = imu_String();
    nh.loginfo(st.c_str());
    imu_msg.data = st.c_str();
    imu_pub.publish(&imu_msg);
  #endif

#endif

#ifndef USE_ROS
    Serial.print("odom msg :");
    Serial.println(odom.msg);

  #ifdef USE_IMU
    Serial.print("\tIMU :");
    Serial.println(imu_String());
  #endif

#endif
  }

#ifdef USE_ROS
  nh.spinOnce();
#endif
}

String imu_String() {
  String s = "";

#ifdef USE_IMU

  #ifdef USE_MAGNETOMETER
  s = s + String(mag_msg);
  #endif

  #ifdef USE_ACCELEROMETER
  s = s + ":" + String(acc_msg);
  #endif

  #ifdef USE_GYROSCOPE
  s = s + ":" + String(gyro_msg);
  #endif

  #ifdef USE_BAROMETER
  s = s + ":" + String(baro_msg);
  #endif
#endif

  return s;
}

void blink() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_blink >= interval_blink) {
    previousMillis_blink = currentMillis;

    state_blink = !state_blink;

    digitalWrite(12, state_blink);
  }
}

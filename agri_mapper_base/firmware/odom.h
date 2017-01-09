#ifndef __ODOM_H__
#define __ODOM_H__

#include <Arduino.h>
#include "Encoder.h"

class Odom {
  private:
    Encoder left;
    Encoder right;
    long oldL;
    long oldR;

  public:
    void init(int e1A, int e1B, int e2A, int e2B);
    void loop();
    char msg[50];
};
#endif

#include "encoder.h"
#include <Arduino.h>

void Encoder::init(int pinA_, int pinB_){
  pinA = pinA_;
  pinB = pinB_;
  count = 0;
  pinALast = LOW;
  n = LOW;

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
}

long Encoder::getPos(){
  return count;
}

void Encoder::loop(){
  n = digitalRead(pinA);
  if ((pinALast == LOW) && (n == HIGH)) {
    if (digitalRead(pinB) == LOW) {
      count--;
    } else {
      count++;
    }
  }
  pinALast = n;
}

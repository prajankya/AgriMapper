#include "Encoder.h"
#include <Arduino.h>

void Encoder::init_set(int pinA_, int pinB_) {
  count = 0;
  pinALast = LOW;
  n = LOW;
  increment_val = 1;

  pinA = pinA_;
  pinB = pinB_;

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
}

void Encoder::init(int pinA_, int pinB_) {
  init_set(pinA_, pinB_);

  th_A = 0;
  th_B = 0;
  is_A_Analog = false;
  is_B_Analog = false;
}

void Encoder::init(int pinA_, int pinB_, int th_A_) {
  init_set(pinA_, pinB_);

  th_A = th_A_;
  th_B = 0;
  is_A_Analog = true;
  is_B_Analog = false;
}

void Encoder::init(int pinA_, int pinB_, int th_A_, int th_B_) {
  init_set(pinA_, pinB_);

  th_A = th_A_;
  th_B = th_B_;
  is_A_Analog = true;
  is_B_Analog = true;
}

long Encoder::getPos() {
  return count;
}

void Encoder::loop() {
  n = readPin(pinA);

  if ((pinALast == LOW) && (n == HIGH)) {
    if (readPin(pinB) == LOW) {
      count -= increment_val;
    } else {
      count += increment_val;
    }
  }

  pinALast = n;
}

void Encoder::negate(bool set_) {
  increment_val = (set_) ? -1 : 1;
}

bool Encoder::readPin(int pin) {
  if (pin == pinA) {
    if (is_A_Analog) {
      return analogRead(pin) > th_A;
    } else {
      return digitalRead(pin);
    }
  } else if (pin == pinB) {
    if (is_B_Analog) {
      return analogRead(pin) > th_B;
    } else {
      return digitalRead(pin);
    }
  }
}

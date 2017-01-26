#include "Odom.h"
#include <Arduino.h>

Odom::Odom(int e1A, int e1B, int e2A, int e2B):left(e1A, e1B), right(e2A, e2B) {

  /*
  Encoder En_L(e1A,e1B);
  left = &En_L;

  Encoder En_R(e2A,e2B);
  right = &En_R;

  *//*
  oldL = 0;
  oldR = 0;

  String s = String(0) + "," + String(0);
  s.toCharArray(msg, 50);*/
}

void Odom::loop() {
  /*
  long nL = left->read();
  long nR = right->read();

  if (nL != oldL || nR != oldR) {
    char l[10];
    dtostrf(nL, 6, 2, l);

    char r[10];
    dtostrf(nR, 6, 2, r);

    String s = String(l) + "," + String(r);
    s.toCharArray(msg, 50);

    oldL = nL;
    oldR = nR;
  }
  */
}

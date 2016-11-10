#include "odom.h"
#include <Arduino.h>

void Odom::init(int e1A, int e1B, int e2A, int e2B){
        left.init(e1A, e1B);
        right.init(e2A, e2B);

        oldL = 0;
        oldR = 0;

        String s = String(0) + "," + String(0);
        s.toCharArray(msg, 50);
}

void Odom::loop(){
        left.loop();
        right.loop();
        long nL = left.getPos();
        long nR = right.getPos();

        if(nL != oldL || nR != oldR) {
                char l[10];
                dtostrf(nL, 6, 2, l);

                char r[10];
                dtostrf(nR, 6, 2, r);

                String s = String(l) + "," + String(r);
                s.toCharArray(msg, 50);

                oldL = nL;
                oldR = nR;
        }
}

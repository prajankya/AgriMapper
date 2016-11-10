#ifndef __ENCODER_H__
#define __ENCODER_H__

class Encoder {
  private:
    int pinA;
    int pinB;
    int pinALast;
    int n;
    long count;

  public:
    void init(int pinA_, int pinB_);
    void loop();
    long getPos();
};
#endif

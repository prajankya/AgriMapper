#ifndef __ENCODER_H__
#define __ENCODER_H__

class Encoder {
  private:
    int pinA;
    int pinB;

    int th_A;
    int th_B;

    int increment_val;

    bool is_A_Analog;
    bool is_B_Analog;

    int pinALast;
    int n;
    long count;

    void init_set(int pinA_, int pinB_);
    bool readPin(int pin);

  public:
    void init(int pinA_, int pinB_);
    void init(int pinA_, int pinB_, int th_A_);
    void init(int pinA_, int pinB_, int th_A_, int th_B_);

    void loop();
    long getPos();
    void negate(bool set_);
};
#endif /* ifndef __ENCODER_H__ */

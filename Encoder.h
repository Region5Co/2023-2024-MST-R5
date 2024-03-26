#ifndef ENCODER_H
#define ENCODER_H

#define WHEEL_RADIUS 0.935
#define PI 3.14159265

class Encoder {
friend class Motor;
  public:
    Encoder();
    Encoder(int encIntPin, int encDirPin);

    void init();

    int getEncCount();
    int getEncIntPin();
    void incEncCount();
    int getEncDist();
    float getRPM();
  
  private:  
    int encCount;
    int intPin;
    int dirPin;
    float wheelCircum;
    float numRev;
    float distInch;
    float angV;
    uint32_t currTime;
    uint32_t lastTime;
    float RPM;
};

#endif
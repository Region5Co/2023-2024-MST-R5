#include <Arduino.h>

class Encoder {
  public:
    Encoder(int encIntPin, int encDirPin);

    void init();

    int getEncCount();
    int getEncIntPin();
    void incEncCount();
    int getEncDist();
    float getRPM();
  
  private:  
    Encoder();

    int encCount;
    int intPin;
    int dirPin;
    float wheelCircum;
    float numRev;
    float distInch;
    uint32_t currTime;
    uint32_t lastTime;
    float RPM;
};
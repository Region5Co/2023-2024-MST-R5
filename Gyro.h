#ifndef IEEE_GYRO_H
#define IEEE_GYRO_H

#include "src/Adafruit_LSM6DS/Adafruit_LSM6DS3TRC.h"

#define CALIBRATION_DELAY 300

#if !defined(ACCEL_RANGE) && !defined(ACCEL_RATE) 
#define ACCEL_RANGE LSM6DS_ACCEL_RANGE_16_G
#define ACCEL_RATE  LSM6DS_RATE_833_HZ
#endif

#if !defined(GYRO_RANGE) && !defined(GYRO_RATE)
#define GYRO_RANGE  LSM6DS_GYRO_RANGE_2000_DPS  //Please dont change very sensitive
#define GYRO_RATE   LSM6DS_RATE_26_HZ          //This one too!
#endif

class Gyro{
friend class Robot;
public: 
    Gyro(){};
    Gyro(bool acc_en,bool gyro_en);
    void init();
    void calibrate();
    double update();
    float getGyroZ();
private:
    //Devices
    Adafruit_LSM6DS3TRC imu;
    bool acc_en = false;
    bool gyro_en = true;
    float gyroZ,             // units dps (degrees per second)
          gyroDriftZ,        // units dps
           gyroYaw,           // units degrees (expect major drift)
           gyroCorrectedYaw;  // units degrees (expect minor drift)
    long lastTime;
    long lastInterval;
};

#endif
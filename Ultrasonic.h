#ifndef IEEE_ULTRASONIC_H
#define IEEE_ULTRASONIC_H

#include <Adafruit_VL53L1X.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <vl53l1x_class.h>
#include <vl53l1x_error_codes.h>

#define US_TIMING_BUDGET 500
#define US_DISTANCE_MODE 2
#define US_ROI_X 4
#define US_ROI_Y 16

class Ultrasonic{
friend class Robot;
public:
    Ultrasonic();
    Ultrasonic(uint8_t i2c_addr);
    void init();
    uint16_t getDistance();
private:
    Adafruit_VL53L1X vl53;
    uint8_t addr;
};

#endif
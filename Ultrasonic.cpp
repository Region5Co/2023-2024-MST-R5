#include "Ultrasonic.h"


Ultrasonic::Ultrasonic(){

}

Ultrasonic::Ultrasonic(uint8_t i2c_addr){
    this->addr = i2c_addr;
    vl53 = Adafruit_VL53L1X();
}

void Ultrasonic::init(){
    if (!vl53.begin(0x29)) {
        #if IEEE_SERIAL
            Serial.print(F("Error on addr init"));
            Serial.println(vl53.vl_status);
        #endif
        Serial.print(F("Error on addr init"));
        Serial.println(vl53.vl_status);
    }
    if (!vl53.startRanging()) {
        #if IEEE_SERIAL
            Serial.print(F("Ranging not working"));
            Serial.println(vl53.vl_status);
        #endif
    }
    vl53.setTimingBudget(US_TIMING_BUDGET);
    vl53.VL53L1X_SetDistanceMode(US_DISTANCE_MODE);
    vl53.VL53L1X_SetROI(US_ROI_X, US_ROI_Y);
}

uint16_t Ultrasonic::getDistance(){
    return vl53.distance();
}
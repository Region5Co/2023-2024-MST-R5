#include "Gyro.h"

Gyro::Gyro(bool acc_en,bool gyro_en): acc_en(acc_en), gyro_en(gyro_en){
    //Imu initialization
    if(!imu.begin_I2C(0x6A)){
        #if IEEE_SERIAL
            Serial.println("ERROR Initializing I2C for IMU");
        #endif
    }
    if(acc_en){
        imu.setAccelRange(ACCEL_RANGE);
        imu.setAccelDataRate(ACCEL_RATE);
    }
    if(gyro_en){
        imu.setGyroRange(GYRO_RANGE);
        imu.setGyroDataRate(GYRO_RATE);
    }
}

void Gyro::init(){
  //not using Accel.
  //imu.configInt1(false, false, true); // accelerometer DRDY on INT1 of the imu
  imu.configInt2(false, true, false); // gyro DRDY on INT1 of the imu

}

void Gyro::calibrate(){
    int calibrationCount = 0;
    delay(CALIBRATION_DELAY); // to avoid shakes after pressing reset button
    lastTime = micros();
    float sumX, sumY, sumZ;
    int startTime = millis();
    while (millis() < startTime + CALIBRATION_DELAY) {
        if (update()) { 
            sumZ += gyroZ;
            calibrationCount++;
        }
    }

    if (calibrationCount == 0) {
        #if IEEE_SERIAL
            Serial.println("Failed to calibrate");
        #endif
    } 
    gyroDriftZ = sumZ / calibrationCount;
    
}

bool Gyro::update(){
    float  x,y;
    if(imu.gyroscopeAvailable()){
        imu.readGyroscope(x,y,gyroZ);
        long currentTime = micros();
        lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
        lastTime = currentTime;
        float lastFrequency = (float) 1000000.0 / lastInterval;
        gyroYaw = gyroYaw + (gyroZ / lastFrequency);
        gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

    return true;
    }else return false;
}

float Gyro::getGyroZ(){
    return this->gyroCorrectedYaw;
}

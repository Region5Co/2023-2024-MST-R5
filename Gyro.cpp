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
    
     gyroZ=0.0;           
 gyroDriftZ=0.0;    
  gyroYaw=0.0;       
  gyroCorrectedYaw=0.0;
}

void Gyro::init(){
    //not using Accel.
    //imu.configInt1(false, false, true); // accelerometer DRDY on INT1 of the imu
    imu.configInt2(false, true, false); // gyro DRDY on INT1 of the imu
   delay(100);
}

void Gyro::calibrate(){
    int calibrationCount = 0;
    delay(CALIBRATION_DELAY); // to avoid shakes after pressing reset button
    lastTime = micros();
    float sumX=0, sumY=0, sumZ=0;
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

double Gyro::update(){
    float  x,y;
    if(imu.readGyroscope(x,y, gyroZ)){

        long currentTime = micros();
        lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
        lastTime = currentTime;
        float lastFrequency = (float) 1000000.0 / lastInterval;
        gyroYaw = gyroYaw + (gyroZ / lastFrequency);
        gyroCorrectedYaw +=  ((gyroZ - gyroDriftZ) / lastFrequency);
        //gyroCorrectedYaw = (gyroCorrectedYaw>=360)? (gyroCorrectedYaw-360):gyroCorrectedYaw;
        //gyroCorrectedYaw = (gyroCorrectedYaw<0)? (gyroCorrectedYaw+360):gyroCorrectedYaw;
    return gyroCorrectedYaw;
    }else return 0.0;
}

float Gyro::getGyroZ(){
    return update();
}

#ifndef IEEE_PINOUT_H

//esp config
/* #define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15
#define SD1 MOSI
#define SD2 9
#define SD3 10 */

#define D0 0
#define D1 1
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define D8 8
#define D9 9
#define D10 10
#define D11 11
#define D12 12
#define D13 13
#define D14 14
#define D15 15
#define D16 16
#define D17 17
#define D18 18
#define D19 19
#define D20 20
#define D21 21

//#define D0 0
//#define D1 1
//#define D2 2
//#define D3 3
//#define D4 4
//#define D5 5
//#define D6 6
//#define D7 7
//#define D8 8
//#define D9 9
//#define D10 10
//#define D11 11
//#define D12 12
//#define D13 13

//#define A0 A0 
//#define A1 A1
//#define A2 A2
//#define A3 A3
//#define A4 A4
//#define A5 A5

#define FRONT_MOTORS_ENABLE   0
#define BACK_MOTORS_ENABLE    0
#define FRONT_RIGHT_PWM       4
#define FRONT_RIGHT_DIR       28
#define FRONT_LEFT_PWM        5
#define FRONT_LEFT_DIR        26
#define BACK_RIGHT_PWM        6
#define BACK_RIGHT_DIR        24
#define BACK_LEFT_PWM         7
#define BACK_LEFT_DIR         22
//gyro
#define SCL_GYRO              21
#define SDA_GYRO              20
//tof
#define SCL_TOF               21
#define SDA_TOF               20

#define HC_TRIGGER            50
#define HC_ECHO               50
#define SERVO_PIN             50

#endif

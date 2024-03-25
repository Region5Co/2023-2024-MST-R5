#ifndef IEEE_PINOUT_H
#define IEEE_DEBUG false

#if IEEE_DEBUG
    #define IEEE_SERIAL 1
#endif


#ifdef ESP_H
    #define D0 16
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
    #define SD3 10
#else
    #undef LED_BUILTIN
    #define LED_BUILTIN 13
    #define D0 0 
    #define D1 1 
    #define D2 2 
    #define D3 3 
    #define D4 4 
    #define D5 5 
    #define D6 6 
    #define D7 7 
    #define D8 8 
#endif




#define FRONT_MOTORS_ENABLE   D0
#define BACK_MOTORS_ENABLE    D0
#define FRONT_RIGHT_PWM       D4
#define FRONT_RIGHT_DIR       D3
#define FRONT_LEFT_PWM        D7
#define FRONT_LEFT_DIR        D8
#define BACK_RIGHT_PWM        D1
#define BACK_RIGHT_DIR        D2
#define BACK_LEFT_PWM         D5
#define BACK_LEFT_DIR         D6
#define IEEE_B0               A1 //Test for uno
#define IEEE_B1               A0 //Test for uno


#endif

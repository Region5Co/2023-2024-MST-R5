#ifndef IEEE_PINOUT_H
#define IEEE_DEBUG false

#define D0   0
#define D1   1
#define D2   2
#define D3   3
#define D4   4
#define D5   5
#define D6   6
#define D7   7
#define D8   8
#define D9   9
#define D10  10
#define D11  11
#define D12  12  
#define D13  13 
#define D14  14 
#define D15  15 
#define D16  16 
#define D17  17 
#define D18  18 
#define D19  19 
#define D20  20 
#define D21  21 
#define D22  22 
#define D23  23 
#define D20  20 
#define D21  21 
#define D22  22 
#define D23  23 
#define D24  24 
#define D25  25 
#define D26  26 
#define D27  27
#define D28  28 
#define D29  29 
#define D30  30 
#define D31  31  
 
#define FRONT_MOTORS_ENABLE   D23
#define BACK_MOTORS_ENABLE    D23
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
#define FRONT_RIGHT_DIR       D28
#define FRONT_LEFT_PWM        D6
#define FRONT_LEFT_DIR        D24
#define BACK_RIGHT_PWM        D5
#define BACK_RIGHT_DIR        D26
#define BACK_LEFT_PWM         D7
#define BACK_LEFT_DIR         D22

#define IEEE_B0               A1 //Test for uno
#define IEEE_B1               A0 //Test for uno


#endif

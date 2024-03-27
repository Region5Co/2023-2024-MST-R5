#ifndef IEEE_PINOUT_H


#if defined(_ATMEGA2560_H_)
  //Pinout for the arduino Due
  #define D0 0
  #define D1 1
  #define D2 2
  #define D3 3
  #define D4 4
  #define D5 5
  #define D6 6
  #define D7 7
  #define D8 8

#elif defined(ESP_H)

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

#endif

//Motors
#define FRONT_MOTORS_ENABLE     D0
#define BACK_MOTORS_ENABLE      D0
#define M_FRONT_RIGHT_PWM       D3
#define M_FRONT_RIGHT_DIR       D4
#define M_FRONT_LEFT_PWM        D7
#define M_FRONT_LEFT_DIR        D8
#define M_BACK_RIGHT_PWM        D1
#define M_BACK_RIGHT_DIR        D2
#define M_BACK_LEFT_PWM         D5
#define M_BACK_LEFT_DIR         D6

//Encoders
#define ENCODERS_ENABLE         true
#define E_FRONT_RIGHT_INT       D9
#define E_FRONT_RIGHT_DIR       D10
#define E_FRONT_LEFT_INT        D11
#define E_FORNT_LEFT_DIR        D12
#define E_BACK_RIGHT_INT        D13
#define E_BACK_RIGHT_DIR        D14  
#define E_BACK_LEFT_INT         D15  
#define E_BACK_LEFT_DIR         D16

#endif

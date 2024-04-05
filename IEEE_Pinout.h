#ifndef IEEE_PINOUT_H
#define IEEE_DEBUG true

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
#define D32  32  
#define D33  33  
#define D34  34  
#define D35  35  
#define D36  36  
#define D37  37  
#define D38  38  
#define D39  39  
#define D40  40  
#define D41  41  
#define D42  42  
#define D43  43  
#define D44  44  
#define D45  45 
#define D46  46 
#define D47  47  
#define D48  48 
#define D49  49 
#define D50  50 
#define D51  51
#define D52  52 
#define D53  53 
#if IEEE_DEBUG
    #define IEEE_SERIAL 0
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

#define ENCODERS_ENABLE         true
#define E_FRONT_RIGHT_INT       D3
#define E_FRONT_RIGHT_DIR       D53
#define E_FRONT_LEFT_INT        D19
#define E_FORNT_LEFT_DIR        D49
#define E_BACK_RIGHT_INT        D2
#define E_BACK_RIGHT_DIR        D51
#define E_BACK_LEFT_INT         D18  
#define E_BACK_LEFT_DIR         D47

#define IEEE_B0               A1 //Test for uno
#define IEEE_B1               A0 //Test for uno
#define IEEE_US               false
#define SERVO_PIN             D8

#endif



#ifndef IEEE_TYPES_H
#define IEEE_TYPES_H


enum class drivetrain {
  mecanum,
  twoWheel
};

enum class turnDirection {
  cw,
  ccw
};

enum class moveDirection {
  forward,
  backward,
  left,
  right
};

enum class WHEEL{
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT
};

#endif

enum States{
  DRIVE,
  STOP
};

#endif

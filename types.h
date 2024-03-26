#ifndef TYPES_H_
#define TYPES_H_

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
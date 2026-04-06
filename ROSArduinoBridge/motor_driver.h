#ifdef L298_MOTOR_DRIVER
  #define FRONT_LEFT_MOTOR_FORWARD   2
  #define FRONT_LEFT_MOTOR_BACKWARD  3
  #define FRONT_RIGHT_MOTOR_FORWARD  4
  #define FRONT_RIGHT_MOTOR_BACKWARD 5
  #define REAR_LEFT_MOTOR_FORWARD    6
  #define REAR_LEFT_MOTOR_BACKWARD   7
  #define REAR_RIGHT_MOTOR_FORWARD   8
  #define REAR_RIGHT_MOTOR_BACKWARD  9

  #define FRONT_LEFT_MOTOR_ENABLE    22
  #define FRONT_RIGHT_MOTOR_ENABLE   23
  #define REAR_LEFT_MOTOR_ENABLE     24
  #define REAR_RIGHT_MOTOR_ENABLE    25
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int flSpeed, int frSpeed, int rlSpeed, int rrSpeed);

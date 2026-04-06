#ifdef USE_BASE
#ifdef L298_MOTOR_DRIVER

void initMotorController() {
  pinMode(FRONT_LEFT_MOTOR_ENABLE,  OUTPUT); digitalWrite(FRONT_LEFT_MOTOR_ENABLE,  HIGH);
  pinMode(FRONT_RIGHT_MOTOR_ENABLE, OUTPUT); digitalWrite(FRONT_RIGHT_MOTOR_ENABLE, HIGH);
  pinMode(REAR_LEFT_MOTOR_ENABLE,   OUTPUT); digitalWrite(REAR_LEFT_MOTOR_ENABLE,   HIGH);
  pinMode(REAR_RIGHT_MOTOR_ENABLE,  OUTPUT); digitalWrite(REAR_RIGHT_MOTOR_ENABLE,  HIGH);
}

void setMotorSpeed(int i, int spd) {
  uint8_t reverse = 0;
  if (spd < 0)  { spd = -spd; reverse = 1; }
  if (spd > 255)  spd = 255;

  switch (i) {
    case FRONT_LEFT:
      if (!reverse) { analogWrite(FRONT_LEFT_MOTOR_FORWARD, spd);  analogWrite(FRONT_LEFT_MOTOR_BACKWARD, 0); }
      else          { analogWrite(FRONT_LEFT_MOTOR_BACKWARD, spd); analogWrite(FRONT_LEFT_MOTOR_FORWARD,  0); }
      break;
    case FRONT_RIGHT:
      if (!reverse) { analogWrite(FRONT_RIGHT_MOTOR_FORWARD, spd);  analogWrite(FRONT_RIGHT_MOTOR_BACKWARD, 0); }
      else          { analogWrite(FRONT_RIGHT_MOTOR_BACKWARD, spd); analogWrite(FRONT_RIGHT_MOTOR_FORWARD,  0); }
      break;
    case REAR_LEFT:
      if (!reverse) { analogWrite(REAR_LEFT_MOTOR_FORWARD, spd);  analogWrite(REAR_LEFT_MOTOR_BACKWARD, 0); }
      else          { analogWrite(REAR_LEFT_MOTOR_BACKWARD, spd); analogWrite(REAR_LEFT_MOTOR_FORWARD,  0); }
      break;
    case REAR_RIGHT:
      if (!reverse) { analogWrite(REAR_RIGHT_MOTOR_FORWARD, spd);  analogWrite(REAR_RIGHT_MOTOR_BACKWARD, 0); }
      else          { analogWrite(REAR_RIGHT_MOTOR_BACKWARD, spd); analogWrite(REAR_RIGHT_MOTOR_FORWARD,  0); }
      break;
  }
}

void setMotorSpeeds(int flSpeed, int frSpeed, int rlSpeed, int rrSpeed) {
  setMotorSpeed(FRONT_LEFT,  flSpeed);
  setMotorSpeed(FRONT_RIGHT, frSpeed);
  setMotorSpeed(REAR_LEFT,   rlSpeed);
  setMotorSpeed(REAR_RIGHT,  rrSpeed);
}

#else
  #error A motor driver must be selected!
#endif
#endif

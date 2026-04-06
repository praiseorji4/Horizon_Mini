#define USE_BASE
#define RIGHT_SCALE 1.0365

#ifdef USE_BASE
   #define ARDUINO_ENC_COUNTER
   #define L298_MOTOR_DRIVER
#endif

#undef USE_SERVOS

#define BAUDRATE     57600
#define MAX_PWM      255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "commands.h"
#include "sensors.h"

#ifdef USE_BASE
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h"

  #define PID_RATE      30
  const int PID_INTERVAL = 1000 / PID_RATE;
  unsigned long nextPID = PID_INTERVAL;

  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[32];
char argv2[32];
long arg1;
long arg2;

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg  = 0;
  index = 0;
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case PING:
      Serial.println(Ping(arg1));
      break;

#ifdef USE_BASE
    case READ_ENCODERS:
      Serial.print(readEncoder(FRONT_LEFT));   Serial.print(" ");
      Serial.print(readEncoder(FRONT_RIGHT));  Serial.print(" ");
      Serial.print(readEncoder(REAR_LEFT));    Serial.print(" ");
      Serial.println(readEncoder(REAR_RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
    {
      char *p = argv1;
      char *str;
      int vals[4] = {0, 0, 0, 0};
      int idx = 0;
      while ((str = strtok_r(p, ":", &p)) != NULL && idx < 4)
        vals[idx++] = atoi(str);
      lastMotorCommand = millis();
      if (vals[0] == 0 && vals[1] == 0 && vals[2] == 0 && vals[3] == 0) {
        setMotorSpeeds(0, 0, 0, 0);
        resetPID();
        moving = 0;
      } else {
        moving = 1;
      }
      int left_target  = (vals[0] + vals[2]) / 2;
      int right_target = (vals[1] + vals[3]) / 2;

      right_target = right_target * RIGHT_SCALE;

      frontLeftPID.TargetTicksPerFrame  = left_target;
      rearLeftPID.TargetTicksPerFrame   = left_target;
      frontRightPID.TargetTicksPerFrame = right_target;
      rearRightPID.TargetTicksPerFrame  = right_target;
      Serial.println("OK");
      break;
    }
    case MOTOR_RAW_PWM:
    {
      char *p = argv1;
      char *str;
      int vals[4] = {0, 0, 0, 0};
      int idx = 0;
      while ((str = strtok_r(p, ":", &p)) != NULL && idx < 4)
        vals[idx++] = atoi(str);
      lastMotorCommand = millis();
      resetPID();
      moving = 0;
      setMotorSpeeds(vals[0], vals[1], vals[2], vals[3]);
      Serial.println("OK");
      break;
    }
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial.println("OK");
      break;
#endif

    default:
      Serial.println("Invalid Command");
      break;
  }
}

void setup() {
  Serial.begin(BAUDRATE);

#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    pinMode(10, INPUT_PULLUP);
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(13, INPUT_PULLUP);
    pinMode(50, INPUT_PULLUP);
    pinMode(51, INPUT_PULLUP);
    pinMode(52, INPUT_PULLUP);
    pinMode(53, INPUT_PULLUP);

    PCMSK0 = 0xFF;
    PCICR |= (1 << PCIE0);
  #endif
  initMotorController();
  resetPID();
#endif
}

void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();

    if (chr == 13) {  // carriage return '\r'
      if (arg == 1) argv1[index] = '\0';
      else if (arg == 2) argv2[index] = '\0';

      runCommand();
      resetCommand();
      break;  // process one command per loop
    }
    else if (chr == ' ') {
      if (arg == 0) {
        arg = 1;
      }
      else if (arg == 1) {
        argv1[index] = '\0';
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        if (index < sizeof(argv1) - 1) {
          argv1[index++] = chr;
        } else {
          // overflow protection: reset everything
          resetCommand();
          return;
        }
      }
      else if (arg == 2) {
        if (index < sizeof(argv2) - 1) {
          argv2[index++] = chr;
        } else {
          // overflow protection
          resetCommand();
          return;
        }
      }
    }
  }

#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0, 0, 0);
    moving = 0;
  }
#endif
}

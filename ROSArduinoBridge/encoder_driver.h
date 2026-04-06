#ifdef ARDUINO_ENC_COUNTER
  // All encoder pins on PCINT0 (PORTB on Mega 2560)
  // Pin 10=PB4  Pin 11=PB5  Pin 12=PB6  Pin 13=PB7
  // Pin 50=PB3  Pin 51=PB2  Pin 52=PB1  Pin 53=PB0
  #define FRONT_LEFT_ENC_PIN_A   4   // PB4 = pin 10
  #define FRONT_LEFT_ENC_PIN_B   5   // PB5 = pin 11
  #define FRONT_RIGHT_ENC_PIN_A  6   // PB6 = pin 12
  #define FRONT_RIGHT_ENC_PIN_B  7   // PB7 = pin 13
  #define REAR_LEFT_ENC_PIN_A    3   // PB3 = pin 50
  #define REAR_LEFT_ENC_PIN_B    2   // PB2 = pin 51
  #define REAR_RIGHT_ENC_PIN_A   1   // PB1 = pin 52
  #define REAR_RIGHT_ENC_PIN_B   0   // PB0 = pin 53
#endif

#define FRONT_LEFT   0
#define FRONT_RIGHT  1
#define REAR_LEFT    2
#define REAR_RIGHT   3

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER

volatile long front_left_enc_pos  = 0L;
volatile long front_right_enc_pos = 0L;
volatile long rear_left_enc_pos   = 0L;
volatile long rear_right_enc_pos  = 0L;

static const int8_t ENC_STATES[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

ISR(PCINT0_vect) {
  static uint8_t last_portb = 0;
  uint8_t curr_portb = PINB;
  uint8_t changed = curr_portb ^ last_portb;

  // Front-left: PB4 (A) and PB5 (B) — adjacent bits, read as pair
  if (changed & ((1 << FRONT_LEFT_ENC_PIN_A) | (1 << FRONT_LEFT_ENC_PIN_B))) {
    static uint8_t fl_last = 0;
    fl_last <<= 2;
    fl_last |= (curr_portb >> FRONT_LEFT_ENC_PIN_A) & 0x03;
    front_left_enc_pos += ENC_STATES[fl_last & 0x0f];
  }

  // Front-right: PB6 (A) and PB7 (B) — adjacent bits, read as pair
  if (changed & ((1 << FRONT_RIGHT_ENC_PIN_A) | (1 << FRONT_RIGHT_ENC_PIN_B))) {
    static uint8_t fr_last = 0;
    fr_last <<= 2;
    fr_last |= (curr_portb >> FRONT_RIGHT_ENC_PIN_A) & 0x03;
    front_right_enc_pos += ENC_STATES[fr_last & 0x0f];
  }

  // Rear-left: PB3 (A) and PB2 (B) — non-adjacent order, read each bit separately
  if (changed & ((1 << REAR_LEFT_ENC_PIN_A) | (1 << REAR_LEFT_ENC_PIN_B))) {
    static uint8_t rl_last = 0;
    rl_last <<= 2;
    uint8_t a = (curr_portb >> REAR_LEFT_ENC_PIN_A) & 0x01;
    uint8_t b = (curr_portb >> REAR_LEFT_ENC_PIN_B) & 0x01;
    rl_last |= (a << 1) | b;
    rear_left_enc_pos += ENC_STATES[rl_last & 0x0f];
  }

  // Rear-right: PB1 (A) and PB0 (B) — adjacent bits, read as pair
  if (changed & ((1 << REAR_RIGHT_ENC_PIN_A) | (1 << REAR_RIGHT_ENC_PIN_B))) {
    static uint8_t rr_last = 0;
    rr_last <<= 2;
    uint8_t a = (curr_portb >> REAR_RIGHT_ENC_PIN_A) & 0x01;
    uint8_t b = (curr_portb >> REAR_RIGHT_ENC_PIN_B) & 0x01;
    rr_last |= (a << 1) | b;
    rear_right_enc_pos += ENC_STATES[rr_last & 0x0f];
  }

  last_portb = curr_portb;
}

long readEncoder(int i) {
  long val;
  switch(i) {
    case FRONT_LEFT:  noInterrupts(); val = front_left_enc_pos;  interrupts(); return val;
    case FRONT_RIGHT: noInterrupts(); val = front_right_enc_pos; interrupts(); return val;
    case REAR_LEFT:   noInterrupts(); val = rear_left_enc_pos;   interrupts(); return val;
    case REAR_RIGHT:  noInterrupts(); val = rear_right_enc_pos;  interrupts(); return val;
    default:          return 0;
  }
}

void resetEncoder(int i) {
  switch(i) {
    case FRONT_LEFT:  front_left_enc_pos  = 0L; break;
    case FRONT_RIGHT: front_right_enc_pos = 0L; break;
    case REAR_LEFT:   rear_left_enc_pos   = 0L; break;
    case REAR_RIGHT:  rear_right_enc_pos  = 0L; break;
  }
}

#else
  #error An encoder driver must be selected!
#endif

void resetEncoders() {
  resetEncoder(FRONT_LEFT);
  resetEncoder(FRONT_RIGHT);
  resetEncoder(REAR_LEFT);
  resetEncoder(REAR_RIGHT);
}

#endif

// interrupt service routine
volatile int encoder_count_motor1 = 0;
void readEncoder1() {
  int b1 = digitalRead(ENC_B_M1);
  if (b1 > 0) {
    encoder_count_motor1++;
  } else {
    encoder_count_motor1--;
  }
}

// interrupt service routine
volatile int encoder_count_motor2 = 0;
void readEncoder2() {
  int b2 = digitalRead(ENC_B_M2);
  if (b2 > 0) {
    encoder_count_motor2++;
  } else {
    encoder_count_motor2--;
  }
}
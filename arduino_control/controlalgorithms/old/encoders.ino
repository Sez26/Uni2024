volatile int encoder_count_volatile_motor1 = 0;
void readEncoder1() {
  int b1 = digitalRead(ENC_B_M1);
  if (b1 == 1) {
    encoder_count_volatile_motor1++;
  } else {
    encoder_count_volatile_motor1--;
  }
}

volatile int encoder_count_volatile_motor2 = 0;
void readEncoder2() {
  int b2 = digitalRead(ENC_B_M2);
  if (b2 == 1) {
    encoder_count_volatile_motor2++;
  } else {
    encoder_count_volatile_motor2--;
  }
}

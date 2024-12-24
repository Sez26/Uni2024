#include <SimplyAtomic.h>  // For the position read, to avoid missed counts

#define ENCA1 0      // Encoder A for Motor 1
#define ENCB1 1      // Encoder B for Motor 1
#define PWM_pin1 2   // PWM pin for Motor 1
#define IN1_1 3      // Direction pin 1 for Motor 1
#define IN2_1 4      // Direction pin 2 for Motor 1

#define ENCA2 6      // Encoder A for Motor 2
#define ENCB2 7      // Encoder B for Motor 2
#define PWM_pin2 10   // PWM pin for Motor 2
#define IN1_2 8      // Direction pin 1 for Motor 2
#define IN2_2 9      // Direction pin 2 for Motor 2

volatile int posi1 = 0, posi2 = 0;  // Encoder counts for Motor 1 and Motor 2
int pos1 = 0, pos2 = 0;             // Position variables
int e1 = 0, e2 = 0;                 // Errors for Motor 1 and Motor 2
int u1 = 0, u2 = 0;                 // Control signals for Motor 1 and Motor 2
int u_sign1 = 0, u_sign2 = 0;       // Control signal signs for Motor 1 and Motor 2
int u_amplitude1 = 150, u_amplitude2 = 150;  // Control signal amplitudes for Motor 1 and Motor 2
float ref1 = 0, ref2 = 0;           // Reference positions for Motor 1 and Motor 2
int rotations1 = 0, rotations2 = 0; // Rotations counter for both motors
long time_start = 0;
float time_per_rotation = 5000;  // time allowed per rotation, in milliseconds
float counts_per_rotation = 131.25 * 16;
float ref_amplitude = counts_per_rotation;
int rotations = 0;
int interval_count = 0;
int print_interval = 100;  // define how often values are sent to the serial monitor

// function to send signal to motor driver (could also be defined at top of code)
// set inputs and their classifications
void setMotor(int dir, float pwmVal, int pwm_pin, int in1, int in2) {
  //set the pwm pin to the appropriate level (0 - 255)
  analogWrite(pwm_pin, pwmVal);
  //set direction
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// ISR for Motor 1 encoder
void readEncoder1() {
  int b1 = digitalRead(ENCB1);
  if (b1 > 0) {
    posi1++;
  } else {
    posi1--;
  }
}

// ISR for Motor 2 encoder
void readEncoder2() {
  int b2 = digitalRead(ENCB2);
  if (b2 > 0) {
    posi2++;
  } else {
    posi2--;
  }
}

void setup() {
  Serial.begin(230400); // set baud rate for communication between USB & raspberry pi pico

  // Motor 1 encoder and motor setup
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);

  pinMode(PWM_pin1, OUTPUT);
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);

  // Motor 2 encoder and motor setup
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);

  pinMode(PWM_pin2, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);

  time_start = millis();
}

//this loop implements bang-bang code 
void loop() {
  // Motor 1 control
  if ((millis() - time_start) > time_per_rotation) {
    rotations1++;
    time_start = millis();
  }
  ref1 = ref_amplitude * rotations1;

  ATOMIC() { pos1 = posi1; }
  e1 = ref1 - pos1;

  u_sign1 = (e1 > 0) ? 1 : (e1 < 0) ? -1 : 0;
  setMotor(u_sign1, u_amplitude1, PWM_pin1, IN1_1, IN2_1);

  // Motor 2 control
  if ((millis() - time_start) > time_per_rotation) {
    rotations2++;
    time_start = millis();
  }
  ref2 = ref_amplitude * rotations2;

  ATOMIC() { pos2 = posi2; }
  e2 = ref2 - pos2;

  u_sign2 = (e2 > 0) ? 1 : (e2 < 0) ? -1 : 0;
  setMotor(u_sign2, u_amplitude2, PWM_pin2, IN1_2, IN2_2);

  // Print data for both motors
  interval_count++;
  if (interval_count >= print_interval) {
    interval_count = 0;
    Serial.print("Motor 1: ");
    Serial.print(ref1); Serial.print(" "); Serial.print(pos1); Serial.print(" "); Serial.print(e1); Serial.print(" "); Serial.print(u_sign1 * u_amplitude1); Serial.println();
    
    Serial.print("Motor 2: ");
    Serial.print(ref2); Serial.print(" "); Serial.print(pos2); Serial.print(" "); Serial.print(e2); Serial.print(" "); Serial.print(u_sign2 * u_amplitude2); Serial.println();
  }
}

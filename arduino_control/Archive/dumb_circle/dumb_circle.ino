#include <SimplyAtomic.h>  // For the position read, to avoid missed counts
#include <math.h>

#define ENC_A_M1 0      // Encoder A for Motor 1
#define ENC_B_M1 1      // Encoder B for Motor 1
#define PWM_pin_M1 2   // PWM pin for Motor 1
#define IN1_M1 3      // Direction pin 1 for Motor 1
#define IN2_M1 4      // Direction pin 2 for Motor 1

#define ENC_A_M2 6      // Encoder A for Motor 2
#define ENC_B_M2 7      // Encoder B for Motor 2
#define PWM_pin_M2 10   // PWM pin for Motor 2
#define IN1_M2 8      // Direction pin 1 for Motor 2
#define IN2_M2 9      // Direction pin 2 for Motor 2

volatile int posi1 = 0, posi2 = 0;    // specify posi as volatile, integer because counts are discrete
int print_interval = 100;  // define how often values are sent to the serial monitor
int interval_count = 0;
float interval_start = 0;
int pos1 = 0, pos2 = 0;             // Position variables
int e1 = 0, e2 = 0;                 // Errors for Motor 1 and Motor 2
int u1 = 0, u2 = 0;                 // Control signals for Motor 1 and Motor 2
int u_sign1 = 0, u_sign2 = 0;       // Control signal signs for Motor 1 and Motor 2
int u_amplitude1 = 150, u_amplitude2 = 150;  // Control signal amplitudes for Motor 1 and Motor 2
float ref1 = 0, ref2 = 0;           // Reference positions for Motor 1 and Motor 2
int rotations = 0; // Rotations counter for both motors
long time_start = 0;
float counts_per_rotation = 131.25 * 16;
float ref_amplitude = counts_per_rotation;
float time_per_rotation = 10000;// time allowed per rotation, in milliseconds
const unsigned long rot_time = 10000;

// reference signal stuff
float th_0 = 0;
float ox = 1.0;
float oy = 0;
float r = 1.0;
float L1 = 2.0;
float L2 = 2.0;
float Pi = 3.14159;

void setup() {
  Serial.begin(230400);  // set baud rate for communication between USB & raspberry pi pico
  pinMode(ENC_A_M1, INPUT);
  pinMode(ENC_B_M1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M1), readEncoder1, RISING);

  // pwm and direction pins are outputs that go to the motor
  pinMode(PWM_pin_M1, OUTPUT);
  pinMode(IN1_M1, OUTPUT);
  pinMode(IN2_M1, OUTPUT);

  pinMode(ENC_A_M2, INPUT);
  pinMode(ENC_B_M2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M2), readEncoder2, RISING);

  // pwm and direction pins are outputs that go to the motor
  pinMode(PWM_pin_M2, OUTPUT);
  pinMode(IN1_M2, OUTPUT);
  pinMode(IN2_M2, OUTPUT);
  
  time_start = millis();
}

// This loop implements 
void loop() {
  // Set a reference angle position for the motor.
  if ((millis() - time_start) > time_per_rotation) {
    rotations++;
    time_start = millis();
  }
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - time_start;
  // th_0 increase
  th_0 = (elapsed_time * 2 * Pi) / rot_time;
  // th_0 = ((millis()%rot_time)/rot_time) * 2 * Pi;
  float tar_x = tarx(ox, r, th_0);
  float tar_y = tary(oy, r, th_0);
  float th_1 = theta_1(tar_x, tar_y, L1, L2, counts_per_rotation, Pi);
  float th_2 = theta_2(tar_x, tar_y, L1, L2, counts_per_rotation, Pi);
  ref1 = th_1;
  ref2 = th_2;

  ATOMIC() {  // lines between these brackets are executed even if an interrupt occurs
    pos1 = posi1;
    pos2 = posi2;
  }

  // calculate position error
  e1 = ref1 - pos1;
  e2 = ref2 - pos2;
  
  // calculate sign of control signal
  u_sign1 = 0;
  if (e1 < 0) {
    u_sign1 = -1;
  }
  if (e1 > 0) {
    u_sign1 = 1;
  }

  u_sign2 = 0;
  if (e2 < 0) {
    u_sign2 = -1;
  }
  if (e2 > 0) {
    u_sign2 = 1;
  }

  // call setMotor function to drive the motor at a set direction and torque
  u_amplitude1 = 100;
  u_amplitude2 = 100;
  setMotor(u_sign1, u_amplitude1, PWM_pin_M1, IN1_M1, IN2_M1);
  setMotor(u_sign2, u_amplitude2, PWM_pin_M2, IN1_M2, IN2_M2);

  // print target and position to see the response every print_interval times around the loop
  interval_count = interval_count + 1;
  if (interval_count >= print_interval) {
    interval_count = 0;
    Serial.print("th_0 ");
    Serial.print(th_0);
    Serial.print(" ref ");
    Serial.print(ref1);
    Serial.print(", pos  ");
    Serial.print(pos1);
    Serial.print(", error  ");
    Serial.print(e1);
    Serial.print(", u  ");
    Serial.print(u_sign1 * u_amplitude1);
    Serial.print(" ");
    Serial.println();
  }
}

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
  int b1 = digitalRead(ENC_B_M1);
  if (b1 > 0) {
    posi1++;
  } else {
    posi1--;
  }
}

// ISR for Motor 2 encoder
void readEncoder2() {
  int b2 = digitalRead(ENC_B_M2);
  if (b2 > 0) {
    posi2++;
  } else {
    posi2--;
  }
}

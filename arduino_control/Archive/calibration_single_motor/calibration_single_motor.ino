#include <SimplyAtomic.h>  // For the position read, to avoid missed counts

// Define Encoder pins, pwm pin, and motor driver pins
#define ENCA 0     // interrupt pin encoder A
#define ENCB 1     // interrupt pin encoder B
#define PWM_pin 2  // pin to send pwm value to motor driver
#define IN1 3      // motor driver direction pins
#define IN2 4

volatile int posi = 0;     // specify posi as volatile, integer because counts are discrete
int print_interval = 100;  // define how often values are sent to the serial monitor
int interval_count = 0;
int cal_interval = 2000;
float interval_start = 0;
float ref = 0;
int pos = 0;            // pos is the angular position of the output from the gearbox, in pulses
int prev_pos;           // needed for calibration
int e = 0;              // e is the error between ref and pos
int u = 0;              // u is the control signal
int u_amplitude = 150;  // control signal amplitude (must be < 256)
int u_sign = 1;         // used to specify required sign of control signal
long time_start = 0;
float counts_per_rotation = 131.25 * 16;  //gear ratio * encoding count
float ref_amplitude = counts_per_rotation;
int rotations = 0;
float time_per_rotation = 5000;  // time allowed per rotation, in milliseconds

void setup() {
  Serial.begin(230400);  // set baud rate for communication between USB & raspberry pi pico
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);  //if position changes, read it - no matter what

  // pwm and direction pins are outputs that go to the motor
  pinMode(PWM_pin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  time_start = millis();
  //Calibration sequence
  calibrate(rotations, pos, prev_pos);
  Serial.println("---------------------------  SETUP COMPLETE  --------------------------------");

}

// This loop implements bang-bang code
void loop() {
  

  // print target and position to see the response every print_interval times around the loop
  interval_count = interval_count + 1;
  if (interval_count >= 5000) {
    interval_count = 0;
    Serial.print("Main loop");
  }
}

//Don't mess with these fuctions, they just send things to the motors
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

// Interrupt service routine 
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}

void calibrate(int rotations, int pos, int prev_pos) {
  u_amplitude = 50;
  bool cal = false;
  bool move = false;
  int num_hits = 0;
  int hit_1_pos = 0;
  int hit_2_pos = 0;
  int mid_point = 0;
  while (cal == false) {
    
    ATOMIC() {
      pos = posi; //update position //about 6 points per degree
    }

    setMotor(u_sign, u_amplitude, PWM_pin, IN1, IN2); // rotate motor slowly

    interval_count = interval_count + 1;  
    if (interval_count >= cal_interval) {
      interval_count = 0;
      Serial.print("| Pos ");
      Serial.print(pos);
      Serial.print("| Prev pos ");
      Serial.print(prev_pos);
      Serial.println();

      //if position is the same as previous position, stick has been hit
      if ((pos == prev_pos) && (pos != 0)) { 
        if (num_hits == 0) {
          Serial.print("| hit 1 ");
          Serial.print(pos);
          hit_1_pos = pos;
          u_sign = u_sign * -1;      //switch motor direction
          num_hits ++;
        }
        else if (num_hits == 1) {
          Serial.print("| hit 2 ");
          Serial.print(pos);
          hit_2_pos = pos;
          mid_point = (hit_1_pos + hit_2_pos) / 2; //calculate mid point
          move = true;
          num_hits ++;
        }
      }

      if (move == true) {
        Serial.print("| Avg: ");
        Serial.print(mid_point);
        //move arm to start point
        if (pos > mid_point) {
          u_sign = -1;
        }
        else if (pos < mid_point) {
          u_sign = 1;
        }
        else if (pos == mid_point) {
          Serial.println("calibration complete");
          u_amplitude = 0;
          cal = true; //calibration complete for one arm
        }
      }
      prev_pos = pos;
    }
  } 
  setMotor(u_sign, u_amplitude, PWM_pin, IN1, IN2); // rotate motor slowly
}



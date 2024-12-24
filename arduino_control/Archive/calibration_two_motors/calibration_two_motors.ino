#include <SimplyAtomic.h>  // For the position read, to avoid missed counts

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

#include "/Users/herra/OneDrive - University of Bristol/EDes/Year 4/Multivariable and Nonlinear Control/MNC shared folder/arduinocode/controlalgorithms/Encoder.h"

int interval_count = 0;
float interval_start = 0;
long time_start = 0;

int ENC_A = 0;      
int ENC_B = 0;      
int PWM_pin = 0;   
int INI1 = 0;     
int INI2 = 0;

void setup() {
  Serial.begin(230400);  // set baud rate for communication between USB & raspberry pi pico
  
  pinMode(ENC_A_M1, INPUT);
  pinMode(ENC_B_M1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M1), readEncoder1, RISING); 

  pinMode(ENC_A_M2, INPUT);
  pinMode(ENC_B_M2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M2), readEncoder2, RISING);

  // pwm and direction pins are outputs that go to the motor
  pinMode(PWM_pin_M1, OUTPUT);
  pinMode(IN1_M1, OUTPUT);
  pinMode(IN2_M1, OUTPUT);

  pinMode(PWM_pin_M2, OUTPUT);
  pinMode(IN1_M2, OUTPUT);
  pinMode(IN2_M2, OUTPUT);

  time_start = millis();

  //Calibration sequence
  motor_calibration(1);
  motor_calibration(2);

  Serial.println("---------------------------  SETUP COMPLETE  --------------------------------");
}

// This loop implements bang-bang code
void loop() {
  interval_count = interval_count + 1;
  if (interval_count >= 5000) {
    interval_count = 0;
    Serial.print("Main loop");
  }
}

// Move the motor
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

void motor_calibration(int motor_num) {
  bool motor_cal = false;
  bool move = false;
  int u_amplitude = 50;
  int u_sign = 1;         // used to specify required sign of control signal
  int cal_interval = 2000;
  int num_hits = 0;
  int hit_1_pos = 0;
  int hit_2_pos = 0;
  int mid_point = 0;
  int pos = 0;            // pos is the angular position of the output from the gearbox, in pulses
  int prev_pos;           // needed for calibration

  if (motor_num == 1) {
    ENC_A = ENC_A_M1;      
    ENC_B = ENC_B_M1;      
    PWM_pin = PWM_pin_M1;   
    INI1 = IN1_M1;     
    INI2 = IN2_M1;
  } else if (motor_num == 2) {
    ENC_A = ENC_A_M2;      
    ENC_B = ENC_B_M2;      
    PWM_pin = PWM_pin_M2;   
    INI1 = IN1_M2;     
    INI2 = IN2_M2;
  }

  while (motor_cal == false) {
    
    ATOMIC() {
      if (motor_num == 1) {
        pos = encoder_count_volatile_motor1; //update position //about 6 points per degree
      } else if (motor_num == 2) {
        pos = encoder_count_volatile_motor2;
      }
    }

    setMotor(u_sign, u_amplitude, PWM_pin, INI1, INI2); // rotate motor slowly

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
          motor_cal = true; //calibration complete for one arm
        }
      }
      prev_pos = pos;
    }
  } 
  setMotor(u_sign, u_amplitude, PWM_pin, INI1, INI2); // rotate motor slowly
}
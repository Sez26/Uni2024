// Definitions ----------------------------------------------------------------------------------------------------
//#define ON_SWITCH 11    // On switch connect here!!

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

//Include the following files and libraries -----------------------------------------------------------------
#include <SimplyAtomic.h>
#include <math.h>

//Izzy's file paths
#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/controlalgorithms.h"
#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/encoders.h"
//#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/startsequence.h"
#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_circ_14.h"
//#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_sq_26.h"
//#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_tri_12.h"
#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/arduino_control/calibration/calibration.ino"

//Lizzy's file paths
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/controlalgorithms.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/encoders.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_circ_10.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_sq_5.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_tri_3.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/arduino_control/calibration/calibration.ino"

//Serena's file paths
// #include "/Users/Sez26/Documents/Arduino/MVNLC/control/Uni2024_MVNLC/arduino_control/controlalgorithms/controlalgorithms.h"
// #include "/Users/Sez26/Documents/Arduino/MVNLC/control/Uni2024_MVNLC/arduino_control/controlalgorithms/encoders.h"
// #include "/Users/Sez26/Documents/Arduino/MVNLC/control/Uni2024_MVNLC/reference_signals/ref_circ_8.h"
// #include "/Users/Sez26/Documents/Arduino/MVNLC/control/Uni2024_MVNLC/arduino_control/calibration/calibration.ino"

// Instantiate Variables ------------------------------------------------------------------------------------------------
//float counts_per_rotation = 131.25 * 16;
bool run_continuously = true;

// timestep in microseconds
long delta_T = 1500;
long previous_T = micros();

int ref_index = 0;
double running_time = 0;
int arrayLength = sizeof(th_1) / sizeof(th_1[0]);

// Targets
int target_counts_1 = 0, target_counts_2 = 0;
int pwm_1 = 0, pwm_2 = 0;

double prev_counts1 = 0;
double prev_prev_counts1 = 0;
double prev_counts2 = 0;
double prev_prev_counts2 = 0;
int calibration_pos1=0, calibration_pos2=0;
int offset_testing = 0, num_of_circles = 0;

// // Arrays to store error values when troubleshootingmain.c is added to loop
// double error1_array[num_samples];
// double error2_array[num_samples];
// const int num_samples = 5000;
// int error_index = 0;      
// bool errors_collected = false;
// double min_error1 = 0, max_error1 = 0, sum_error1 = 0;
// double min_error2 = 0, max_error2 = 0, sum_error2 = 0;

// Instantiate motors
MotorController_c motor_controller1;
MotorController_c motor_controller2;

//Setup ---------------------------------------------------------------------------------
void setup() {
  Serial.begin(230400);  // set baud rate for communication between USB & raspberry pi pico

  motor_controller1 = MotorController_c();
  motor_controller1.SetupMotorController(PWM_pin_M1, IN1_M1, IN2_M1);
  pinMode(ENC_A_M1, INPUT);
  pinMode(ENC_B_M1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M1), readEncoder1, RISING); 
  
  motor_controller2 = MotorController_c();
  motor_controller2.SetupMotorController(PWM_pin_M2, IN1_M2, IN2_M2);
  pinMode(ENC_A_M2, INPUT);
  pinMode(ENC_B_M2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M2), readEncoder2, RISING);

  motor_controller1.SetupPIDController(150, 0.45, 3, 100, delta_T/1e6);
  motor_controller2.SetupPIDController(150, 0.45, 3, 100, delta_T/1e6);
}

//Loop -----------------------------------------------------------------------------------
void loop() {

  do{       } // running an empty loop until the current time- prev time is the desired timestep
  while ((micros() - previous_T) < delta_T);
  previous_T = micros();

  if (running_time >= 5){
    target_counts_1 = th_1[ref_index];
    target_counts_2 = th_2[ref_index];

    if (ref_index == arrayLength - 1) {
      if (run_continuously) {
        ref_index = 0;  // Loop back to the start if running continuously
      }
      // Otherwise, keep ref_index at the last element
    } else {
      ref_index++;
    }

  }
    
  motor_controller1.SetTargetCounts(target_counts_1);
  motor_controller2.SetTargetCounts(target_counts_2);

  // Controller algorithms
  // PID
  double output1 = motor_controller1.pid_controller(encoder_count_motor1);
  double output2 = motor_controller2.pid_controller(encoder_count_motor2);

  //Update running time
  running_time += delta_T/1e6;
}




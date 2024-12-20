//Notes to team/self ---------------------------------------------------------------------------------------------
// filtering the differentiator signal is important
// removing the integrator term when the error is low and adding it when the error is high

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
#include <SimplyAtomic.h>  // For the position read, to avoid missed counts
#include <math.h>
//#include <BasicLinearAlgebra.h>

//Izzy's file paths
#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/controlalgorithms.h"
#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/Encoder.h"
//#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/startsequence.h"
#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_circ_14.h"
//#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_sq_26.h"
//#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_tri_12.h"
#include "/Users/Izzy Popiolek/Documents/GitHub/Uni2024_MVNLC/arduino_control/calibration/calibration.ino"

//Lizzy's file paths
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/controlalgorithms.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/arduino_control/controlalgorithms/Encoder.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_circ_10.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_sq_5.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/reference_signals/ref_tri_3.h"
//#include "/Users/herra/Documents/GitHub/Uni2024_MVNLC/arduino_control/calibration/calibration.ino"

//Serena's file paths
// #include "/Users/Sez26/Documents/Arduino/MVNLC/control/Uni2024_MVNLC/arduino_control/controlalgorithms/controlalgorithms.h"
// #include "/Users/Sez26/Documents/Arduino/MVNLC/control/Uni2024_MVNLC/arduino_control/controlalgorithms/Encoder.h"
// #include "/Users/Sez26/Documents/Arduino/MVNLC/control/Uni2024_MVNLC/reference_signals/ref_circ_8.h"
// #include "/Users/Sez26/Documents/Arduino/MVNLC/control/Uni2024_MVNLC/arduino_control/calibration/calibration.ino"

// Instantiate Variables ------------------------------------------------------------------------------------------------
//float counts_per_rotation = 131.25 * 16;
bool run_continuously = true;

// timestep in microseconds
long delta_T = 1500; // it was 1500  
// 2000 = 2 sec circle, 1500 = 1.5 sec circle 1000 = 1 sec circle
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

// Define constants for the number of samples to record
const int num_samples = 5000;

// Arrays to store error values
double error1_array[num_samples];
double error2_array[num_samples];
int error_index = 0;            // To track the number of collected samples
bool errors_collected = false;  // Flag to track if we have collected enough samples
double min_error1 = 0, max_error1 = 0, sum_error1 = 0;
double min_error2 = 0, max_error2 = 0, sum_error2 = 0;

// Instantiate motors
MotorController_c motor_controller1;
MotorController_c motor_controller2;

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

// // there are combinations that work together well
//   motor_controller1.SetupPIDController(26, 0, 1.8, 100, delta_T/1e6);
//   motor_controller2.SetupPIDController(26, 0, 1.8, 100, delta_T/1e6);

  // best square yet -> try bspline/ corner ramping
  // motor_controller1.SetupPIDController(50, 0.9, 2.5, 100, delta_T/1e6);
  // motor_controller2.SetupPIDController(50, 0.9, 2.5, 100, delta_T/1e6);
  motor_controller1.SetupPIDController(150, 0.45, 3, 100, delta_T/1e6);
  motor_controller2.SetupPIDController(150, 0.45, 3, 100, delta_T/1e6);
  // motor_controller1.SetupPIDController(299.37, 0.24588, 167.18, 100, delta_T/1e6);
  // motor_controller2.SetupPIDController(8.52, 0.19994, 170.23, 100, delta_T/1e6);
}

// This loop implements 
void loop() {

  do{       } // running an empty loop until the current time- prev time is the desired timestep
  while ((micros() - previous_T) < delta_T);
  previous_T = micros();
  // Step input

  // //start_sequence
  // if (running_time <= 5){
  //   target_counts_1 = start_1[ref_index];
  //   target_counts_2 = start_2[ref_index];
  //   if (ref_index == 999) {
  //     target_counts_1 = -450;
  //     target_counts_2 = -900;
  //   } else {
  //       ref_index++;
  //   }
  // }

  if (running_time >= 5){
    target_counts_1 = th_1[ref_index];
    target_counts_2 = th_2[ref_index];

      // Move to the next reference signal, handling continuous or single-run mode
    if (ref_index == arrayLength - 1) {
    // if (ref_index == 0){
      if (run_continuously) {
        ref_index = 0;  // Loop back to the start if running continuously
      }
      // Otherwise, keep ref_index at the last element to stop updating
    } else {
      ref_index++;
      // motor_controller1.TurnMotorOff();
      // motor_controller2.TurnMotorOff();
      // return;
    }


    if (ref_index == 999){
      num_of_circles += 1;
    }

    if (ref_index == 999 && num_of_circles % 2 == 0){
      offset_testing += -50;
    }

    if (ref_index == 999 && run_continuously == false){
      Serial.print("time to print once:");Serial.print(running_time - 5, 4); Serial.print(";");
      delay(1000000);
    }

  }
    
  motor_controller1.SetTargetCounts(target_counts_1);
  motor_controller2.SetTargetCounts(target_counts_2);

  // Controller algorithms
  // PID
  double output1 = motor_controller1.pid_controller(encoder_count_volatile_motor1);
  double output2 = motor_controller2.pid_controller(encoder_count_volatile_motor2);

  // Store errors after 5 seconds, up to 5000 samples
  if (running_time >= 10 && error_index < num_samples && !errors_collected) {
    error1_array[error_index] = output1;
    error2_array[error_index] = output2;
    error_index++;

    // After collecting 5000 samples, calculate min, max, and average
    if (error_index == num_samples) {
      errors_collected = true;

      // Initialize min and max values
      min_error1 = error1_array[0];
      max_error1 = error1_array[0];
      min_error2 = error2_array[0];
      max_error2 = error2_array[0];

      // Calculate min, max, and sum
      for (int i = 0; i < num_samples; i++) {
        // For error1
        if (error1_array[i] < min_error1) min_error1 = error1_array[i];
        if (error1_array[i] > max_error1) max_error1 = error1_array[i];
        sum_error1 += abs(error1_array[i]);

        // For error2
        if (error2_array[i] < min_error2) min_error2 = error2_array[i];
        if (error2_array[i] > max_error2) max_error2 = error2_array[i];
        sum_error2 += abs(error2_array[i]);
      }
          // Calculate average
      double avg_error1 = sum_error1 / num_samples;
      double avg_error2 = sum_error2 / num_samples;

      // Print the results
      Serial.println("Collected 5000 error samples:");
      Serial.print("Error1 - Min: "); Serial.print(min_error1);
      Serial.print(", Max: "); Serial.print(max_error1);
      Serial.print(", Average: "); Serial.println(avg_error1);

      Serial.print("Error2 - Min: "); Serial.print(min_error2);
      Serial.print(", Max: "); Serial.print(max_error2);
      Serial.print(", Average: "); Serial.println(avg_error2);

      delay(10000);
    }
  }

  // Print target and position to see the response every print_interval times around the loop
  //interval_count = interval_count + 1;
  //if (interval_count >= print_interval) {
    //Serial.print("ref index:"); Serial.print(ref_index); Serial.println();

    // //old printing
    // Serial.print("motor1 ref; "); Serial.print(th_1[ref_index]);Serial.print(";");
    // Serial.print("motor2 ref; "); Serial.print(th_2[ref_index]);Serial.print(";");
    // Serial.print("Time;"); Serial.print(running_time, 4); Serial.print(";");
    // Serial.print("encoder1;"); Serial.print(encoder_count_volatile_motor1); Serial.print(";");
    // Serial.print("encoder2;"); Serial.print(encoder_count_volatile_motor2);Serial.print(";");
    // //Serial.print("Target_counts_1;"); Serial.print(target_counts_1); Serial.print(";");
    // Serial.print("Error1;"); Serial.print(output1); Serial.print(";");
    // //Serial.print("Target_counts_2;"); Serial.print(target_counts_2); Serial.print(";");
    // Serial.print("Error2;"); Serial.print(output2); Serial.print(";");Serial.println();

    // better formatting printing
    // Serial.print("motor1 ref; "); Serial.print(";");
    // Serial.print("motor2 ref; "); Serial.print(";");
    // Serial.print("Time;"); Serial.print(";");
    // Serial.print("encoder1;"); Serial.print(";");
    // Serial.print("encoder2;"); Serial.print(";");
    // Serial.print("Error1;"); Serial.print(";");
    // Serial.print("Error2;"); Serial.print(";");
    Serial.print("Time;"); Serial.print(";");
    Serial.print(running_time, 4); Serial.print(";");
    // Serial.print(th_1[ref_index]);Serial.print(";");
    // Serial.print(th_2[ref_index] + calibration_pos2);Serial.print(";");
    Serial.print("encoder1;"); Serial.print(";");
    Serial.print(encoder_count_volatile_motor1); Serial.print(";");
    Serial.print("encoder2;"); Serial.print(";");
    Serial.print(encoder_count_volatile_motor2);Serial.print(";");
    Serial.print("Error1;"); Serial.print(";");
    Serial.print(output1); Serial.print(";");
    Serial.print("Error2;"); Serial.print(";");
    Serial.print(output2); Serial.print(";");
    Serial.print(ref_index);
    Serial.println();
  //}

  //Update running time
  running_time += delta_T/1e6;
}




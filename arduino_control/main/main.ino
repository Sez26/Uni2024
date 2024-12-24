// Definitions ----------------------------------------------------------------------------------------------------

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
bool isRunningContinuously = true;

// timestep in microseconds
long timeStep = 1500;
long lastUpdateTime = micros();

int referenceIndex = 0;
double runningTime = 0;
int arrayLength = sizeof(th_1) / sizeof(th_1[0]);

// Targets
int targetCounts1 = 0, targetCounts2 = 0;

// //Calibration and other testing variables
// double prev_counts1 = 0;
// double prev_prev_counts1 = 0;
// double prev_counts2 = 0;
// double prev_prev_counts2 = 0;
// int calibration_pos1=0, calibration_pos2=0;
// int offset_testing = 0, num_of_circles = 0;

// // Arrays to store error values when troubleshootingmain.c is added to loop
// double error1_array[num_samples];
// double error2_array[num_samples];
// const int num_samples = 5000;
// int error_index = 0;      
// bool errors_collected = false;
// double min_error1 = 0, max_error1 = 0, sum_error1 = 0;
// double min_error2 = 0, max_error2 = 0, sum_error2 = 0;

// Instantiate motors
MotorController motor1controller;
MotorController motor2controller;

//Setup ---------------------------------------------------------------------------------
void setup() {
  Serial.begin(230400);  // set baud rate for communication between USB & raspberry pi pico

  motor1controller = MotorController();
  motor1controller.initialiseMotorController(PWM_pin_M1, IN1_M1, IN2_M1);
  pinMode(ENC_A_M1, INPUT);
  pinMode(ENC_B_M1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M1), readEncoder1, RISING); 
  
  motor2controller = MotorController();
  motor2controller.initialiseMotorController(PWM_pin_M2, IN1_M2, IN2_M2);
  pinMode(ENC_A_M2, INPUT);
  pinMode(ENC_B_M2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M2), readEncoder2, RISING);

  motor1controller.configurePIDController(150, 0.45, 3, 100, timeStep/1e6);
  motor2controller.configurePIDController(150, 0.45, 3, 100, timeStep/1e6);
}

//Loop -----------------------------------------------------------------------------------
void loop() {

  do{       } // running empty loop until current time - lastUpdateTime is the length of the desired timestep
  while ((micros() - lastUpdateTime) < timeStep);
  lastUpdateTime = micros();

  if (runningTime >= 5){
    targetCounts1 = th_1[referenceIndex];
    targetCounts2 = th_2[referenceIndex];

    if (referenceIndex == arrayLength - 1 && isRunningContinuously) {
      referenceIndex = 0;  // Loop back to the start if running continuously
      // Otherwise, keep referenceIndex at the last element
    } else {
      referenceIndex++;
    }   
  }
    
  motor1controller.setTargetPosition(targetCounts1);
  motor2controller.setTargetPosition(targetCounts2);

  // PID
  double output1 = motor1controller.pidController(encoder_count_motor1);
  double output2 = motor2controller.pidController(encoder_count_motor2);

  //Update running time
  runningTime += timeStep/1e6;   //Update running time
}




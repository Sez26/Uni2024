// Definitions ----------------------------------------------------------------------------------------------------
#define ON_SWITCH 11    // On switch connect here!!

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

// Button Stuff
volatile bool isOn = false;
const int buttonPin = 16;
unsigned long lastInterruptTime = 0; // Used for debouncing in the interrupt

// timestep in microseconds
long timeStep = 1500;
long lastUpdateTime = micros();

int referenceIndex = 0;
double runningTime = 0;
int arrayLength = sizeof(th_1) / sizeof(th_1[0]);

// Targets
int targetCounts1 = 0, targetCounts2 = 0;

double prev_counts1 = 0;
double prev_prev_counts1 = 0;
double prev_counts2 = 0;
double prev_prev_counts2 = 0;
int calibration_pos1=0, calibration_pos2=0;

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

  while (!Serial) {
    // Wait for Serial to initialise (only necessary on some boards)
  }
  Serial.println("Beginning set up loop.");
  // button set up
  pinMode(buttonPin, INPUT_PULLUP);    // Set button pin as input with pull-up resistor

  // if (digitalRead(buttonPin) == LOW) {
  //   // If button 1 is pressed initially
  //   isOn = true; // Set the system on
  //   // digitalWrite(ledPin, HIGH); // Turn on the LED
  // }

  // Attach interrupt to the button pin, triggering on falling edge (button press)
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleOnOff, FALLING);

  Serial.println("Button functionality added.");

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

  //Set up complete
  Serial.println("Set up complete. Awaiting calibration cue.");

  // // wait for on switch before running loop
  // while (!isOn) {
  //  // Do nothing, keep waiting for the button to be pressed
  // }

  // When the button is pressed, change the state to ON
  isOn = true;
  Serial.println("Calibration cue received. Starting calibration.");

  //Calibration sequence
  calibration_pos1 = motor_calibration(1);  //calibrates motor 1
  calibration_pos2 = motor_calibration(2);  //calibrates motor 2

  Serial.print("calibration position 1 "); Serial.println(calibration_pos1);
  Serial.print("calibration position 2 "); Serial.println(calibration_pos2);
  Serial.print("target 1 "); Serial.println(th_1[referenceIndex]);
  Serial.print("target 2 "); Serial.println(th_2[referenceIndex]);
  Serial.print("new target 1 "); Serial.println(th_1[referenceIndex] + calibration_pos1);
  Serial.print("new target 2 "); Serial.println(th_2[referenceIndex] + calibration_pos2);

  isOn = false;
  Serial.println("Calibration complete. Waiting start cue.");

  // wait for on switch before running loop
  while (!isOn) {
   // Do nothing, keep waiting for the button to be pressed
  }

  // When the button is pressed, change the state to ON
  isOn = true;
  Serial.println("Start cue received. Starting run time in 3 seconds. Button will now act as a kill switch.");
  delay(3000);
}

//Loop -----------------------------------------------------------------------------------
void loop() {

  do{       } // running empty loop until current time - lastUpdateTime is the length of the desired timestep
  while ((micros() - lastUpdateTime) < timeStep);
  lastUpdateTime = micros();

  if (runningTime >= 5){
    targetCounts1 = th_1[referenceIndex] + calibration_pos1;
    targetCounts2 = th_2[referenceIndex] + calibration_pos2;

    // Move to the next reference signal, handling continuous or single-run mode
    if (referenceIndex == arrayLength - 1) {
      isOn = false;
      Serial.println("waiting for button press to go again");
      while (!isOn) {
      // Do nothing, keep waiting for the button to be pressed
        motor1controller.disableMotor();
        motor2controller.disableMotor();
        // Serial.println("System has been stopped.");
        return;
      }
      if (isOn){
        referenceIndex = 0;
      }
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

// Interrupt Service Routine to toggle on/off
void toggleOnOff() {
  // Debounce by checking time since last interrupt
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 50) { // 50 ms debounce time
    isOn = !isOn; // Toggle the on/off state
    // digitalWrite(ledPin, isOn ? HIGH : LOW); // LED reflects state
    lastInterruptTime = currentTime; // Update last interrupt time
  }
}

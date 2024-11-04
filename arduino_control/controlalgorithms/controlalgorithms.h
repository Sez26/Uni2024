#include "Filters.h"  // Ensure you have the correct path
Filters_c filter;

class MotorController_c {
    public:
        MotorController_c() {
            // Constructor, must exist.
        }
    //  Pin setup
    int PWM_PIN = 0;
    int IN_1_PIN = 0;
    int IN_2_PIN = 0;

    // System dynamics
    double M1 = 1;
    double M2 = 1;
    double L1 = 1;
    double L2 = 1;
    double r1 = 0.5;
    double r2 = 0.5;
    double J1 = 0.03;
    double J2 = 0.15;
    double g = 0;

    // Controller constants
    // PID
    double KP = 0;
    double KI = 0;
    double KD = 0; 

    // DFB
    double prev_unfiltered_dfb = 0;
    double prev_filtered_dfb = 0;
    
    double prev_unfiltered_output = 0;
    double previous_filtered_output = 0;

    double delta_T;
    double T_c = 0;

    float target_counts = 0;
    double error = 0;
    double previous_error = 0;
    double prev_counts = 0;
    
    // Frequently altered variables
    int u = 0;
    int u_amplitude = 0;
    int u_sign = 0;
    double integral = 0;
    double integralFlag = 0;

    void SetupMotorController(int pwm_pin, int in_1, int in_2) {
        // Assign local variables to MotorController class attributes
        PWM_PIN = pwm_pin;
        IN_1_PIN = in_1;
        IN_2_PIN = in_2;

        // Set up input and output pins
        pinMode(PWM_PIN, OUTPUT);
        pinMode(IN_1_PIN, OUTPUT);
        pinMode(IN_2_PIN, OUTPUT);
    }

    void SetMotorPower(int dir, int pwmVal) {
        // Set the PWM pin to the appropriate level
        analogWrite(PWM_PIN, pwmVal);

        // Set direction
        if (dir == 1) {
            digitalWrite(IN_1_PIN, HIGH);
            digitalWrite(IN_2_PIN, LOW);
        } else if (dir == -1) {
            digitalWrite(IN_1_PIN, LOW);
            digitalWrite(IN_2_PIN, HIGH);
        } else {
            digitalWrite(IN_1_PIN, LOW);
            digitalWrite(IN_2_PIN, LOW);
        }
    }

    void SetupPIDController(double kp, double ki, double kd, double w_c, double d_t){
        KP = kp;
        KI = ki;
        KD = kd;
        T_c = 1/w_c;
        delta_T = d_t;

        filter.setCutoffTimeConstant(T_c, delta_T);
    }

    void bang_bang_controller(int encoder_count){
        error = target_counts - encoder_count;
        // current position error
        if (error < 0) u = -255;
        else if (error > 0) u = 255;

        u_amplitude = abs(u);
        u_sign = 1;
        if (u < 0){
            u_sign = -1;
        }
        SetMotorPower(u_sign, u_amplitude);
    }

    double pid_controller(int encoder_count){

        error = target_counts - encoder_count;

        int count_toggle = 3;
        // Toggling the integral to only apply when the counts are within 10
        if (abs(error) < count_toggle && abs(previous_error) < count_toggle){
            integralFlag = 1;
        }
        if (abs(error) > count_toggle && abs(previous_error) > count_toggle){
            integralFlag = 0;
            // Not sure about this but want to reset the integral counter when a new ref is given so a left over windup doesnt affect new point
            integral = 0;
        }
        // Reset after each data point timestep
        // Adjust reference for backlash


        integral += (integralFlag * error)*delta_T;
        // Think there are several ways to take the derivative of something in discrete. We did it in fluids last year but can test either to see if there's a difference.
        double derivative = (error - previous_error)/delta_T;
        // double derivative = (error - (2 * previousError) + previousPreviousError) ;

        // Applying low pass filter to pid output
        double raw_output = (KP*error) + (KI*integralFlag*integral) + (KD*derivative);
        double filtered_output = filter.lowpass_leaky_integrator(raw_output, previous_filtered_output);

        prev_unfiltered_output = raw_output;
        previous_filtered_output = filtered_output;

        // Convert theta_dot to pwm in
        double scaled = (filtered_output * 0.0075 * 255) / 9;

        // Determining the direction and amplitude of signal to send to motor
        u_amplitude = abs(scaled);
        u_sign = 1;
        if (scaled < 0){
            u_sign = -1;
        }
        // PWM cannot exceed 255. Saturates at max it if control signal is higher
        //Serial.print("Error; "); Serial.print(error); Serial.print(";");
        //Serial.print("raw output; "); Serial.print(raw_output); Serial.print(";");
       // Serial.print("filtered output; "); Serial.print(filtered_output); Serial.print(";");
        //Serial.print("U u_amplitude; "); Serial.print(scaled); Serial.print(";");
        //Serial.print("theta_dot_des "); Serial.print(filtered_output); Serial.println();
        //Serial.print("kp; "); Serial.print(KP); Serial.print(";");
       //Serial.print("kd;"); Serial.print(KD); Serial.print(";");
        //Serial.print("ki; "); Serial.print(KI); Serial.print(";");
        double saturation = min(u_amplitude, 255);

        SetMotorPower(u_sign, saturation);
        previous_error = error;  
        return error;
    }

    void SetTargetCounts(int counts){
        target_counts = counts;
    }

};
#include "filters.h"
Filter motor_filter;

class MotorController {
    public:
        MotorController() {
            // constructor to initialise motor controller instance
        }

    // pin setup
    int pwm_pin = 0;
    int dir_pin_1 = 0;
    int dir_pin_2 = 0;

    // system dynamics
    double mass1 = 1;
    double mass2 = 1;
    double length1 = 1;
    double length2 = 1;
    double radius1 = 0.5;
    double radius2 = 0.5;
    double inertia1 = 0.03;
    double inertia2 = 0.15;
    double gravity = 0;

    // controller constants
    // pid parameters
    double kp = 0;
    double ki = 0;
    double kd = 0;

    // dfb variables
    double prev_raw_dfb = 0;
    double prev_filtered_dfb = 0;

    double prev_raw_output = 0;
    double prev_filtered_output = 0;

    double time_step = 0;
    double cutoff_time = 0;

    float target_position = 0;
    double position_error = 0;
    double prev_position_error = 0;
    double prev_position = 0;

    // frequently modified variables
    int control_signal = 0;
    int signal_magnitude = 0;
    int signal_direction = 0;
    double integral_sum = 0;
    double integral_enabled = 0;

    void initialiseMotorController(int pwm, int dir1, int dir2) {
        // assign pin parameters to class attributes
        pwm_pin = pwm;
        dir_pin_1 = dir1;
        dir_pin_2 = dir2;

        // configure pins as input or output
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin_1, OUTPUT);
        pinMode(dir_pin_2, OUTPUT);
    }

    void SetMotorPower(int direction, int pwm_value) {
        // write pwm value to the motor control pin
        analogWrite(pwm_pin, pwm_value);

        // set motor direction
        if (direction == 1) {
            digitalWrite(dir_pin_1, HIGH);
            digitalWrite(dir_pin_2, LOW);
        } else if (direction == -1) {
            digitalWrite(dir_pin_1, LOW);
            digitalWrite(dir_pin_2, HIGH);
        } else {
            digitalWrite(dir_pin_1, LOW);
            digitalWrite(dir_pin_2, LOW);
        }
    }

    void configurePIDController(double p_gain, double i_gain, double d_gain, double cutoff_frequency, double step_time) {
        kp = p_gain;
        ki = i_gain;
        kd = d_gain;
        cutoff_time = 1 / cutoff_frequency;
        time_step = step_time;

        motor_filter.configureWithTimeConstant(cutoff_time, time_step);
    }

    double pidController(int encoder_value) {
        position_error = target_position - encoder_value;

        int integral_toggle_threshold = 3;
        // enable integral only when error is small and stable
        if (abs(position_error) < integral_toggle_threshold && abs(prev_position_error) < integral_toggle_threshold) {
            integral_enabled = 1;
        }
        if (abs(position_error) > integral_toggle_threshold && abs(prev_position_error) > integral_toggle_threshold) {
            integral_enabled = 0;
            // reset integral when error exceeds threshold
            integral_sum = 0;
        }

        // update integral term
        integral_sum += (integral_enabled * position_error) * time_step;

        // compute derivative term
        double derivative_term = (position_error - prev_position_error) / time_step;

        // calculate raw output using pid formula
        double raw_pid_output = (kp * position_error) + (ki * integral_enabled * integral_sum) + (kd * derivative_term);

        // apply low-pass filter to raw output
        double filtered_pid_output = motor_filter.applyLowPass(raw_pid_output, prev_filtered_output);

        prev_raw_output = raw_pid_output;
        prev_filtered_output = filtered_pid_output;

        // scale the filtered output to a pwm value
        double scaled_output = (filtered_pid_output * 0.0075 * 255) / 9;

        // determine signal direction and magnitude
        signal_magnitude = abs(scaled_output);
        signal_direction = (scaled_output < 0) ? -1 : 1;

        // saturate pwm at 255
        double saturated_output = min(signal_magnitude, 255);

        SetMotorPower(signal_direction, saturated_output);
        prev_position_error = position_error;

        return position_error;
    }

    void setTargetPosition(int position) {
        target_position = position;
    }

    void disableMotor() {
        signal_magnitude = 0;
        SetMotorPower(signal_direction, signal_magnitude);
    }
};
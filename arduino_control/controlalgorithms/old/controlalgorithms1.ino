class PIDController {
public:
    // PID constants
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float tau = 0;
    float T = 0;
    float limMin = -255;
    float limMax = 255;
    float limMinInt = -50;
    float limMaxInt = 50;

    // PID state variables
    float integrator = 0.0f;
    float prevError = 0.0f;
    float differentiator = 0.0f;
    float prevMeasurement = 0.0f;
    float out = 0.0f;

    void SetupPIDController(double kp, double ki, double kd, double w_c, double d_t){
        KP = kp;
        KI = ki;
        KD = kd;
        T_c = 1/w_c;
        delta_T = d_t;

        filter.setCutoffTimeConstant(T_c, delta_T);
    }

    // PID Update method
    float update(float setpoint, float measurement) {
        float error = setpoint - measurement;

        // Proportional term
        float proportional = Kp * error;

        // Integral term with anti-windup
        integrator += 0.5f * Ki * T * (error + prevError);
        if (integrator > limMaxInt) integrator = limMaxInt;
        else if (integrator < limMinInt) integrator = limMinInt;

        // Derivative term (band-limited differentiator)
        differentiator = -(2.0f * Kd * (measurement - prevMeasurement) +
            (2.0f * tau - T) * differentiator) / (2.0f * tau + T);

        // Output
        out = proportional + integrator + differentiator;

        // Clamp output to limits
        if (out > limMax) out = limMax;
        else if (out < limMin) out = limMin;

        // Store state
        prevError = error;
        prevMeasurement = measurement;

        return out;
    }
};

class MotorController_c {
public:
    // Pin setup
    int PWM_PIN = 0;
    int IN_1_PIN = 0;
    int IN_2_PIN = 0;

    // Controller constants
    float target_counts = 0;
    double error = 0;
    int u = 0;
    int u_amplitude = 0;
    int u_sign = 0;

    PIDController* pid; // Pointer to PID controller

    // Default constructor (no arguments)
    MotorController_c() {
        // Optionally initialize values or leave them as default
    }

    // Parameterized constructor
    MotorController_c(int pwm_pin, int in_1, int in_2, PIDController* pidCtrl) {
        PWM_PIN = pwm_pin;
        IN_1_PIN = in_1;
        IN_2_PIN = in_2;
        pid = pidCtrl;

        pinMode(PWM_PIN, OUTPUT);
        pinMode(IN_1_PIN, OUTPUT);
        pinMode(IN_2_PIN, OUTPUT);
    }

    // Method to set up motor controller pins after default constructor
    void SetupMotorController(int pwm_pin, int in_1, int in_2) {
        PWM_PIN = pwm_pin;
        IN_1_PIN = in_1;
        IN_2_PIN = in_2;

        pinMode(PWM_PIN, OUTPUT);
        pinMode(IN_1_PIN, OUTPUT);
        pinMode(IN_2_PIN, OUTPUT);
    }

    void SetMotorPower(int dir, int pwmVal) {
        analogWrite(PWM_PIN, pwmVal);
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

    void bang_bang_controller(int encoder_count) {
        error = target_counts - encoder_count;
        u = (error < 0) ? -255 : 255;
        SetMotorPower((u < 0) ? -1 : 1, abs(u));
    }

    double pid_controller(int encoder_count) {
        // Update PID controller
        double pid_output = pid->update(target_counts, encoder_count);

        // Convert PID output to motor control signal
        u_amplitude = abs(pid_output);
        u_sign = (pid_output < 0) ? -1 : 1;

        // Saturate the motor control signal to within PWM limits
        double saturation = min(u_amplitude, 255.0);

        SetMotorPower(u_sign, saturation);
        return saturation;
    }

    void SetTargetCounts(float counts) {
        target_counts = counts;
    }
};

// Example usage
void setup() {
    PIDController pid(1.0, 0.01, 0.1, 0.02, 0.1); // Example values for kp, ki, kd, tau, dt
    MotorController_c motor(3, 4, 5, &pid);      // Assign motor control pins and link PID controller
    motor.SetTargetCounts(100);                  // Set desired encoder count

    // Example control loop (replace with actual motor control logic)
    int current_encoder_count = 80;
    motor.pid_controller(current_encoder_count);  // PID control
}

void loop() {
    // Main loop code here
}

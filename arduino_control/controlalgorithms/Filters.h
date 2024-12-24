#include "math.h"

// class to implement low-pass filtering
// when initialised, specify the cutoff time constant

class Filter {
    public:
        Filter() {
            // Constructor to initialise filter instance
        }

    // storing the cutoff properties
    double cutoff_rate = 0; // Cutoff frequency in angular rate (rad/s)
    double time_constant = 0; // Cutoff time constant (seconds)
    // alpha is derived from the cutoff time constant and the sampling interval
    double sampling_interval = 0; // Sampling period (seconds)
    double alpha_coefficient = 0; // Weighting factor for the filter
    double complement_alpha = 0; // To optimise computations, "1 - alpha" is pre-calculated

    // Configures the filter parameters when the cutoff is defined by the time constant
    void configureWithTimeConstant(double Tc, double Ts) {
        cutoff_rate = 1 / Tc; // Convert time constant to cutoff frequency
        time_constant = Tc;
        sampling_interval = Ts;
        alpha_coefficient = sampling_interval / time_constant;
        complement_alpha = 1 - alpha_coefficient;
    }

    // Implements a simple low-pass filter using a leaky integrator approach
    double applyLowPass(double input_signal, double previous_filtered_signal) {
        return (alpha_coefficient * input_signal) + (complement_alpha * previous_filtered_signal);
    }
};

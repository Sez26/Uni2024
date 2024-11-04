#include "math.h"

// A filter class provides methods to implement a low/high pass filter. When it is initalised you pass in the break frequency which will be used for both types. So as an example if you want to use a low and high pass of different break frequencies you will need to implement an instance of this class for each.

class Filters_c {
    public:
        Filters_c(){
            // Constructor, must exist.
        }

    // These could be used interchangeably 
    double cutoff_frequency = 0;
    double cutoff_time_constant = 0;

    // alpha is calculated from the desired break frequency and the timestep. 1-alpha is precomputed and stored to make things quicker
    double sampling_time = 0;
    double alpha = 0;
    double one_minus_alpha = 0;

    // sets all the variables above according to if the break point known in angular frequency
    void setBreakFrequency(double w_c, double T_s){
        cutoff_frequency = w_c;
        cutoff_time_constant = 1/w_c;
        sampling_time = T_s;
        alpha = sampling_time/cutoff_time_constant;
        one_minus_alpha = 1-alpha;
    }

    // sets all the variables above according to if the break point is known in time constant
    void setCutoffTimeConstant(double T_c, double T_s){
        cutoff_frequency = 1/T_c;
        cutoff_time_constant = T_c;
        sampling_time = T_s;
        alpha = sampling_time/cutoff_time_constant;
        one_minus_alpha = 1-alpha;
    }

    // Leaky integrator is a a simple low pass filter often used. Good links to explain it:
        // https://dsp.stackexchange.com/questions/43063/first-order-low-pass-filter
        // https://uk.mathworks.com/help/sps/ref/lowpassfilterdiscreteorcontinuous.html
        // https://dsp.stackexchange.com/questions/3179/is-a-leaky-integrator-the-same-thing-as-a-low-pass-filter
    double lowpass_leaky_integrator(double current_u, double prev_filtered_u){
        return (alpha * current_u) + (one_minus_alpha * prev_filtered_u);
    }

    // Setting high passes as the complement of low pass.
    double highpass_leaky_integrator(double current_u, double prev_filtered_u){
        return (1 - lowpass_leaky_integrator(current_u, prev_filtered_u));
    }

    // I think this is the direct equivalence of a low pass from laplace to disrete - idk so maybe we test
        // https://dsp.stackexchange.com/questions/60277/is-the-typical-implementation-of-low-pass-filter-in-c-code-actually-not-a-typica  
        // 𝑦[𝑛]=(1−𝑎)⋅𝑦[𝑛−1]+𝑎⋅(𝑥[𝑛]+𝑥[𝑛−1])/2  
    double proper_lowpass(double unfiltered_u, double prev_unfiltered_u, double prev_filtered_u){
        return (one_minus_alpha*prev_filtered_u) + (alpha*(unfiltered_u + prev_unfiltered_u)/2);
    }

    double proper_highpass(double unfiltered_u, double prev_unfiltered_u, double prev_filtered_u){
        return (1 - proper_lowpass(unfiltered_u, prev_unfiltered_u, prev_filtered_u));
    }

};

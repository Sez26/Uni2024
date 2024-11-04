#include <StateSpaceControl.h>

/*
 * This example shows how StateSpaceControl can be used to control the position of a DC motor.
 * The model being used comes from this analysis:
 * http://ctms.engin.umich.edu/CTMS/index.php?example=MotorPosition&section=SystemModeling which defines the state as: x
 * = [angle angular_velocity current]^T
 */

// Start by defining a state space model, This particular model describes a DC motor but you can define any model by
// just declaring a Model object and then filling out the state matrices. See Model.h for examples on how to do this.
MotorPositionModel model(0.01, 0.1, 0.01, 1, 0.5);

// Next define a state space controller. The motor position model uses 3 states and 1 input so we'll need to specify
// these inside the <> brackets when declaring the controller
StateSpaceController<3, 1> controller(model);

// Lastly, since the controller isn't actually connected to a motor, we'll need to simulate one to show how the
// controller works. The Simulation class handles this by accepting the control inputs generated by the controller (u)
// and returning observations from the motor (y).
Simulation<3, 1> sim(model);

Matrix<3> y;
const float dt = 0.01;

void setup()
{
    Serial.begin(115200);

    // To parameterise the controller, in this case we'll just need to fill out the control law matrix K.
    // K defines feedback gains which are loosely similar to PID gains. If you're wondering where I pulled these
    // numbers from, head over to TuneThoseGains.ipynb
    controller.K = {9.75, 0.97, 0.40};

    // Once the system and gain matrices are filled in, the controller needs to be initialised so that it can
    // precalculate Nbar
    controller.initialise();

    // Set a target motor position of 2.2 radians
    controller.r(0) = 2.2;
}

// Now we can start the control loop
void loop()
{
    // Firstly generate some measurements from the simulator. Since this model assumes that the entire state can be
    // observed y is the same size as x and contains measurements for motor angle, velocity and current. If we were
    // controlling an actual motor, these observations would measured from physical sensors attached to the motor.
    y = sim.step(controller.u, dt);

    // Now update the state space controller, which causes it to update its u member (the control input). When
    // controlling an actual system, the updated control input would be used to set the motor voltage.
    controller.update(y, dt);

    // Print the current system output to serial. If the controller is doing its job properly then y should settle at
    // whatever r was set to after a short transient.
    Serial << "angle = " << y(0) << " velocity = " << y(1) << " current = " << y(2) << '\n';

    delay(dt * 1000);
}
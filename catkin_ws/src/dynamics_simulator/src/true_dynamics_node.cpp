#include <ros/ros.h>
#include <dynamics_simulator/true_dynamics.h>

#include <string.h>
#include <stdlib.h>
#include <math.h>

// Global constants
float A = 3.00;  // Assume cross section area is 3.00m^2
float C = 0.35;
float P = 1.225;  // 1.225g/L air density at 15 degrees Celsius
float M = 120.00;  // 100kg + 20kg = 120kg
float Fup = 1500.00;
float G = 9.81; // Acceleration of gravity

// Starting Conditions for X coordinate calculations
float startingV = 0;
float startingD = 0;
bool newConditions = false;

// Acceleration functions
float getXAcceleration(float z) {
    float Vw;
    if (z<=1000) {
        Vw = -5.00;  // Wind velocity 1000m and below
    } else {
        Vw = 5.00;  // Wind velocity above 1000m
        z -= 1000;
    }

    return ((Vw*C*P*A*exp((-C*P*A*z) / (2*M))) / (2*M)) / G;
}

float getZAcceleration(float time) {
    return (((Fup - M*G)*pow(1.0 / (cosh((sqrt(Fup - M*G)*sqrt(0.5*C*P*A)*time) / M)), 2)) / M) / G;
}

// Velocity functions
float xVelocityFunction(float z) {
    // Constants
    float Vw;
    if (z<=1000) {
        Vw = -5.00;  // Wind velocity 1000m and below
    } else {
        Vw = 5.00;  // Wind velocity above 1000m

        z -= 1000;
    }

    float Vx = -Vw*exp((-C*P*A*z)/(2*M)) + Vw + startingV;

    return Vx;
}

float zVelocityFunction(float time) {
    return (sqrt(Fup - M*G) * tanh((sqrt(Fup - M*G) * sqrt(0.5*C*P*A) * time) / M)) / (sqrt(0.5*C*P*A));
}

// Position functions
float getZCoordinate(float time) {
    float z = (M*log(cosh((sqrt(Fup - M*G) * sqrt(0.5*C*P*A) * time) / M))) / (0.5*C*P*A);
    return z;
}

float getXCoordinate(float z, float time) {
    // Initial conditions
    float Vw;
    if (z<=1000) {
        Vw = -5.00;  // Wind velocity 1000m and below
    } else {
        Vw = 5.00;  // Wind velocity above 1000m

        if (!newConditions) {
            // Conditions above 1000m
            float lastZCoordinate = getZCoordinate(time-0.01);
            startingD = getXCoordinate(lastZCoordinate, time-0.01);
            startingV = xVelocityFunction(lastZCoordinate);
            newConditions = true;
        }

        z -= 1000;
    }

    float x = Vw*exp((-C*P*A*z)/(2*M))*((2*M)/(C*P*A)) + Vw*z - Vw*((2*M) / (C*P*A)) + startingV * z + startingD;

    return x;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamics_simulation_node");
    ros::NodeHandle n;

    std::string publisher_topic = "true_dynamics";

    ros::Publisher dynamics_publisher = n.advertise<dynamics_simulator::true_dynamics>(publisher_topic, 1000);

    dynamics_simulator::true_dynamics msg;

    float simulationTime = 0.0000000;
    int simulationCycles = 0;

    float coordinateZ;
    float coordinateX;

    // Update at rate of 1Hz for start time count down
    ros::Rate delay_start_rate(1);
    while (ros::ok() && simulationCycles < 10) {
        ROS_INFO("Simulation starting in: %i", 10 - simulationCycles);
        simulationCycles++;
        delay_start_rate.sleep();
    }

    simulationCycles = 0;

    // Update at rate of 1Hz for simulation
    ros::Rate loop_rate(100);
    while (ros::ok() && coordinateZ < 2000) {
        // Calculate coordinates
        coordinateZ = getZCoordinate(simulationTime);
        coordinateX = getXCoordinate(coordinateZ, simulationTime);
        float velocityX = xVelocityFunction(coordinateZ);
        float velocityZ = zVelocityFunction(simulationTime);
        float accelerationX = getXAcceleration(coordinateZ);
        float accelerationZ = getZAcceleration(simulationTime);

        msg.xPosition = coordinateX;
        msg.zPosition = coordinateZ;
        msg.xVelocity = velocityX;
        msg.zVelocity = velocityZ;
        msg.zAcceleration = accelerationX;
        msg.zAcceleration = accelerationZ;
        msg.time = simulationTime;

        dynamics_publisher.publish(msg);

        // Print simulation status every 2 seconds in simulation time
        if (simulationCycles % 200 == 0) {
            ROS_INFO("Simulation time: %f \t (Current position: Z: %f, X: %f)", simulationTime, coordinateZ, coordinateX);
            simulationCycles = 0;
        }

        ros::spinOnce();

        loop_rate.sleep();

        simulationTime+=0.01;
        simulationCycles++;
    }

    return 0;
}




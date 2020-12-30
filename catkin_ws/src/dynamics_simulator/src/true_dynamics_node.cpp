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

float xVelocityFunction(float z) {
    // Constants
    float Vw = -5.00;  // Wind velocity 1000m and below

    float Vx = -Vw*exp((-C*P*A*z)/(2*M)) + Vw;

    return Vx;
}

float getZCoordinate(int time) {
    float z = (M*log(cosh((sqrt(Fup - M*G) * sqrt(0.5*C*P*A) * time) / M))) / (0.5*C*P*A);
    return z;
}

float getXCoordinate(float z, int time) {
    // Initial conditions
    float Vw;
    if (z<=1000) {
        Vw = -5.00;  // Wind velocity 1000m and below
    } else {
        Vw = 5.00;  // Wind velocity above 1000m
    }

    if (z>1000) {
        if (!newConditions) {
            // Conditions above 1000m
            float lastZCoordinate = getZCoordinate(time-1);
            startingD = getXCoordinate(lastZCoordinate, time-1);
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

    int simulationTime = 0;

    float coordinateZ;
    float coordinateX;

    // Update at rate of 1Hz for start time count down
    ros::Rate delay_start_rate(1);
    while (ros::ok() && simulationTime < 9) {
        ROS_INFO("Simulation starting in: %i", 10 - simulationTime);
        simulationTime++;
        delay_start_rate.sleep();
    }

    simulationTime = 0;

    // Update at rate of 1Hz for simulation
    ros::Rate loop_rate(10);
    while (ros::ok() && coordinateZ < 2000) {
        // Calculate coordinates
        coordinateZ = getZCoordinate(simulationTime);
        coordinateX = getXCoordinate(coordinateZ, simulationTime);

        msg.x = coordinateX;
        msg.z = coordinateZ;
        msg.time = simulationTime;

        dynamics_publisher.publish(msg);

        // Print simulation status every 2 seconds in simulation time
        if (simulationTime % 1 == 0) {
            ROS_INFO("Simulation time: %i \t (Current position: Z: %f, X: %f)", simulationTime, coordinateZ, coordinateX);
        }

        ros::spinOnce();

        loop_rate.sleep();

        simulationTime++;
    }

    return 0;
}




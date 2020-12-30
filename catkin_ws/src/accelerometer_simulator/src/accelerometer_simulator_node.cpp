#include <ros/ros.h>
#include <dynamics_simulator/true_dynamics.h>

#include <string.h>
#include <stdlib.h>
#include <random>
#include <iostream>
#include <math.h>

// Make publisher global, so that it can be used in callback function
ros::Publisher accelerometer_publisher;

// Global constants
float A = 3.00;  // Assume cross section area is 3.00m^2
float C = 0.35;
float P = 1.225;  // 1.225g/L air density at 15 degrees Celsius
float M = 120.00;  // 100kg + 20kg = 120kg
float Fup = 1500.00;
float G = 9.81; // Acceleration of gravity

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

// Define random generator with Gaussian distribution
const double mean = 0.0;
const double stddev = 0.1;
std::default_random_engine generatorX;
std::default_random_engine generatorZ;
std::normal_distribution<double> distX(mean, stddev);
std::normal_distribution<double> distZ(mean, stddev);

dynamics_simulator::true_dynamics generateNoise(const dynamics_simulator::true_dynamics& msg) {
    // Get acceleration values without noise
    float x = getXAcceleration(msg.z);
    float z = getZAcceleration(msg.time);

    // Add Gaussian noise
    dynamics_simulator::true_dynamics noiseMsg;
    noiseMsg.x = x + distX(generatorX);
    noiseMsg.z = z + distZ(generatorZ);
    noiseMsg.time = msg.time;

    return noiseMsg;
}

void coordinatesCallback(const dynamics_simulator::true_dynamics& msg) {
    accelerometer_publisher.publish(generateNoise(msg));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "accelerometer_simulator_node");
    ros::NodeHandle n;

    std::string publisher_topic = "accelerometer_sensor";
    accelerometer_publisher = n.advertise<dynamics_simulator::true_dynamics>(publisher_topic, 1000);

    std::string subscriber_topic = "/true_dynamics";
    ros::Subscriber dynamics_subscriber = n.subscribe(subscriber_topic, 1000, coordinatesCallback);

    // Update at rate of 40Hz
    ros::Rate loop_rate(40);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}




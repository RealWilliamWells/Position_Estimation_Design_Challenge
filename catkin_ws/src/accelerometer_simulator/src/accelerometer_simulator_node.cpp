#include <ros/ros.h>
#include <dynamics_simulator/true_dynamics.h>

#include <string.h>
#include <stdlib.h>
#include <random>
#include <iostream>
#include <math.h>

// Make publisher global, so that it can be used in callback function
ros::Publisher accelerometer_publisher;

// Define random generator and Gaussian distribution
const double mean = 0.0;
const double stddev = 0.1;
std::default_random_engine generatorX;
std::default_random_engine generatorZ;
std::normal_distribution<double> distribution(mean, stddev);

dynamics_simulator::true_dynamics generateNoise(const dynamics_simulator::true_dynamics& msg) {
    // Get acceleration values without noise
    float x = msg.xAcceleration;
    float z = msg.zAcceleration;

    // Add Gaussian noise
    dynamics_simulator::true_dynamics noiseMsg;
    noiseMsg.xAcceleration = x + distribution(generatorX);
    noiseMsg.zAcceleration = z + distribution(generatorZ);
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




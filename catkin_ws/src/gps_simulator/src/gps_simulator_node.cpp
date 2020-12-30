#include <ros/ros.h>
#include <dynamics_simulator/true_dynamics.h>

#include <string.h>
#include <stdlib.h>
#include <random>
#include <iostream>

// Make publisher global, so that it can be used in callback function
ros::Publisher gps_publisher;

// Define random generator with Gaussian distribution
const double mean = 0.0;
const double stddev = 60.0;
std::default_random_engine generator;
std::normal_distribution<double> dist(mean, stddev);

dynamics_simulator::true_dynamics generateNoise(const dynamics_simulator::true_dynamics& msg) {
    // Add Gaussian noise
    dynamics_simulator::true_dynamics noiseMsg;
    noiseMsg.x = msg.x + dist(generator);
    noiseMsg.z = msg.z + dist(generator);
    noiseMsg.time = msg.time;

    return noiseMsg;
}

void coordinatesCallback(const dynamics_simulator::true_dynamics& msg) {
    gps_publisher.publish(generateNoise(msg));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_simulator_node");
    ros::NodeHandle n;

    std::string publisher_topic = "gps_sensor";
    gps_publisher = n.advertise<dynamics_simulator::true_dynamics>(publisher_topic, 1000);

    std::string subscriber_topic = "/true_dynamics";
    ros::Subscriber dynamics_subscriber = n.subscribe(subscriber_topic, 1000, coordinatesCallback);

    // Update at rate of 1Hz
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}




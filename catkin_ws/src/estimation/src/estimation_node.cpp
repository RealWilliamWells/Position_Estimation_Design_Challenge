#include <ros/ros.h>
#include <dynamics_simulator/true_dynamics.h>

#include <string.h>
#include <stdlib.h>
#include <random>
#include <iostream>
#include <math.h>

// Make publisher global, so that it can be used in callback function
ros::Publisher estimation_publisher;

// Global constants
float A = 3.00;  // Assume cross section area is 3.00m^2
float C = 0.35;
float P = 1.225;  // 1.225g/L air density at 15 degrees Celsius
float M = 120.00;  // 100kg + 20kg = 120kg
float Fup = 1500.00;
float G = 9.81; // Acceleration of gravity

float PositionUpdate(float mean1, float variance1, float mean2, float variance2) {
    return (variance2*mean1 + variance1*mean2) / (variance2+variance1);
}

float varianceUpdate(float variance1, float variance2) {
    return 1/(1 / variance1 + 1 / variance2);
}

float meanPredict(float mean1, float mean2) {
    return mean1 + mean2;
}

float variancePredict(float variance1, float variance2) {
    return variance1 + variance2;
}

// Initial parameters
const float measurement_variance = pow(60,2);
const float motion_variance = pow(0.1,2);
float mean = 0;
float variance = 10000000; // Start with high number because of initial high uncertainty

// Filter for one direction
float kalmanFilter(float measurement, float motion) {
    // Update values
    mean = meanUpdate(mean, variance, motion, measurement_variance);
    variance = varianceUpdate(variance, measurement_variance);

    // Predict
    mean = meanPredict(mean, measurement);
    variance = variancePredict(variance, motion_variance);

    return mean;
}

// Data from sensors management
bool receivedGPS = false;
dynamics_simulator::true_dynamics gps_msg;
dynamics_simulator::true_dynamics accelerometer_msg;

dynamics_simulator::true_dynamics estimated_msg;

// SensorID is 0 for GPS sensor, SensorID is 1 for accelerometer sensor
void gpsCallback(const dynamics_simulator::true_dynamics& msg) {
    if (!receivedGPS) {
        gps_msg = msg;
        receivedGPS = true;
    }
}

void accelerometerCallback(const dynamics_simulator::true_dynamics& msg) {
    if (receivedGPS) {
        accelerometer_msg = msg;

        std::cout<<"TIME: "<<gps_msg.time<<" A: "<<accelerometer_msg.time<<"\n";

        // X direction
        estimated_msg.x = kalmanFilter(gps_msg.x, accelerometer_msg.x);
        estimated_msg.time = accelerometer_msg.time;

        estimation_publisher.publish(estimated_msg);

        receivedGPS = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "accelerometer_simulator_node");
    ros::NodeHandle n;

    std::string publisher_topic = "position_estimation";
    estimation_publisher = n.advertise<dynamics_simulator::true_dynamics>(publisher_topic, 1000);

    std::string gps_subscriber_topic = "/gps_sensor";
    ros::Subscriber gps_subscriber = n.subscribe(gps_subscriber_topic, 1000, gpsCallback);

    std::string accelerometer_subscriber_topic = "/accelerometer_sensor";
    ros::Subscriber accelerometer_subscriber = n.subscribe(accelerometer_subscriber_topic, 1000, accelerometerCallback);

    // Update at rate of 1Hz
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}




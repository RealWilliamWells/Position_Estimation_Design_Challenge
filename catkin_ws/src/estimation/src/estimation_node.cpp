#include <ros/ros.h>
#include <dynamics_simulator/true_dynamics.h>

#include <string.h>
#include <stdlib.h>
#include <random>
#include <iostream>
#include <math.h>
#include <eigen/Eigen/Dense>

// Make publisher global, so that it can be used in callback function
ros::Publisher estimation_publisher;

// Initial parameters for filter
float lastTime = 0;

Eigen::MatrixXf I(2,2);

Eigen::MatrixXf processVariance(2,2);

Eigen::MatrixXf measureVariance(2,2);

// Initial state
Eigen::MatrixXf stateX(2,1);

Eigen::MatrixXf H(2,2);

// Initial error covariance
Eigen::MatrixXf errorCovarianceX(2,2);

// Equations for filter predict stage
Eigen::MatrixXf getStateTransitionMatrix(float deltaTime) {
    Eigen::MatrixXf stateTransition(2,2);
    stateTransition(0,0) = 1.00;
    stateTransition(0,1) = deltaTime;
    stateTransition(1,0) = 0.00;
    stateTransition(1,1) = 1;

    return stateTransition;
}

Eigen::MatrixXf getControlMatrix(float deltaTime) {
    Eigen::MatrixXf controlMatrix(2,1);
    controlMatrix(0,0) = 0.5 * pow(deltaTime, 2);
    controlMatrix(1,0) = deltaTime;

    return controlMatrix;
}

Eigen::MatrixXf getControlVector(float value) {
    Eigen::MatrixXf controlVector(1,1);
    controlVector(0,0) = value;

    return controlVector;
}

void predictState(Eigen::MatrixXf controlMatrix, Eigen::MatrixXf controlVector, Eigen::MatrixXf stateTransition) {
    stateX = stateTransition * stateX + controlMatrix * controlVector;
}

void predictErrorCovariance(Eigen::MatrixXf stateTransition) {
    errorCovarianceX = stateTransition * errorCovarianceX * stateTransition.transpose() + processVariance;
}

void kalmanPredict(float deltaTime, float acceleration) {
    Eigen::MatrixXf stateTransition = getStateTransitionMatrix(deltaTime);
    Eigen::MatrixXf controlMatrix = getControlMatrix(deltaTime);
    Eigen::MatrixXf controlVector = getControlVector(acceleration);

    predictState(controlMatrix, controlVector, stateTransition);

    predictErrorCovariance(stateTransition);

    ROS_INFO("Test0");
}

// Equations for filter update stage
Eigen::MatrixXf getMeasurementVector(float position, float velocity) {
    Eigen::MatrixXf measurementVector(2,1);
    measurementVector(0,0) = position;
    measurementVector(1,0) = velocity;

    return measurementVector;
}

Eigen::MatrixXf getKalmanGain() {
    return errorCovarianceX * H.transpose() * (H * errorCovarianceX * H.transpose() + measureVariance).inverse();
}

void updateEstimate(Eigen::MatrixXf kalmanGain, Eigen::MatrixXf measurementVector) {
    stateX = stateX + kalmanGain * (measurementVector - H*stateX);
}

void updateErrorCovariance(Eigen::MatrixXf kalmanGain) {
    errorCovarianceX = (I - kalmanGain * H) * errorCovarianceX;
}

void kalmanUpdate(float position, float velocity) {
    Eigen::MatrixXf kalmanGain = getKalmanGain();
    Eigen::MatrixXf measurementVector = getMeasurementVector(position, velocity);

    updateEstimate(kalmanGain, measurementVector);

    ROS_INFO("Test1");

    updateErrorCovariance(kalmanGain);

    ROS_INFO("Test2");
}

// Main kalman function
void kalmanFilter(float position, float velocity, float acceleration, float time) {
    float deltaTime = time - lastTime;

    kalmanPredict(deltaTime, acceleration);

    kalmanUpdate(position, velocity);

    lastTime = time;
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

        // X direction
        kalmanFilter(gps_msg.xPosition, gps_msg.xVelocity, accelerometer_msg.xPosition, accelerometer_msg.time);
        estimated_msg.xPosition = stateX(0,0);
        estimated_msg.time = accelerometer_msg.time;

        estimation_publisher.publish(estimated_msg);

        receivedGPS = false;
    }
}

int main(int argc, char **argv) {
    // Initial parameters for filter
    I(0,0) = 1.00;
    I(0,1) = 0.00;
    I(1,0) = 0.00;
    I(1,1) = 1.00;

    processVariance(0,0) = pow(0.1,2);
    processVariance(0,1) = 0.00;
    processVariance(1,0) = 0.00;
    processVariance(1,1) = pow(0.1,2);


    measureVariance(0,0) = pow(60.00,2);
    measureVariance(0,1) = 0.00;
    measureVariance(1,0) = 0.00;
    measureVariance(1,1) = pow(60.00,2);

    Eigen::MatrixXf stateX(2,1);
    stateX(0,0) = 0.00; // Position
    stateX(1,0) = 0.00; // Velocity

    H(0,0) = 1.00;
    H(0,1) = 0.00;
    H(1,0) = 0.00;
    H(1,1) = 1.00;

    // Initial error covariance
    errorCovarianceX(0,0) = 1000.00;
    errorCovarianceX(0,1) = 0.00;
    errorCovarianceX(1,0) = 0.00;
    errorCovarianceX(1,1) = 1000.00;

    // Initialize node
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




#include "main.h"
#include "lemlib/api.hpp"
#include "Eigen/Eigen"
#include "config.hpp"

float trackWidth = 10.2;
float wheelDiameter = 3.25;



void moveChassis(float linearVelocity, float angularVelocity) {
    // compute left and right velocities
    float leftVelocity = (2 * linearVelocity - angularVelocity * trackWidth) / 2; // inches/sec
    float rightVelocity = (2 * linearVelocity + angularVelocity * trackWidth) / 2; // inches/sec

    // calculate left and right wheel rpm
    float leftRPM = leftVelocity * 60.0 / (wheelDiameter * M_PI); // rpm
    float rightRPM = rightVelocity * 60.0 / (wheelDiameter * M_PI); // rpm

    // calculate the left and right motor rpm
    float leftMotorRPM = leftRPM * (60.0 / 36.0);
    float rightMotorRPM = rightRPM * (60.0 / 36.0);

    // move chassis
    leftDrive.move_velocity(leftMotorRPM);
    rightDrive.move_velocity(rightMotorRPM);
}


/**
 * @brief Ramsete controller
 *
 * @param targetPose the target pose (inches, radians)
 * @param currentPose the current pose (inches, radians)
 * @param targetAngularVelocity the target angular velocity (radians/sec)
 * @param targetLateralVelocity the target lateral velocity (inches/sec)
 * @param beta Proportional gain. 0 <= beta
 * @param zeta Damping factor. 0.0 = no damping, 1.0 = critical damping. 0 <= beta <= 1
 */
void ramsete(lemlib::Pose targetPose, lemlib::Pose currentPose, float targetAngularVelocity, float targetLinearVelocity, float beta, float zeta) {
    // compute global error
    Eigen::MatrixXd globalError(1, 3);
    globalError <<
        targetPose.x - currentPose.x,
        targetPose.y - currentPose.y,
        targetPose.theta - currentPose.theta;

    // compute transformation matrix
    Eigen::MatrixXd transformationMatrix(3, 3);
    transformationMatrix <<
        cos(currentPose.theta),  sin(currentPose.theta), 0,
        -sin(currentPose.theta), cos(currentPose.theta), 0,
        0,                       0,                      1;

    // compute local error
    Eigen::MatrixXd localError = globalError * transformationMatrix;
    // compute k gain
    float k = 2 * zeta * std::sqrt(targetAngularVelocity * targetAngularVelocity + beta + targetLinearVelocity * targetLinearVelocity);
    // compute angular velocity
    float angularVelocity = targetAngularVelocity * cos(localError(0, 2)) + k * localError(0, 0);
    // compute linear velocity
    float linearVelocity = targetLinearVelocity + k * localError(0, 2) + (beta * linearVelocity * sin(localError(0, 2)) * localError(0, 1) / localError(0, 2));

    // move chassis
    moveChassis(linearVelocity, angularVelocity);
}


void MoveToPath(){

}


void initialize() {
	pros::lcd::initialize();
    std::string text;
    std::ifstream file("index.txt");
    std::map<

    while (std::getline(file, text)){
        Point one = parseString(text);
        list.push_back(one);
    }

    // Close the file
    file.close();
    desPath1.points = list;
}

void autonomous() {

}

void opcontrol() {


}



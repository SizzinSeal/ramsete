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

int findClosest(lemlib::Pose pose, std::vector<lemlib::Pose>* pPath, int prevCloseIndex=0) {
    //Find the closest point to the robot
    int closeIndex = 0;
    float minDistance = INT_MAX;
    for(int i = prevCloseIndex; i<pPath->size(); i++){
        float dist = pose.distance(pPath->at(i));
        if(dist < minDistance){
            closeIndex = i;
            minDistance = dist;
        }
    }
    return closeIndex;
}

/**
 * @brief Follow a precalculated path using the Ramsete controller
 *
 * @param pPath pointer to the path object with velocities
 * @param timeOut longest time the robot can spend moving
 * @param errorRange how close the robot must be to the desired point before stopping
 * @param beta Proportional gain. 0 <= beta (Ramsete Controller)
 * @param zeta Damping factor. 0.0 = no damping, 1.0 = critical damping. 0 <= beta <= 1 (Ramsete Controller)
 * @param reversed whether the robot should turn in the opposite direction. false by default
 * 
 */
void FollowPath(std::vector<lemlib::Pose>* pPath, float timeOut, float errorRange, float beta, float zeta, bool reversed = false){
    float offFromPose = INT_MAX;
    
    // set up the timer
    timeOut *= CLOCKS_PER_SEC;
    clock_t startTime = clock(); 
    float runtime = 0;

    // initialise loop variables
    int prevCloseIndex=0;
    lemlib::Pose pose(0,0,0);

    // keep running the controller until either time expires or the bot is within the error range
    while(runtime <= timeOut && offFromPose >= errorRange){
        // update runtime
        runtime = clock() - startTime;

        // update pose
        // *****ASK LIAM HOW TO UPDATE POSE USING LEMLIB******


        // find the closest index
        int closeIndex = findClosest(pose, pPath, prevCloseIndex);

        // get the closest pose velocities
        lemlib::Pose closestPose = pPath->at(closeIndex);
        float targetAngularVelocity = closestPose.angularVel;
        float targetLinearVelocity = closestPose.linearVel;
        
        // set the desired pose to one ahead (so the robot is always moving forward) *****TEST******
        int targetIndex = std::min(closeIndex+1, (int)pPath->size()-1); // ensures no out of range error
        lemlib::Pose targetPose = pPath->at(targetIndex);

        // run the controller function
        ramsete(targetPose, pose, targetAngularVelocity, targetLinearVelocity, beta, zeta);

        pros::delay(20);
    }
}



void initialize() {
	pros::lcd::initialize();
    // std::string text;
    // std::ifstream file("index.txt");
    // while (std::getline(file, text)){
    //     Point one = parseString(text);
    //     list.push_back(one);
    // }

    // // Close the file
    // file.close();
    // desPath1.points = list;
}

void autonomous() {

}

void opcontrol() {


}



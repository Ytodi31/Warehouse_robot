/**
 *BSD 3-Clause License
 *
 *Copyright (c) 2019, Yashaarth Todi
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file pidController.cpp
 * @brief This file provides the implementation of the Controller module
 *
 * This project contains the execution to navigate Turtlebot3 in a warehouse
 * environment using A star path planning, picks up a package and drops it in
 * user defined postitions. Turtebot3 uses OpenManipulator to perform thus
 * pick-place task.
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Suyash Yeotikar
 * @author Gautam Balachandran
 * @author Yashaarth Todi
 *
 * @date 11-28-2019
 */

#include <math.h>
#include <geometry_msgs/Twist.h>
#include "pidController.hpp"

/**
 * @brief Setter method for the Ros Node
 * @param New Node to be set
 * @return none
 */
void PidController::setControllerNode(ros::NodeHandle n) {
  PidController::controllerNode = n;
}

/**
 * @brief Setter method for the velocity publisher
 * @param New velocity publisher to be set
 * @return none
 */
void PidController::setVelocityPub() {
  velocityPub = controllerNode.advertise<geometry_msgs
          ::Twist>("/om_with_tb3/cmd_vel", 100);
}

/**
 * @brief Setter method for the pose subscriber
 * @param New pose subscriber to be set
 * @return none
 */
void PidController::setPoseSub() {
  poseSub = controllerNode.subscribe("/rawPose", 100,
          &PidController::distCallBack, this);
}
/**
 * @brief Getter method for the pose
 * @param none
 * @return The current pose of the turtlebot
 */
tf::Pose PidController::getPose() {
  return PidController::pose;
}

/**
 * @brief Getter method for the linear velocity
 * @param none
 * @return The current linear velocity of the turtlebot
 */
double PidController::getLinearVel() {
  return PidController::linearVel;
}

/**
 * @brief Getter method for the angular velocity
 * @param none
 * @return The current angular velocity of the turtlebot
 */
double PidController::getAngularVel() {
  return PidController::angularVel;
}

/**
 * @brief Getter method for the Propotional gain
 * @param none
 * @return The current propotional gain of the controller
 */
std::vector<double> PidController::getKP() {
  return PidController::kP;
}
/**
 * @brief Setter method for the propotional gain
 * @param New propotional gain to be set
 * @return none
 */
void PidController::setKP(double kP_linear, double kP_angular) {
  kP.push_back(kP_linear);
  kP.push_back(kP_angular);
}
/**
 * @brief Getter method for the derivative gain
 * @param none
 * @return The current derivative gain of the controller
 */
std::vector<double> PidController::getKD() {
    return kD;
}
/**
 * @brief Setter method for the derivative gain
 * @param New derivative gain to be set
 * @return none
 */
void PidController::setKD(double kD_linear, double kD_angular) {
  kD.push_back(kD_linear);
  kD.push_back(kD_angular);
}
/**
 * @brief Getter method for the integral gain
 * @param none
 * @return The current integral gain of the controller
 */
std::vector<double> PidController::getKI() {
  return PidController::kI;
}
/**
 * @brief Setter method for the integral gain
 * @param New integral gain to be set
 * @return none
 */
void PidController::setKI(double kI_linear, double kI_angular) {
  kI.push_back(kI_linear);
  kI.push_back(kI_angular);
}
/**
 * @brief Callback function to get the pose data from the Turtlebot
 * @param Turtlebot Pose
 * @return none
 */
void PidController::distCallBack(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
    pose.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y,
            msg->pose.position.z));
    pose.setRotation(tf::Quaternion(msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z, msg->pose.orientation.w));
    return;
}
/**
 * @brief Mock of the euclidean distance calculator
 * @param Current pose and desired pose
 * @return Calculated distance
 */
double PidController::euclideanDist(tf::Pose currentPose,
                                    tf::Pose desiredPose) {
    double dist = 0;
    dist = currentPose.getOrigin().distance(desiredPose.getOrigin());
    return dist;
}

/**
 * @brief Mock method to calculate the linear and angular velocity for the robot
 * @param Current pose and desired pose
 * @return Calculated linear and angular velocity
 */
void PidController::calcVel(tf::Pose currentPose, tf::Pose desiredPose) {
    double dist = euclideanDist(currentPose, desiredPose);
    double linearError = dist;
    double angularDesired = atan2(desiredPose.getOrigin().y() -
                                currentPose.getOrigin().y(),
                                 desiredPose.getOrigin().x() -
                                 currentPose.getOrigin().x()) + M_PI;
    ros::start();
    tf::Matrix3x3 rotMat(currentPose.getRotation());
    double roll, pitch, yaw;
    rotMat.getRPY(roll, pitch, yaw);
    double angularError;
    angularError = yaw - angularDesired;
    ROS_DEBUG_STREAM("angular Error: " << angularError);
    double linearErrorDiff;
    linearErrorDiff = linearError - lastLinearError;
    double angularErrorDiff;
    angularErrorDiff = angularError - lastAngularError;
    linearVel = kP[0] * linearError + kI[0] * sumLinearError +
                kD[0] * linearErrorDiff;
    angularVel = kP[1] * angularError + kI[1] * sumAngularError +
                 kD[1] * angularErrorDiff;
    geometry_msgs::Twist msg;
    if ( linearVel > linearVelThreshold ) {
        linearVel = linearVelThreshold;
    } else if ( linearVel < -linearVelThreshold ) {
        linearVel = -linearVelThreshold;
    }
    if ( angularVel > angularVelThreshold ) {
        angularVel = angularVelThreshold;
    } else if ( angularVel < -angularVelThreshold ) {
        angularVel = -angularVelThreshold;
    }
    std::cout << "Angular vel: " << angularVel;
    msg.linear.x = linearVel;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angularVel;
    velocityPub.publish(msg);
    lastAngularError = angularError;
    lastLinearError = linearError;
    sumLinearError = sumLinearError + linearError;
    sumAngularError = sumAngularError + angularError;
}

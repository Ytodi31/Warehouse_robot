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

#include "pidController.hpp"

/**
 * @brief Getter method for the Ros Node
 * @param none
 * @return The current node handle for the controller
 */
ros::NodeHandle PidController::getControllerNode() {
  return PidController::controllerNode;
}
/**
 * @brief Setter method for the Ros Node
 * @param New Node to be set
 * @return none
 */
void PidController::setControllerNode(ros::NodeHandle n) {
  PidController::controllerNode = n;
}
/**
 * @brief Getter method for the velocity publisher
 * @param none
 * @return The current velocity publisher
 */
ros::Publisher PidController::getVelocityPub() {
  return PidController::velocityPub;
}
/**
 * @brief Setter method for the velocity publisher
 * @param New velocity publisher to be set
 * @return none
 */
void PidController::setVelocityPub(ros::Publisher pub) {
  PidController::velocityPub = pub;
}
/**
 * @brief Getter method for the pose subscriber
 * @param none
 * @return The current pose subscriber
 */
ros::Subscriber PidController::getPoseSub() {
  return PidController::poseSub;
}
/**
 * @brief Setter method for the pose subscriber
 * @param New pose subscriber to be set
 * @return none
 */
void PidController::setPoseSub(ros::Subscriber sub) {
  PidController::poseSub = sub;
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
 * @brief Setter method for the pose
 * @param New pose to be set
 * @return none
 */
void PidController::setPose(tf::Pose pos) {
  PidController::pose = pos;
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
 * @brief Setter method for the linear velocity
 * @param New linear velocity to be set
 * @return none
 */
void PidController::setLinearVel(double vel) {
  PidController::linearVel = vel;
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
 * @brief Setter method for the angular velocity
 * @param New angular velocity to be set
 * @return none
 */
void PidController::setAngularVel(double angVel) {
  PidController::angularVel = angVel;
}
/**
 * @brief Getter method for the Propotional gain
 * @param none
 * @return The current propotional gain of the controller
 */
double PidController::getKP() {
  return PidController::kP;
}
/**
 * @brief Setter method for the propotional gain
 * @param New propotional gain to be set
 * @return none
 */
void PidController::setKP(double kP) {
  PidController::kP = kP;
}
/**
 * @brief Getter method for the derivative gain
 * @param none
 * @return The current derivative gain of the controller
 */
double PidController::getKD() {
  return PidController::kD;
}
/**
 * @brief Setter method for the derivative gain
 * @param New derivative gain to be set
 * @return none
 */
void PidController::setKD(double kD) {
  PidController::kD = kD;
}
/**
 * @brief Getter method for the integral gain
 * @param none
 * @return The current integral gain of the controller
 */
double PidController::getKI() {
  return PidController::kI;
}
/**
 * @brief Setter method for the integral gain
 * @param New integral gain to be set
 * @return none
 */
void PidController::setKI(double kI) {
  PidController::kI = kI;
}
/**
 * @brief Callback function to get the pose data from the Turtlebot
 * @param Turtlebot Pose
 * @return none
 */
void PidController::distCallBack(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("Calling Distance Call Back Pose: [%s]", msg->pose);
}
/**
 * @brief Mock of the euclidean distance calculator
 * @param Current pose and desired pose
 * @return Calculated distance
 */
double PidController::euclideanDist(tf::Pose currentPose,
                                    tf::Pose desiredPose) {
  double dist = 5;  // MOCK VALUE FOR TESTING!
  return dist;
}

/**
 * @brief Mock method to calculate the linear and angular velocity for the robot
 * @param Current pose and desired pose
 * @return Calculated linear and angular velocity
 */
void PidController::calcVel(tf::Pose currentPose, tf::Pose desiredPose) {
  PidController::linearVel = 10.0;  // Mock values for testing
  PidController::angularVel = 10.0;  // Mock values for testing

}

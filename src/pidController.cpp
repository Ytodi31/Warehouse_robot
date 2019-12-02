/*
 * pidController.cpp
 *
 *  Created on: Nov 28, 2019
 *      Author: gautam
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
 * @brief Getter method for the position subscriber
 * @param none
 * @return The current position subscriber
 */
ros::Subscriber PidController::getPositionSub() {
  return PidController::positionSub;
}
/**
 * @brief Setter method for the position subscriber
 * @param New position subscriber to be set
 * @return none
 */
void PidController::setPositionSub(ros::Subscriber sub) {
  PidController::positionSub = sub;
}
/**
 * @brief Getter method for the position
 * @param none
 * @return The current position of the turtlebot
 */
tf::Point PidController::getPosition() {
  return PidController::position;
}
/**
 * @brief Setter method for the position
 * @param New position to be set
 * @return none
 */
void PidController::setPosition(tf::Point pos) {
  PidController::position = pos;
}
/**
 * @brief Getter method for the orientation
 * @param none
 * @return The current orientation of the turtlebot
 */
double PidController::getOrientation() {
  return PidController::orientation;
}
/**
 * @brief Setter method for the orientation
 * @param New orientation to be set
 * @return none
 */
void PidController::setOrientation(double orient) {
  PidController::orientation = orient;
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
 * @brief Euclidean distance calculator
 * @param Current position and desired position
 * @return Calculated distance
 */
/**
 * @brief Mock of the euclidean distance calculator
 * @param Current position and desired position
 * @return Calculated distance
 */
double PidController::euclideanDist(tf::Point currentPosition,
                                    tf::Point desiredPosition) {
  double dist = 5;  // MOCK VALUE FOR TESTING!
  return dist;
}

/**
 * @brief Mock method to calculate the linear and angular velocity for the robot
 * @param Current position and desired position
 * @return Calculated linear and angular velocity
 */
void PidController::calcVel(tf::Point currentPosition,
                            tf::Point desiredPosition) {
  PidController::linearVel = 10.0;  // Mock values for testing
  PidController::angularVel = 10.0;  // Mock values for testing

}

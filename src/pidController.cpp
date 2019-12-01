/*
 * pidController.cpp
 *
 *  Created on: Nov 28, 2019
 *      Author: gautam
 */

#include "pidController.hpp"

/**
 * @brief Getter method for the Propotional gain
 * @param  none
 * @return The propotional gain for the controller
 */
double PidController::getKP(){
  return PidController::kP;
}

/**
 * @brief Getter method for the Derivative gain
 * @param  none
 * @return The derivative gain for the controller
 */
double PidController::getKD(){
  return PidController::kD;
}

/**
 * @brief Getter method for the Integral gain
 * @param  none
 * @return The integral gain for the controller
 */
double PidController::getKI(){
  return PidController::kI;
}

/**
 * @brief Mock of the method to calculate the controller parameters
 * @param Trajectory position and current position
 * @return none
 */
void PidController::calcPID(tf::Point trajPosition, tf::Point currentPosition) {
  double kP, kD, kI;
  kP = 0.3;
  kD = 0.05;
  kI = 1.02;  //  MOCK VALUES FOR TESTING!
  PidController::kP = kP;
  PidController::kD = kD;
  PidController::kI = kI;
}

/**
 * @brief Mock of the euclidean distance calculator
 * @param Current position and desired position
 * @return calculated distance
 */
double PidController::euclideanDist(tf::Point currentPosition,
                                    tf::Point desiredPosition) {
  double dist = 5;  // MOCK VALUE FOR TESTING!
  return dist;
}

/**
 * @brief Mock of the method to calculate the steering angle for the robot
 * @param Trajectory position and the angular velocity
 * @return calculated steering angle
 */
double PidController::calcSteeringAng(tf::Point trajPosition,
                                      double angVelocity) {
  double steeringAng = 30;  // MOCK VALUE FOR TESTING!
  return steeringAng;
}

/*
 * pidController.hpp
 *
 *  Created on: Nov 27, 2019
 *  Author: Gautam Balachandran
 */

#ifndef INCLUDE_PIDCONTROLLER_HPP_
#define INCLUDE_PIDCONTROLLER_HPP_

#include <iostream>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

/*
 * pidController.hpp
 *
 *  Created on: Nov 26, 2019
 *  Author: Gautam Balachandran
 */

class PidController {
 private:
  // ROS Node handle object for controller
  ros::NodeHandle controllerNode;
  // ROS publisher object for velocity publishing
  ros::Publisher velocityPub;
  // ROS subscriber object for getting position
  ros::Subscriber positionSub;
  // Transform Object for position
  tf::Point position;
  double orientation, linearVel, angularVel, kD, kI, kP;

 public:
  /**
   * @brief Getter method for the Ros Node
   * @param none
   * @return The current node handle for the controller
   */
  ros::NodeHandle getControllerNode();
  /**
   * @brief Setter method for the Ros Node
   * @param New Node to be set
   * @return none
   */
  void setControllerNode(ros::NodeHandle n);
  /**
   * @brief Getter method for the velocity publisher
   * @param none
   * @return The current velocity publisher
   */
  ros::Publisher getVelocityPub();
  /**
   * @brief Setter method for the velocity publisher
   * @param New velocity publisher to be set
   * @return none
   */
  void setVelocityPub(ros::Publisher pub);
  /**
   * @brief Getter method for the position subscriber
   * @param none
   * @return The current position subscriber
   */
  ros::Subscriber getPositionSub();
  /**
   * @brief Setter method for the position subscriber
   * @param New position subscriber to be set
   * @return none
   */
  void setPositionSub(ros::Subscriber sub);
  /**
   * @brief Getter method for the position
   * @param none
   * @return The current position of the turtlebot
   */
  tf::Point getPosition();
  /**
   * @brief Setter method for the position
   * @param New position to be set
   * @return none
   */
  void setPosition(tf::Point pos);
  /**
   * @brief Getter method for the orientation
   * @param none
   * @return The current orientation of the turtlebot
   */
  double getOrientation();
  /**
   * @brief Setter method for the orientation
   * @param New orientation to be set
   * @return none
   */
  void setOrientation(double orient);
  /**
   * @brief Getter method for the linear velocity
   * @param none
   * @return The current linear velocity of the turtlebot
   */
  double getLinearVel();
  /**
   * @brief Setter method for the linear velocity
   * @param New linear velocity to be set
   * @return none
   */
  void setLinearVel(double vel);
  /**
   * @brief Getter method for the angular velocity
   * @param none
   * @return The current angular velocity of the turtlebot
   */
  double getAngularVel();
  /**
   * @brief Setter method for the angular velocity
   * @param New angular velocity to be set
   * @return none
   */
  void setAngularVel(double angVel);
  /**
   * @brief Getter method for the Propotional gain
   * @param none
   * @return The current propotional gain of the controller
   */
  double getKP();
  /**
   * @brief Setter method for the propotional gain
   * @param New propotional gain to be set
   * @return none
   */
  void setKP(double kP);
  /**
   * @brief Getter method for the derivative gain
   * @param none
   * @return The current derivative gain of the controller
   */
  double getKD();
  /**
   * @brief Setter method for the derivative gain
   * @param New derivative gain to be set
   * @return none
   */
  void setKD(double kD);
  /**
   * @brief Getter method for the integral gain
   * @param none
   * @return The current integral gain of the controller
   */
  double getKI();
  /**
   * @brief Setter method for the integral gain
   * @param New integral gain to be set
   * @return none
   */
  void setKI(double kI);
  /**
   * @brief Method to calculate the controller parameters
   * @param Trajectory position and current position
   * @return none
   */
  void calcPID(tf::Point trajPosition, tf::Point currentPosition);
  /**
   * @brief Euclidean distance calculator
   * @param Current position and desired position
   * @return calculated distance
   */
  double euclideanDist(tf::Point currentPos, tf::Point desiredPos);
  /**
   * @brief Method to calculate the steering angle for the robot
   * @param Trajectory position and the angular velocity
   * @return calculated steering angle
   */
  double calSteeringAng(tf::Point trajPosition, double angularVel);

};

#endif /* INCLUDE_PIDCONTROLLER_HPP_ */

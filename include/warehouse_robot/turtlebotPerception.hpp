/*
 * turtlebotPerception.hpp
 *
 *  Created on: Nov 27, 2019
 *  Author: Gautam Balachandran
 */

#ifndef INCLUDE_TURTLEBOTPERCEPTION_HPP_
#define INCLUDE_TURTLEBOTPERCEPTION_HPP_

#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

class TurtlebotPerception {
 private:
  // ROS Node handle object for perception
  // ros::NodeHandle perceptionNode;
  // ROS publisher object for perception
  // ros::Publisher distPub;
  // ROS subscriber object for perception
  // ros::Subscriber distSub;
  // Boolean object to detect collision
  bool collide;

 public:
  /**
   * @brief Getter method for the Ros Node
   * @param  none
   * @return The current node handle for the perception
   */
  // ros::NodeHandle getPerceptionNode();
  /**
   * @brief Setter method for the Ros Node
   * @param  New Node to be set
   * @return none
   */
  // void setPerceptionNode(ros::NodeHandle n);
  /**
   * @brief Getter method for the distance publisher
   * @param  none
   * @return The current distance publisher
   */
  // ros::Publisher getDistPub();
  /**
   * @brief Setter method for the distance publisher
   * @param  New distance publisher to be set
   * @return none
   */
  // void setDistPub(ros::Publisher pub);
  /**
   * @brief Getter method for the distance subscriber
   * @param  none
   * @return The current distance subscriber
   */
  // ros::Subscriber getDistSub();
  /**
   * @brief Setter method for the distance subscriber
   * @param  New distance subscriber to be set
   * @return none
   */
  // void setDistSub(ros::Subscriber sub);
  /**
   * @brief Getter method for the collision parameter
   * @param  none
   * @return The boolean value for collision
   */
  bool getCollide();
  /**
   * @brief Setter method for the collision parameter
   * @param boolean value for collision
   * @return none
   */
  void setCollide(bool collision);
  /**
   * @brief Function to get the Laser scan data from the Turtlebot
   * @param Planar laser range-finder data
   * @return none
   */
  void sensorData(const sensor_msgs::LaserScan::ConstPtr &msg);
  /**
   * @brief Function to detect collision
   * @param none
   * @return true if obstacle detected
   */
  bool detectCollision();
};

#endif /* INCLUDE_TURTLEBOTPERCEPTION_HPP_ */

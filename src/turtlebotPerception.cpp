/*
 * turtlebotPerception.cpp
 *
 *  Created on: Nov 28, 2019
 *      Author: gautam
 */

#include "turtlebotPerception.hpp"

/**
 * @brief Getter method for the Ros Node
 * @param  none
 * @return The current node handle for the perception
 */
ros::NodeHandle TurtlebotPerception::getPerceptionNode() {
  return TurtlebotPerception::perceptionNode;
}
/**
 * @brief Setter method for the Ros Node
 * @param  New Node to be set
 * @return none
 */
void TurtlebotPerception::setPerceptionNode(ros::NodeHandle n) {
  TurtlebotPerception::perceptionNode = n;
}
/**
 * @brief Getter method for the distance publisher
 * @param  none
 * @return The current distance publisher
 */
ros::Publisher TurtlebotPerception::getDistPub() {
  return TurtlebotPerception::distPub;
}
/**
 * @brief Setter method for the distance publisher
 * @param  New distance publisher to be set
 * @return none
 */
void TurtlebotPerception::setDistPub(ros::Publisher pub) {
  TurtlebotPerception::distPub = pub;
}
/**
 * @brief Getter method for the distance subscriber
 * @param  none
 * @return The current distance subscriber
 */
ros::Subscriber TurtlebotPerception::getDistSub() {
  return TurtlebotPerception::distSub;
}
/**
 * @brief Setter method for the distance subscriber
 * @param  New distance subscriber to be set
 * @return none
 */
void TurtlebotPerception::setDistSub(ros::Subscriber sub) {
  TurtlebotPerception::distSub = sub;
}
/**
 * @brief Getter method for the collision parameter
 * @param  none
 * @return The boolean value for collision
 */
bool TurtlebotPerception::getCollide() {
  return TurtlebotPerception::collide;
}
/**
 * @brief Setter method for the collision parameter
 * @param boolean value for collision
 * @return none
 */
void TurtlebotPerception::setCollide(bool collision) {
  TurtlebotPerception::collide = collision;
}
/**
 * @brief Function to get the Laser scan data from the Turtlebot
 * @param Planar laser range-finder data
 * @return none
 */
void TurtlebotPerception::sensorData(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  // MOCK FUNCTION
}
/**
 * @brief Mock of the function to detect collision
 * @param none
 * @return true if obstacle detected
 */
bool TurtlebotPerception::detectCollision() {
  return true;  // MOCK VALUE FOR TESTING!
}
/**
 * @brief Function to detect the Aruco Marker
 * @param Frame containing the image to be processed and the markerId
 *  to be compared with
 * @return True if the marker detected matches the package required
 */
bool TurtlebotPerception::detectArucoMarker(cv::Mat imageFrame,
                                            double markerId) {
  return true;  // Mock
}
/**
 * @brief Function to detect the depth of the package for grasping
 * @param Frame containing the image to be processed
 * @return The calculated depth of the package
 */
double TurtlebotPerception::packageDepth(cv::Mat packageImage) {
  return 10.0;  // Mock
}


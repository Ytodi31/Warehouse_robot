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
 * @file turtlebotPerception.hpp
 * @brief This file provides the header file for the Perception module
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
 * @date 11-27-2019
 */

#ifndef WAREHOUSE_ROBOT_SRC_WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_TURTLEBOTPERCEPTION_HPP_
#define WAREHOUSE_ROBOT_SRC_WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_TURTLEBOTPERCEPTION_HPP_

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <aruco/markerdetector.h>
#include <aruco/arucofidmarkers.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/exports.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include <pidController.hpp>


class TurtlebotPerception : public PidController {
 private:
  // Ros perception node
  ros::NodeHandle perceptionNode;
  // Distance publisher for ROS
  ros::Publisher distPub;
  // Image subscriber
  ros::Subscriber imageSub;
  // Image variable
  cv::Mat img;
  // Flag set when the Aruco marker is detected
  bool markerDetected = false;
  // Marker x location
  double marker_x = 0;
  // Marker Y location
  double marker_y = 0;
  // Marker Area
  double marker_area = 0;
  // Controller Propotional gain
  double kp;
  // Controller Integral gain
  double ki;
  // Controller differential gain
  double kd;

 public:
  // Image translation matrix
  cv::Mat translation;
  // Image rotation matrix
  cv::Mat rotMat;
  // velocity function derived from PID controller module
  using PidController::calcVel;
  // Propotional gain setter derived from PID controller module
  using PidController::setKP;
  // Integral gain setter derived from PID controller module
  using PidController::setKI;
  // Derivative gain setter derived from PID controller module
  using PidController::setKD;

  /**
   * @brief Setter method for the derivative gain
   * @param  Derivative gain
   * @return none
   */
  void setKD(double kD);
  /**
   * @brief Setter method for the propotional gain
   * @param  Propotional gain
   * @return none
   */
  void setKP(double kP);
  /**
   * @brief Setter method for the integral gain
   * @param  Integral gain
   * @return none
   */
  void setKI(double kI);
  /**
   * @brief Angular velocity function
   * @param none
   * @return Gives the angular velocity that is required to track the marker
   */
  geometry_msgs::Twist calcVel();
  /**
   * @brief Setter method for the Ros Node
   * @param  New Node to be set
   * @return none
   */
  void setPerceptionNode(ros::NodeHandle n);
  /**
   * @brief Setter method for the Ros Subscribers
   * @param none
   * @return none
   */
  void setSubscribers();
  /**
   * @brief Callback function to get the images from the Turtlebot kinect
   * @param RGB image ddata
   * @return none
   */
  void sensorImageData(const sensor_msgs::Image::ConstPtr msg);
  /**
   * @brief Function to detect the Aruco Marker
   * @param Frame containing the image to be processed and the markerId
   *  to be compared with
   * @return True if the marker detected matches the package required
   */
  bool detectArucoMarker(cv::Mat imageFrame, double markerId);
  /**
     * @brief Function to calculate the marker area
     * @param none
     * @return Area of the marker detected
     */
  double getMarkerArea();
};

#endif  // WAREHOUSE_ROBOT_SRC_WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_TURTLEBOTPERCEPTION_HPP_

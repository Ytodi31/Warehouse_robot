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
 * @file turtlebotPerception.cpp
 * @brief This file provides the implementation for the perception module
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
#include <turtlebotPerception.hpp>

void TurtlebotPerception::setPerceptionNode(ros::NodeHandle n) {
  TurtlebotPerception::perceptionNode = n;
}

void TurtlebotPerception::setSubscribers() {
  imageSub = perceptionNode.subscribe("/om_with_tb3/camera/rgb/image_raw", 100,
                                      &TurtlebotPerception::sensorImageData,
                                      this);
}

void TurtlebotPerception::sensorImageData(
    const sensor_msgs::Image::ConstPtr msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg);
  img = cv_ptr->image;
}

bool TurtlebotPerception::detectArucoMarker(cv::Mat imageFrame,
                                            double markerId) {
  aruco::MarkerDetector detector;
  std::vector < aruco::Marker > markers;
  ofstream myfile;
  myfile.open("/home/suyash/Desktop/example.txt");
  cv::Mat camParams = cv::Mat::zeros(3, 3, CV_32F);
  cv::Mat distortion = cv::Mat::zeros(1, 5, CV_32F);
  camParams.at<float>(0, 0) = 530.4669406576809;
  camParams.at<float>(0, 1) = 0.0;
  camParams.at<float>(0, 2) = 320.5;
  camParams.at<float>(1, 0) = 0.0;
  camParams.at<float>(1, 1) = 530.4669406576809;
  camParams.at<float>(1, 2) = 240.5;
  camParams.at<float>(2, 2) = 1;
  aruco::CameraParameters params;
  detector.setMinMaxSize(0.01, 1);

  detector.detect(imageFrame, markers, camParams, distortion, 0.073, false);
  myfile << "Percpeption data.\n";
  myfile << markers.size() << std::endl;
  for (auto mark : markers) {
    if (mark.id == 985) {
      markerDetected = true;
      marker_x = mark.getCenter().x;
      marker_y = mark.getCenter().y;
      marker_area = mark.getArea();
      translation = mark.Tvec;
      rotMat = mark.Rvec;
    }
  }
  myfile.close();
  return markerDetected;  // Mock
}

void TurtlebotPerception::setKP(double kpin) {
  kp = kpin;
}

void TurtlebotPerception::setKI(double kiin) {
  ki = kiin;
}

void TurtlebotPerception::setKD(double kdin) {
  kd = kdin;
}

geometry_msgs::Twist TurtlebotPerception::calcVel() {
  geometry_msgs::Twist vel;
  if (markerDetected) {
    double error = 0;
    double error_diff = 0;
    error = img.rows / 2 - marker_x;
    if (abs(error) > 10) {
      error_diff = error - lastAngularError;
      angularVel = kp * error + ki * sumAngularError + kd * error_diff;
      lastAngularError = error;
      geometry_msgs::Twist vel;
      vel.linear.x = 0;
      vel.linear.y = 0;
      vel.linear.z = 0;
      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = angularVel;
    } else {
      vel.linear.x = 0;
      vel.linear.y = 0;
      vel.linear.z = 0;
      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = 0;
    }
  } else {
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0.08;
  }
  return vel;
}

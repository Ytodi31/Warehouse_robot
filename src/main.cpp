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
 * @file main.cpp
 * @brief This file is the main file of project AuWaMaR
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
 * @date 11-20-2019
 */
#include <ros/ros.h>
#include "localisation.hpp"
#include "PathPlanner.hpp"
#include "pidController.hpp"
#include "turtlebotPerception.hpp"
#include "PickPlace.hpp"
#include "warehouse_robot/user_input.h"
#include <boost/bind.hpp>
 /**
  * @brief Main file to call classes
  * @param Parameter 1, Number of inputs
  * @param Parameter 2, Input
  * @return int, 0 if executed successfully
  */



bool queryUserInput(warehouse_robot::user_input::Request & req,
                    warehouse_robot::user_input::Response &res) {
    ros::NodeHandle n;
    double mapBox_x = (req.boxPose.position.x)/0.05;
    double mapBox_y = (req.boxPose.position.y)/0.05;
    double goal_x = (req.goalPose.position.x)/0.05;
    double goal_y = (req.goalPose.position.y)/0.05;
    Localisation loc;
    PathPlanner pathBox;
    PidController controller;
    TurtlebotPerception perception;
    PickPlace pp;
    perception.setPerceptionNode(n);
    perception.setControllerNode(n);
    perception.setSubscribers();
    controller.setKP(0.5, 0.5);
    controller.setKI(0.0, 0.0);
    controller.setKD(0, 0);
    bool status = false;
    int stage = 1;
    int index = 0;
    loc.initSubscribers(n);
    loc.SetEntropyThreshold(25);
    controller.setControllerNode(n);
    controller.setVelocityPub();
    //controller.setPoseSub();
    ros::Rate loop_rate(100);
    pathBox.setGoal(std::make_pair(mapBox_x, mapBox_y));
    std::vector<std::pair<double, double>> shortestPathBox;
    while (ros::ok()) {
        loc.ExecuteLocalisation();
        if ( stage == 1 ) {
            if (loc.localisationPose.position.x != 0) {
                double start_x = loc.localisationPose.position.x/0.05;
                double start_y = loc.localisationPose.position.y/0.05;
                pathBox.setStart(std::make_pair(start_x, start_y));
                shortestPathBox = pathBox.plannerMain();
                ROS_ERROR_STREAM("GOAL: "<< shortestPathBox.size());
                stage = 2;
            }
        } else if ( stage == 2 ) {
            if (loc.localisationPose.position.x != 0) {
                tf::Pose desired;
                tf::Pose current;
                std::pair<double, double> currentGoal = shortestPathBox[index];
                current.setOrigin(tf::Vector3(loc.localisationPose.position.x,
                        loc.localisationPose.position.y,
                        loc.localisationPose.position.z));
                current.setRotation(tf::Quaternion(loc.localisationPose.orientation.x,
                        loc.localisationPose.orientation.y,
                        loc.localisationPose.orientation.z,
                        loc.localisationPose.orientation.w));

                desired.setOrigin(tf::Vector3(currentGoal.first*0.05,
                            currentGoal.second*0.05, 0));
                controller.calcVel(current, desired);
                ROS_ERROR_STREAM( "Error is: " << controller.euclideanDist(current, desired));
                ROS_ERROR_STREAM( "Node is: " << index);
                ROS_ERROR_STREAM( "Nodes :" << shortestPathBox[14].first << " " << shortestPathBox[14].second);

                if ( controller.euclideanDist(current, desired) < 0.10 ) {
                    controller.firstPoseFlag = true;
                    index = index+2;
                    geometry_msgs::Twist msg;
                      msg.linear.x = 0;
                      msg.linear.y = 0;
                      msg.linear.z = 0;
                      msg.angular.x = 0;
                      msg.angular.y = 0;
                      msg.angular.z = 0;
                      controller.velocityPub.publish(msg);

                }
          }
                if(index == shortestPathBox.size()) {
                    status = true;
                    stage = 3;
                }

        } else if (stage == 3) {
            perception.setKP(0.001);
            perception.setKI(0);
            perception.setKD(0);
            geometry_msgs::Twist vel;
            perception.detectArucoMarker(perception.img, 0);
            vel=perception.calcVel();
            ROS_ERROR_STREAM("Perception translation:"<< perception.translation.at<float>(2,0));
            if(perception.marker_area < 16000 ) {
                vel.linear.x = 0.25;
            } else {
                stage = 4;
            }
            ROS_ERROR_STREAM("marker area:" << perception.marker_area);

            controller.velocityPub.publish(vel);
            status = perception.markerDetected;
        } else if (stage == 4) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
            controller.velocityPub.publish(msg);
            geometry_msgs::Pose pickPose;
            perception.detectArucoMarker(perception.img, 0);
            pickPose.position.x = perception.translation.at<float>(2,0);
            pickPose.position.y = 0;
            pickPose.position.z = 0.03;
            pickPose.orientation.x = 0;
            pickPose.orientation.y = 0;
            pickPose.orientation.z = 0;
            pickPose.orientation.w = 1;
            pp.setPose(pickPose);
            pp.executePick(n);
            break;
        }
//        if (perception.markerDetected == true) {
//            break;
//        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return status;
}


 int main(int argc, char **argv) {
     ros::init(argc, argv, "picknplace");
     ros::NodeHandle n;
     ros::ServiceServer service = n.advertiseService("user_input",
             queryUserInput);
     ros::spin();


   return 0;
 }

/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 Suyash Yeotikar, Yashaarth Todi, Gautam Balachandran
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  @file localisation.hpp
 *  @date Nov 28, 2019
 *  @author Suyash Yeotikar (test-driver)
 *  @brief Localisation module header file
 *  Copyright 2019 Suyash Yeotikar, Yashaarth Todi, Gautam Balachandran  [legal/copyright]
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

#ifndef WAREHOUSE_ROBOT_SRC_WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_LOCALISATION_HPP_
#define WAREHOUSE_ROBOT_SRC_WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_LOCALISATION_HPP_

/**
 * @class Localisation
 * @ingroup warehouse_robot
 * @brief Class declaration for Localisation module
 */
class Localisation {
 private:
   /**
    * @brief ros Node handle for local operations
    */
    ros::NodeHandle localizationNode;

    /**
     * @brief double value holding uncertainity of pose received from SLAM
     */
    double entropy;

    /**
     * @brief entropy limit above which pose values will be discarded
     */
    double entropyThreshold = 0;

    /**
     * @brief publisher to publish the pose of the robot in image coordinates
     */
    ros::Publisher mapPosePublisher;

    /**
     * @brief publisher to publish the pose of the robot in world coordinates
     */
    ros::Publisher rawPosePublisher;

    /**
     * @brief subscriber to receive the entropy from slam module
     */
    ros::Subscriber entropySubscriber;

    /**
     * @brief Transform listener, for frame transform between map and robot base
     */
    tf::TransformListener mapToBaseTfListen;

    /*
     * @brief Callback function when subscribed to the topic which provides entropy (measure of accuracy) of the SLAM algorithm
     * @param msg entropy message holding the entropy value
     * @return none
     */
    void EntropyCallback(const std_msgs::Float64::ConstPtr& msg);

    /*
     * @brief function to publish the pose that was obtained in
     * map coordinates (class variable)(extracted from tf listener)
     * @param none
     * @return none
     */
    void PublishMapPose();

    /*
     * @brief function to publish the raw pose
     * (actual pose of the robot in the map)
     * @param none
     * @return none
     */
    void PublishRawPose();

    /*
     * @brief function to obtain the position of Robot by passing
     * the transform between frames
     * /map /om_with_tb3/robot_base_footprint
     * @param mapToRobot the transform obtained by the listener callback
     * @return none
     */
    void GetRobotCoordinate(tf::StampedTransform mapToRobot);

 public:
   /**
    * @brief Robot pose holding the robot pose received from SLAM
    */
    geometry_msgs::Pose localisationPose;

   /*
    * @brief Function to initialise members
    * @param ros node handle
    * @return none
    */
    void initSubscribers(ros::NodeHandle n);

    /*
     * @brief function to set the threshold of entropy
     * @param thresholdValue threshold value to be set
     * @return bool Boolean value specifying if the threshold was set or not
     */
    bool SetEntropyThreshold(double thresholdValue);

    /*
     * @brief function to execute the localisation functionality
     * assuming that the slam nodes of the robot have been launched
     * @param none
     * @return none
     */
    void ExecuteLocalisation();
};

#endif  // WAREHOUSE_ROBOT_SRC_WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_LOCALISATION_HPP_

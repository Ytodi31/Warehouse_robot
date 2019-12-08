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
 *  @file localisationTest.cpp
 *  @date Nov 28, 2019
 *  @author Suyash Yeotikar (test-driver)
 *  @brief Localisation module test file
 */

#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "localisationTestClass.hpp"
#include "localisation.hpp"
/**
 * @brief test to check if the entropy threshold variable is set properly or not
 * in case if appriopriate value is passed
 */

TEST(LocalisationTest, EntopyTestPositive) {
    double thresh = 100;
    Localisation locObj;
    EXPECT_TRUE(locObj.SetEntropyThreshold(thresh));
}

/**
 * @brief test to check the if the entropy thresold is set appropriately in case an
 * an inappropriate value is passed.
 */
TEST(LocalisationTest, EntropyTestNegative) {
    double thresh1 = -0.01;
    Localisation locObj;
    EXPECT_FALSE(locObj.SetEntropyThreshold(thresh1));
    EXPECT_FALSE(locObj.SetEntropyThreshold(0));
}

/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed (positive test case where localisation is correct)
 */
TEST(LocalisationTest, ExecuteLocalisationPositive) {
    LocalisationTest testObj1;
    LocalisationTest testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(2.5, 9.7, 0));
    tf::Quaternion q;
    q.setRPY(-0.003, 0.135, 0.295);
    transform.setRotation(q);
    locObj.SetEntropyThreshold(100);
    testObj1.InitPose();
    testObj2.InitPose();
    locObj.initSubscribers(nh);
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
            100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();

        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 7) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 2.5, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 9.7, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, -0.011, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0.066, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0.144, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0.987, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 50, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 194, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, -0.011, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0.066, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0.144, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0.987, 0.1);
}

/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed but Robot is fallen (incorrectPose)
 */
TEST(LocalisationTest, ExecuteLocalisationFallenRobotYaxis) {
    LocalisationTest testObj1;
    LocalisationTest testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(2.5, 9.7, 0));
    tf::Quaternion q;
    q.setRPY(-0.003, 1.57, 0.295);
    transform.setRotation(q);
    locObj.initSubscribers(nh);
    locObj.SetEntropyThreshold(100);
    testObj1.InitPose();
    testObj2.InitPose();
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
                100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();
        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 7) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0, 0.1);
}

/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed but Robot is fallen (incorrectPose)
 */
TEST(LocalisationTest, ExecuteLocalisationFallenRobotYaxisNeg) {
    LocalisationTest testObj1;
    LocalisationTest testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(2.5, 9.7, 0));
    tf::Quaternion q;
    q.setRPY(-0.003, -1.57, 0.295);
    transform.setRotation(q);
    locObj.initSubscribers(nh);
    locObj.SetEntropyThreshold(100);
    testObj2.InitPose();
    testObj1.InitPose();
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
                100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();
        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 7) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0, 0.1);
}

/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed but Robot is fallen (incorrectPose)
 */
TEST(LocalisationTest, ExecuteLocalisationFallenRobotXaxis) {
    LocalisationTest testObj1, testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(2.5, 9.7, 0));
    tf::Quaternion q;
    q.setRPY(1.57, 0.135, 0.295);
    transform.setRotation(q);
    locObj.SetEntropyThreshold(100);
    testObj1.InitPose();
    testObj2.InitPose();
    locObj.initSubscribers(nh);
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
                100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();
        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 7) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0, 0.1);
}

/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed but Robot is fallen (incorrectPose)
 */
TEST(LocalisationTest, ExecuteLocalisationFallenRobotXaxisNeg) {
    LocalisationTest testObj1, testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(2.5, 9.7, 0));
    tf::Quaternion q;
    q.setRPY(-1.57, 0.135, 0.295);
    transform.setRotation(q);
    locObj.SetEntropyThreshold(100);
    locObj.initSubscribers(nh);
    testObj1.InitPose();
    testObj2.InitPose();
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
                100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();
        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 5) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0, 0.1);
}


/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed but Robot is out of map bounds (incorrectPose)
 */
TEST(LocalisationTest, ExecuteLocalisationRobotOutOfMapCorner1) {
    LocalisationTest testObj1, testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(-1, -1, 0));
    tf::Quaternion q;
    q.setRPY(0.003, 0.135, 0.295);
    transform.setRotation(q);
    locObj.SetEntropyThreshold(100);
    locObj.initSubscribers(nh);
    locObj.ExecuteLocalisation();
    testObj1.InitPose();
    testObj2.InitPose();
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
                100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();
        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 5) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0, 0.1);
}


/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed but Robot is out of map bounds (incorrectPose)
 */
TEST(LocalisationTest, ExecuteLocalisationRobotOutOfMapCorner2) {
    LocalisationTest testObj1, testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(40, 25, 0));
    tf::Quaternion q;
    q.setRPY(0.003, 0.135, 0.295);
    transform.setRotation(q);
    locObj.SetEntropyThreshold(100);
    locObj.initSubscribers(nh);
    locObj.ExecuteLocalisation();
    testObj1.InitPose();
    testObj2.InitPose();
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
                100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();

        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 5) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0, 0.1);
}

/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed but Robot is out of map bounds (incorrectPose)
 */
TEST(LocalisationTest, ExecuteLocalisationRobotZPos) {
    LocalisationTest testObj1, testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(2.5, 9.7, 2));
    tf::Quaternion q;
    q.setRPY(0.003, 0.135, 0.295);
    transform.setRotation(q);
    locObj.SetEntropyThreshold(100);
    locObj.initSubscribers(nh);
    testObj1.InitPose();
    testObj2.InitPose();
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
                100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();
        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 7) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0, 0.1);
}


/**
 * @brief test to check if the Robot Coordinate sets appropriate
 * values or not when localisation is executed but Robot is out of map bounds (incorrectPose)
 */
TEST(LocalisationTest, ExecuteLocalisationRobotZPosNeg) {
    LocalisationTest testObj1, testObj2;
    Localisation locObj;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(2.5, 9.7, -2));
    tf::Quaternion q;
    q.setRPY(0.003, 0.135, 0.295);
    transform.setRotation(q);
    locObj.SetEntropyThreshold(100);
    locObj.initSubscribers(nh);
    br.sendTransform(tf::StampedTransform(transform,
    ros::Time::now(), "map", "/om_with_tb3/base_footprint"));
    locObj.ExecuteLocalisation();
    testObj1.InitPose();
    testObj2.InitPose();
    testObj1.poseSubscriber = nh.subscribe("/rawPose",
                100, &LocalisationTest::PoseCallback, &testObj1);
    testObj2.poseSubscriber = nh.subscribe("/mapPose",
                100, &LocalisationTest::PoseCallback, &testObj2);
    ros::Rate loop_rate(10);
    ros::Time begin = ros::Time::now();
    while (1) {
        br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), "/map", "/om_with_tb3/base_footprint"));
         ros::spinOnce();
         locObj.ExecuteLocalisation();

        loop_rate.sleep();
        double diff = (ros::Time::now() - begin).toSec();
        if ((testObj2.receivedPose && testObj1.receivedPose) || diff > 5) {
            break;
        }
    }
    EXPECT_TRUE(testObj1.receivedPose);
    EXPECT_NEAR(testObj1.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj1.robotPose.orientation.w, 0, 0.1);
    EXPECT_TRUE(testObj2.receivedPose);
    EXPECT_NEAR(testObj2.robotPose.position.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.position.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.x, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.y, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.z, 0, 0.1);
    EXPECT_NEAR(testObj2.robotPose.orientation.w, 0, 0.1);
}







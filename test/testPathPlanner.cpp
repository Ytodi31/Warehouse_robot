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
 * @file testPathPlanner.cpp
 * @brief This file tests the member functions of class PathPlanner
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
#include <vector>
#include <utility>
#include "../include/warehouse_robot/PathPlanner.hpp"
#include "ros/ros.h"
#include "gtest/gtest.h"

/**
 * @brief Test to checl the setter and getter of private member goal of class
 * PathPlanner
 */
TEST(TestPathPlanner, testSetterGetterGoal) {
  PathPlanner path;
  std::pair<double, double> goal = std::make_pair(10, 10);
  path.setGoal(goal);
  std::pair<double, double> goalTest = path.getGoal();
  ASSERT_EQ(goalTest, goal);
}

/**
 * @brief This tests the setter and getter of private member start of class
 * PathPlanner
 */
TEST(TestPathPlanner, testSetterGetterStart) {
  PathPlanner path;
  std::pair<double, double> start = std::make_pair(90, 90);
  path.setStart(start);
  std::pair<double, double> startTest = path.getStart();
  ASSERT_EQ(startTest, start);
}

/**
 * @brief This tests the setter and getter of private member PathFound of class
 * PathPlanner
 */
TEST(TestPathPlanner, testSetterGetterPathFound) {
  PathPlanner path;
  path.setPathFound(true);
  bool testPathFound = path.getPathFound();
  ASSERT_EQ(testPathFound, true);
}

/**
 * @brief This tests the function hashCoordinates of the class PathPlanner
 */
TEST(TestPathPlanner, testHashToCoordinates) {
  PathPlanner path;
  std::size_t rows = path.mapSize.first;   // y coordinates
  std::size_t cols = path.mapSize.second;    // x cooordinates
  std::pair<double, double> coordinates;
  coordinates.second = (rows - 1);
  coordinates.first = (cols - 1);
  std::size_t testIndex = (rows * coordinates.first * 0.05
      + coordinates.second * 0.05) / 0.05;
  std::pair<double, double> coordinatesTest = path.hashCoordinates(testIndex);
  EXPECT_EQ(coordinatesTest, coordinates);
}

/**
 * @brief This tests the function hashIndex of the class PathPlanner
 */
TEST(TestPathPlanner, testCoordinatesToHash) {
  PathPlanner path;
  std::size_t rows = path.mapSize.first;   // y coordinates
  std::size_t cols = path.mapSize.second;    // x coordinates
  std::pair<double, double> testCoordinates = std::make_pair(1, rows);
  std::size_t indexTest = path.hashIndex(testCoordinates);
  testCoordinates.first = testCoordinates.first * 0.05;
  testCoordinates.second = testCoordinates.second * 0.05;
  std::size_t index = (testCoordinates.first * rows + testCoordinates.second)
      / 0.05;
  EXPECT_EQ(indexTest, index);
}

/**
 * @brief This tests the function boundaryCheck of the class PathPlanner
 */
TEST(TestPathPlanner, testBoundaryCheck) {
  PathPlanner path;
  std::pair<double, double> node = std::make_pair(90, 90);
  bool testBoundary = path.boundaryCheck(node);
  ASSERT_EQ(testBoundary, true);
}

/**
 * @brief This tests the function shortestPath of the class PathPlanner
 */
TEST(TestPathPlanner, testShortestPath) {
  PathPlanner path;
  std::pair<double, double> start = std::make_pair(0, 0);
  std::pair<double, double> goal = std::make_pair(300, 200);
  path.setStart(start);
  path.setGoal(goal);
  path.plannerMain();  // Calling main function to generate all nodes
  std::size_t localGoalIndex = path.hashIndex(path.localGoal);
  std::vector<std::pair<double, double>> shortestPath = path.shortestPath(
      localGoalIndex);
  int length = shortestPath.size();
  std::pair<double, double> expectedGoal = shortestPath.at(length - 1);
  double diff = sqrt(pow((goal.first-expectedGoal.first), 2) +
  pow((goal.second-expectedGoal.second), 2));
  EXPECT_TRUE(diff < path.goalThreshold);
//  EXPECT_EQ(path.localGoal, shortestPath.at(length - 1));
//  double testCost = path.totalCost.at(localGoalIndex);
//  auto lowestCost = std::min_element(std::begin(path.totalCost),
//                                     std::end(path.totalCost));
//  // Checking the lowest cost in stack versus cost of goal reached
//  EXPECT_EQ(testCost, *lowestCost);
  // Checking coordinates of goal and the end/last waypoint
}

/**
 * @brief This tests the function updateCost of the class PathPlanner
 */
TEST(TestPathPlanner, testUpdateCost) {
  PathPlanner path;
  std::size_t testIndex = 0;
  path.totalCost.push_back(1000);
  path.parentNode.push_back(5);
  double current_cost1 = 900;
  bool test1 = path.updateCost(testIndex, testIndex + 10, current_cost1);
  // Checking the case when current cost is less than total cost at node and
  // the node has not been visited
  EXPECT_EQ(test1, true);
  path.costCome[testIndex] = 900;
  double current_cost2 = 10000;
  bool test2 = path.updateCost(testIndex, testIndex + 10, current_cost2);
  // Checking the case when current cost is more than total cost at node when
  // the node has been visited
  EXPECT_EQ(test2, false);
  double current_cost3 = 800;
  bool test3 = path.updateCost(testIndex, testIndex + 10, current_cost3);
  // Checking the case when current cost is less than total cost at node when
  //  the node has been visited
  EXPECT_EQ(test3, true);
  // Checking if parent node is updated
  EXPECT_EQ(path.parentNode.at(testIndex), testIndex + 10);
}

/**
 * @brief This tests the function differential of the class PathPlanner
 */
TEST(TestPathPlanner, testDifferential) {
  PathPlanner p;
  double ul = 2;
  double ur = 3;
  double r = 0.033;
  double length = 0.16;
  double dt = 1;
  std::size_t currentNode = 50;
  double theta = 0;
  double dtheta = (r / length) * (ur - ul) * dt;
  std::pair<double, double> coordTest = p.differential(ul, ur, theta, length, r,
                                                       dt, currentNode, dtheta);
  coordTest.first = static_cast<int>(coordTest.first * 100);
  coordTest.second = static_cast<int>(coordTest.second * 100);
  coordTest.first /= 100;
  coordTest.second /= 100;
  // Checking the new x coordinates from the movement
  EXPECT_EQ(coordTest.first, -0.06);
  // Checking the new y coordinates from the movement
  EXPECT_EQ(coordTest.second, 50.78);
}

/**
 * @brief  This tests the function allActions of the class PathPlanner
 */
TEST(TestPathPlanner, testAllActions) {
  PathPlanner p;
  std::size_t initialSize = p.stack.size();
  p.allActions(p.startIndex);
  std::size_t finalSize = p.stack.size();
  // Checks if the initial size of the stack is greater or equal to stack size
  // after exploring all possible movement options
  EXPECT_GT(finalSize, initialSize);
}

/**
 * @brief This tests the function showMap of the class PathPlanner
 */
TEST(TestPathPlanner, testShowMap) {
  PathPlanner p;
  std::vector<std::vector<std::size_t>> testMap = p.showMap();
  std::size_t rows = p.mapSize.first;
  std::size_t cols = p.mapSize.second;
  std::size_t testSize = rows * cols;
  // Checking the size of returned vector to be equal to map size
  ASSERT_EQ(testMap.size() * testMap[0].size(), testSize);
}

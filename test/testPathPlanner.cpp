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
#include "ros/ros.h"
#include "gtest/gtest.h"
#include <utility>
#include <vector>
#include <../include/warehouse_robot/PathPlanner.h>

/**
 * @brief Test to checl the setter and getter of private member goal of class
 * PathPlanner
 */
TEST(TestPathPlanner, testSetterGetterGoal) {
  PathPlanner path;
  std::pair<double, double> goal = std::make_pair(10,10);
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
  std::size_t rows = path.mapSize.second;   // y coordinates
  std::size_t cols = path.mapSize.first;    // x cooordinates
  std::pair<double, double> coordinates;
  coordinates.second = rows -1;
  coordinates.first = cols - 1;
  std::size_t testIndex = rows*coordinates.first + coordinates.second;
  std::pair<double, double> coordinatesTest = path.hashCoordinates(testIndex);
  EXPECT_EQ(coordinatesTest, coordinates);
}

/**
 * @brief This tests the function hashIndex of the class PathPlanner
 */
TEST(TestPathPlanner, testCoordinatesToHash) {
  PathPlanner path;
  std::size_t rows = path.mapSize.second;   // y coordinates
  std::size_t cols = path.mapSize.first;    // x coordinates
  std::pair<double, double> testCoordinates = std::make_pair(1, rows);
  std::size_t indexTest = path.hashIndex(testCoordinates);
  std::size_t index = testCoordinates.first*rows + testCoordinates.second;
  EXPECT_EQ(indexTest,index);
}

/**
 * @brief This tests the function boundaryCheck of the class PathPlanner
 */
TEST(TestPathPlanner, testBoundaryCheck) {
  PathPlanner path;
  std::size_t rows = path.mapSize.second;
  // Creating a coordinate outside of map
  std::size_t cols = path.mapSize.first + 1;
  std::size_t index = path.mapSize.second*cols + rows;
  bool testBoundary = path.boundaryCheck(index);
  ASSERT_EQ(testBoundary, true);
}

/**
 * @brief This tests the function shortestPath of the class PathPlanner
 */
TEST(TestPathPlanner, testShortestPath) {
  PathPlanner path;
  std::vector<std::pair<double, double>> shortestPath;
  std::size_t localGoalIndex = path.localGoal.first*path.mapSize.second +
                               path.localGoal.second;
  shortestPath = path.shortestPath(localGoalIndex);
  int length = shortestPath.size();
  double testCost = path.totalCost.at(localGoalIndex);
  auto lowestCost = std::min_element(std::begin(path.totalCost),
                    std::end(path.totalCost));
  // Checking the lowest cost in stack versus cost of goal reached
  EXPECT_EQ(testCost, *lowestCost);
  // Checking coordinates of goal and the end/last waypoint
  EXPECT_EQ(path.localGoal, shortestPath.at(length-1));
}

/**
 * @brief This tests the function updateCost of the class PathPlanner
 */
TEST(TestPathPlanner, testUpdateCost) {
  PathPlanner path;
  std::size_t testIndex = 0;
  path.totalCost.push_back(1000);
  path.parentNode.push_back(5);
  double current_cost1 = 10000;
  bool test1 = path.updateCost(testIndex, testIndex+10, current_cost1);
  // Checking the case when current cost is more than total cost at node
  EXPECT_EQ(test1, false);
  double current_cost2 = 900;
  bool test2 = path.updateCost(testIndex, testIndex+10, current_cost2);
  // Checking the case when current cost is less than total cost at node
  EXPECT_EQ(test2, true);
  // Checking if parent node is updated
  EXPECT_EQ(path.parentNode.at(0), testIndex+10);
}

/**
 * @brief This tests the function differential of the class PathPlanner
 */
TEST(TestPathPlanner, testDifferential) {
  PathPlanner p;
  double ul = 2;
  double ur = 3;
  double r = 0.2;
  double length = 25;
  double dt = 1;
  std::size_t currentNode = 50;
  double theta = 0;
  double dtheta = (r/length)*(ur - ul)*dt;
  std::pair<double, double> coordTest = p.differential(ul, ur, theta, length,
    r,dt, currentNode, dtheta);
  // Checking the new x coordinates from the movement
  EXPECT_EQ(coordTest.first, 0.5);
  // Checking the new y coordinates from the movement
  EXPECT_EQ(coordTest.second, 50);
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
  ASSERT_EQ(testMap.size(), testSize);
}

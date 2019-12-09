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
 * @file PathPlanner.cpp
 * @brief This file contains the function definitions of class PathPlanner
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
 * @date 11-27-20 19
 */
#include<math.h>
#include <ros/package.h>
#include<iostream>
#include<vector>
#include<utility>
#include<algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "../include/warehouse_robot/PathPlanner.hpp"

PathPlanner::PathPlanner() {
  setPathFound(false);
  goalThreshold = 5;  // Threshold distance around the goal
  mapSize = std::make_pair(401, 779);
  // Preassigned size for the vectors
  int vectorSize = (mapSize.first * mapSize.second) / 0.05;
  costGo.assign(vectorSize, INFINITY);
  costCome.assign(vectorSize, 0.0);
  totalCost.assign(vectorSize, 0.0);
  visitStatus.assign(vectorSize, 0);
  parentNode.assign(vectorSize, 0);
  actionSequence.assign(vectorSize, 0);
  thetaSequence.assign(vectorSize, 45.0);
  currentIndex = 0;
  localGoal = std::make_pair(0, 0);

  // Reading Image file to populate the map
  std::string imagePath = ros::package::getPath("warehouse_robot");
  imagePath.append("/data/maps/ariac_load.png");
  // Reading the image in Greyscale
  cv::Mat mapImage = cv::imread(imagePath, CV_LOAD_IMAGE_GRAYSCALE);
  int erosion_type = 0, erosion_size = 0;
  erosion_type = cv::MORPH_ELLIPSE;
  cv::Mat erodedImage;
  erosion_size = 5;
  // Eroding the image to get thicker boundaries around obstacles
  cv::Mat element = cv::getStructuringElement(
      erosion_type, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
      cv::Point(erosion_size, erosion_size));
  int rows = mapImage.rows;
  int cols = mapImage.cols;

  for (auto i = 0; i < rows; i++) {
    for (auto j = 0; j < cols; j++) {
      if (static_cast<int>(mapImage.at < uchar > (i, j) > 240)) {
        mapImage.at < uchar > (i, j) = 255;
      } else {
        mapImage.at < uchar > (i, j) = 0;
      }
    }
  }
  // Eroding original image as a substitute for Minkowski Sum
  cv::erode(mapImage, erodedImage, element);
  erodedImage = 1 - (erodedImage / 255);
  int val;
  for (int i = 0; i < rows; i++) {
      std::vector<int> tempCol;
      for (int j = 0; j < cols; j++) {
      val = static_cast<int>(erodedImage.at < uchar > (i, j));
      tempCol.push_back(val);
    }
    map.push_back(tempCol);
  }
  // Populating Map with the image values
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      if (static_cast<int>(erodedImage.at < uchar > (i, j) == 0)) {
        map[i][j] = 0;
      } else {
        map[i][j] = 1;
      }
    }
  }
  ROS_ERROR_STREAM("rEACHED DERE");
}

std::vector<std::pair<double, double>> PathPlanner::plannerMain() {
  int index = 0;
  double stackLength = 0, thetaOld = 0;
  double lowestCost = 1000000, currentCost = INFINITY;
  std::vector<std::pair<double, double>> shortPath;
  std::vector<std::size_t>::iterator it;
  std::pair<double, double> startNode = getStart();
  std::pair<double, double> goalNode = getGoal();
  std::pair<double, double> currentNode = startNode;

  // Checking if the start and goal points are in the map
  // and are not in obstacles
  if (boundaryCheck(startNode) == 0) {
    ROS_ERROR_STREAM("INVALID START POINT!");
    return shortPath;
  } else if (boundaryCheck(goalNode) == 0) {
    ROS_ERROR_STREAM("INVALID GOAL POINT!");
    return shortPath;
  } else if (startNode.first == goalNode.first
      && startNode.second == goalNode.second) {
    ROS_ERROR_STREAM("START AND GOAL NODES ARE SAME!");
    return shortPath;
  }
  startIndex = hashIndex(getStart());
  goalIndex = hashIndex(getGoal());
  stack.push_back(startIndex);
  costCome.at(startIndex) = 0;
  costGo.at(startIndex) = euclideanDist(start, goal);
  totalCost.at(startIndex) = costCome.at(startIndex) + costGo.at(startIndex);
  currentIndex = startIndex;

  // While goal has not been reached
  while (goalFlag == 0) {
    if (goalCheck(currentNode) == true) {
      setPathFound(true);
      ROS_DEBUG_STREAM("Within goal threshold!");
      break;
    }
    stackLength = stack.size();
    if (stackLength == 0) {
      ROS_DEBUG_STREAM("All nodes explored. Goal could not be reached!");
      break;
    }
    for (std::vector<std::size_t>::iterator it = stack.begin();
        it != stack.end(); ++it) {
      auto loc = *it;
      auto totCost = totalCost.at(loc);
      if (totCost < lowestCost) {
        // Taking the next node from stack as the current one
        currentIndex = *it;
        currentNode = hashCoordinates(currentIndex);
        currentCost = costGo.at(currentIndex);
        lowestCost = totCost;
      }
    }
    currentNode = hashCoordinates(currentIndex);
    currentCost = costGo.at(currentIndex);
    thetaOld = thetaSequence.at(currentIndex);
    // Calling the function to explore all possible nodes
    allActions(currentIndex);
  }
  // If goal has been reached, calculate the shortest path
  if (goalFlag == 1) {
    std::size_t localGoalIndex = hashIndex(localGoal);
    shortPath = shortestPath(localGoalIndex);
  } else {
    ROS_DEBUG_STREAM("Could not reach goal!");
  }
  return shortPath;
}

void PathPlanner::setGoal(std::pair<double, double> g) {
  goal.first = g.first;
  goal.second = g.second;
  goalIndex = hashIndex(getGoal());
}

std::pair<double, double> PathPlanner::getGoal() {
  return goal;
}

void PathPlanner::setStart(std::pair<double, double> s) {
  start.first = s.first;
  start.second = s.second;
  startIndex = hashIndex(s);
}

std::pair<double, double> PathPlanner::getStart() {
  return start;
}

void PathPlanner::setPathFound(bool flag) {
  pathFound = flag;
}

bool PathPlanner::getPathFound() {
  return pathFound;
}

std::size_t PathPlanner::hashIndex(std::pair<double, double> node) {
  node.first = abs(round(node.first));
  node.second = abs(round(node.second));
  std::size_t mapRow = mapSize.first;
  std::size_t index = (node.first * mapRow + node.second);
  return index;
}

std::pair<double, double> PathPlanner::hashCoordinates(std::size_t hashIndex) {
  std::size_t mapRow = mapSize.first;
  std::pair<double, double> cartesianCoord;
  double x = (hashIndex / mapRow);  // Quotient
  double y = (hashIndex % mapRow);  // Remainder
  cartesianCoord.first = x;
  cartesianCoord.second = y;
  return cartesianCoord;
}

bool PathPlanner::boundaryCheck(std::pair<double, double> node) {
  // Checking if the node is within the map
  if ((node.first <= mapSize.second && node.first >= 0)
      && (node.second <= mapSize.first && node.second >= 0)) {
    // Checking if the new node is not in an obstacle
    if (map[node.second][node.first] == 0) {
      return 1;
    }
  }
  return 0;
}

std::vector<std::pair<double, double>> PathPlanner::shortestPath(
    std::size_t goalIndex) {
  std::vector<std::pair<double, double>> shortPath;
  std::pair<double, double> currentNode;
  currentIndex = goalIndex;

  if (parentIndexList.size() == 0) {
    ROS_ERROR_STREAM("NO PATH FOUND!");
    return shortPath;
  }

  while (currentIndex != startIndex) {
    currentNode = hashCoordinates(currentIndex);
    shortPath.push_back(currentNode);
    for (int i = 1; i < parentIndexList.size(); i++) {
      if (parentIndexList[i].second == currentIndex) {
        currentIndex = parentIndexList[i].first;
        currentNode = hashCoordinates(currentIndex);
        shortPath.push_back(currentNode);
      }
    }
  }
  std::reverse(shortPath.begin(), shortPath.end());
  shortPath.erase(unique(shortPath.begin(), shortPath.end()), shortPath.end());
  return shortPath;
}

bool PathPlanner::updateCost(std::size_t newIndex, std::size_t parentIndex,
                             double cost) {
  double size = stack.size();
  std::pair < std::size_t, std::size_t > indexes;
  // Checks if node has already been visited
  if (visitStatus.at(newIndex) == 0) {
    currentIndex = newIndex;
    visitStatus.at(currentIndex) = 1;
    indexes.first = parentIndex;
    indexes.second = newIndex;
    parentIndexList.push_back(indexes);
    parentNode[newIndex] = parentIndex;
    stack.push_back(currentIndex);  // Pushing the current index to stack
    actionSequence.at(currentIndex) = actionNumber;
    std::pair<double, double> nodeParent = hashCoordinates(parentIndex);
    std::pair<double, double> nodeCurrent = hashCoordinates(currentIndex);

    // Updating the cost
    costCome[currentIndex] = euclideanDist(nodeParent, nodeCurrent);
    costGo[currentIndex] = euclideanDist(nodeCurrent, goal);
    totalCost[currentIndex] = costCome.at(currentIndex)
        + costGo.at(currentIndex);

  } else if (cost < costCome.at(newIndex)) {
    ROS_INFO_STREAM("COST UPDATED. COST : " << cost);
    costCome.at(newIndex) = cost;
    currentIndex = newIndex;
    indexes.first = parentIndex;
    indexes.second = newIndex;
    parentIndexList.push_back(indexes);
    parentNode[newIndex] = parentIndex;
  } else {
    return false;
  }
  return true;
}

double PathPlanner::euclideanDist(std::pair<double, double> point1,
                                  std::pair<double, double> point2) {
  double dist = 0;
  dist = sqrt(
      pow((point1.first - point2.first), 2)
          + pow(((point1.second - point2.second)), 2));
  return dist;
}

std::pair<double, double> PathPlanner::differential(double leftRpm,
                                                    double rightRpm,
                                                    double heading, double l,
                                                    double r, double dt,
                                                    std::size_t oldIndex,
                                                    double dTheta) {
  double sumX = 0.0, sumY = 0.0, costInc = 0.0, dx = 0.0, dy = 0.0;
  bool boundary = 0;
  currentIndex = oldIndex;
  std::size_t newIndex = 0;
  std::pair<double, double> oldNode = hashCoordinates(oldIndex);
  std::pair<double, double> newNode = std::make_pair(0, 0);
  std::pair<double, double> firstNode = hashCoordinates(oldIndex);

  // Calculates the differential path from one node to the next in 20 steps
  for (int i = 0; i <= 20; i++) {
    dx = ((r / 2) * (leftRpm + rightRpm) * cos(heading) * dt);
    newNode.first = oldNode.first + dx;
    sumX += dx;
    dy = ((r / 2) * (leftRpm + rightRpm) * sin(heading) * dt);
    newNode.second = oldNode.second + dy;
    sumY += dy;
    heading += dTheta;
    newIndex = hashIndex(newNode);
    boundary = boundaryCheck(newNode);
    if (boundary == 0) {  // Out of Map and Obstacle check
      break;
    }
    if (goalCheck(newNode)) {
      goalFlag = 1;
      setPathFound(true);
      break;
    }
    oldNode = newNode;
    oldIndex = newIndex;
  }

  if (boundary == 1) {
    thetaSequence.at(newIndex) = heading;  // Updating the heading to vector
  }
  return newNode;
}

void PathPlanner::allActions(std::size_t currentIndex) {
  double leftRpm, rightRpm, newTheta, dTheta, currentCost, rpm1, rpm2, dt, l, r,
      oldTheta;
  rpm1 = 10;
  rpm2 = 15;
  currentCost = totalCost.at(currentIndex);
  dt = 0.05;
  r = 0.033 / 0.05;
  l = 0.16 / 0.05;
  oldTheta = thetaSequence.at(currentIndex);
  std::pair<double, double> newNode;
  std::size_t newIndex = 0;
  newTheta = oldTheta;

  // Action 1
  actionNumber = 1;
  rightRpm = rpm1;
  leftRpm = 0;
  dTheta = 0.10472 * (rightRpm - leftRpm) * (r / l) * dt;
  newNode = differential(leftRpm, rightRpm, newTheta, l, r, dt, currentIndex,
                         dTheta);

  newIndex = hashIndex(newNode);

  if (boundaryCheck(newNode) == 1) {
    updateCost(newIndex, currentIndex, currentCost);
    map[newNode.first][newNode.second] = 3;
  }

  // Action 2
  actionNumber = 2;
  rightRpm = 0;
  leftRpm = rpm1;
  dTheta = 0.10472 * (rightRpm - leftRpm) * (r / l) * dt;
  newNode = differential(leftRpm, rightRpm, newTheta, l, r, dt, currentIndex,
                         dTheta);
  newIndex = hashIndex(newNode);
  if (boundaryCheck(newNode) == 1) {
    updateCost(newIndex, currentIndex, currentCost);
    map[newNode.first][newNode.second] = 3;
  }

  // Action 3
  actionNumber = 3;
  rightRpm = rpm1;
  leftRpm = rpm1;
  dTheta = 0.10472 * (rightRpm - leftRpm) * (r / l) * dt;
  newNode = differential(leftRpm, rightRpm, newTheta, l, r, dt, currentIndex,
                         dTheta);
  newIndex = hashIndex(newNode);
  if (boundaryCheck(newNode) == 1) {
    updateCost(newIndex, currentIndex, currentCost);
    map[newNode.first][newNode.second] = 3;
  }

  // Action 4
  actionNumber = 4;
  rightRpm = rpm2;
  leftRpm = 0;
  dTheta = 0.10472 * (rightRpm - leftRpm) * (r / l) * dt;
  newNode = differential(leftRpm, rightRpm, newTheta, l, r, dt, currentIndex,
                         dTheta);
  newIndex = hashIndex(newNode);
  if (boundaryCheck(newNode) == 1) {
    updateCost(newIndex, currentIndex, currentCost);
    map[newNode.first][newNode.second] = 3;
  }

  // Action 5
  actionNumber = 5;
  rightRpm = 0;
  leftRpm = rpm2;
  dTheta = 0.10472 * (rightRpm - leftRpm) * (r / l) * dt;
  newNode = differential(leftRpm, rightRpm, newTheta, l, r, dt, currentIndex,
                         dTheta);
  newIndex = hashIndex(newNode);
  if (boundaryCheck(newNode) == 1) {
    updateCost(newIndex, currentIndex, currentCost);
    map[newNode.first][newNode.second] = 3;
  }

  // Action 6
  actionNumber = 6;
  rightRpm = rpm2;
  leftRpm = rpm2;
  dTheta = 0.10472 * (rightRpm - leftRpm) * (r / l) * dt;
  newNode = differential(leftRpm, rightRpm, newTheta, l, r, dt, currentIndex,
                         dTheta);
  newIndex = hashIndex(newNode);
  if (boundaryCheck(newNode) == 1) {
    updateCost(newIndex, currentIndex, currentCost);
    map[newNode.first][newNode.second] = 3;
  }

  // Action 7
  actionNumber = 7;
  rightRpm = rpm2;
  leftRpm = rpm1;
  dTheta = 0.10472 * (rightRpm - leftRpm) * (r / l) * dt;
  newNode = differential(leftRpm, rightRpm, newTheta, l, r, dt, currentIndex,
                         dTheta);
  newIndex = hashIndex(newNode);
  if (boundaryCheck(newNode) == 1) {
    updateCost(newIndex, currentIndex, currentCost);
    map[newNode.first][newNode.second] = 3;
  }

  // Action 8
  actionNumber = 8;
  rightRpm = rpm1;
  leftRpm = rpm2;
  dTheta = 0.10472 * (rightRpm - leftRpm) * (r / l) * dt;
  newNode = differential(leftRpm, rightRpm, newTheta, l, r, dt, currentIndex,
                         dTheta);
  newIndex = hashIndex(newNode);
  if (boundaryCheck(newNode) == 1) {
    updateCost(newIndex, currentIndex, currentCost);
    map[newNode.first][newNode.second] = 3;
  }
}

bool PathPlanner::goalCheck(std::pair<double, double> currentNode) {
  if (sqrt(
      pow((currentNode.first - goal.first), 2)
          + pow(((currentNode.second - goal.second)), 2)) < goalThreshold) {
    localGoal = currentNode;
    return true;
  }
  return false;
}

std::vector<std::vector<int>> PathPlanner::showMap() {
  return map;
}

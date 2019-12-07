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
#include "../include/warehouse_robot/PathPlanner.hpp"
#include <iostream>

PathPlanner::PathPlanner() {
  setPathFound(false);
  goalThreshold = 5;  // Euclidean distance threshold to the goal
  mapSize = std::make_pair(402, 780);
  std::vector<std::vector<std::size_t> > vec(780,
                                             std::vector<std::size_t>(402, 0));
  map = vec;
  int vectorSize = (mapSize.first * mapSize.second) / 0.05;
  costGo.assign(vectorSize, INFINITY);
  costCome.assign(vectorSize, 0.0);  // Euclidean distance
  totalCost.assign(vectorSize, 0.0);
  visitStatus.assign(vectorSize, 0);
  parentNode.assign(vectorSize, 0);
  actionSequence.assign(vectorSize, 0);
  thetaSequence.assign(vectorSize, 45.0);
//  stack.assign(vectorSize,0);
  startIndex = hashIndex(getStart());
  goalIndex = hashIndex(getGoal());
  currentIndex = 0;
  localGoal = std::make_pair(0, 0);
}

void PathPlanner::plannerMain() {
  int index = 0;
  double stackLength = 0, thetaOld = 0;
  double lowestCost = 1000000, currentCost = INFINITY;
  std::vector<std::size_t>::iterator it;
  std::pair<double, double> currentNode = getStart();
  startIndex = hashIndex(getStart());
  std::cout << "START INDEX : " << startIndex << std::endl;  // REMOVE!!!
  goalIndex = hashIndex(getGoal());
  std::cout << "GOAL INDEX : " << goalIndex << std::endl;  // REMOVE!!!
  stack.push_back(startIndex);
  costCome.at(startIndex) = 0;
  costGo.at(startIndex) = euclideanDist(start, goal);
  totalCost.at(startIndex) = costCome.at(startIndex) + costGo.at(startIndex);
  currentIndex = startIndex;

  while (goalFlag == 0) {
    if (goalCheck(currentNode) == true) {
      std::cout << "Within goal threshold!" << std::endl;
      break;
      //      ROS_DEBUG("Within goal threshold!");
    }
    stackLength = stack.size();
    if (stackLength == 0) {
      std::cout << "Goal could not be reached!" << std::endl;
//      ROS_DEBUG("Goal could not be reached!");
      std::cout << "Current Position : " << currentNode.first
                << currentNode.second << std::endl;
      break;
    }
    for (int i = 0; i < stackLength; i++) {
      double totCost = totalCost.at(stack[i]);
      if (totCost < lowestCost) {
        currentIndex = stack[i];
        currentNode = hashCoordinates(currentIndex);
        currentCost = costGo.at(currentIndex);
        lowestCost = totCost;
      }
    }
    currentNode = hashCoordinates(currentIndex);
    currentCost = costGo.at(currentIndex);
    thetaOld = thetaSequence.at(currentIndex);
    allActions(currentIndex);
  }
}

void PathPlanner::setGoal(std::pair<double, double> g) {
  goal.first = g.first;
  goal.second = g.second;
}

std::pair<double, double> PathPlanner::getGoal() {
  return goal;
}

void PathPlanner::setStart(std::pair<double, double> s) {
  start.first = s.first;
  start.second = s.second;
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
//  x = x * 0.05;
//  y = y * 0.05;
  cartesianCoord.first = x;
  cartesianCoord.second = y;
  return cartesianCoord;
}

bool PathPlanner::boundaryCheck(std::pair<double, double> node) {
  if ((node.first <= mapSize.second and node.first >= 0)
      and (node.second <= mapSize.first and node.second >= 0)) {
    return 1;
  }
  return 0;
}

std::vector<std::pair<double, double>> PathPlanner::shortestPath(
    std::size_t goalIndex) {
  std::vector<std::pair<double, double>> shortPath;
  std::pair<double, double> currentNode;
  currentIndex = goalIndex;
//  std::cout << "Parent Node size : " << parentNode.size() << std::endl;  // REMOVE!
//  std::cout << "Goal Index : " << currentIndex << std::endl;  // REMOVE!

//  for (int z = 1; z < parentIndexList.size(); z++) {
//    std::cout << "Parent INDEX NODE " << z << " : " << parentIndexList[z].first<<":"<<parentIndexList[z].second<< std::endl;  // REMOVE!
//  }

  while (currentIndex != startIndex) {
    currentNode = hashCoordinates(currentIndex);
    shortPath.push_back(currentNode);
    for (int i = 1; i < parentIndexList.size(); i++) {
      if (parentIndexList[i].second == currentIndex) {
        std::cout << "NEXT INDEX : " << parentIndexList[i].first << std::endl;
        currentIndex = parentIndexList[i].first;
        currentNode = hashCoordinates(currentIndex);
        shortPath.push_back(currentNode);
      }
    }
  }
  std::reverse(shortPath.begin(), shortPath.end());
  shortPath.erase(unique(shortPath.begin(), shortPath.end()), shortPath.end());
  std::cout << "Path size : " << shortPath.size() << std::endl;  // REMOVE!
  return shortPath;
}

bool PathPlanner::updateCost(std::size_t newIndex, std::size_t parentIndex,
                             double cost) {
  double size = stack.size();
  std::cout << "CURRENT COST : " << costCome.at(newIndex) << std::endl;  // REMOVE!
  std::cout << "NEW COST : " << cost << std::endl;  // REMOVE!
  std::pair<std::size_t, std::size_t> indexes;
  if (visitStatus.at(newIndex) == 0) {
    std::cout << "VISITED" << std::endl;  // REMOVE!
    currentIndex = newIndex;
    visitStatus.at(currentIndex) = 1;
    indexes.first = parentIndex;
    indexes.second = newIndex;
    parentIndexList.push_back(indexes);
    parentNode[newIndex] = parentIndex;
    stack.push_back(currentIndex);
    actionSequence.at(currentIndex) = actionNumber;
    std::pair<double, double> nodeParent = hashCoordinates(parentIndex);
    std::pair<double, double> nodeCurrent = hashCoordinates(currentIndex);
    costCome[currentIndex] = euclideanDist(nodeParent, nodeCurrent);
    costGo[currentIndex] = euclideanDist(nodeCurrent, goal);
    totalCost[currentIndex] = costCome.at(currentIndex)
        + costGo.at(currentIndex);

    std::cout << "PARENT OF " << newIndex << " is " << parentNode.at(newIndex)
              << std::endl;

  } else if (cost < costCome.at(newIndex)) {
    std::cout << "COST UPDATE!" << std::endl;  // REMOVE!
    costCome.at(newIndex) = cost;
    currentIndex = newIndex;
    indexes.first = parentIndex;
    indexes.second = newIndex;
    parentIndexList.push_back(indexes);
    parentNode[newIndex] = parentIndex;
    actionSequence[newIndex] = actionNumber;
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
    if (boundary == 0) {
      break;
    }
    if (goalCheck(newNode)) {
      goalFlag = 1;
      break;
    }
    oldNode = newNode;
    oldIndex = newIndex;
  }

  if (boundary == 1) {
    thetaSequence.at(newIndex) = heading;
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
  newNode.second += 1;
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
  newNode.first -= 1;
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

std::vector<std::vector<std::size_t>> PathPlanner::showMap() {
  return map;
}

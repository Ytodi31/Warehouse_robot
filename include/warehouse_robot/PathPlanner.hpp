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
 * @file PathPlanner.hpp
 * @brief This file contains the class declaration of PathPlanner, which will be
 * using A-star algorithm to compute its path
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
 * @date 11-24-2019
 */
#ifndef INCLUDE_WAREHOUSE_ROBOT_PATHPLANNER_HPP_
#define INCLUDE_WAREHOUSE_ROBOT_PATHPLANNER_HPP_

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
/**
 * @brief PathPlanner class computes the shortest path from start to goal
 */
class PathPlanner {
 private:
  /**
   * @brief pair holding cartesian coordinates of the start node
   */
  std::pair<double, double> start;

  /**
   * @brief pair holding cartesian coordinates of the goal node
   */
  std::pair<double, double> goal;

  /**
   * @brief boolean variable, 1 if path found from start to goal threshold, 0
   * otherwise
   */
  bool pathFound;

 public:
  /**
   * @brief 2D Vector, holding the occupancy grid of the environment
   */
  std::vector<std::vector<int>> map;

  /**
   * @brief pair holding the rows and columns size of the map environment
   */
  std::pair<std::size_t, std::size_t> mapSize;

  /**
   * @brief vector holding the cost to go heuristic for Astar algorithm
   */
  std::vector<double> costGo;

  /**
   * @brief vector holding the cost to come heuristic for Astar algorithm
   */
  std::vector<double> costCome;

  /**
   * @brief vector holding the total, cost i.e. cost to come + cost to go from
   * A-star algorithm
   */
  std::vector<double> totalCost;

  /**
   * @brief boolean vector, holding visit status, 1 if node is previously
   * visited, 0 otherwise
   */
  std::vector<bool> visitStatus;

  /**
   * @brief vector holding the information of parent node for each child node
   */
  std::vector<std::size_t> parentNode;

  /**
   * @brief vector holding the information of each parent child index pairs
   */
  std::vector<std::pair<std::size_t, std::size_t>> parentIndexList;

  /**
   * @brief vector holding the action of the node that  was used to get from
   * the parent node to the child node and has the least cost
   */
  std::vector<std::size_t> actionSequence;

  /**
   * @brief integer value that indicates the action being taken
   */
  int actionNumber;

  /**
   * @brief vector holding the theta value of the node that  was used to get from
   * the parent node to the child node
   */
  std::vector<double> thetaSequence;

  /**
   * @brief vector holding the list of nodes to be expanded, i.e. the queue
   */
  std::vector<std::size_t> stack;

  /**
   * @brief float value, holding the value to create the goal region around
   * goal node
   */
  float goalThreshold;

  /**
   * @brief variable holding hash index of start node
   */
  std::size_t startIndex;

  /**
   * @brief variable holding hash index of goal node
   */
  std::size_t goalIndex;

  /**
   * @brief variable holding hash index of goal node
   */
  std::size_t currentIndex;

  /**
   * @brief variable that is set if goal is reached
   */
  int goalFlag = 0;

  /**
   * @brief pair holding cartesian coordinates of the node in the goal
   * threshold at which path search terminated
   */
  std::pair<double, double> localGoal;

  /**
   * @brief Constructor for the class
   */
  PathPlanner();

  /**
   * @brief Method to coompute the Euclidean distance between two nodes
   * @param Two node coordinates
   * @return Euclidean distance
   */
  double euclideanDist(std::pair<double, double> point1,
                       std::pair<double, double> point2);

  /**
   * @brief Main function that calls all the other functions.
   * @param none
   * @return Shortest path from start to goal
   */
  std::vector<std::pair<double, double>> plannerMain();

  /**
   * @brief Setter for private member, goal
   * @param pair holding x and y coordinates of goal point
   * @return none
   */
  void setGoal(std::pair<double, double> g);

  /**
   * @brief Getter for private member, goal
   * @param none
   * @return pair holding x and y coordinates of goal point
   */
  std::pair<double, double> getGoal();

  /**
   * @brief Setter for private member, start
   * @param pair holding x and y coordinates of start point
   * @return none
   */
  void setStart(std::pair<double, double> s);

  /**
   * @brief Getter for private member, start
   * @param none
   * @return pair holding x and y coordinates of start point
   */
  std::pair<double, double> getStart();

  /**
   * @brief Setter for private member pathFound
   * @param boolean value, true if path found, false otherwise
   * @return none
   */
  void setPathFound(bool flag);

  /**
   * @brief Getter for private member pathFound
   * @param none
   * @return boolean value, true if path found, false otherwise
   */
  bool getPathFound();

  /**
   * @brief Function to convert cartesian coordinates to hash index
   * @param pair holding x and y coordinates of node
   * @return hash index
   */
  std::size_t hashIndex(std::pair<double, double> node);

  /**
   * @brief Function to convert hash index to cartesian coordinates
   * @param hash index to be converted
   * @return pair holding x and y coordinates of node
   */
  std::pair<double, double> hashCoordinates(std::size_t hashInd);

  /**
   * @brief Function to conduct boundary check and free space check
   * @param Node coordinates
   * @return bool, true if in map and free space, false otherwise
   */
  bool boundaryCheck(std::pair<double, double> node);

  /**
   * @brief Function to find the shortest path once goal is reached
   * @param hash index of node in goal threshold
   * @return vector of pairs containing x and y coordinates/waypoints
   */
  std::vector<std::pair<double, double>> shortestPath(std::size_t goalHash);

  /**
   * @brief Function to update the cost of node and parent of node
   * @param1 hash index of the current node
   * @param2 hash index of the parent node
   * @param3 cost of the node
   * @return bool, true if cost was updated, false otherwise
   */
  bool updateCost(std::size_t currentHash, std::size_t parentHash, double cost);

  /**
   * @brief Function to calculate the new node using differential constrains
   * @param1 left wheel velocity in rpm
   * @param2 right wheel velocity in rpm
   * @param3 heading angle
   * @param4 Length of wheel base of robot
   * @param5 wheel radius of robot
   * @param6 time interval
   * @param7 current node hash index
   * @param8 change in heading angle
   * @return pair containing x and y of new cooordinates
   */

  std::pair<double, double> differential(double leftRpm, double rightRpm,
                                         double heading, double l, double r,
                                         double dt, std::size_t currentHash,
                                         double dTheta);

  /**
   * @brief Function to calculate new nodes in all directions from current node
   * @param hash index of the current node
   * @return none
   */
  void allActions(std::size_t newIndex);

  /**
   * @brief Function to check if the goal has been reached
   * @param1 current node to be checked
   * @return True if goal reached, false otherwise
   */
  bool goalCheck(std::pair<double, double>);

  /**
   * @brief Function to display the map
   * @param1 none
   * @return 2D vector holding the map as an occupancy grid
   */
  std::vector<std::vector<int>> showMap();
};
#endif  // INCLUDE_WAREHOUSE_ROBOT_PATHPLANNER_HPP_

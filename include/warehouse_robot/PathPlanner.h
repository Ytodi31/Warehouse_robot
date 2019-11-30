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
 * @file PathPlanner.h
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
#ifndef WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_PATHPLANNER_H_
#define WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_PATHPLANNER_H_

#include <vector>
#include <utility>
 /**
  * @brief PathPlanner class computes the shortest path from start to goal
  */
  class PathPlanner {
   public:

  /**
    * @brief Setter for private member, goal
    * @param pair holding x and y coordinates of goal point
    * @return none
    */
    void setGoal(std::pair<double , double>);

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
    void setStart(std::pair<double, double>);

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
    void setPathFound(bool);

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
    std::size_t hashIndex(std::pair<double, double>);

  /**
    * @brief Function to convert hash index to cartesian coordinates
    * @param hash index
    * @return pair holding x and y coordinates of node
    */
    std::pair<double, double> hashCoordinates(std::size_t);

  /**
    * @brief Function to conduct boundary check and free space check
    * @param hash index
    * @return bool, true if in map and free space, false otherwise
    */
    bool boundaryCheck(std::size_t);

  /**
    * @brief Function to find the shortest path once goal is reached
    * @param hash index of node in goal threshold
    * @return vector of pairs containing x and y coordinates/waypoints
    */
    std::vector<std::pair<double, double>> shortestPath(std::size_t);

  /**
    * @brief Function to update the cost of node  and parent of node
    * @param1 hash index of the current node
    * @param2 hash index of the parent node
    * @param3 cost of the node
    * @return bool, true if cost was updated, false otherwise
    */
    bool updateCost(std::size_t, std::size_t, double);

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
    std::pair<double, double> differential(double, double, double,
      double, double, double, std::size_t, double);

  /**
    * @brief Function to calculate new nodes in all directions from current node
    * @param1 hash index of the current node
    * @return none
    */
    void allActions(std::size_t);

  /**
    * @brief Function to display the map
    * @param1 none
    * @return 2D vector holding the map as an occupancy grid
    */
  std::vector<std::vector<std::size_t>> showMap();

  /**
    * @brief 2D Vector, holding the occupancy grid of the environment
    */
    std::vector<std::vector<std::size_t>> map;

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
    * visited, 0 otherwise
    */
    std::vector<std::size_t> parentNode;

  /**
    * @brief vector holding the action of the node that  was used to get from
    * the parent node to the child node and has the least cost
    */
    std::vector<std::size_t> actionSequence;

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
    * @brief pair holding the rows and columns size of the map environment
    */
    std::pair<std::size_t, std::size_t> mapSize = std::make_pair(100, 50);

  /**
    * @brief variable holding hash index of start node
    */
    std::size_t startIndex = 0;

  /**
    * @brief variable holding hash index of goal node
    */
    std::size_t goalIndex;

  /**
    * @brief pair holding cartesian coordinates of the node in the goal
    * threshold at which path search terminated
    */
    std::pair<double, double> localGoal = std::make_pair(0, 0);

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
  };
  #endif  // WAREHOUSE_ROBOT_INCLUDE_WAREHOUSE_ROBOT_PATHPLANNER_H_

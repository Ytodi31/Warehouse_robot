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
 * @date 11-27-2019
 */
#include <../include/warehouse_robot/PathPlanner.h>
#include <iostream>

 void PathPlanner::setGoal(std::pair<double , double>) {
  goal.first = 100;
  goal.second = 100;
 }


 std::pair<double , double> PathPlanner::getGoal() {
  std::pair<double , double> stub;
  stub.first = 1;
  stub.second = 1;
  return stub;
 }

 void PathPlanner::setStart(std::pair<double , double>) {
  start.first = 100;
  start.second = 100;
 }

 std::pair<double , double> PathPlanner::getStart() {
  std::pair<double , double> stub;
  stub.first = 1;
  stub.second = 1;
  return stub;
 }

 void PathPlanner::setPathFound(bool) {
  pathFound = false;
 }

 bool PathPlanner::getPathFound() {
  return false;
 }

 std::size_t PathPlanner::hashIndex(std::pair<double , double>) {
  return 0;
 }

std::pair<double , double> PathPlanner::hashCoordinates(std::size_t) {

  return std::make_pair(0,0);
 }

 bool PathPlanner::boundaryCheck(std::size_t) {
   return false;
 }

std::vector<std::pair<double , double>> PathPlanner::
                                                    shortestPath(std::size_t) {
std::vector<std::pair<double , double>> shortPath;
std::pair<double , double> p = std::make_pair(0,0);
shortPath.push_back(p);
PathPlanner::totalCost.push_back(0);
PathPlanner::totalCost.push_back(-1);
return shortPath;
}

bool PathPlanner::updateCost(std::size_t, std::size_t, double) {
  return true;
}

std::pair<double, double> PathPlanner::differential(double, double, double, double,
   double, double, size_t, double) {
return std::make_pair(0,0);
}

void PathPlanner::allActions(std::size_t) {

}

std::vector<std::vector<std::size_t>> PathPlanner::showMap() {
  std::vector<std::vector<std::size_t>> stubVector;
  std::vector<std::size_t> intermediateVector;
  intermediateVector.push_back(0);
  stubVector.push_back(intermediateVector);
  return stubVector;
}

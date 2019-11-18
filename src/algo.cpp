/**
 * Copyright (c) 2019, Naman Gupta
 *
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *@file       algo.cpp
 *@author     Naman Gupta
 *@copyright  GNU
 *@brief      Implementation of obstacle avoidance class
 */

#include <iostream>
#include "algo.hpp"

/**
 * @brief	Constructs the object.
 */
Algo::Algo() {
  wall = false;
  // Setting linear velocity and angular velocity
  linVel = 0.2;
  angVel = 1.0;
  // Publishing velocities
  pubVel = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);
  // Subscribing to laser scanner
  subLaser = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1000,
      &Algo::laserScan, this);
  // Initial velocities
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // Publish msg to the turtlebot
  pubVel.publish(msg);
}

/**
 * @brief	Destructs the objects
 */
Algo::~Algo() {
  // Stopping the turtlebot
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // Publish msg to the turtlebot
  pubVel.publish(msg);
}

/**
 * @brief	function which tells that wall detected or not
 * @param	none
 * @return	returns true if wall detected else false if not of type bool
 */
bool Algo::obstacle() {
  return wall;
}

/**
 * @brief 	function to callback laser scanner
 * @return 	none
 * @param	stat pointer which stores scanner status of type constant
 */
void Algo::laserScan(const sensor_msgs::LaserScan::ConstPtr& stat) {
  int i = 0;
  // Looking for obstacle
  while (i < stat->ranges.size()) {
    if (stat->ranges[i] < 0.8) {
      wall = true;
      return;
    }
    i++;
  }
  wall = false;
}

/**
 * @brief	function to move the robot
 * @param	none
 * @return 	none
 */
void Algo::movement() {
  // Frequency 10 Hz
  ros::Rate loop(10);
  while (ros::ok()) {
    // Case for wall status true
    if (obstacle()) {
      // Wall detected
      ROS_INFO_STREAM("Oh, wall detected!! Turning anti-clockwise");
      // Stopping and turning the robot
      msg.linear.x = 0.0;
      msg.angular.z = angVel;
    } else {
      ROS_INFO_STREAM("I am moving forward!!");
      // Moving forward only
      msg.angular.z = 0.0;
      msg.linear.x = linVel;
    }
    // Publish msg to turtlebot
    pubVel.publish(msg);
    ros::spinOnce();
    loop.sleep();
  }
}

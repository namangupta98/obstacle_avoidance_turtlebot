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
 *@file       algo.hpp
 *@author     Naman Gupta
 *@copyright  GNU
 *@brief      Class to implement obstacle avoidance algorithm
 */

#ifndef INCLUDE_ALGO_HPP_
#define INCLUDE_ALGO_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

/**
 * @brief	Class for the obstacle avoidance algorithm
 */
class Algo{
 private:
        // Wall container to store boolean value of status
        bool wall;
        // Object of Twist
        geometry_msgs::Twist msg;
        // Object of NodeHandle
        ros::NodeHandle nh;
        // Object publisher
        ros::Publisher pubVel;
        // Object of subscriber
        ros::Subscriber subLaser;
        // Container to store linear velocity
        float linVel;
        // Container to store angular velocity
        float angVel;

 public:
	/**
 	 * @brief 	Constructor for objects
 	 */
    Algo();

	/**
	 * @brief	Destructor for objects
	 */
    ~Algo();

	/**
 	 * @brief 	function to callback laser scanner
 	 * @return 	none
 	 * @param	stat pointer which stores scanner status of type constant
 	 */
    void laserScan(const sensor_msgs::LaserScan::ConstPtr& stat);

	/**
	 * @brief	function which tells that wall detected or not
	 * @param	none
	 * @return	returns true if wall detected else false if not of type bool
	 */
    bool obstacle();

	/**
	 * @brief	function to move the robot
	 * @param	none
	 * @return 	none
	 */
    void movement();
};

#endif  // INCLUDE_ALGO_HPP_

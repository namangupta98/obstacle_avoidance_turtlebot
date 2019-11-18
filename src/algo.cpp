#include <iostream>
#include "algo.hpp"

/**
 * @brief	Constructs the object.
 */
Algo::Algo() {
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
    // check wall
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
#ifndef INCLUDE_ALGORITHM_HPP
#define INCLUDE_ALGORITHM_HPP

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

#endif
#include "algo.hpp"

/**
 * @brief      main function
 * @param      argc
 * @param      argv
 * @return     none
 */
int main(int argc, char* argv[]) {
	// Initializing ROS node
	ros::init(argc, argv, "obstacle_avoidance_turtlebot");
	// Object robot of Algo class
	Algo robot;
	// Moving robot
	robot.movement();
	return 0;
}
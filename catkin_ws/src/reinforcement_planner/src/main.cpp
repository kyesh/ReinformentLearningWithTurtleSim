#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

ros::Publisher action;

void turtleSimPos_callback(const turtlesim::Pose::ConstPtr& pos){

}

int main(int argc, char **argv){
	
	
	ros::init(argc, argv, "reinforcement_planner");
	ros::NodeHandle n;
	action = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 0);
	ros::Subscriber sub1 = n.subscribe("/turtle1/pose", 0, turtleSimPos_callback);
	
	ros::spin();
	
	return 0;
	
}

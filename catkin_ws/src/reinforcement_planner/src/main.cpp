#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

ros::Publisher action;
double XG = 1;
double YG = 1;

//Forces range between - pi and pi
double fixThetaRange(double theta){

	theta = fmod(theta,2*M_PI);//Brings between -2pi and pi
	if(theta > M_PI){//convert to negative pi if greater than pi
		theta =  theta - 2*M_PI;
	}else if(theta < -M_PI){
		theta = 2*M_PI - theta;
	}

	return theta;

}

//Gets anglular difference from a to be between -pi/2 and pi/2
double angleDiff(double a , double b){
	return fixThetaRange(b-a);
}

geometry_msgs::Twist policy(double xt, double yt, double xg, double yg, double thetat){

	double thetag = atan2(yg - yt, xg - xt);
	double thetad = angleDiff(thetat, thetag);
	double a = yg - yt;
	double b =  xg - xt;
	double d = sqrt(a*a + b*b);

	geometry_msgs::Twist msg;

	msg.linear.x = 1;
	msg.angular.z = 1;

	return msg;
}

void turtleSimPos_callback(const turtlesim::Pose::ConstPtr& pos){

	geometry_msgs::Twist msg = policy(pos->x, pos->y, XG, YG, pos->theta);
	action.publish(msg);

}

int main(int argc, char **argv){
	
	
	ros::init(argc, argv, "reinforcement_planner");
	ros::NodeHandle n;
	action = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 0);
	ros::Subscriber sub1 = n.subscribe("/turtle1/pose", 0, turtleSimPos_callback);
	
	ros::spin();
	
	return 0;
	
}

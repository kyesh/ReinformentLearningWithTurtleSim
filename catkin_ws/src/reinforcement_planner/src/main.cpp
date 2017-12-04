#include <math.h>
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/SetPen.h"


ros::Publisher action;
double XG = 1;
double YG = 1;

double A = 0;
double B = 0;
double C = 0;
double D = 0;

double lastA, lastB, lastC, lastD;

turtlesim::Pose lastPos;

void updateCoef( double &Coef, double stepSize){
	int n = rand() % 3;
	if(n == 3){
		Coef = Coef + stepSize;
	}else if(n == 0){
		Coef = Coef - stepSize;
	}
	return;
}

void rosSpinFor(int seconds){

	ros::Time end = ros::Time::now() + ros::Duration(seconds);

	while(ros::Time::now()  <= end){
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
	}

	return;
}

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

double distance(double x1, double y1, double x2, double y2){

        double a = y1 - y2;
        double b =  x1 - x2;
        double d = sqrt(a*a + b*b);
	return d;

}

double reward(turtlesim::Pose state){

	return  (5.0 - distance(state.x, state.y, XG, YG));

}

geometry_msgs::Twist policy(double xt, double yt, double xg, double yg, double thetat){

	double thetag = atan2(yg - yt, xg - xt);
	double thetad = angleDiff(thetat, thetag);
	double d = distance(xt, yt, xg, yg);

	geometry_msgs::Twist msg;

	msg.linear.x = A*thetad + B*d;
	msg.angular.z = C*thetad + D*d;

	return msg;
}

void turtleSimPos_callback(const turtlesim::Pose::ConstPtr& pos){

	geometry_msgs::Twist msg = policy(pos->x, pos->y, XG, YG, pos->theta);
	action.publish(msg);
	lastPos = *pos;

}

int main(int argc, char **argv){
	
	
	srand(time(NULL));
	double lastScore = -10000000000;
        double score = 0;
	geometry_msgs::Twist stop;

        stop.linear.x = 0;
        stop.angular.z = 0;


	ros::init(argc, argv, "reinforcement_planner");
	ros::NodeHandle n;

	ros::ServiceClient client1 = n.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
	ros::ServiceClient client2 = n.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

	action = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
	ros::Subscriber sub1 = n.subscribe("/turtle1/pose", 1, turtleSimPos_callback);

	turtlesim::SetPen pen;
	pen.request.r = 255;

	turtlesim::TeleportAbsolute startPos;
	startPos.request.x = 5;
	startPos.request.y = 5;
	startPos.request.theta = 0;
	while(true){
		pen.request.off = true;
		client2.call(pen);
		if(client1.call(startPos)){
			ROS_INFO_STREAM("Should have teleported");
			pen.request.off = false;
			client2.call(pen);
		}else{
			ROS_INFO_STREAM("Something went wrong");
		}
		rosSpinFor(1);
		action.publish(stop);
		score = reward(lastPos);
		ROS_INFO_STREAM("Score:" << score << " A:" << A << " B:" << B << " C:" << C << " D:" << D);

		if(lastScore > score){
			A = lastA;
			B = lastB;
			C = lastC;
			D = lastD;
		}else{
			lastScore = score;
		}

		lastA = A;
		lastB = B;
		lastC = C;
		lastD = D;

		updateCoef(A, .1);
		updateCoef(B, .1);
		updateCoef(C, .1);
		updateCoef(D, .1);

	}
	
	return 0;
	
}

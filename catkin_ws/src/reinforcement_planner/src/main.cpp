#include <math.h>
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/Kill.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <fstream>
#include <string>

ros::Publisher action;
double XG = 6;
double YG = 1;

std::vector<double> linCoef(3,.3);
std::vector<double> thetaCoef(3,0);
std::vector<double> lastLinCoef(3,.3);
std::vector<double> lastThetaCoef(3,0);

turtlesim::Pose lastPos;

void updateCoef( double &Coef, double stepSize){
	int n = rand() % 3;
	if(n == 2){
		Coef = Coef + stepSize;
	}else if(n == 0){
		Coef = Coef - stepSize;
	}
	return;
}

void bound(double &var, double min , double max){
	if(var > max)
		var = max;
	if(var < min)
		var = min;
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

double getRandomRange(double range, double offset){

	double random = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/range));
	return random + offset;

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
	double x = linCoef[0] + linCoef[1]*abs(thetad) + linCoef[2]*d;
	double z = thetaCoef[0] + thetaCoef[1]*thetad + thetaCoef[2]*d;

	bound(x, 0, 5);
	bound(z, -5, 5);

	msg.linear.x = x;
	msg.angular.z = z;

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

	ros::ServiceClient client3 = n.serviceClient<turtlesim::Spawn>("/spawn");
	ros::ServiceClient client4 = n.serviceClient<turtlesim::Kill>("/kill");

	action = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
	ros::Subscriber sub1 = n.subscribe("/turtle1/pose", 1, turtleSimPos_callback);

	turtlesim::SetPen pen;
	pen.request.r = 255;

	turtlesim::TeleportAbsolute startPos;
	turtlesim::Spawn goal;
	turtlesim::Kill goalk;
	goal.request.name = "goal";
	goalk.request.name = "goal";
	char fileName[] = "run00.csv"; 

	for(int p = 0; p<10; p++){
		std::ofstream myfile;
		fileName[4] = 64+p;
		//std::string fileName ="run"+std::to_string(p)+".txt"; 
		myfile.open(fileName);


		for(int n = 0; n<60; n++){
	

						startPos.request.x = getRandomRange(10,.5);
			startPos.request.y = getRandomRange(10,.5);
			startPos.request.theta = getRandomRange(2*M_PI,-M_PI);
	
			XG = getRandomRange(10,.5);
			YG = getRandomRange(10,.5);
	
			goal.request.x = XG;
			goal.request.y = YG;
	
			client3.call(goal);
	
			for(int i=0; i <10; i++)
			{
	
				pen.request.off = true;
				client2.call(pen);
				if(client1.call(startPos)){
					ROS_INFO_STREAM("Should have teleported");
					pen.request.off = false;
					client2.call(pen);
				}else{
					ROS_INFO_STREAM("Something went wrong");
				}
				rosSpinFor(2);
				action.publish(stop);
				score = reward(lastPos);
				ROS_INFO_STREAM("Score:" << score << " linCoef:" << linCoef[0]  << " " << linCoef[1]  << " " << linCoef[2]  << " thetaCoef:" << thetaCoef[0] << " " << thetaCoef[1] << " " << thetaCoef[2]);
				myfile << score << std::endl;
				
				if(lastScore > score && i > 0){
					linCoef.assign(lastLinCoef.begin(),lastLinCoef.end());
					thetaCoef.assign(lastThetaCoef.begin(),lastThetaCoef.end());
				}else{
					lastScore = score;
				}
		
		                lastLinCoef.assign(linCoef.begin(),linCoef.end());
		                lastThetaCoef.assign(thetaCoef.begin(),thetaCoef.end());
		
		
				updateCoef(linCoef[0], .1);
				updateCoef(linCoef[1], .1);
				updateCoef(linCoef[2], .1);
				updateCoef(thetaCoef[0], .1);
				updateCoef(thetaCoef[1], .1);
				updateCoef(thetaCoef[2], .1);
	
			}
	
			client4.call(goalk);
		}
		
		myfile.close();
	}
	
	return 0;
	
}

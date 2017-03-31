#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <stdio.h>
#include <string.h>

#define MAX_SPEED 0.1
#define DEADZONE 0.0025

void clamp(double&, double, double);
void dead(double&, double);
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&);

ros::Publisher bebop_velocity;
ros::Subscriber ar_pose;
bool visible;

typedef struct vec{
	vec(double X, double Y, double Z) {
		x = X;
		y = Y;
		z = Z;
	}
	double x,y,z;
} vec;

vec offset(0.0,0.0,0.75);
geometry_msgs::Twist move;
int backCount;
double lastBack;

void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
	//markers is a vector<AlvarMarker>
	if(msg->markers.empty()) {
		if((ros::Time::now().toSec() - lastBack) > 1.0 && backCount < 10) {
			move.linear.x = -0.1;
			move.linear.y = 0;
			move.linear.z = 0;
			move.angular.z = 0;
			bebop_velocity.publish(move);
			ROS_INFO("BACKING UP!");

			lastBack = ros::Time::now().toSec();
			backCount++;
		}
		ROS_ERROR("OUT OF VIEW!");
		return;
	}

	backCount = 0;

	ar_track_alvar_msgs::AlvarMarker marker = msg->markers.at(0);
	geometry_msgs::Pose pose = marker.pose.pose;

	double forward = pose.position.z - offset.z;
	double left = -(pose.position.x - offset.x);
	double up = -(pose.position.y - offset.y);

	double dis = sqrt(forward*forward+left*left+up*up);
	double sp = 0.8;

	if (dis > 5) {
		ROS_ERROR("Distance too far!");
		return;
	}

	forward = forward > 0 ? sp*forward*forward : -sp*forward*forward;
	left = left > 0 ? 1.25*sp*left*left : -1.25*sp*left*left;
	up = up > 0 ? sp*up*up : -sp*up*up;

	//apply clamp and thresh
	clamp(forward, -MAX_SPEED, MAX_SPEED);
	clamp(left, -MAX_SPEED, MAX_SPEED);
	clamp(up, -MAX_SPEED, MAX_SPEED);
	dead(forward, DEADZONE);
	dead(left, DEADZONE);
	dead(up, DEADZONE);

	fprintf(stdout, "---\nDIS: %f\nPOS:\nX: %f\nY: %f\nZ: %f\n\n", dis, pose.position.x, pose.position.y, pose.position.z);
	fprintf(stdout, "MOVE:\nforward: %f\nleft: %f\nup: %f\n---\n", forward, left, up);



	move.linear.x = forward;
	move.linear.y = left;
	move.linear.z = up;
	// double sp = 0.2;
	// move.linear.x = forward > 0.05 ? sp : 0.0;
	// move.linear.y = left > 0.05 ? sp : 0.0;
	// move.linear.z = up > 0.05 ? sp : 0.0;
	move.angular.z = 0;
	ROS_INFO("Publishing");
	bebop_velocity.publish(move);

}

void clamp(double& toClamp, double min, double max) {
	if(toClamp > max) toClamp = max;
	else if(toClamp < min) toClamp = min;
}

void dead(double& toDead, double deadThresh) {
	if(toDead < deadThresh && toDead > -deadThresh) toDead = 0.0;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_tag");
	ros::NodeHandle nh;

	// ros::Rate r(60);

	bebop_velocity = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
	ar_pose = nh.subscribe("ar_pose_marker", 1, poseCallback);
	geometry_msgs::Twist vel;

	ros::spin();

	// while(ros::ok()) {
	// 	ros::spinOnce();
	// 	r.sleep();
	// }


	// ros::shutdown();
	return 0;
}

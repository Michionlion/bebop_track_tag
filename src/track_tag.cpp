#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <stdio.h>

#define MAX_SPEED 0.08
#define DEADZONE 0.0025

void clamp(double&, double, double);
void dead(double&, double);
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&);

ros::Publisher cmd_vel;
ros::Subscriber ar_pose;
ros::Subscriber odom;
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
geometry_msgs::Twist velocity;
int backCount = 10;
double lastBack;

void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
	//markers is a vector<AlvarMarker>
	if(msg->markers.empty()) {
		if((ros::Time::now().toSec() - lastBack) > .4 && backCount < 10) {
			move.linear.x = -0.35;
			move.linear.y = 0;
			move.linear.z = 0;
			move.angular.z = 0;
			cmd_vel.publish(move);
			ROS_INFO("BACKING UP!");

			lastBack = ros::Time::now().toSec();
			backCount++;
		}
		return;
	}

	backCount = 0;

	ar_track_alvar_msgs::AlvarMarker marker = msg->markers.at(0);
	geometry_msgs::Pose pose = marker.pose.pose;
	double x,y,z;
	double forward = z =pose.position.z - offset.z;
	double left = x = -(pose.position.x - offset.x);
	double up = y = -(pose.position.y - offset.y);

	double dis = sqrt(forward*forward+left*left+up*up);
	double sp = 0.8;

	if (dis > 5) {
		ROS_ERROR("Distance too far!");
		return;
	}

	double p = 1.5;
	forward = forward > 0 ? sp*pow(fabs(forward), p) : -sp*pow(fabs(forward), p);
	left = left > 0 ? sp*pow(fabs(left), p) : -sp*pow(fabs(left), p);
	up = up > 0 ? 1.5*sp*pow(fabs(up), p) : -1.5*sp*pow(fabs(up), p);

	//apply clamp and thresh
	clamp(forward, -MAX_SPEED, MAX_SPEED);
	clamp(left, -MAX_SPEED, MAX_SPEED);
	clamp(up, -MAX_SPEED*3, MAX_SPEED*3);
	dead(forward, DEADZONE);
	dead(left, DEADZONE);
	dead(up, DEADZONE);

	double lim = 0.1;
	if(fabs(velocity.linear.x) > lim) {
		forward = 0;
		fprintf(stdout, "CUT FORWARD\n");
	}
	if(fabs(velocity.linear.y) > lim) {
		left = 0;
		fprintf(stdout, "CUT LEFT\n");
	}
	if(fabs(velocity.linear.z) > lim) {
		up = 0;
		fprintf(stdout, "CUT UP\n");
	}

	fprintf(stdout, "---\nPOS:\nX: %f\nY: %f\nZ: %f\n\n", pose.position.x, pose.position.y, pose.position.z);
	fprintf(stdout, "VEL:\nforward: %f\nleft: %f\nup: %f\n\n", velocity.linear.x, velocity.linear.y, velocity.linear.z);
	fprintf(stdout, "DIS: %f\nOFFSET:\nforward: %f\nleft: %f\nup: %f\n\n", dis, z, x, y);
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
	cmd_vel.publish(move);

}

void clamp(double& toClamp, double min, double max) {
	if(toClamp > max) toClamp = max;
	else if(toClamp < min) toClamp = min;
}

void dead(double& toDead, double deadThresh) {
	if(toDead < deadThresh && toDead > -deadThresh) toDead = 0.0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	velocity = msg->twist.twist;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_tag");
	ros::NodeHandle nh;

	// ros::Rate r(60);

	cmd_vel = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
	ar_pose = nh.subscribe("ar_pose_marker", 1, poseCallback);
	odom = nh.subscribe("bebop/odom", 1, odomCallback);

	ros::spin();

	// while(ros::ok()) {
	// 	ros::spinOnce();
	// 	r.sleep();
	// }


	// ros::shutdown();
	return 0;
}

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
#define DEBUG 1

//declare static function stubs
void clamp(double&, double, double);
void dead(double&, double);
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&);

//topics
ros::Publisher cmd_vel;
ros::Subscriber ar_pose;
ros::Subscriber odom;

//vector3 struct for ease of organization
typedef struct vec{
	vec(double X, double Y, double Z) {
		x = X;
		y = Y;
		z = Z;
	}
	double x,y,z;
} vec;

//target offset of bebop - x is left/right, y is height, z is backward/forward
vec offset(0.0,0.0,1.25);

//Twist messages
geometry_msgs::Twist move;
geometry_msgs::Twist velocity;

//utility vars for backing up when tracking is lost
int backCount = 10;
double lastBack;

//ar_pose_marker callback function - update and publish move message
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
	//markers is a vector<AlvarMarker>
<<<<<<< HEAD
	if(msg->markers.empty()) {
		if((ros::Time::now().toSec() - lastBack) > 0.5 && backCount < 6) {
			// move.linear.x = -0.35;
			move.linear.x = 0;
=======
	if(msg->markers.empty()) { // if no tag is in view
		if((ros::Time::now().toSec() - lastBack) > .4 && backCount < 10) {  // constraint so bebop isn't moving backwards forever
			move.linear.x = -0.35;  // move backward
>>>>>>> c8a40e845311846c8f982a2709f308a7e33f5285
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

	ar_track_alvar_msgs::AlvarMarker marker = msg->markers.at(0);  // get the AR tag in view
	geometry_msgs::Pose pose = marker.pose.pose;
	double x,y,z;
	double forward = z = pose.position.z - offset.z;
	double left = x = -(pose.position.x - offset.x);
	double up = y = -(pose.position.y - offset.y);

	double dis = sqrt(forward*forward+left*left+up*up);
	double sp = 1;

	if (dis > 5) {  // if the bebop is much too far from tag
		ROS_ERROR("Distance too far!");
		return;
	}


	double p = 1.5;
	// if differences between current x, y, or z positions and desired positions are greater than 0, set amounts to move forward, scaled by distance from the tag
	forward = forward > 0 ? sp*pow(fabs(forward), p) : -sp*pow(fabs(forward), p);
	left = left > 0 ? sp*pow(fabs(left), p) : -sp*pow(fabs(left), p);
	up = up > 0 ? 1.5*sp*pow(fabs(up), p) : -1.5*sp*pow(fabs(up), p);

	//apply clamp and thresh
	clamp(forward, -MAX_SPEED, MAX_SPEED);
	clamp(left, -MAX_SPEED, MAX_SPEED);
	clamp(up, -MAX_SPEED, MAX_SPEED);
	dead(forward, DEADZONE);
	dead(left, DEADZONE);
	dead(up, DEADZONE);

	fprintf(stdout, "---\n");

	double lim = 0.2;
	if(fabs(velocity.linear.x) > lim) {
		forward = 0;
		// fprintf(stdout, "CUT FORWARD\n");
	}
	if(fabs(velocity.linear.y) > lim) {
		left = 0;
		// fprintf(stdout, "CUT LEFT\n");
	}
	// if(fabs(velocity.linear.z) > lim) {
	// 	up = 0;
	// 	// fprintf(stdout, "CUT UP\n");
	// }

	#ifdef DEBUG

	fprintf(stdout, "\nPOS:\nX: %f\nY: %f\nZ: %f\n\n", pose.position.x, pose.position.y, pose.position.z);
	fprintf(stdout, "VEL:\nforward: %f\nleft: %f\nup: %f\n\n", velocity.linear.x, velocity.linear.y, velocity.linear.z);
	fprintf(stdout, "DIS: %f\nOFFSET:\nforward: %f\nleft: %f\nup: %f\n\n", dis, z, x, y);
	fprintf(stdout, "MOVE:\nforward: %f\nleft: %f\nup: %f\n---\n", forward, left, up);

	#endif

	// publishing velocity movements to "cmd_vel" to make the bebop move
	move.linear.x = forward;
	move.linear.y = left;
	move.linear.z = up;
	move.angular.z = 0;
	cmd_vel.publish(move);

}

//clamp toClamp between min and max - toClamp is passed by reference so no need to return anything
void clamp(double& toClamp, double min, double max) {
	if(toClamp > max) toClamp = max;
	else if(toClamp < min) toClamp = min;
}

//create a deadzone for toDead - if it is within deadThresh of 0, clamp it to 0
void dead(double& toDead, double deadThresh) {
	if(toDead < deadThresh && toDead > -deadThresh) toDead = 0.0;
}

//bebop/odom callback function, updates velocity Twist
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	velocity = msg->twist.twist;
}

//main method, entrance to ROS node
int main(int argc, char** argv) {
	//Initialize ROS, name node track_tag
	ros::init(argc, argv, "track_tag");
	//create the NodeHandle
	ros::NodeHandle nh;

	//start publishing to cmd_vel
	cmd_vel = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);

	//subscribe to ar_pose_marker
	ar_pose = nh.subscribe("ar_pose_marker", 1, poseCallback);
	// subscribe to bebop/odom
	odom = nh.subscribe("bebop/odom", 1, odomCallback);

	//call callbacks until ROS shutdown
	try {
		ros::spin();
	} catch(...) {
		ROS_ERROR("--- ERROR IN spin(), shutting down! ---");
		ros::shutdown();
	}
	return 0;
}

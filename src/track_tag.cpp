#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <stdio.h>
#include <string.h>


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

vec offset(0.0,0.0,0.38);
double move_speed = 0.2;
geometry_msgs::Twist move;


void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
	//markers is a vector<AlvarMarker>
	if(msg->markers.empty()) {
		return;
	}

	ar_track_alvar_msgs::AlvarMarker marker = msg->markers.at(0);
	geometry_msgs::Pose pose = marker.pose.pose;

	double forward = pose.position.z - offset.z;
	double left = -(pose.position.x - offset.x);
	double up = -(pose.position.y - offset.y);

	double dis = sqrt(forward*forward+left*left+up*up);

	// forward *= move_speed;
	// left *= move_speed;
	// up *= move_speed;

	if(dis < 0.04) {
		forward = left = up = 0;
	}

	fprintf(stdout, "---\nDIS: %f\nPOS:\nX: %f\nY: %f\nZ: %f\n\n", dis, pose.position.x, pose.position.y, pose.position.z);
	fprintf(stdout, "MOVE:\nforward: %f\nleft: %f\nup: %f\n---\n", forward, left, up);

	// move.linear.x = forward;
	// move.linear.y = left;
	// move.linear.z = up;
	double sp = 0.01;
	move.linear.x = forward > 0.05 ? sp : 0.0;
	move.linear.y = left > 0.05 ? sp : 0.0;
	move.linear.z = up > 0.05 ? sp : 0.0;
	move.angular.z = 0;
	bebop_velocity.publish(move);

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_tag");
	ros::NodeHandle nh;

	ros::Rate r(60);

	bebop_velocity = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
	ar_pose = nh.subscribe("ar_pose_marker", 1, poseCallback);
	geometry_msgs::Twist vel;


	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}


	// ros::shutdown();
	return 0;
}

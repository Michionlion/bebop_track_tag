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
// ros::Duration interval(1.5);
bool visible;

typedef struct vec{
	vec(double X, double Y, double Z) {
		x = X;
		y = Y;
		z = Z;
	}
	double x,y,z;
} vec;

vec offset(0.4,0.0,0.0);
double move_speed = 1; // higher than 1 = slower, lower than 1 = inaccurate but faster
geometry_msgs::Twist move;


void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
	//markers is a vector<AlvarMarker>
	if(msg->markers.empty()) return;
	for(int i=0; i < msg->markers.size(); i++) {
		ar_track_alvar_msgs::AlvarMarker marker = msg->markers.at(i);

		if(marker.id == 8) {
			geometry_msgs::Pose pose = marker.pose.pose;

			double forward = pose.position.x - offset.x;
			double left = -(pose.position.y - offset.y);
			double up = pose.position.z - offset.z;

			double dis = sqrt(forward*forward+left*left+up*up);

			//normalize
			forward /= dis * move_speed;
			left /= dis * move_speed;
			up /= dis * move_speed;

			fprintf(stdout, "---\nDIS: %f\nPOS:\nX: %f\nY: %f\nZ: %f\n\n", dis, pose.position.x, pose.position.y, pose.position.z);
			fprintf(stdout, "MOV:\nforward: %f\nleft: %f\nup: %f\n---\n", forward, left, up);

			move.linear.x = forward;
			move.linear.y = left;
			move.linear.z = up;
			move.angular.z = 0;
			bebop_velocity.publish(move);

		}


	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_tag");
	ros::NodeHandle nh;

	ros::Rate r(30);

	bebop_velocity = nh.advertise<geometry_msgs::Twist>("bebop/debug_twist", 1);
	ar_pose = nh.subscribe("ar_pose_marker", 1, poseCallback);
	geometry_msgs::Twist vel;


	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}


	ros::shutdown();
	return 0;
}

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <stdio.h>
#include <string.h>


ros::Publisher bebop_velocity;
ros::Subscriber ar_pose;
ros::Duration interval(5.0);
bool visible;

void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {

	//markers is a vector<AlvarMarker>

	if(msg->markers.size() > 0) {

		ROS_INFO("ID: %d", msg->markers[0].id);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_tag");
	ros::NodeHandle nh;s

	ros::Rate r(30);

	bebop_velocity = nh.advertise<geometry_msgs::Twist>("bebop/debug_twist", 1);
	ar_pose = nh.subscribe("ar_pose_marker", 1, poseCallback);
	geometry_msgs::Twist vel;

	ros::spin();

	while(ros::ok()) {
		try {

		} catch(tf2::LookupException ex) {
			ROS_ERROR("Exception: %s", ex.what());
		} catch(tf2::ExtrapolationException ex) {
			ROS_ERROR("Exception: %s", ex.what());
		}
		r.sleep();

		ros::spinOnce();
	}

	ros::shutdown();
	return 0;
}

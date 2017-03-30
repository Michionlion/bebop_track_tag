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
	//if(!visible) {
	//	ROS_ERROR("MARKER NOT VISIBLE");
	//	return;
	//}

	ROS_INFO("ID: %d", msg->markers[0].id);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_tag");
	ros::NodeHandle nh;

	tf::TransformListener tfl;

	ros::Rate r(30);

	bebop_velocity = nh.advertise<geometry_msgs::Twist>("bebop/debug_twist", 1);
	ar_pose = nh.subscribe("ar_pose_marker", 1, poseCallback);
	geometry_msgs::Twist vel;

	while(ros::ok()) {
		try {
			tfl.lookupTwist("camera_optical", "ar_marker_8", ros::Time(0), interval, vel);
			bebop_velocity.publish(vel);

			visible = true;
		} catch(tf2::LookupException ex) {
			visible = false;
			ROS_ERROR("Exception: %s", ex.what());
		} catch(tf2::ExtrapolationException ex) {
			visible = false;
			ROS_ERROR("Exception: %s", ex.what());
		}
		r.sleep();

		ros::spinOnce();
	}

	ros::shutdown();
	return 0;
}

#include "ros/ros.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_tag");
	ros::NodeHandle nh;

	ros::Rate r(60);


	while(ros::ok()) {
		ros::spinOnce();

		r.sleep();
	}

	ros::shutdown();
	return 0;
}

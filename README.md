# Installing dependencies

The bebop_track_tag node requires several ROS packages. 
Install the following:
- [bebop_driver](http://wiki.ros.org/bebop_autonomy)
- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)

# Installing bebop_track_tag

Clone this repository into the `src` directory of a catkin workspace (optimally, a new workspace). Then, run `$ catkin_make`. 

Configure the parameters within the `launch/alvar.launch` file, if desired. (The current configuration assumes an AR tag marker size of 20 cm).

# Running bebop_track_tag

To run the track_tag node, the following steps must be completed:

First, the bebop_driver node must be started, using `$ roslaunch bebop_driver bebop_node.launch`.

Then, you will need to start the ar_track_alvar package libraries.
We have provided a launch file that does default configuration and should work for any AR tag that is 20 cm in size. To launch it, use `$ roslaunch bebop_track_tag alvar.launch`.

Finally, you can launch the track_tag node by using `$ rosrun bebop_track_tag track_tag`.

This will start tracking, however if the bebop is grounded the velocity commands will not be obeyed, so the bebop must be launched by either using the [bebop_teleop](https://github.com/Michionlion/bebop_teleop) package, or by running `$ rostopic pub --once bebop/takeoff std_msgs/Empty`.

The teleop package is highly recommended, as it allows you to set the starting point of the tracking by positioning the drone before starting tracking. However, you will need to disable velocity publishing when using the tracking. To do this, press `0` in the teleop input window.

Additionally, you may want to see a view of the camera to ensure the AR tag is in view. To do so, you can use the image_view node, `$ rosrun image_view image_view image:=bebop/image_raw compressed`.

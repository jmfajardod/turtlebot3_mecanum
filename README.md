# Files implementing the localization and navigation of a Turtlebot3 with mecanum wheels

Code of ROS and Arduino packages for the turtlebot3 waffle with four mecanum wheels.

Some fixes for the compiler of Arduino for the OpenCR board are documented in the fixes folder.

Because the original robot only has two wheels, some changes have been made in the turtlebot3_msgs, the modified versions are located in the turtlebot3_msgs_mecanum folder which need to replace the original in the turtlebot3_msgs package. Also because this messages are used in the arduino implementation it is needed to run the command:

><code>  cd arduino_sketchbook/libraries</code> 
>
><code>	rm -rf ros_lib </code> 
>
><code>	rosrun rosserial_arduino make_libraries.py </code> 

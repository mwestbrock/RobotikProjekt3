# PlotbotEncoderRosSerial

Code for nodemcu attached to a simple portal robot with rotary encoders on each axis.

This reads the position of the robot head and publishes them via the serial interface as messages that can be relayed to a ROS2 network via the code in the [ros2_serial_example repo](https://github.com/osrf/ros2_serial_example).
The message description can be found at [ro45 portalrobot interfaces repo](https://github.com/matl-hsk/ro45_portalrobot_interfaces).


## Building
1. Check out the code including submodule
2. Open the folder in platformIO
3. Run build and upload to the nodemcu board with the encoders attached to the pins mentioned in the pin_definitions.h header.
4. Run ros2_serial_example with config file adapted to the serial port and plotbot position messages.

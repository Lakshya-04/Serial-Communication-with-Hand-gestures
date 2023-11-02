# Serial-Communication-with-Hand-gestures
Here i am Using ROS serial to establish communication between my python script and the Arduino car. I was facing issues with Pyserial so thought to shift to ros.
I am basically using Mediapipe to track my hand and detect what gesture it is, eg. One, Two, Three, Four, Five , Yo. 
Then that is sent to a Two wheeled basic Arduino car which has basic motor control functions which are called when the particular gesture is received.

Steps to get this running(after installing Ros in Ubuntu 20.04):
S1) roscore
S2) rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=9600 ( setting up port number and baud rate)
S3) rosrun lakshya_bot_pkg file_name.py ( package name, file name)

Troubleshooting:
1) Test with different Arduino Cable
2) Check if Roscore is running or not
3) Don't open serial monitor in Arduino Ide, rather print in the python terminal itself

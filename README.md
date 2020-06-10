# Object Catching Robot
* This work was conducted as a study of future veichle and robot capstone design, a subject of Department of Software Convergence.

![rviz_screenshot_2019_11_11-09_01_40](https://user-images.githubusercontent.com/45928371/84251252-10701f80-ab48-11ea-8975-5bfe35a657cc.png)
Use rviz to check if data is coming out right

## Getting Started
### Dependencies
#### HardWare
* Microsoft Azure Kinect
* Raspberry Pi 3 B+
* Arduino Board
* Robot Platform
#### Software
* Linux (Tested in Ubuntu 18.04, Mint 19)
> * ROS
>> * vision_opencv
>> * message_filters
>> * cv_bridge
>> * sensor_msgs
>> * geometry_msgs(for visualization)
>> * k4a
> * Azure Kinect SDK
* Raspbian Buster
> * ROS
> * ros_serial

### Run
* First you need to obtain the HSV color value for the object you want to catch. Modifiy the value in percept_object.cpp\
* If you downloaded all the dependencies and hardware setting, first run ROS
`$roscore`
* And execute
`$rosrun <package name> catching_robot`
* Also turn on the power of the arduino board with Raspberry Pi, upload the arduino code in the Pi for receive data.
* When connected, you can test and calib some macros in percept.object.cpp

## License
* [MIT](https://choosealicense.com/licenses/mit/)

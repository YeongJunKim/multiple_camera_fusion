#### ros_qt_basic_image_view

##### **Requirement**

###### 1. Install ROS package: [IntelRealSense/realsense-ros](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwj0-IKpn8_tAhWOvpQKHbWzDjAQFjABegQIBBAC&url=https%3A%2F%2Fgithub.com%2FIntelRealSense%2Frealsense-ros&usg=AOvVaw1ponQvWssrtydQInjvGcEZ)
I was uisng the realsense2 SDK version as **2.40.0** and realsense-ros package as **2.20.0**


###### 2. Install qt requirements: 
* qt-build
```
sudo apt-get install ros-melodic-qt-build
```
* libqt4-dev
```
sudo apt-get install libqt4-dev
```

###### 3. ROS, execute node

* run
```
rosrun multi_cam_fusion cam_fusion_node
```
#### ROS install

1. 스크립트를 다운받아 ROS 설치 후 재부팅
```
~$ sudo apt update
~$ sudo apt upgrade
~$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
~$ chmod 755 ./install_ros_melodic.sh 
~$ bash ./install_ros_melodic.sh
~$ sudo reboot

재부팅 후 테스트
~$ roscore
```
2. 스크립트 설치가 완료되면, src 폴더로 이동.
```
~& ~/catkin_ws
```
3. 해당 폴더에 포함되어있는 디렉토리
```
~$ sudo apt-get install tree
~$ tree -d -L 1
.
├── build
├── devel
└── src
```
4. src 폴더로 이동
```
~$ cd src
```
src 폴더에 있는 패키지들을 catkin_make를 통해 빌드를 해야 실행시킬 수 있습니다.
5. 이번에 제작한 패키지를 포함하여 필요한 패키지는 다음과 같습니다.
```
~$ tree -d -L 1
.
├── lepton_camera               <Thermal camera package>
├── multiple_camera_fusion      <3개 카메라 통합 뷰어 UI package>
├── realsense-ros               <color 와 depth 카메라 뷰어 package>
├── ddynamic_reconfigure        <realsense-ros package에 필요한 추가 package>
```
* lepton_camera 패키지를 **정확히** 설치학 위해서는 해당 설치 방법이 나와있는 페이지를 꼭 숙지하셔야합니다. 해당 페이지의 libuvc.so 파일의 include path 를 설정해야 할 수도 있습니다.
* 여기서 realsense-ros 패키지를 **정확히** 설치하기 위해서는 README.md의 버전을 꼭 확인하시고 설치하셔야합니다. 현재 사용한 버전은 realsense2 SDK의 경우 **2.40.0** 이고 realsense-ros 패키지의 경우 **2.20.0** 입니다.
* ddynamic_reconfigure 패키지는 [링크](https://github.com/pal-robotics/ddynamic_reconfigure) 여기에서 다운받으시면 됩니다.
* multiple_camera_fusion은 현재 보고계신 패키지입니다.
6. 위의 3개의 패키지에 대한 내용을 **build** 해야지만 비로소 roslaunch 또는 rosrun 명령어를 통해 해당 패키지의 노드를 실행시킬 수 있습니다.
```
~$ cd ~/catkin_ws                   <해당 폴더의 하위 폴더인 src에 대하여 빌드하기 위해 workspace 접근>
~$ catkin_make                      <build 하기>
```
7. 만약 노드를 찾을 수 없다는 에러가 나올 겅우
```
~$ rospack profile
또는
~$ cd ~/catkin_ws 
~$ sudo rm -r /build /devel
~$ catkin_make
```
8. 참고
catkin_ws 폴더 내부 구조
```
~$ cd ~/catkin_ws
~$ tree -d -L 2
├── build
│   ├── CMakeFiles
│   ├── atomic_configure
│   ├── catkin
│   ├── catkin_generated
│   ├── ddynamic_reconfigure
│   ├── gtest
│   ├── lepton_camera
│   ├── multiple_camera_fusion
│   ├── realsense-ros
│   ├── test_results
│   └── usb_cam
├── devel
│   ├── include
│   ├── lib
│   └── share
└── src
    ├── ddynamic_reconfigure
    ├── lepton_camera
    ├── multiple_camera_fusion
    ├── realsense-ros
```


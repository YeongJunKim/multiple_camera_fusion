/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/imageView_example/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace enc = sensor_msgs::image_encodings;

namespace imageView_example {
bool isRecv = false;
bool isIrRecv = false;
bool isLidarRecv = false;

int mode = MODE_ONLY_COLOR;

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"kuro_vision");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
        image_color_sub = it.subscribe("/camera/color/image_raw", 100, &QNode::imageCallback, this);
        image_lidar_sub = it.subscribe("/camera/depth/image_rect_raw", 100, &QNode::LidarImageCallback, this);
    start();
    return true;
}


void QNode::run() {
    ros::Rate loop_rate(33);

    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg_img)
{
    if(img_qnode == NULL && !isRecv)
    {
        img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img,enc::BGR8)->image);
        if(img_qnode != NULL)
        {
            isRecv = true;
            Q_EMIT recvImg();
        }
    }
}
void QNode::IrImageCallback(const sensor_msgs::ImageConstPtr& msg_img)
{
  if(ir_img_qnode == NULL && !isIrRecv)
  {
      ir_img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img,enc::BGR8)->image);

      if(ir_img_qnode != NULL)
      {
        isIrRecv = true;
        Q_EMIT recvImgIr();
      }
  }
}
void QNode::LidarImageCallback(const sensor_msgs::ImageConstPtr& msg_img)
{
//  ROS_INFO("LIDAR calblack");
  if(lidar_img_qnode == NULL && !isLidarRecv)
  {
lidar_img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img,enc::TYPE_16UC1)->image);
//      ROS_INFO("lidar : col %d, row %d", lidar_img_qnode->cols, lidar_img_qnode->rows);
      if(lidar_img_qnode != NULL)
      {
        isLidarRecv = true;
        Q_EMIT recvImgLidar();
      }
  }
}

}  // namespace imageView_example

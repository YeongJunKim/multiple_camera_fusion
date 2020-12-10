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
#include "../include/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace enc = sensor_msgs::image_encodings;

namespace imageView_example {
bool isRecv = false;
bool isIrRecv = false;
bool is16IrRecv = false;
bool isLidarRecv = false;

int mode = MODE_COLOR_THERMAL;

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
//        image_color_sub = it.subscribe("/usb_cam/image_raw", 100, &QNode::imageCallback, this);
        image_lidar_sub = it.subscribe("/camera/depth/image_rect_raw", 100, &QNode::LidarImageCallback, this);
//        image_lidar_sub = it.subscribe("/depth/out", 100, &QNode::LidarImageCallback, this);
//        image_ir_sub = it.subscribe("/lepton/out", 100, &QNode::IrImageCallback, this);
        image_ir_sub = it.subscribe("/ir/usb_cam/image_raw", 100, &QNode::IrImageCallback, this);
        image_ir_16bit_img_sub = it.subscribe("/lepton/out", 100, &QNode::Ir16bImageCallbcak, this);
    image_color_pub = it.advertise("/fusion/color/out",1);
    image_depth_pub = it.advertise("/fusion/depth/out",1);
    image_ir_pub    = it.advertise("/fusion/ir/out",1);
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
//      ir_img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img,enc::TYPE_8UC1)->image);
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
lidar_img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img)->image);
      if(lidar_img_qnode != NULL)
      {
        isLidarRecv = true;
        Q_EMIT recvImgLidar();
      }
  }
}

void QNode::Ir16bImageCallbcak(const sensor_msgs::ImageConstPtr& msg_img)
{
  if(ir_16bit_img_qnode == NULL && !is16IrRecv)
  {
    ROS_INFO("16 ir1");
      ir_16bit_img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img)->image);
      if(ir_16bit_img_qnode != NULL)
      {
        ROS_INFO("16 ir2");
        is16IrRecv = true;
        Q_EMIT recvImgIr16();
      }
  }
}

}  // namespace imageView_example

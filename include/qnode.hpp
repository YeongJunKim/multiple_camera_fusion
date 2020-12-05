/**
 * @file /include/imageView_example/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef imageView_example_QNODE_HPP_
#define imageView_example_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#endif

#define MODE_NONE           0
#define MODE_ONLY_COLOR     1
#define MODE_ONLY_THERMAL   2
#define MODE_ONLY_LIDAR     3
#define MODE_COLOR_THERMAL  4
#define MODE_COLOR_LIDAR    5

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace imageView_example {

/*****************************************************************************
** Class
*****************************************************************************/


class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();
    cv::Mat *img_qnode;
    cv::Mat *ir_img_qnode;
    cv::Mat *lidar_img_qnode;

Q_SIGNALS:
    void recvImg();
    void recvImgIr();
    void recvImgLidar();
    void rosShutdown();
private:
    int init_argc;
    char** init_argv;

    image_transport::Subscriber image_color_sub;
    image_transport::Subscriber image_ir_sub;
    image_transport::Subscriber image_lidar_sub;

    void imageCallback(const sensor_msgs::ImageConstPtr &msg_img);
    void IrImageCallback(const sensor_msgs::ImageConstPtr& msg_img);
    void LidarImageCallback(const sensor_msgs::ImageConstPtr& msg_img);
};

}  // namespace imageView_example

#endif /* imageView_example_QNODE_HPP_ */

#ifndef MY_FLIR_H
#define MY_FLIR_H

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

typedef struct minmaxloc
{
  double min;
  double max;
  cv::Point min_point;
  cv::Point max_point;
}minmaxloc_t;

class my_flir
{
public:
  my_flir();

  double ktof(int val);

  double ktoc(int val);

  void display(cv::Mat *img, cv::Rect rect, cv::Scalar scalar, int thickness, int linetype, int shift);

  minmaxloc_t min_max_location(cv::Mat *img_, cv::Rect interestmask);

};

#endif // MY_FLIR_H

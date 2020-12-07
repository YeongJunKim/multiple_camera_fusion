#include "my_flir.h"

my_flir::my_flir()
{

}

double my_flir::ktof(int val)
{
  return 1.8 * ktoc(val) + 32.0;
}

double my_flir::ktoc(int val)
{
  return (val - 27315) / 100.0;
}


void my_flir::display(cv::Mat *img, cv::Rect rect, cv::Scalar scalar, int thickness, int linetype, int shift)
{

}

minmaxloc_t my_flir::min_max_location(cv::Mat *img_, cv::Rect interestmask)
{
  minmaxloc_t returntype;

  cv::Mat img = *img_;
  cv::Mat gray, roi;
  cv::cvtColor(img, gray, CV_RGB2GRAY);
  cv::Rect bounds(0,0,img.cols, img.rows);
  roi = gray(interestmask & bounds);

  cv::Point offset(interestmask.x,interestmask.y);
  double minVal = 0;
  double maxVal = 0;
  cv::Point minPoint;
  cv::Point maxPoint;
  cv::minMaxLoc(roi,&returntype.min,&returntype.max,&returntype.min_point,&returntype.max_point);

  returntype.min_point = returntype.min_point + offset;
  returntype.max_point = returntype.max_point + offset;

  ROS_INFO("min=%f,max=%f,minP=(%d,%d), maxP=(%d,%d)", minVal, maxVal,minPoint.x,minPoint.y,maxPoint.x,maxPoint.y);




  return returntype;
}

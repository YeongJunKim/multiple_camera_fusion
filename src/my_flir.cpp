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
  cv::Mat gray, roi, normalized;
  cv::cvtColor(img, gray, CV_RGB2GRAY);

  gray.convertTo(normalized, CV_16U, 255);

  cv::Rect bounds(0,0,img.cols, img.rows);

  roi = normalized(interestmask & bounds);

  cv::Point offset(interestmask.x,interestmask.y);
  cv::minMaxLoc(roi,&returntype.min,&returntype.max,&returntype.min_point,&returntype.max_point);

  returntype.min_point = returntype.min_point + offset;
  returntype.max_point = returntype.max_point + offset;
  returntype.min_degC = pixel2degC(returntype.min);
  returntype.max_degC = pixel2degC(returntype.max);
//  returntype.min_degC = pixel2degC((double)(roi.at<uint16_t>(returntype.min_point)));
//  returntype.max_degC = pixel2degC((double)(roi.at<uint16_t>(returntype.max_point)));

//  ROS_INFO("min_deg = %f, max_deg = %f", returntype.min_degC, returntype.max_degC);

//  ROS_INFO("min=%f,max=%f,minP=(%d,%d), maxP=(%d,%d)", minVal, maxVal,minPoint.x,minPoint.y,maxPoint.x,maxPoint.y);




  return returntype;
}

double my_flir::pixel2degC(double val)
{
  ROS_INFO("val = %f", val);
  return (val - 27315.0) / 100.0;
}

double my_flir::pixel2degF(double val)
{
  return (1.8 * pixel2degC(val)+32.0);
}

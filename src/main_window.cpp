/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>

#include <iostream>
#include "../include/main_window.hpp"
#include "../include/my_qlabel.h"
#include "../include/my_flir.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace imageView_example {
extern bool isRecv;
extern bool isIrRecv;
extern bool is16IrRecv;
extern bool isLidarRecv;
extern int mode;

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  qnode.init();

  if(qnode.img_qnode != NULL) qnode.img_qnode = NULL;
  if(qnode.ir_img_qnode != NULL) qnode.ir_img_qnode = NULL;
  if(qnode.ir_16bit_img_qnode != NULL) qnode.ir_16bit_img_qnode = NULL;
  if(qnode.lidar_img_qnode != NULL) qnode.lidar_img_qnode = NULL;

  QObject::connect(&qnode, SIGNAL(recvImg()), this, SLOT(updateImg()));
  QObject::connect(&qnode, SIGNAL(recvImgIr()), this, SLOT(updateImgIr()));
  QObject::connect(&qnode, SIGNAL(recvImgLidar()), this, SLOT(updateImgLidar()));
  QObject::connect(&qnode, SIGNAL(recvImgIr16()), this, SLOT(updateImgIr16()));

  connect(ui.labelOrg, SIGNAL(Mouse_Pos()), this, SLOT(Mouse_current_pos()));
  connect(ui.labelOrg, SIGNAL(Mouse_Pressed()), this, SLOT(Mouse_Pressed()));
  connect(ui.labelOrg, SIGNAL(Mouse_Left()), this, SLOT(Mouse_left()));
  connect(ui.labelOrg, SIGNAL(Mouse_Released()),this, SLOT(Mouse_Released()));

  ui.horizontalSlider_IR_leftside->setValue(ui.lineEdit_IR_roi_leftside->text().toInt());
  ui.horizontalSlider_IR_rightside->setValue(ui.lineEdit_IR_roi_rightside->text().toInt());
  ui.horizontalSlider_IR_upperside->setValue(ui.lineEdit_IR_roi_upperside->text().toInt());
  ui.horizontalSlider_IR_bottomside->setValue(ui.lineEdit_IR_roi_bottomside->text().toInt());
  ui.horizontalSlider_depth_leftside->setValue(ui.lineEdit_depth_roi_leftside->text().toInt());
  ui.horizontalSlider_depth_rightside->setValue(ui.lineEdit_depth_roi_rightside->text().toInt());
  ui.horizontalSlider_depth_upperrside->setValue(ui.lineEdit_depth_roi_upperside->text().toInt());
  ui.horizontalSlider_depth_bottomside->setValue(ui.lineEdit_depth_roi_bottomside->text().toInt());
  ui.horizontalSlider_color_leftside->setValue(ui.lineEdit_color_roi_leftside->text().toInt());
  ui.horizontalSlider_color_rightside->setValue(ui.lineEdit_color_roi_rightside->text().toInt());
  ui.horizontalSlider_color_upperside->setValue(ui.lineEdit_color_roi_upperside->text().toInt());
  ui.horizontalSlider_color_bottomside->setValue(ui.lineEdit_color_roi_bottomside->text().toInt());

  myModel = new QStandardItemModel(0,0,this);
  QStringList horzHeaders;
  horzHeaders << "id" << "x1" << "y1" << "x2" << "y2" << "MAX" << "MIN" << "MAX_x" << "MAX_y" << "MIN_x" << "MIN_y" << "threshold";

  myModel->setHorizontalHeaderLabels(horzHeaders);

  ui.tableView->setModel(myModel);
  ui.tableView->resizeColumnsToContents();
  ui.tableView->resizeRowsToContents();

  //    QStandardItem *item1 = new QStandardItem("Test");
  //    QStandardItem *item2 = new QStandardItem("Test");
  //    QStandardItem *item3 = new QStandardItem("Test");
  //    myModel->setItem(0,0,item1);
  //    myModel->setItem(0,2,item2);
  //    myModel->setItem(2,0,item3);





  m_pTimer = make_shared<QTimer>();
  connect(m_pTimer.get(), SIGNAL(timeout()), this, SLOT(OnTimerCallbackFunction()));
  m_pTimer->start(100);
}

MainWindow::~MainWindow() {}

void MainWindow::updateImg()
{
  cv::Mat dst, crop, roi, output;

  int l,r,u,b;
  l = ui.lineEdit_color_roi_leftside->text().toInt();
  r = ui.lineEdit_color_roi_rightside->text().toInt();
  u = ui.lineEdit_color_roi_upperside->text().toInt();
  b = ui.lineEdit_color_roi_bottomside->text().toInt();
  int width  = ui.labelOrg->width();
  int height = ui.labelOrg->height();
  cv::resize(*qnode.img_qnode, dst, cv::Size(width,height),0,0,CV_INTER_NN);
  int x1,x2,y1,y2;
  x1 = l;  y1 = u;  x2 = width - r;  y2 = height - b;
  cv::Rect bounds(0,0,width, height);
  cv::Rect intermask(cv::Point(x1,y1), cv::Point(x2,y2));
  roi = dst(intermask & bounds);
  cv::resize(roi, output, cv::Size(width,height),0,0,CV_INTER_NN);

  Mat imgOrg(output);
  drawLabels(&imgOrg,1);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",imgOrg).toImageMsg();
  qnode.image_color_pub.publish(msg);


  if(!qnode.img_qnode->empty() && !imgOrg.empty() && isRecv)
  {
    if(mode == MODE_ONLY_COLOR)
    {
      QImage qimageOrg((const unsigned char*)(imgOrg.data), imgOrg.cols, imgOrg.rows, QImage::Format_RGB888);
      ui.labelOrg->setPixmap(QPixmap::fromImage(qimageOrg.rgbSwapped()));
    }
  }
  delete qnode.img_qnode;
  if(qnode.img_qnode != NULL) qnode.img_qnode = NULL;
  isRecv = false;
}
void MainWindow::updateImgIr()
{
  ROS_INFO("debug1");
}

void MainWindow::updateImgLidar()
{
  //  ROS_INFO("update IR");
  cv::Mat dst, roi, output;
  Mat temp(*qnode.lidar_img_qnode);

  int l,r,u,b;
  l = ui.lineEdit_depth_roi_leftside->text().toInt();
  r = ui.lineEdit_depth_roi_rightside->text().toInt();
  u = ui.lineEdit_depth_roi_upperside->text().toInt();
  b = ui.lineEdit_depth_roi_bottomside->text().toInt();
  int width = ui.labelOrg->width();
  int height = ui.labelOrg->height();
  int x1,x2,y1,y2;
  x1 = l;  y1 = u;  x2 = width - r;  y2 = height - b;
  cv::Rect bounds(0,0,width, height);
  cv::Rect intermask(cv::Point(x1,y1), cv::Point(x2,y2));





  cv::resize(*qnode.lidar_img_qnode, dst, cv::Size(width,height),0,0,CV_INTER_NN);
  roi = dst(intermask & bounds);
  cv::resize(roi, output, cv::Size(width,height),0,0,CV_INTER_NN);
  // 1920 1080
  // 1024 768
  // width height
  // 1920:1080=width:x
  // 1080 * 1024 /1920 = 576
  cv::Mat norm, color, t8;
  cv::normalize(output,norm,0,255,NORM_MINMAX);
  norm.convertTo(t8, CV_8UC1);
  cv::cvtColor(t8,color, cv::COLOR_GRAY2RGB);
  Mat imgOrg(color);
  drawLabels(&imgOrg,0);


  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8",imgOrg).toImageMsg();
  qnode.image_depth_pub.publish(msg);

  if(!qnode.lidar_img_qnode->empty() && !imgOrg.empty() && isLidarRecv)
  {
    if(mode == MODE_ONLY_LIDAR)
    {
      QImage qimageOrg((const unsigned char*)(imgOrg.data), imgOrg.cols, imgOrg.rows, QImage::Format_RGB888);
      ui.labelOrg->setPixmap(QPixmap::fromImage(qimageOrg));
    }
  }
  delete qnode.lidar_img_qnode;
  if(qnode.lidar_img_qnode != NULL) qnode.lidar_img_qnode = NULL;
  isLidarRecv = false;
}

void MainWindow::updateImgIr16()
{
  ROS_INFO("hello");
  cv::Mat get(*qnode.ir_16bit_img_qnode);
  cv::Mat dst, norm, t8, roi, output;


  int l,r,u,b;
  l = ui.lineEdit_IR_roi_leftside->text().toInt();
  r = ui.lineEdit_IR_roi_rightside->text().toInt();
  u = ui.lineEdit_IR_roi_upperside->text().toInt();
  b = ui.lineEdit_IR_roi_bottomside->text().toInt();
  int width = ui.labelOrg->width();
  int height = ui.labelOrg->height();
  cv::resize(get, dst, cv::Size(width,height),0,0,CV_INTER_NN);
  int x1,x2,y1,y2;
  x1 = l;  y1 = u;  x2 = width - r;  y2 = height - b;
  cv::Rect bounds(0,0,width, height);
  cv::Rect intermask(cv::Point(x1,y1), cv::Point(x2,y2));
  roi = dst(intermask & bounds);
  cv::resize(roi, output, cv::Size(width,height),0,0,CV_INTER_NN);


  cv::Mat color = updateLabelsGray(&output);
  drawLabels(&color,0);



  Mat imgOrg(color);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8",imgOrg).toImageMsg();
  qnode.image_ir_pub.publish(msg);
  if(!qnode.ir_16bit_img_qnode->empty() && !imgOrg.empty() && is16IrRecv)
  {
    if(mode == MODE_COLOR_THERMAL)
    {
      QImage qimageOrg((const unsigned char*)(imgOrg.data), imgOrg.cols, imgOrg.rows, QImage::Format_RGB888);
      ui.labelOrg->setPixmap(QPixmap::fromImage(qimageOrg));
    }
  }
  delete qnode.ir_16bit_img_qnode;
  if(qnode.ir_16bit_img_qnode != NULL) qnode.ir_16bit_img_qnode = NULL;
  is16IrRecv = false;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

void MainWindow::on_button_update_clicked()
{
  ROS_INFO("ON_BUTTON_UPDATE_CLICKED");
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);

  QStringListModel *model = new QStringListModel();
  QStringList list;

  for (unsigned long i = 0; i < topic_infos.size(); i++) {
    string a = topic_infos.at(i).name;
    QString qstr = QString::fromStdString(a);
    a = topic_infos.at(i).datatype;
    std::cout<<a<<std::endl;
    list << qstr;
  }
  model->setStringList(list);
  ui.listView_topic_list->setModel(model);
  //  ui.listView_topic_list.setModel(model);
}

void imageView_example::MainWindow::on_checkBox_only_color_clicked()
{
  ui.checkBox_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_thermal->setCheckState(Qt::CheckState::Unchecked);
  //  ui.checkBox_only_color->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_gray_thermal->setCheckState(Qt::CheckState::Unchecked);
  mode = MODE_ONLY_COLOR;
  ROS_INFO("MODE_ONLY_COLOR");
}

void imageView_example::MainWindow::on_checkBox_only_thermal_clicked()
{

  ui.checkBox_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_color->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_lidar->setCheckState(Qt::CheckState::Unchecked);
  //  ui.checkBox_only_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_gray_thermal->setCheckState(Qt::CheckState::Unchecked);
  mode = MODE_ONLY_THERMAL;
  ROS_INFO("MODE_ONLY_THERMAL");
}

void imageView_example::MainWindow::on_checkBox_only_lidar_clicked()
{

  ui.checkBox_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_color->setCheckState(Qt::CheckState::Unchecked);
  //  ui.checkBox_only_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_gray_thermal->setCheckState(Qt::CheckState::Unchecked);
  mode = MODE_ONLY_LIDAR;
  ROS_INFO("MODE_ONLY_LIDAR");
}

void imageView_example::MainWindow::on_checkBox_thermal_clicked()
{

  ui.checkBox_lidar->setCheckState(Qt::CheckState::Unchecked);
  //  ui.checkBox_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_color->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_gray_thermal->setCheckState(Qt::CheckState::Unchecked);
  mode = MODE_COLOR_THERMAL;
  ROS_INFO("MODE_COLOR_THERMAL");
}

void imageView_example::MainWindow::on_checkBox_lidar_clicked()
{

  //  ui.checkBox_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_color->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_gray_thermal->setCheckState(Qt::CheckState::Unchecked);
  mode = MODE_COLOR_LIDAR;
  ROS_INFO("MODE_COLOR_LIDAR");
}

void imageView_example::MainWindow::on_checkBox_gray_thermal_clicked()
{
  ui.checkBox_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_thermal->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_color->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_lidar->setCheckState(Qt::CheckState::Unchecked);
  ui.checkBox_only_thermal->setCheckState(Qt::CheckState::Unchecked);
  mode = MODE_ONLY_THERMAL_GRAY;
  ROS_INFO("MODE_ONLY_THERMAL_GRAY");
}


float imageView_example::MainWindow::map(float value, float istart, float istop, float ostart, float ostop) {
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

void MainWindow::Mouse_current_pos()
{
  ROS_INFO("current_pos X=%d, Y=%d", ui.labelOrg->x, ui.labelOrg->y);
}

void MainWindow::Mouse_Pressed()
{
  ROS_INFO("Mouse Pressed X=%d, Y=%d", ui.labelOrg->x, ui.labelOrg->y);
  if(this->button_flag == 1)
  {
    ui.lineEdit_x1->setText(QString::number(ui.labelOrg->x));
    ui.lineEdit_y1->setText(QString::number(ui.labelOrg->y));
    button_flag = 2;
  }
}

void MainWindow::Mouse_left()
{
  ROS_INFO("Mouse Left");
}

void MainWindow::Mouse_Released()
{
  ROS_INFO("Mouse Released");
  if(this->button_flag == 2)
  {
    ui.lineEdit_x2->setText(QString::number(ui.labelOrg->x));
    ui.lineEdit_y2->setText(QString::number(ui.labelOrg->y));
    ui.button_delete_id->setDisabled(false);
    ui.button_update_clear->setDisabled(false);
    button_flag = 0;
  }
}


int imageView_example::MainWindow::checkMode()
{
  return 0;
}

void imageView_example::MainWindow::on_button_select_position_clicked()
{
  ROS_INFO("select button clicked");
  this->button_flag = 1;
  ui.button_delete_id->setDisabled(true);
  ui.button_update_clear->setDisabled(true);
}

void imageView_example::MainWindow::on_button_update_clear_clicked()
{
  ROS_INFO("clear button clicked");
  ui.tableView->model()->removeRows(0,ui.tableView->model()->rowCount());
}

void imageView_example::MainWindow::on_button_manually_add_clicked()
{
  ROS_INFO("add button clicked");
  QList<QStandardItem*> items;
  int id = rand()%1000;
  int x1,y1,x2,y2;
  double th;
  int x1_ = ui.lineEdit_x1->text().toInt();
  int y1_ = ui.lineEdit_y1->text().toInt();
  int x2_ = ui.lineEdit_x2->text().toInt();
  int y2_ = ui.lineEdit_y2->text().toInt();
  th = ui.lineEdit_threshold->text().toDouble();


  if(x1_ > x2_)
  {
      x1 = x2_;
      x2 = x1_;
  }
  else{
      x1 = x1_;
      x2 = x2_;
  }
  if(y1_ > y2_)
  {
    y1 = y2_;
    y2 = y1_;
  }
  else
  {
    y1 = y1_;
    y2 = y2_;
  }

  items.append(new QStandardItem(QString::number(id)));
  items.append(new QStandardItem(QString::number(x1)));
  items.append(new QStandardItem(QString::number(y1)));
  items.append(new QStandardItem(QString::number(x2)));
  items.append(new QStandardItem(QString::number(y2)));
  items.append(new QStandardItem(QString::number(0)));
  items.append(new QStandardItem(QString::number(0)));
  items.append(new QStandardItem(QString::number(0)));
  items.append(new QStandardItem(QString::number(0)));
  items.append(new QStandardItem(QString::number(0)));
  items.append(new QStandardItem(QString::number(0)));
  items.append(new QStandardItem(QString::number(th)));
  myModel->appendRow(items);


  ui.tableView->resizeColumnsToContents();
  ui.tableView->resizeRowsToContents();

  ui.lineEdit_x1->clear();
  ui.lineEdit_y1->clear();
  ui.lineEdit_x2->clear();
  ui.lineEdit_y2->clear();
}
void imageView_example::MainWindow::on_button_delete_id_clicked()
{
  ROS_INFO("delete button clicked");

  int id = ui.lineEdit_delete_id->text().toInt();

  for(int i = 0 ; i < ui.tableView->model()->rowCount(); i++)
  {
    int dst = ui.tableView->model()->index(i,0).data().toInt();
    if(id == dst)
    {
      ui.tableView->model()->removeRow(i);
    }
  }
}



void imageView_example::MainWindow::updateLabels(cv::Mat *img_)
{
}
void imageView_example::MainWindow::drawLabels(Mat *img_, int order)
{
  double secs = ros::Time::now().toSec();
  nowTick = static_cast<uint32_t>(secs * 1000.0);
  ROS_INFO("nowTick = %d, pastTick = %d", nowTick, pastTick);
  if(nowTick - pastTick > 100)
  {
    if(blinkflag == 0)
      blinkflag = 1;
    else
      blinkflag = 0;
    pastTick = nowTick;
  }

  Scalar *bluelabel;
  Scalar *redlabel;
  Scalar *thermalColor;
  Scalar *blacklabel = new Scalar(0,0,0);

  if(order == 0)
  {
    bluelabel = new Scalar(0,0,255);
    redlabel = new Scalar(255,0,0);
    thermalColor = new Scalar(100,0,50);
  }
  else
  {
    bluelabel = new Scalar(255,0,0);
    redlabel = new Scalar(0,0,255);
    thermalColor = new Scalar(50,0,100);
  }
  cv::Mat *img;
  img = img_;
  if(this->button_flag == 0) {
    int x1,x2,y1,y2;
    x1 = ui.lineEdit_x1->text().toInt();
    y1 = ui.lineEdit_y1->text().toInt();
    x2 = ui.lineEdit_x2->text().toInt();
    y2 = ui.lineEdit_y2->text().toInt();
    if(x1 * x2 * y1 * y2 == 0)
    {}
    else {
      rectangle(*img,Rect(Point(x1,y1),Point(x2,y2)),*bluelabel,3,4,0);
    }
  }
  if(this->button_flag == 2) {
    int x1,x2,y1,y2;
    x1 = ui.lineEdit_x1->text().toInt();
    y1 = ui.lineEdit_y1->text().toInt();
    x2 = ui.labelOrg->x;
    y2 = ui.labelOrg->y;
    rectangle(*img,Rect(Point(x1,y1),Point(x2,y2)),*bluelabel,3,4,0);
  }

  int rows = ui.tableView->model()->rowCount();

  if (rows > 0)
  {
    for(int i = 0; i < rows; i++) {
      int x1, x2, y1, y2;
      int id = ui.tableView->model()->index(i,0).data().toInt();
      x1 = ui.tableView->model()->index(i,1).data().toInt();
      y1 = ui.tableView->model()->index(i,2).data().toInt();
      x2 = ui.tableView->model()->index(i,3).data().toInt();
      y2 = ui.tableView->model()->index(i,4).data().toInt();
      int max_x = ui.tableView->model()->index(i,7).data().toInt();
      int max_y = ui.tableView->model()->index(i,8).data().toInt();
      int min_x = ui.tableView->model()->index(i,9).data().toInt();
      int min_y = ui.tableView->model()->index(i,10).data().toInt();


      double max_deg, min_deg, th;
      max_deg = ui.tableView->model()->index(i,5).data().toDouble();
      min_deg = ui.tableView->model()->index(i,6).data().toDouble();
      th = ui.tableView->model()->index(i,11).data().toDouble();
      ROS_INFO("max = %f, th = %f", max_deg,th);
      if(max_deg > th)
      {
        if(blinkflag)
          cv::rectangle(*img,Rect(Point(x1,y1),Point(x2,y2)),*redlabel,3,4,0);
        else
          cv::rectangle(*img,Rect(Point(x1,y1),Point(x2,y2)),*blacklabel,3,4,0);
      }
      else
      {
        cv::rectangle(*img,Rect(Point(x1,y1),Point(x2,y2)),Scalar(0,255,0),3,4,0);
      }


      cv::putText(*img,to_string(id), Point(x1, y1), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),3);
      cv::putText(*img,to_string(max_deg), Point(max_x, max_y), FONT_HERSHEY_SIMPLEX, 1,*thermalColor,3);
      cv::putText(*img,to_string(min_deg), Point(min_x, min_y), FONT_HERSHEY_SIMPLEX, 1,*thermalColor,3);

      minmaxloc_t mmloc;
      mmloc.max_point = cv::Point(ui.tableView->model()->index(i,7).data().toInt(),ui.tableView->model()->index(i,8).data().toInt());
      mmloc.min_point = cv::Point(ui.tableView->model()->index(i,9).data().toInt(),ui.tableView->model()->index(i,10).data().toInt());
      cv::circle(*img,mmloc.min_point,5,*bluelabel,3,4,0);
      cv::circle(*img,mmloc.max_point,5,*redlabel,3,4,0);
    }
  }
}

cv::Mat imageView_example::MainWindow::updateLabelsGray(Mat *gray_)
{
  cv::Mat *img;
  cv::Mat norm, t8, color,color2;
  img = gray_;
  cv::normalize(*img,norm,0,255, NORM_MINMAX);
  norm.convertTo(t8, CV_8UC1);
  cv::cvtColor(t8,color,cv::COLOR_GRAY2BGR);
  cv::cvtColor(t8,color2,cv::COLOR_GRAY2BGR);

  if(this->button_flag == 0) {
    int x1,x2,y1,y2;
    x1 = ui.lineEdit_x1->text().toInt();
    y1 = ui.lineEdit_y1->text().toInt();
    x2 = ui.lineEdit_x2->text().toInt();
    y2 = ui.lineEdit_y2->text().toInt();
    if(x1 * x2 * y1 * y2 == 0)
    {}
    else {
      rectangle(color,Rect(Point(x1,y1),Point(x2,y2)),Scalar(0,0,255),3,4,0);
    }
  }
  if(this->button_flag == 2) {
    int x1,x2,y1,y2;
    x1 = ui.lineEdit_x1->text().toInt();
    y1 = ui.lineEdit_y1->text().toInt();
    x2 = ui.labelOrg->x;
    y2 = ui.labelOrg->y;
    rectangle(color,Rect(Point(x1,y1),Point(x2,y2)),Scalar(0,0,255),3,4,0);
  }

  int rows = ui.tableView->model()->rowCount();

  if (rows == 0)
  {

  }
  else
  {
    for(int i = 0; i < rows; i++) {
      int x1, x2, y1, y2;
      int id = ui.tableView->model()->index(i,0).data().toInt();
      x1 = ui.tableView->model()->index(i,1).data().toInt();
      y1 = ui.tableView->model()->index(i,2).data().toInt();
      x2 = ui.tableView->model()->index(i,3).data().toInt();
      y2 = ui.tableView->model()->index(i,4).data().toInt();

      my_flir flir;
      minmaxloc_t mmloc;
      mmloc = flir.min_max_location_gray(img, Rect(Point(x1,y1),Point(x2,y2)));
      cv::circle(color,mmloc.min_point,5,Scalar(0,0,255),3,4,0);
      cv::circle(color,mmloc.max_point,5,Scalar(255,0,0),3,4,0);
      QStandardItem *max = new QStandardItem(QString::number(mmloc.max_degC));
      QStandardItem *min = new QStandardItem(QString::number(mmloc.min_degC));
      QStandardItem *max_x = new QStandardItem(QString::number(mmloc.max_point.x));
      QStandardItem *max_y = new QStandardItem(QString::number(mmloc.max_point.y));
      QStandardItem *min_x = new QStandardItem(QString::number(mmloc.min_point.x));
      QStandardItem *min_y = new QStandardItem(QString::number(mmloc.min_point.y));
      myModel->setItem(i,5,max);
      myModel->setItem(i,6,min);
      myModel->setItem(i,7,max_x);
      myModel->setItem(i,8,max_y);
      myModel->setItem(i,9,min_x);
      myModel->setItem(i,10,min_y);
      cv::rectangle(color,Rect(Point(x1,y1),Point(x2,y2)),Scalar(0,255,0),3,4,0);
      cv::putText(color,to_string(id), Point(x1, y1), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0),1);
    }
  }


  return color2;
}

void imageView_example::MainWindow::updateImage(cv::Mat *img_)
{
  cv::Mat *img;
  img = img_;
  switch(mode)
  {
  case MODE_ONLY_COLOR:

    break;
  case MODE_ONLY_LIDAR:

    break;
  case MODE_ONLY_THERMAL:

    break;
  }
}
void MainWindow::OnTimerCallbackFunction()
{
//  ROS_INFO("TimerCallbackFunction Called");
  // check thermal in labels

}



























void imageView_example::MainWindow::on_horizontalSlider_color_leftside_sliderMoved(int position)
{
  ui.lineEdit_color_roi_leftside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_color_rightside_sliderMoved(int position)
{
  ui.lineEdit_color_roi_rightside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_color_upperside_sliderMoved(int position)
{
  ui.lineEdit_color_roi_upperside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_color_bottomside_sliderMoved(int position)
{
  ui.lineEdit_color_roi_bottomside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_IR_leftside_sliderMoved(int position)
{
  ui.lineEdit_IR_roi_leftside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_IR_rightside_sliderMoved(int position)
{
  ui.lineEdit_IR_roi_rightside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_IR_upperside_sliderMoved(int position)
{
  ui.lineEdit_IR_roi_upperside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_IR_bottomside_sliderMoved(int position)
{
  ui.lineEdit_IR_roi_bottomside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_depth_leftside_sliderMoved(int position)
{
  ui.lineEdit_depth_roi_leftside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_depth_rightside_sliderMoved(int position)
{
  ui.lineEdit_depth_roi_rightside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_depth_upperrside_sliderMoved(int position)
{
  ui.lineEdit_depth_roi_upperside->setText(QString::number(position));
}
void imageView_example::MainWindow::on_horizontalSlider_depth_bottomside_sliderMoved(int position)
{
  ui.lineEdit_depth_roi_bottomside->setText(QString::number(position));
}
}  // namespace imageView_example







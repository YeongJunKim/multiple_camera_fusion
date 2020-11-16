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
#include "../include/imageView_example/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace imageView_example {
extern bool isRecv;
extern bool isIrRecv;
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
    if(qnode.lidar_img_qnode != NULL) qnode.lidar_img_qnode = NULL;

    QObject::connect(&qnode, SIGNAL(recvImg()), this, SLOT(updateImg()));
    QObject::connect(&qnode, SIGNAL(recvImgIr()), this, SLOT(updateImgIr()));
    QObject::connect(&qnode, SIGNAL(recvImgLidar()), this, SLOT(updateImgLidar()));


    int value = ui.horizontalSlider_threshold_thermal->value();
    QString qstr = QString::number(value);
    ui.lineEdit->setText(qstr);
    value = ui.horizontalSlider_threshold_distance->value();
    QString qstr2 = QString::number(value);
    ui.lineEdit_2->setText(qstr2);

}

MainWindow::~MainWindow() {}

void MainWindow::updateImg()
{
    cv::Mat dst, crop;



    int width  = ui.labelOrg->width();
    int height = ui.labelOrg->height();
    cv::resize(*qnode.img_qnode, dst, cv::Size(width,height),0,0,CV_INTER_NN);



//    Mat imgOrg(*qnode.img_qnode); //qnode-> receive
    Mat imgOrg(dst);

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
  isIrRecv = false;
}

void MainWindow::updateImgLidar()
{
//  ROS_INFO("update IR");
  cv::Mat dst;
  Mat temp(*qnode.lidar_img_qnode);

  int width = ui.labelOrg->width();
  int height = ui.labelOrg->height();
  // 1920 1080
  // 1024 768
  // width height
  // 1920:1080=width:x
  // 1080 * 1024 /1920 = 576
  int offset1 = 30;
  int offset2 =10;
  cv::Rect bounds(0,0+offset1,temp.cols,576+offset1-offset2);
  Mat roi = temp(bounds);


  cv::resize(roi, dst, cv::Size(width,height),0,0,CV_INTER_NN);

  Mat imgOrg(dst);

  if(!qnode.lidar_img_qnode->empty() && !imgOrg.empty() && isLidarRecv)
  {
    if(mode == MODE_ONLY_LIDAR)
    {
      QImage qimageOrg((const unsigned char*)(imgOrg.data), imgOrg.cols, imgOrg.rows, QImage::Format_RGB16);
      ui.labelOrg->setPixmap(QPixmap::fromImage(qimageOrg));
    }
  }
  delete qnode.lidar_img_qnode;
  if(qnode.lidar_img_qnode != NULL) qnode.lidar_img_qnode = NULL;

  isLidarRecv = false;
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
  mode = MODE_COLOR_LIDAR;
  ROS_INFO("MODE_COLOR_LIDAR");
}

void imageView_example::MainWindow::on_horizontalSlider_threshold_thermal_sliderMoved(int position)
{
  int value = ui.horizontalSlider_threshold_thermal->value();
  QString qstr = QString::number(value);
  ui.lineEdit->setText(qstr);
  ROS_INFO("value : %d", value);
}

void imageView_example::MainWindow::on_horizontalSlider_threshold_distance_sliderMoved(int position)
{
  int value = ui.horizontalSlider_threshold_distance->value();
  QString qstr = QString::number(value);
  ui.lineEdit_2->setText(qstr);
  ROS_INFO("value : %d", value);
}





int imageView_example::MainWindow::checkMode()
{
  return 0;
}

}  // namespace imageView_example







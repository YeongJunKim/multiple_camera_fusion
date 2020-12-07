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

  connect(ui.labelOrg, SIGNAL(Mouse_Pos()), this, SLOT(Mouse_current_pos()));
  connect(ui.labelOrg, SIGNAL(Mouse_Pressed()), this, SLOT(Mouse_Pressed()));
  connect(ui.labelOrg, SIGNAL(Mouse_Left()), this, SLOT(Mouse_left()));
  connect(ui.labelOrg, SIGNAL(Mouse_Released()),this, SLOT(Mouse_Released()));


  int value = ui.horizontalSlider_threshold_thermal->value();
  QString qstr = QString::number(value);
  ui.lineEdit->setText(qstr);
  value = ui.horizontalSlider_threshold_distance->value();
  QString qstr2 = QString::number(value);
  ui.lineEdit_2->setText(qstr2);

  myModel = new QStandardItemModel(0,0,this);
  QStringList horzHeaders;
  horzHeaders << "id" << "x1" << "y1" << "x2" << "y2" << "MAX_TH" << "MIN_TH";

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
  cv::Mat dst, crop;

  int width  = ui.labelOrg->width();
  int height = ui.labelOrg->height();
  cv::resize(*qnode.img_qnode, dst, cv::Size(width,height),0,0,CV_INTER_NN);


  //    Mat imgOrg(*qnode.img_qnode); //qnode-> receive
  Mat imgOrg(dst);
  updateLabels(&imgOrg);
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
  //  ROS_INFO("debug1");
  cv::Mat dst, cvt,cvt2, th;
  int width  = ui.labelOrg->width();
  int height = ui.labelOrg->height();
  cv::resize(*qnode.ir_img_qnode, dst, cv::Size(width,height),0,0,CV_INTER_NN);


//  int value = ui.horizontalSlider_threshold_thermal->value();
//  double t = (double)map(value,0,100,0,255);
//  cv::threshold(dst,th,t,65535,cv::THRESH_BINARY);

//  cv::cvtColor(th, cvt2, cv::COLOR_GRAY2BGR);

  Mat imgOrg(dst);
  updateLabels(&imgOrg);
  //  ROS_INFO("debug1");
  if(!qnode.ir_img_qnode->empty() && !imgOrg.empty() && isIrRecv)
  {
    if(mode == MODE_ONLY_THERMAL)
    {
      QImage qimageOrg((const unsigned char*)(imgOrg.data), imgOrg.cols, imgOrg.rows, QImage::Format_RGB888);
      ui.labelOrg->setPixmap(QPixmap::fromImage(qimageOrg.rgbSwapped()));
    }
  }
  //  ROS_INFO("debug1");
  delete qnode.ir_img_qnode;
  if(qnode.ir_img_qnode != NULL) qnode.ir_img_qnode = NULL;
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
  cv::Mat th;
  cv::Mat gray;
  cv::Mat step1;
  cv::Mat cvt;
  int value = ui.horizontalSlider_threshold_distance->value();
  double t = (double)map(value,0,100,0,255);
  cv::threshold(dst,th,t,65535,cv::THRESH_BINARY);
  cv::cvtColor(th, cvt, cv::COLOR_GRAY2BGR);
  Mat imgOrg(cvt);
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
  int id = rand();
  int x1,y1,x2,y2;

  int x1_ = ui.lineEdit_x1->text().toInt();
  int y1_ = ui.lineEdit_y1->text().toInt();
  int x2_ = ui.lineEdit_x2->text().toInt();
  int y2_ = ui.lineEdit_y2->text().toInt();

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
//  items.append(new QStandardItem(ui.lineEdit_x1->text()));
//  items.append(new QStandardItem(ui.lineEdit_y1->text()));
//  items.append(new QStandardItem(ui.lineEdit_x2->text()));
//  items.append(new QStandardItem(ui.lineEdit_y2->text()));

  items.append(new QStandardItem(QString::number(x1)));
  items.append(new QStandardItem(QString::number(y1)));
  items.append(new QStandardItem(QString::number(x2)));
  items.append(new QStandardItem(QString::number(y2)));

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
  cv::Mat *img;
  img = img_;
  //get tablerowsize
  int rows = ui.tableView->model()->rowCount();

  if(this->button_flag == 2)
  {
    int x1,x2,y1,y2;
    x1 = ui.lineEdit_x1->text().toInt();
    y1 = ui.lineEdit_y1->text().toInt();
    x2 = ui.labelOrg->x;
    y2 = ui.labelOrg->y;
    rectangle(*img,Rect(Point(x1,y1),Point(x2,y2)),Scalar(255,0,0),3,4,0);
  }
  if(rows == 0)
    return;
  //get label data x1 x2 y1 y2
  for(int i = 0; i < rows; i++)
  {
    int x1, x2, y1, y2;
    int id = ui.tableView->model()->index(i,0).data().toInt();
    x1 = ui.tableView->model()->index(i,1).data().toInt();
    y1 = ui.tableView->model()->index(i,2).data().toInt();
    x2 = ui.tableView->model()->index(i,3).data().toInt();
    y2 = ui.tableView->model()->index(i,4).data().toInt();
    rectangle(*img,Rect(Point(x1,y1),Point(x2,y2)),Scalar(0,0,255),3,4,0);
    putText(*img,to_string(id), Point(x1, y1), FONT_HERSHEY_SIMPLEX, 1, Scalar(200,0,0),1);
  }
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
  ROS_INFO("TimerCallbackFunction Called");
  // check thermal in labels

}
}  // namespace imageView_example




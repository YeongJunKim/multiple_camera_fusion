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

    QObject::connect(&qnode, SIGNAL(recvImg()), this, SLOT(updateImg()));
}

MainWindow::~MainWindow() {}

void MainWindow::updateImg()
{
    Mat imgOrg(*qnode.img_qnode); //qnode-> receive
    if(!qnode.img_qnode->empty() && !imgOrg.empty() && isRecv)
    {

        QImage qimageOrg((const unsigned char*)(imgOrg.data), imgOrg.cols, imgOrg.rows, QImage::Format_RGB888);
        ui.labelOrg->setPixmap(QPixmap::fromImage(qimageOrg.rgbSwapped()));

    }

    delete qnode.img_qnode;
    if(qnode.img_qnode != NULL) qnode.img_qnode = NULL;
    isRecv = false;
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


}  // namespace imageView_example




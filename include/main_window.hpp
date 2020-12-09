/**
 * @file /include/imageView_example/main_window.hpp
 *
 * @brief Qt based gui for imageView_example.
 *
 * @date November 2010
 **/
#ifndef imageView_example_MAIN_WINDOW_H
#define imageView_example_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <my_qlabel.h>
#include <QStandardItemModel>
#include <QTimer>
/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;
using namespace cv;
namespace imageView_example {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();


	void closeEvent(QCloseEvent *event); // Overloaded function


public Q_SLOTS:
        void updateImg();
        void updateImgIr();
        void updateImgLidar();
        void updateImgIr16();

//private slots:
        void on_button_update_clicked();

        void on_checkBox_only_color_clicked();

        void on_checkBox_lidar_clicked();

        void on_checkBox_thermal_clicked();

        void on_checkBox_only_lidar_clicked();

        void on_checkBox_only_thermal_clicked();

        void on_horizontalSlider_threshold_thermal_sliderMoved(int position);

        void on_horizontalSlider_threshold_distance_sliderMoved(int position);

        void on_button_select_position_clicked();

        void on_button_update_clear_clicked();

        void on_button_manually_add_clicked();

        void on_button_delete_id_clicked();

        void on_checkBox_gray_thermal_clicked();

        float map(float value, float istart, float istop, float ostart, float ostop);

        void Mouse_current_pos();
        void Mouse_Pressed();
        void Mouse_Released();
        void Mouse_left();

        void updateLabels(cv::Mat *img_);
        void drawLabels(Mat *img_);
        cv::Mat updateLabelsGray(cv::Mat *gray_);

        void updateImage(cv::Mat *img_);
        void OnTimerCallbackFunction();

        void on_horizontalSlider_color_bottomside_sliderMoved(int position);
        void on_horizontalSlider_color_upperside_sliderMoved(int position);
        void on_horizontalSlider_color_rightside_sliderMoved(int position);
        void on_horizontalSlider_color_leftside_sliderMoved(int position);
        void on_horizontalSlider_depth_bottomside_sliderMoved(int position);
        void on_horizontalSlider_depth_upperrside_sliderMoved(int position);
        void on_horizontalSlider_depth_rightside_sliderMoved(int position);
        void on_horizontalSlider_depth_leftside_sliderMoved(int position);
        void on_horizontalSlider_IR_bottomside_sliderMoved(int position);
        void on_horizontalSlider_IR_upperside_sliderMoved(int position);
        void on_horizontalSlider_IR_rightside_sliderMoved(int position);
        void on_horizontalSlider_IR_leftside_sliderMoved(int position);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  int checkMode(void);

  int mWidth = 640;
  int mHeight = 480;

  QStandardItemModel *myModel;

  int button_flag = 0;

  //timer
  shared_ptr<QTimer> m_pTimer;

};

}  // namespace imageView_example

#endif // imageView_example_MAIN_WINDOW_H

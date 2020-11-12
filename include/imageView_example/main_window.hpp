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

//private slots:
        void on_button_update_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    int mWidth = 640;
    int mHeight = 480;

};

}  // namespace imageView_example

#endif // imageView_example_MAIN_WINDOW_H

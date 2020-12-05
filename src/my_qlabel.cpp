//#include "my_qlabel.h"
#include "../include/my_qlabel.hpp"

my_qlabel::my_qlabel(QWidget *parent):
  QLabel(parent)
{

}

void my_qlabel::mouseMoveEvent(QMouseEvent *ev)
{
  this->x = ev->x();
  this->y = ev->y();

  Q_EMIT Mouse_Pos();
}

void my_qlabel::mousePressEvent(QMouseEvent *ev)
{
  Q_EMIT Mouse_Pressed();
}

void my_qlabel::leaveEvent(QEvent *)
{
  Q_EMIT Mouse_Left();
}

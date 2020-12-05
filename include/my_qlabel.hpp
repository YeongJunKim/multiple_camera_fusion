#ifndef MY_QLABEL_H
#define MY_QLABEL_H

#include <QLabel>
#include <QObject>
#include <QWidget>
#include <QEvent>
#include <QMouseEvent>

class my_qlabel : public QLabel
{
  Q_OBJECT
public:
  my_qlabel(QWidget *parent = 0);

  void mouseMoveEvent(QMouseEvent *ev);
  void mousePressEvent(QMouseEvent *ev);
  void leaveEvent(QEvent *);

  int x, y;

Q_SIGNALS:
  void Mouse_Pressed();
  void Mouse_Pos();
  void Mouse_Left();

};

#endif // MY_QLABEL_H

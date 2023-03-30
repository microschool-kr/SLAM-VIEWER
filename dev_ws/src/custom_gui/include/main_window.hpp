/**
 * @file /include/main_window.hpp
 *
 * @brief Qt based gui
 *
 * @date
 **/
#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>

#include <QComboBox>
#include <QDesktopWidget>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QVariant>
#include <map>
#include <QTimer>
#include <QShortcut>

#include <QProcess>

#include <ros/ros.h>
#include <custom_gui/Check.h>
#include <geometry_msgs/Twist.h>

#include "myviz.h"
#include "ui_main_window.h"

#include "mymap.h"
#include "loading_dialog.h"
#include "mycheck.h"
#include "mycheckexit.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();
  void closeEvent(QCloseEvent * event);
  void showLoading();
  void hideLoading();
  void callRobot(std::string data);
  void saveMap();
  

 public slots:
  void onClickRunButton();
  void onClickStopButton();
  void onClickCloseButton();
  void onSaveMapFinished();
  void onResponse(int d);
  void onCheckFailed();
  void onCheckExit();
  void onComboBoxChanged(int idx);
  void exitAction();

 private:
  Ui::MainWindowDesign ui;
  MyViz * myviz_;
  QProcess* qProcess_;
  bool isAllowClose_ = false;
  MapGenerator * mg_;
  LoadingDialog * loadingDialog_;
  ros::NodeHandle nh_;
  ros::Publisher stop_pub_;
  geometry_msgs::Twist msg_;
  MyCheck * my_check_;
  MyCheckExit * my_check_exit_;
  FILE * logfile_;
  QShortcut * sC_;
  std::chrono::system_clock::time_point start_;
};

#endif  // MAIN_WINDOW_H

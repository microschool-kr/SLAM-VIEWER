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

#include "main_window.hpp"
#include <QString>
#include <QtGui>
#include <iostream>
#include <QDebug>
#include <QMessageBox>
#include <shlobj_core.h>
#include <ros/master.h>
#include <xmlrpcpp/XmlRpc.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent) {
  ui.setupUi(this);
  ui.modeComboBox->setCursor(Qt::PointingHandCursor);
  ui.modeComboBox->view()->setCursor(Qt::PointingHandCursor);
  ui.runButton->setCursor(Qt::PointingHandCursor);
  ui.stopButton->setCursor(Qt::PointingHandCursor);
  QFrame* line = new QFrame();
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  ui.verticalLayout->insertWidget(1, line);
  myviz_ = new MyViz(ui.mainHorizontalLayout);
  stop_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  // setWindowFlags(Qt::WindowStaysOnTopHint);
  connect(ui.logoButton, SIGNAL(clicked()), this, SLOT(onClickCloseButton()));
  connect(ui.nameButton, SIGNAL(clicked()), this, SLOT(onClickCloseButton()));
  ui.runButton->setEnabled(false);
  ui.runButton->setVisible(false);
  ui.stopButton->setEnabled(false);
  ui.stopButton->setVisible(false);
  connect(ui.modeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onComboBoxChanged(int)));
  connect(ui.runButton, SIGNAL(clicked()), this, SLOT(onClickRunButton()));
  connect(ui.stopButton, SIGNAL(clicked()), this, SLOT(onClickStopButton()));
  my_check_ = new MyCheck(this);
  connect(my_check_, SIGNAL(checkcheck(int)), this, SLOT(onResponse(int)));
  connect(my_check_, SIGNAL(checkfail()), this, SLOT(onCheckFailed()));
  my_check_exit_ = new MyCheckExit(this);
  connect(my_check_exit_, SIGNAL(checkexit()), this, SLOT(onCheckExit()));
  my_check_exit_->start();
  sC_ = new QShortcut(tr("Ctrl+Alt+F4"), this);
  connect(sC_, SIGNAL(activated()), this, SLOT(exitAction()));
  std::stringstream ss;
  PWSTR path = NULL;
  SHGetKnownFolderPath(FOLDERID_Desktop, 0, NULL, &path);
  char path_buf[260];
  WideCharToMultiByte(CP_ACP,0,path,-1, path_buf,260,NULL, NULL);
  ss << path_buf << "\\slam_viewer_log.txt";
  fopen_s(&logfile_, ss.str().c_str(), "w");
  CoTaskMemFree(path);
  loadingDialog_ = new LoadingDialog(this, Qt::FramelessWindowHint);
  connect(loadingDialog_, SIGNAL(closeWindow()), this, SLOT(exitAction()));
  loadingDialog_->setModal(true);
  showLoading();
  callRobot("open");
}

void MainWindow::onComboBoxChanged(int idx)
{
  if (idx == 1)
  {
    ui.runButton->setEnabled(true);
    ui.runButton->setVisible(true);
    ui.stopButton->setEnabled(false);
    ui.stopButton->setVisible(false);
    ui.runButton->setText("실행");
    ui.stopButton->setText("종료");
  }
  else if (idx == 2)
  {
    ui.runButton->setEnabled(true);
    ui.runButton->setVisible(true);
    ui.stopButton->setEnabled(false);
    ui.stopButton->setVisible(false);
    ui.runButton->setText("실행");
    ui.stopButton->setText("맵 저장 && 종료");
  }
  else if (idx == 3)
  {
    ui.runButton->setEnabled(true);
    ui.runButton->setVisible(true);
    ui.stopButton->setEnabled(false);
    ui.stopButton->setVisible(false);
    ui.runButton->setText("맵 불러오기");
    ui.stopButton->setText("종료");
  }
  else
  {
    ui.runButton->setEnabled(false);
    ui.runButton->setVisible(false);
    ui.stopButton->setEnabled(false);
    ui.stopButton->setVisible(false);
  }
}

void MainWindow::onClickCloseButton()
{
  isAllowClose_ = !isAllowClose_;
}

void MainWindow::onClickRunButton()
{
  int idx = ui.modeComboBox->currentIndex();
  if (idx)
  {
    start_ = std::chrono::system_clock::now();
    ROS_INFO("click runbutton");
    showLoading();
    ui.modeComboBox->setEnabled(false);
    ui.runButton->setEnabled(false);
    ui.stopButton->setEnabled(true);
    myviz_->reset_();
    if (idx == 1)
    {
      callRobot("control");
      fprintf(logfile_, "Control mode : ");
    }
    else if (idx == 2)
    {
      callRobot("mapping");
      fprintf(logfile_, "Mapping mode : ");
    }
    else if (idx == 3)
    {
      callRobot("navigation");
      fprintf(logfile_, "Navigation mode : ");
    }
  } 
  else
  {
    QMessageBox::critical(this, "경고", "모드를 선택하세요.", QMessageBox::Ok);
  }
}

void MainWindow::onResponse(int d)
{
  if (d == 0)
  {
    hideLoading();
  }
  else if (d == 1)
  {
    ui.runButton->setVisible(false);
    ui.stopButton->setVisible(true);
    std::chrono::duration<double>sec = std::chrono::system_clock::now() - start_;
    fprintf(logfile_, "%lf\n", sec.count());
    fflush(logfile_);
    hideLoading();
  }
  else if (d == 2)
  {
    ui.modeComboBox->setEnabled(true);
    ui.runButton->setVisible(true);
    ui.stopButton->setVisible(false);
    ui.modeComboBox->setCurrentIndex(0);
    myviz_->reset_();
    std::chrono::duration<double>sec = std::chrono::system_clock::now() - start_;
    fprintf(logfile_, "%lf\n", sec.count());
    fflush(logfile_);
    hideLoading();
  }
}

void MainWindow::onClickStopButton()
{
  start_ = std::chrono::system_clock::now();
  showLoading();
  stop_pub_.publish(msg_);
  int idx = ui.modeComboBox->currentIndex();
  ui.runButton->setEnabled(true);
  ui.stopButton->setEnabled(false);
  if (idx == 1)
  {
    fprintf(logfile_, "Control mode end : ");
    callRobot("disconnect");
  }
  else if (idx == 2)
  {
    saveMap();
    fprintf(logfile_, "Mapping mode end : ");
  }
  else if (idx == 3)
  {
    fprintf(logfile_, "Navigation mode end : ");
    callRobot("disconnect");
  }
}

void MainWindow::saveMap()
{
  mg_ = new MapGenerator("mymap");
  connect(mg_, SIGNAL(saveMap()), this, SLOT(onSaveMapFinished()));
  mg_->start();
  ROS_INFO("map");
}

void MainWindow::showLoading()
{
  loadingDialog_->show();
  loadingDialog_->startAnimation();
}

void MainWindow::hideLoading()
{
  loadingDialog_->hide();
  loadingDialog_->stopAnimation();
}

void MainWindow::callRobot(std::string data)
{
  my_check_->setData(data);
  my_check_->start();
}

void MainWindow::onCheckFailed()
{
  hideLoading();
  QMessageBox::warning(this, "경고", "SLAM VIEWER 프로그램이 정상 실행되지 않았습니다.\n종료 후 다시 실행해주세요.", QMessageBox::Ok);
  exitAction();
}

void MainWindow::onCheckExit()
{
  QMessageBox::warning(this, "경고", "SLAM 로봇과 연결이 끊어졌습니다.\n로봇을 확인한 후 다시 실행해주세요.", QMessageBox::Ok);
  exitAction();
}

void MainWindow::onSaveMapFinished()
{
  ROS_INFO("map saved");
  delete mg_;
  callRobot("disconnect");
}

void MainWindow::exitAction()
{
  isAllowClose_ = true;
  close();
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  if (isAllowClose_) 
  {
    QProcess* kill_process = new QProcess(this);
    kill_process->start("taskkill /t /f /im cmd.exe");
    kill_process->waitForFinished(-1);
    my_check_->terminate();
    callRobot("off");
    event->accept();
  }
  else 
  {
    event->ignore();
  }
}

MainWindow::~MainWindow()
{
  fclose(logfile_);
  delete myviz_;
  delete qProcess_;
  delete stop_pub_;
  delete my_check_;
  delete loadingDialog_;
}

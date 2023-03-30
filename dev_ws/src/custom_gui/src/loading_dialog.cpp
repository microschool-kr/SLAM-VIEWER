#include "loading_dialog.h"

LoadingDialog::LoadingDialog(QWidget* parent, Qt::WindowFlags f) : QDialog(parent, f), isAllowClose_(false)
{
    resize(300,300);
    setAttribute(Qt::WA_TranslucentBackground);
    pi = new QProgressIndicator(this);
    pi->resize(300, 300);
    sC_ = new QShortcut(tr("Ctrl+Alt+F4"), this);
    connect(sC_, SIGNAL(activated()), this, SLOT(closeLoading()));
}

void LoadingDialog::stopAnimation()
{
  pi->stopAnimation();
}

void LoadingDialog::startAnimation()
{
  pi->startAnimation();
}

void LoadingDialog::closeEvent(QCloseEvent * event)
{
  if (!isAllowClose_) 
  {
    event->ignore();
  }
}

void LoadingDialog::closeLoading()
{
  emit closeWindow();
}


void LoadingDialog::loadingFinished()
{
    isAllowClose_ = true;
    close();
}

LoadingDialog::~LoadingDialog()
{
    delete pi;
}
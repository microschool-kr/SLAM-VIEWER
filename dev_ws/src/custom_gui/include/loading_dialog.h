#ifndef LOADING_DIALOG_H
#define LOADING_DIALOG_H

#include "QProgressIndicator.h"

#include <QCloseEvent>
#include <QShortcut>
#include <QDialog>
  
class LoadingDialog : public QDialog
{
    Q_OBJECT
    public:
        LoadingDialog(QWidget* parent = 0, Qt::WindowFlags f = Qt::WindowFlags());
        ~LoadingDialog();
        void loadingFinished();
        void closeEvent(QCloseEvent * event);
        void stopAnimation();
        void startAnimation();

    public slots:
        void closeLoading();

    signals:
        void closeWindow();

    private:
        QProgressIndicator* pi;
        bool isAllowClose_;
        QShortcut * sC_;
};

#endif // LOADING_DIALOG_H
#include <QMainWindow>
#include <QWidget>
#include <QImage>

#include "main_window.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    m_videoWidget = new VideoWidget(this);
    setCentralWidget(m_videoWidget);
    showMaximized();
}

void MainWindow::updateFrame(const QImage &frame)
{
    // m_videoWidget->updateFrame(frame);
    QMetaObject::invokeMethod(this->m_videoWidget, "updateFrame", Qt::QueuedConnection, Q_ARG(QImage, frame));
}
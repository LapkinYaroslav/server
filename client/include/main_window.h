#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QImage>

#include "video_widget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    void updateFrame(const QImage &frame);

private:
    VideoWidget *m_videoWidget;
};

#endif // MAIN_WINDOW_H
#include <QPainter>
#include <QWidget>
#include <QPaintEvent>
#include <QOpenGLWidget>

#include "video_widget.h"

VideoWidget::VideoWidget(QWidget *parent) : QOpenGLWidget(parent)
{
}

void VideoWidget::updateFrame(const QImage &frame)
{
    m_currentFrame = frame;
    update(); // Вызывает перерисовку виджета
}

void VideoWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    if (!m_currentFrame.isNull())
    {
        painter.drawImage(rect(), m_currentFrame);
    }
}

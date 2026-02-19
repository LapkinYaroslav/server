#include <QApplication>
#include <QThread>
#include <QObject>

#include "main_window.h"
#include "client.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow *window = new MainWindow();

    GStreamerClient *client = new GStreamerClient(window);
    QThread *q = new QThread();
    client->moveToThread(q);
    QObject::connect(q, &QThread::started, client, &GStreamerClient::run_listen);
    QObject::connect(q, &QThread::finished, client, &QObject::deleteLater);
    QObject::connect(q, &QThread::finished, q, &QThread::deleteLater);

    window->show();
    q->start();

    return app.exec();
}
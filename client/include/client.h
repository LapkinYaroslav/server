#ifndef CLIENT_H
#define CLIENT_H

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <QObject>

#include "main_window.h"

class GStreamerClient : public QObject
{
    Q_OBJECT
public:
    GStreamerClient(MainWindow *window);
    ~GStreamerClient();
public slots:
    int run_listen();

private:
    int listen();
    void handle_server_message(const gchar *message);
    MainWindow *m_window;
};

#endif // CLIENT_H
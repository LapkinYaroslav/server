#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/rtp/gstrtcpbuffer.h>
#include <QImage>
#include <iostream>
#include <cstdio>
#include <cstring>

#ifdef __APPLE__
#include <TargetConditionals.h>
#endif

#include "client.h"
#include "main_window.h"

int PORT_CLIENT = 8600;
const char *IP_CLIENT = "192.168.31.251";
const char *SERVER_IP = "192.168.31.149"; // IP сервера для отправки сообщений

// ===== ГЛОБАЛЬНЫЕ ОБЪЯВЛЕНИЯ =====
static GstElement *global_pipeline = NULL;
static GstElement *msg_appsrc = NULL; // Для ОТПРАВКИ сообщений на сервер

typedef struct
{
    GstElement *pipeline;
    GstElement *msg_pipeline;
    GMainLoop *main_loop;
} BusData;

typedef struct
{
    gchar *filename;
    FILE *file;
    long last_position;
    GMainLoop *main_loop;
    guint timeout_id;
} FileWatcherData;

static void adapt_quality(guint8 fraction_lost, guint32 jitter)
{
    // Логика адаптации качества
    g_print("Получено состояние канала: потери=%d%%, джиттер=%u\\n", fraction_lost, jitter);
    
    if (fraction_lost > 10 || jitter > 50000)
    {
        // Плохое качество - уменьшаем битрейт
        g_print("Обнаружены проблемы с качеством связи\\n");
    }
    else if (fraction_lost < 2 && jitter < 10000)
    {
        // Хорошее качество - можно повысить
        g_print("Качество связи хорошее\\n");
    }
}
{
    if (!msg_appsrc)
    {
        g_printerr("msg_appsrc не инициализирован!\n");
        return;
    }

    GstBuffer *buffer = gst_buffer_new_and_alloc(strlen(message));
    gst_buffer_fill(buffer, 0, message, strlen(message));

    GstFlowReturn ret;
    g_signal_emit_by_name(msg_appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK)
    {
        g_printerr("Ошибка отправки сообщения: %d\n", ret);
    }
}

// Инициализация пайплайна ДЛЯ ОТПРАВКИ сообщений на сервер (порт 8602)
GstElement *init_msg_send_pipeline(int port)
{
    GstElement *pipeline, *appsrc, *tcpclientsink;
    pipeline = gst_pipeline_new("msg_send_pipeline");
    if (!pipeline)
    {
        g_printerr("Failed to create message send pipeline\n");
        return NULL;
    }
    appsrc = gst_element_factory_make("appsrc", "msg_src");
    tcpclientsink = gst_element_factory_make("tcpclientsink", "msg_sink");

    if (!appsrc || !tcpclientsink)
    {
        g_printerr("Failed to create message send elements\n");
        gst_object_unref(pipeline);
        return NULL;
    }

    g_object_set(tcpclientsink,
                 "host", SERVER_IP,
                 "port", port,
                 "sync", FALSE,
                 NULL);

    g_object_set(appsrc,
                 "format", GST_FORMAT_TIME,
                 "is-live", TRUE,
                 NULL);

    gst_bin_add_many(GST_BIN(pipeline), appsrc, tcpclientsink, NULL);
    gst_element_link(appsrc, tcpclientsink);

    msg_appsrc = appsrc;
    return pipeline;
}

// Инициализация пайплайна ДЛЯ ПРИЁМА сообщений от сервера (порт 8601)
GstElement *init_msg_receive_pipeline(int port)
{
    GstElement *pipeline, *tcpclientsrc, *appsink;

    pipeline = gst_pipeline_new("msg_receive_pipeline");
    if (!pipeline)
    {
        g_printerr("Failed to create message receive pipeline\n");
        return NULL;
    }

    tcpclientsrc = gst_element_factory_make("tcpclientsrc", "msg_receive_src");
    appsink = gst_element_factory_make("appsink", "msg_receive_sink");

    if (!tcpclientsrc || !appsink)
    {
        g_printerr("Failed to create message receive elements\n");
        gst_object_unref(pipeline);
        return NULL;
    }

    g_object_set(tcpclientsrc,
                 "host", SERVER_IP,
                 "port", port,
                 "timeout", 10000000, // 10 секунд
                 NULL);

    g_object_set(appsink,
                 "emit-signals", TRUE,
                 "sync", FALSE,
                 "drop", TRUE,
                 "max-buffers", 1,
                 NULL);

    gst_bin_add_many(GST_BIN(pipeline), tcpclientsrc, appsink, NULL);

    if (!gst_element_link(tcpclientsrc, appsink))
    {
        g_printerr("Failed to link tcpclientsrc -> appsink\n");
        gst_object_unref(pipeline);
        return NULL;
    }

    // Подключаем обработчик сообщений
    g_signal_connect(appsink, "new-sample", G_CALLBACK([](GstElement *appsink, gpointer) -> GstFlowReturn
                                                       {
        GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
        if (sample) {
            GstBuffer *buffer = gst_sample_get_buffer(sample);
            if (buffer) {
                gsize size = gst_buffer_get_size(buffer);
                gchar *data = (gchar *)g_malloc0(size + 1);
                if (data) {
                    gst_buffer_extract(buffer, 0, data, size);
                    g_print("Получено сообщение от сервера: %s\n", data);
                    
                    // Пример обработки команды
                    if (g_strstr_len(data, -1, "STOP_RECORDING")) {
                        g_print("Получена команда остановки записи\n");
                        // Здесь можно добавить логику остановки записи
                    }
                    g_free(data);
                }
            }
            gst_sample_unref(sample);
        }
        return GST_FLOW_OK; }),
                     NULL);

    g_print("Message receive pipeline initialized: connecting to %s:%d\n", SERVER_IP, port);
    return pipeline;
}

static gboolean bus_msg_handler(GstBus *bus, GstMessage *msg, gpointer user_data)
{
    BusData *data = (BusData *)user_data;
    GError *err;
    gchar *debug_info;

    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_STATE_CHANGED:
        if (GST_MESSAGE_SRC(msg) == GST_OBJECT(data->pipeline))
        {
            GstState old_state, new_state, pending_state;
            gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
            g_print("Pipeline state: %s -> %s (pending: %s)\n",
                    gst_element_state_get_name(old_state),
                    gst_element_state_get_name(new_state),
                    gst_element_state_get_name(pending_state));
        }
        break;
    case GST_MESSAGE_ERROR:
        gst_message_parse_error(msg, &err, &debug_info);
        g_printerr("Error from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
        g_clear_error(&err);
        g_free(debug_info);
        g_main_loop_quit(data->main_loop);
        break;
    case GST_MESSAGE_EOS:
        g_print("End-Of-Stream reached.\n");
        g_main_loop_quit(data->main_loop);
        break;
    default:
        break;
    }
    return TRUE;
}

static gboolean check_and_read_file(gpointer user_data)
{
    FileWatcherData *data = (FileWatcherData *)user_data;
    gchar buffer[1024];

    if (!data->file)
    {
        data->file = fopen(data->filename, "r");
        if (!data->file)
        {
            g_printerr("Не удалось открыть файл: %s\n", data->filename);
            return G_SOURCE_CONTINUE;
        }

        fseek(data->file, 0, SEEK_END);
        data->last_position = ftell(data->file);
        return G_SOURCE_CONTINUE;
    }

    fseek(data->file, 0, SEEK_END);
    long current_position = ftell(data->file);

    if (current_position < data->last_position)
    {
        data->last_position = 0;
        fseek(data->file, 0, SEEK_SET);
    }

    if (current_position > data->last_position)
    {
        fseek(data->file, data->last_position, SEEK_SET);

        while (fgets(buffer, sizeof(buffer), data->file))
        {
            size_t len = strlen(buffer);
            if (len > 0 && buffer[len - 1] == '\n')
            {
                buffer[len - 1] = '\0';
            }

            if (strlen(buffer) > 0)
            {
                g_print("Отправляю сообщение на сервер: %s\n", buffer);
                send_message(buffer);
            }
        }

        data->last_position = ftell(data->file);
    }

    if (ferror(data->file))
    {
        clearerr(data->file);
        fclose(data->file);
        data->file = NULL;
    }

    return G_SOURCE_CONTINUE;
}

void stop_file_monitor(void)
{
    static FileWatcherData *watcher_data = NULL;

    if (watcher_data && watcher_data->timeout_id)
    {
        g_source_remove(watcher_data->timeout_id);

        if (watcher_data->file)
        {
            fclose(watcher_data->file);
        }
        g_free(watcher_data->filename);
        g_free(watcher_data);
        watcher_data = NULL;

        g_print("Мониторинг файла остановлен\n");
    }
}

void start_file_monitor(const gchar *filename, GMainLoop *main_loop)
{
    static FileWatcherData *watcher_data = NULL;
    stop_file_monitor();

    watcher_data = g_new0(FileWatcherData, 1);
    watcher_data->filename = g_strdup(filename);
    watcher_data->main_loop = main_loop;
    watcher_data->last_position = 0;
    watcher_data->file = NULL;

    watcher_data->timeout_id = g_timeout_add(500, check_and_read_file, watcher_data);
    g_print("Начал мониторинг файла: %s\n", filename);
}

// ===== CALLBACK ДЛЯ ОТОБРАЖЕНИЯ КАДРОВ =====
static GstFlowReturn on_new_sample_callback(GstAppSink *appsink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    MainWindow *window = (MainWindow *)user_data;

    if (sample)
    {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ))
        {
            GstCaps *caps = gst_sample_get_caps(sample);
            GstStructure *structure = gst_caps_get_structure(caps, 0);
            int width, height;
            gst_structure_get_int(structure, "width", &width);
            gst_structure_get_int(structure, "height", &height);

            // Важно: формат должен соответствовать настройкам пайплайна (RGB)
            QImage image((const uchar *)map.data, width, height, QImage::Format_RGB888);
            window->updateFrame(image);

            gst_buffer_unmap(buffer, &map);
        }
        gst_sample_unref(sample);
    }

    return GST_FLOW_OK;
}

// ===== КОНСТРУКТОР/ДЕСТРУКТОР =====
GStreamerClient::GStreamerClient(MainWindow *window)
{
    this->m_window = window;
    // Инициализируем элементы записи здесь, чтобы избежать ошибок компиляции
    tee = NULL;
    record_queue = NULL;
    stream_queue = NULL;
    mp4mux = NULL;
    filesink = NULL;
}

GStreamerClient::~GStreamerClient()
{
    stop_file_monitor();
}

// ===== ОСНОВНАЯ ФУНКЦИЯ ПРИЁМА И ЗАПИСИ =====
int GStreamerClient::listen()
{
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;

    /* Initialize GStreamer */
    gst_init(0, 0);

    /* Create the elements */
    GstElement *pipeline = gst_pipeline_new("client_pipeline");
    GstElement *udpsrc = gst_element_factory_make("udpsrc", "source");
    GstElement *rtph264depay = gst_element_factory_make("rtph264depay", "depay");
    GstElement *h264parse = gst_element_factory_make("h264parse", "parse");
    GstElement *avdec_h264 = gst_element_factory_make("avdec_h264", "decoder");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
    GstElement *appsink = gst_element_factory_make("appsink", "appsink");

    // Элементы для записи (логирования)
    GstElement *tee = gst_element_factory_make("tee", "tee");
    GstElement *record_queue = gst_element_factory_make("queue", "record_queue");
    GstElement *stream_queue = gst_element_factory_make("queue", "stream_queue");
    GstElement *mp4mux = gst_element_factory_make("mp4mux", "mp4mux");
    GstElement *filesink = gst_element_factory_make("filesink", "filesink");

    if (!pipeline || !udpsrc || !rtph264depay || !h264parse || !avdec_h264 ||
        !videoconvert || !appsink || !tee || !record_queue || !stream_queue ||
        !mp4mux || !filesink)
    {
        g_printerr("Not all elements could be created.\n");
        return -1;
    }

    // Настройка источника
    g_object_set(udpsrc,
                 "port", PORT_CLIENT,
                 "caps", gst_caps_from_string("application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"),
                 "buffer-size", 2097152,
                 NULL);

    // Настройка декодера
    g_object_set(avdec_h264,
                 "max-threads", 4,
                 "skip-frame", 0,
                 "output-corrupt", FALSE,
                 NULL);

    // Настройка appsink для отображения
    g_object_set(appsink,
                 "emit-signals", TRUE,
                 "sync", FALSE,
                 "max-lateness", -1,
                 "max-buffers", 5,
                 "drop", FALSE,
                 NULL);

    GstCaps *caps = gst_caps_from_string("video/x-raw,format=RGB");
    g_object_set(appsink, "caps", caps, NULL);
    gst_caps_unref(caps);

    // Настройка записи: сохраняем в файл без перекодирования
    g_object_set(filesink,
                 "location", "client_recording.mp4",
                 "sync", FALSE,
                 NULL);

    // Добавляем ВСЕ элементы в пайплайн
    gst_bin_add_many(GST_BIN(pipeline),
                     udpsrc, rtph264depay, h264parse, tee,
                     stream_queue, avdec_h264, videoconvert, appsink,
                     record_queue, mp4mux, filesink,
                     NULL);

    // 1. Связываем основную цепочку до tee
    if (!gst_element_link_many(udpsrc, rtph264depay, h264parse, tee, NULL))
    {
        g_printerr("Failed to link source -> tee chain.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    // 2. Ветка отображения: tee → queue → decoder → convert → appsink
    GstPad *tee_src_pad_display = gst_element_request_pad_simple(tee, "src_%u");
    GstPad *stream_queue_sink = gst_element_get_static_pad(stream_queue, "sink");
    if (!tee_src_pad_display || !stream_queue_sink ||
        gst_pad_link(tee_src_pad_display, stream_queue_sink) != GST_PAD_LINK_OK)
    {
        g_printerr("Failed to link tee to display branch.\n");
        gst_object_unref(pipeline);
        return -1;
    }
    gst_object_unref(tee_src_pad_display);
    gst_object_unref(stream_queue_sink);

    if (!gst_element_link_many(stream_queue, avdec_h264, videoconvert, appsink, NULL))
    {
        g_printerr("Failed to link display branch.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    // 3. Ветка записи: tee → queue → mp4mux → filesink
    GstPad *tee_src_pad_record = gst_element_request_pad_simple(tee, "src_%u");
    GstPad *record_queue_sink = gst_element_get_static_pad(record_queue, "sink");
    if (!tee_src_pad_record || !record_queue_sink ||
        gst_pad_link(tee_src_pad_record, record_queue_sink) != GST_PAD_LINK_OK)
    {
        g_printerr("Failed to link tee to recording branch.\n");
        gst_object_unref(pipeline);
        return -1;
    }
    gst_object_unref(tee_src_pad_record);
    gst_object_unref(record_queue_sink);

    if (!gst_element_link_many(record_queue, mp4mux, filesink, NULL))
    {
        g_printerr("Failed to link recording branch.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    // Подключаем обработчик кадров для отображения
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample_callback), this->m_window);

    GMainLoop *main_loop = g_main_loop_new(NULL, FALSE);

    // Инициализация пайплайна для приёма сообщений
    GstElement *msg_pipeline = init_msg_pipeline(8601);

    BusData *bus_data = g_new0(BusData, 1); // Используем g_new0 вместо g_new
    bus_data->pipeline = pipeline;
    bus_data->msg_pipeline = msg_pipeline;
    bus_data->main_loop = main_loop;

    /* Запуск пайплайнов */
    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        g_printerr("Unable to set main pipeline to PLAYING.\n");
        gst_object_unref(pipeline);
        g_free(bus_data);
        return -1;
    }

    ret = gst_element_set_state(msg_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        g_printerr("Unable to set message pipeline to PLAYING.\n");
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        g_free(bus_data);
        return -1;
    }

    g_print("Client pipeline started. Recording to 'client_recording.mp4'...\n");

    /* Ожидание событий */
    bus = gst_element_get_bus(pipeline);
    GstBus *msg_bus = gst_element_get_bus(msg_pipeline);

    gst_bus_add_watch(bus, bus_msg_handler, bus_data);
    gst_bus_add_watch(msg_bus, bus_msg_handler, bus_data);

    // Запуск мониторинга файла сообщений
    start_file_monitor("messages.txt", main_loop);

    g_main_loop_run(main_loop);

    /* Очистка ресурсов */
    gst_object_unref(bus);
    gst_object_unref(msg_bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_element_set_state(msg_pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_object_unref(msg_pipeline);
    g_main_loop_unref(main_loop);
    g_free(bus_data);

    g_print("Recording saved to 'client_recording.mp4'\n");
    return 0;
}
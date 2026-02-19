#include <gst/gst.h>
#include <gst/rtp/gstrtcpbuffer.h>
#include <signal.h>
#include <stdio.h>

#ifdef __APPLE__
#include <TargetConditionals.h>
#endif

const char *IP_CLIENT = "192.168.31.251";
int PORT_CLIENT = 8600;

static GstElement *global_pipeline = NULL;
static GstElement *msg_appsrc = NULL;

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

static void adapt_quality(GstElement *encoder, guint8 fraction_lost, guint32 jitter)
{
    gint current_bitrate;
    g_object_get(encoder, "bitrate", &current_bitrate, NULL);

    // Логика адаптации
    if (fraction_lost > 10 || jitter > 50000)
    {
        // Плохое качество - уменьшаем битрейт
        gint new_bitrate = current_bitrate * 0.7;
        if (new_bitrate < 500)
            new_bitrate = 500; // Минимум 500 кбит/с

        g_object_set(encoder, "bitrate", new_bitrate, NULL);
        g_print("Снижаем качество: %d кбит/с (потери: %d%%)\n",
                new_bitrate, fraction_lost);
    }
    else if (fraction_lost < 2 && jitter < 10000)
    {
        // Хорошее качество - можно повысить
        gint new_bitrate = current_bitrate * 1.2;
        if (new_bitrate > 4000)
            new_bitrate = 4000; // Максимум 4 Мбит/с

        g_object_set(encoder, "bitrate", new_bitrate, NULL);
        g_print("Повышаем качество: %d кбит/с\n", new_bitrate);
    }
}

static gboolean send_eos_to_source(GstElement *pipeline)
{
    GstElement *source = gst_bin_get_by_name(GST_BIN(pipeline), "source");
    if (source)
    {
        gst_element_send_event(source, gst_event_new_eos());
        gst_object_unref(source);
    }
    return FALSE; // Don't call again
}

static void sigint_handler(int sig)
{
    if (global_pipeline)
    {
        g_print("\nSending EOS to finalize recording...\n");
        GstElement *source = gst_bin_get_by_name(GST_BIN(global_pipeline), "source");
        if (source)
        {
            gst_element_send_event(source, gst_event_new_eos());
            gst_object_unref(source);
        }
    }
}

// Callback
static void on_rtcp_received(GstElement *rtpbin, GstBuffer *buffer, guint session, gpointer user_data)
{
    GstElement *encoder = GST_ELEMENT(user_data);
    GstRTCPBuffer rtcpbuf = GST_RTCP_BUFFER_INIT;
    if (gst_rtcp_buffer_validate(buffer) && gst_rtcp_buffer_map(buffer, GST_MAP_READ, &rtcpbuf))
    {
        GstRTCPPacket packet;
        gst_rtcp_buffer_get_first_packet(&rtcpbuf, &packet);
        do
        {
            if (gst_rtcp_packet_get_type(&packet) == GST_RTCP_TYPE_RR)
            {
                guint32 ssrc = 0;
                guint8 fraction_lost = 0; // Изменено с guint на guint8
                gint32 packets_lost = 0;
                guint32 jitter = 0; // Изменено с guint на guint32
                guint32 delay = 0;  // Изменено с guint на guint32

                gst_rtcp_packet_get_rb(
                    &packet,
                    0,
                    &ssrc,
                    &fraction_lost,
                    &packets_lost,
                    NULL,
                    &jitter,
                    NULL,
                    &delay);

                adapt_quality(encoder, fraction_lost, jitter);
            }
        } while (gst_rtcp_packet_move_to_next(&packet));
        gst_rtcp_buffer_unmap(&rtcpbuf);
    }
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
        g_printerr("Error from %s: %s\n",
                   GST_OBJECT_NAME(msg->src), err->message);
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

void send_message(const gchar *message)
{
    GstBuffer *buffer = gst_buffer_new_and_alloc(strlen(message));
    gst_buffer_fill(buffer, 0, message, strlen(message));

    GstFlowReturn ret;
    g_signal_emit_by_name(msg_appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
}

GstElement *init_msg_pipeline(int port)
{
    GstElement *pipeline, *appsrc, *tcpserversink;
    pipeline = gst_pipeline_new("msg_pipeline");
    if (!pipeline)
    {
        g_printerr("Failed to create message pipeline\n");
        return NULL;
    }
    appsrc = gst_element_factory_make("appsrc", "msg_src");
    tcpserversink = gst_element_factory_make("tcpserversink", "msg_sink");

    // Настраиваем TCP сервер
    g_object_set(tcpserversink,
                 "host", "0.0.0.0", // Слушаем на всех интерфейсах
                 "port", port,      // Порт для сообщений
                 NULL);

    gst_bin_add_many(GST_BIN(pipeline), appsrc, tcpserversink, NULL);
    gst_element_link(appsrc, tcpserversink);
    msg_appsrc = appsrc;
    return pipeline;
}

void handle_connection_lost()
{
    g_print("It's over!");
}

static void on_rtp_timeout(GstElement *rtpbin, guint session, gpointer user_data)
{
    g_print("RTCP timeout detected - connection lost!\n");
    handle_connection_lost();
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

        // Перемещаемся в конец файла при первом открытии
        fseek(data->file, 0, SEEK_END);
        data->last_position = ftell(data->file);
        return G_SOURCE_CONTINUE;
    }

    // Получаем текущий размер файла
    fseek(data->file, 0, SEEK_END);
    long current_position = ftell(data->file);

    // Если файл уменьшился (например, перезаписан)
    if (current_position < data->last_position)
    {
        data->last_position = 0;
        fseek(data->file, 0, SEEK_SET);
    }

    // Если есть новые данные
    if (current_position > data->last_position)
    {
        fseek(data->file, data->last_position, SEEK_SET);

        while (fgets(buffer, sizeof(buffer), data->file))
        {
            // Убираем символ новой строки в конце
            size_t len = strlen(buffer);
            if (len > 0 && buffer[len - 1] == '\n')
            {
                buffer[len - 1] = '\0';
            }

            // Проверяем, не пустая ли строка
            if (strlen(buffer) > 0)
            {
                g_print("Отправляю новую строку: %s\n", buffer);
                send_message(buffer);
            }
        }

        data->last_position = ftell(data->file);
    }

    // Проверяем, не произошла ли ошибка с файлом
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
        // Удаляем таймер по ID
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

// Создание и запуск монитора файла
void start_file_monitor(const gchar *filename, GMainLoop *main_loop)
{
    static FileWatcherData *watcher_data = NULL;

    // Если уже мониторим, останавливаем предыдущий
    stop_file_monitor();

    // Создаем новую структуру данных
    watcher_data = g_new0(FileWatcherData, 1);
    watcher_data->filename = g_strdup(filename);
    watcher_data->main_loop = main_loop;
    watcher_data->last_position = 0;
    watcher_data->file = NULL;

    // Добавляем периодическую проверку каждые 500 мс
    watcher_data->timeout_id = g_timeout_add(500, check_and_read_file, watcher_data);

    g_print("Начал мониторинг файла: %s\n", filename);
}

// Debug only
// static gboolean timeout_function(gpointer user_data)
// {
//     static int counter = 0;
//     g_print("Таймаут %d\n", ++counter);
//     return G_SOURCE_CONTINUE; // Продолжать выполнение
// }

int tutorial_main(int argc, char *argv[])
{
    GstElement *pipeline, *source, *capsfilter, *videoscale, *videoconverter,
        *x264enc_stream, *rtppay, *udpsink, *tee,
        *record_queue, *mp4mux, *filesink, *x264enc_record;
    GstElement *stream_queue; // Добавляем очередь для стриминга
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;
    GstPad *tee_src_pad1, *tee_src_pad2, *queue_sink_pad, *rtppay_sink_pad;

    /* Initialize GStreamer */
    gst_init(&argc, &argv);

    /* Create the elements */
#ifdef _WIN32
    source = gst_element_factory_make("ksvideosrc", "source");
#else
    source = gst_element_factory_make("libcamerasrc", "source");
#endif

    rtppay = gst_element_factory_make("rtph264pay", "rtppay");
    udpsink = gst_element_factory_make("udpsink", "udpsink");
    capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    videoscale = gst_element_factory_make("videoscale", "videoscale");
    videoconverter = gst_element_factory_make("videoconvert", "videoconvert");
    GstElement *rtpbin = gst_element_factory_make("rtpbin", "rtpbin");

    // Два разных кодировщика для стриминга и записи
    x264enc_stream = gst_element_factory_make("x264enc", "x264enc_stream");
    x264enc_record = gst_element_factory_make("x264enc", "x264enc_record");

    tee = gst_element_factory_make("tee", "tee");
    record_queue = gst_element_factory_make("queue", "record_queue");
    stream_queue = gst_element_factory_make("queue", "stream_queue"); // Очередь для стриминга
    mp4mux = gst_element_factory_make("mp4mux", "mp4mux");
    filesink = gst_element_factory_make("filesink", "filesink");

    GstCaps *caps = gst_caps_from_string("video/x-raw,format=I420,width=1280,height=720,framerate=30/1");
    g_object_set(capsfilter, "caps", caps, NULL);
    gst_caps_unref(caps);

    /* Create the empty pipeline */
    pipeline = gst_pipeline_new("adaptive_pipeline");

    if (!pipeline || !source || !capsfilter || !udpsink || !rtppay ||
        !videoscale || !x264enc_stream || !x264enc_record || !videoconverter ||
        !tee || !record_queue || !stream_queue || !mp4mux || !filesink || !rtpbin)
    {
        g_printerr("Not all elements could be created.\n");
        if (!stream_queue)
            g_printerr("Failed to create stream_queue\n");
        if (!x264enc_stream)
            g_printerr("Failed to create x264enc_stream\n");
        if (!x264enc_record)
            g_printerr("Failed to create x264enc_record\n");
        return -1;
    }

    // Добавляем все элементы в пайплайн
    gst_bin_add_many(GST_BIN(pipeline),
                     source, videoconverter, videoscale, capsfilter,
                     tee, rtpbin,
                     stream_queue, x264enc_stream, rtppay, udpsink,  // Ветка стриминга
                     record_queue, x264enc_record, mp4mux, filesink, // Ветка записи
                     NULL);

#ifdef _WIN32
    g_object_set(source, "device-index", 0, NULL);
#endif

    // Настройка RTP
    g_object_set(rtppay,
                 "pt", 96,
                 "config-interval", 1,
                 "mtu", 1400,
                 NULL);

    // Настройка UDP
    g_object_set(udpsink,
                 "host", IP_CLIENT,
                 "port", PORT_CLIENT,
                 "sync", FALSE,
                 "async", FALSE,
                 "buffer-size", 4194304,
                 NULL);

    // Настройка кодировщика для стриминга
    g_object_set(x264enc_stream,
                 "bitrate", 1500,
                 "speed-preset", 1,  // ultrafast
                 "tune", 0x00000004, // zerolatency
                 "key-int-max", 60,
                 NULL);

    // Настройка кодировщика для записи
    g_object_set(x264enc_record,
                 "bitrate", 2500,
                 "speed-preset", 2,  // superfast
                 "tune", 0x00000004, // zerolatency
                 "key-int-max", 60,
                 NULL);

    g_object_set(filesink, "location", "recording.mp4", NULL);

    // Связываем основную цепочку до tee
    if (!gst_element_link_many(source, videoconverter, videoscale, capsfilter, tee, NULL))
    {
        g_printerr("Failed to link source -> tee chain.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    // Связываем ветку стриминга
    if (!gst_element_link_many(stream_queue, x264enc_stream, rtppay, udpsink, NULL))
    {
        g_printerr("Failed to link stream chain.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    // Связываем ветку записи
    if (!gst_element_link_many(record_queue, x264enc_record, mp4mux, filesink, NULL))
    {
        g_printerr("Failed to link record chain.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    // Подключаем первую ветку от tee (стриминг)
    tee_src_pad1 = gst_element_request_pad_simple(tee, "src_%u");
    if (!tee_src_pad1)
    {
        g_printerr("Failed to get first tee src pad\n");
        gst_object_unref(pipeline);
        return -1;
    }

    queue_sink_pad = gst_element_get_static_pad(stream_queue, "sink");
    if (!queue_sink_pad)
    {
        g_printerr("Failed to get stream_queue sink pad\n");
        gst_object_unref(tee_src_pad1);
        gst_object_unref(pipeline);
        return -1;
    }

    if (gst_pad_link(tee_src_pad1, queue_sink_pad) != GST_PAD_LINK_OK)
    {
        g_printerr("Failed to link tee to stream_queue\n");
        gst_object_unref(tee_src_pad1);
        gst_object_unref(queue_sink_pad);
        gst_object_unref(pipeline);
        return -1;
    }

    gst_object_unref(tee_src_pad1);
    gst_object_unref(queue_sink_pad);

    // Подключаем вторую ветку от tee (запись)
    tee_src_pad2 = gst_element_request_pad_simple(tee, "src_%u");
    if (!tee_src_pad2)
    {
        g_printerr("Failed to get second tee src pad\n");
        gst_object_unref(pipeline);
        return -1;
    }

    queue_sink_pad = gst_element_get_static_pad(record_queue, "sink");
    if (!queue_sink_pad)
    {
        g_printerr("Failed to get record_queue sink pad\n");
        gst_object_unref(tee_src_pad2);
        gst_object_unref(pipeline);
        return -1;
    }

    if (gst_pad_link(tee_src_pad2, queue_sink_pad) != GST_PAD_LINK_OK)
    {
        g_printerr("Failed to link tee to record_queue\n");
        gst_object_unref(tee_src_pad2);
        gst_object_unref(queue_sink_pad);
        gst_object_unref(pipeline);
        return -1;
    }

    gst_object_unref(tee_src_pad2);
    gst_object_unref(queue_sink_pad);

    global_pipeline = pipeline;
    signal(SIGINT, sigint_handler);

    GstElement *msg_pipeline = init_msg_pipeline(8601);

    GMainLoop *main_loop = g_main_loop_new(NULL, FALSE);

    BusData *bus_data = g_new(BusData, 1);
    bus_data->pipeline = pipeline;
    bus_data->msg_pipeline = msg_pipeline;
    bus_data->main_loop = main_loop;

    // g_signal_connect(rtpbin, "on-timeout", G_CALLBACK(on_rtp_timeout), NULL);

    /* Start playing */
    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return -1;
    }
    ret = gst_element_set_state(msg_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return -1;
    }

/* Modify the source's properties */
#ifndef _WIN32
    g_object_set(source, "pattern", 0, NULL); // Linux
#endif

    /* Wait until error or EOS */
    bus = gst_element_get_bus(pipeline);
    GstBus *msg_bus = gst_element_get_bus(msg_pipeline);

    gst_bus_add_watch(bus, bus_msg_handler, bus_data);
    gst_bus_add_watch(msg_bus, bus_msg_handler, bus_data);

    // add func to gst main loop
    start_file_monitor("messages.txt", main_loop);

    // run event loop
    g_main_loop_run(main_loop);

    /* Free resources */
    gst_object_unref(bus);
    gst_object_unref(msg_bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_element_set_state(msg_pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_object_unref(msg_pipeline);
    g_main_loop_unref(main_loop);
    return 0;
}

int main(int argc, char *argv[])
{
#if defined(__APPLE__) && TARGET_OS_MAC && !TARGET_OS_IPHONE
    return gst_macos_main((GstMainFunc)tutorial_main, argc, argv, NULL);
#else
    return tutorial_main(argc, argv);
#endif
}
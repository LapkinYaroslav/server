#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QLabel>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDateTimeEdit>
#include <QtCore/QTimer>
#include <QtCore/QDateTime>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

// GStreamer includes
extern "C" {
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
}

#include <thread>
#include <mutex>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

class VideoProcessor {
public:
    VideoProcessor() : modelLoaded(false) {
        // Попробуем загрузить модель YOLO в различных форматах, включая ONNX и конвертацию из PyTorch
        try {
            // Попробуем найти файл модели в различных форматах
            std::vector<std::string> model_paths = {"yolov3-tiny.onnx", "/usr/local/share/opencv4/models/yolov3-tiny.onnx", "./models/yolov3-tiny.onnx", 
                                                   "yolo.pt", "/usr/local/share/opencv4/models/yolo.pt", "./models/yolo.pt",
                                                   "yolov3-tiny.weights", "/usr/local/share/opencv4/models/yolov3-tiny.weights", "./models/yolov3-tiny.weights"};
            
            for(const auto& model_path : model_paths) {
                try {
                    // First try to load as ONNX model
                    if(model_path.find(".onnx") != std::string::npos) {
                        net = cv::dnn::readNetFromONNX(model_path);
                    } else if(model_path.find(".weights") != std::string::npos) {
                        // Load Darknet model if it's .weights
                        std::string cfg_path = model_path;
                        cfg_path.replace(cfg_path.find(".weights"), 8, ".cfg");
                        
                        // Check if corresponding config file exists
                        if(fileExists(cfg_path)) {
                            net = cv::dnn::readNetFromDarknet(cfg_path, model_path);
                        } else {
                            // Try alternative naming
                            cfg_path = model_path.substr(0, model_path.find_last_of('/')) + "/" + 
                                      model_path.substr(model_path.find_last_of('/') + 1, model_path.find_last_of('.') - model_path.find_last_of('/') - 1) + ".cfg";
                            if(fileExists(cfg_path)) {
                                net = cv::dnn::readNetFromDarknet(cfg_path, model_path);
                            } else {
                                continue;
                            }
                        }
                    } else if(model_path.find(".pt") != std::string::npos) {
                        // For PyTorch models, we need to convert to ONNX format first or use alternative approach
                        // Since OpenCV doesn't directly support .pt models, we'll look for converted ONNX version
                        std::string onnx_path = model_path.substr(0, model_path.find_last_of('.')) + ".onnx";
                        if(fileExists(onnx_path)) {
                            net = cv::dnn::readNetFromONNX(onnx_path);
                        } else {
                            // If no ONNX version exists, we could potentially convert here
                            // For now, just skip .pt files since OpenCV DNN doesn't support them directly
                            continue;
                        }
                    } else {
                        continue; // Unsupported format
                    }
                    
                    if(!net.empty()) {
                        modelLoaded = true;
                        std::cout << "YOLO model loaded successfully from " << model_path << std::endl;
                        return;
                    }
                } catch(const std::exception& e) {
                    std::cout << "Failed to load model from " << model_path << ": " << e.what() << std::endl;
                    continue; // Пробуем следующий путь
                }
            }
            
            std::cout << "Warning: Could not load YOLO model, running without detection" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Warning: Could not load YOLO model, running without detection. Error: " << e.what() << std::endl;
        }
    }
    
    // Helper function to check if file exists
    bool fileExists(const std::string& filename) {
        std::ifstream file(filename);
        return file.good();
    }

    void processFrame(cv::Mat& frame) {
        if (!modelLoaded) return;

        // Подготовка изображения для YOLO
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1/255.0, cv::Size(416, 416), cv::Scalar(0,0,0), true, false);
        net.setInput(blob);
        
        std::vector<cv::String> outNames = net.getUnconnectedOutLayersNames();
        std::vector<cv::Mat> outs;
        net.forward(outs, outNames);

        // Обработка результатов детекции
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        for (size_t i = 0; i < outs.size(); ++i) {
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
                cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                cv::Point classId;
                double confidence;
                cv::minMaxLoc(scores, 0, &confidence, 0, &classId);

                if (confidence > 0.5) {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classId.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
        }

        // Подавление немаксимумов
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

        // Рисование рамок вокруг объектов
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            cv::Rect box = boxes[idx];
            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
            std::string label = cv::format("%.2f", confidences[idx]);
            cv::putText(frame, label, cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
    }

private:
    cv::dnn::Net net;
    bool modelLoaded;
};

class RobotControlClient : public QMainWindow {
    Q_OBJECT

public:
    RobotControlClient(QWidget *parent = nullptr) : QMainWindow(parent) {
        gst_init(NULL, NULL);
        setupUI();
        initializeGStreamerPipeline();
        
        // Таймер для обновления видео
        videoTimer = new QTimer(this);
        connect(videoTimer, &QTimer::timeout, this, &RobotControlClient::updateVideoFrame);
        videoTimer->start(33); // ~30 FPS
        
        processor = new VideoProcessor();
    }

private slots:
    void sendCommand(const QString& cmd) {
        GstMapInfo map_info;
        gchar buffer[2];
        buffer[0] = cmd.toStdString()[0];
        buffer[1] = '\n';
        
        GstBuffer *gst_buffer = gst_buffer_new_allocate(NULL, 2, NULL);
        gst_buffer_map(gst_buffer, &map_info, GST_MAP_WRITE);
        memcpy(map_info.data, buffer, 2);
        gst_buffer_unmap(gst_buffer, &map_info);
        
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), gst_buffer);
        if (ret != GST_FLOW_OK) {
            std::cout << "Failed to push buffer to appsrc" << std::endl;
        }
        
        // Логирование команды
        logCommand(cmd);
    }
    
    static GstFlowReturn new_sample_from_sink(GstElement *sink, gpointer user_data) {
        RobotControlClient *client = static_cast<RobotControlClient*>(user_data);
        return client->handleNewSample(sink);
    }
    
    GstFlowReturn handleNewSample(GstElement *sink) {
        GstSample *sample = NULL;
        g_object_get(GST_OBJECT(sink), "last-sample", &sample, NULL);
        
        if (sample) {
            GstBuffer *buffer = gst_sample_get_buffer(sample);
            GstCaps *caps = gst_sample_get_caps(sample);
            
            if (buffer && caps) {
                GstStructure *structure = gst_caps_get_structure(caps, 0);
                gint width, height;
                if (gst_structure_get_int(structure, "width", &width) &&
                    gst_structure_get_int(structure, "height", &height)) {
                    
                    GstMapInfo map_info;
                    if (gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
                        // Create OpenCV Mat from GStreamer buffer
                        frame = cv::Mat(height, width, CV_8UC3, (unsigned char *)map_info.data);
                        
                        // Make a copy of the data to ensure it stays valid
                        frame = frame.clone();
                        
                        gst_buffer_unmap(buffer, &map_info);
                    }
                }
            }
            gst_sample_unref(sample);
        }
        return GST_FLOW_OK;
    }
    
    void updateVideoFrame() {
        if (!frame.empty()) {
            // Обработка кадра с помощью YOLO
            processor->processFrame(frame);
            
            // Преобразование OpenCV Mat в QImage
            QImage qimg;
            if (frame.channels() == 3) {
                cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
                qimg = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            } else {
                qimg = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_Grayscale8);
            }
            
            videoLabel->setPixmap(QPixmap::fromImage(qimg.scaled(videoLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)));
        }
    }

private:
    void setupUI() {
        setWindowTitle("Robot Control Client");
        resize(800, 600);
        
        QWidget *centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);
        
        QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
        
        // Метка для видео
        videoLabel = new QLabel(this);
        videoLabel->setAlignment(Qt::AlignCenter);
        videoLabel->setMinimumSize(640, 480);
        videoLabel->setText("Video Feed");
        videoLabel->setStyleSheet("background-color: black;");
        mainLayout->addWidget(videoLabel);
        
        // Комбинированный список для выбора команды
        QHBoxLayout *controlLayout = new QHBoxLayout();
        
        commandCombo = new QComboBox(this);
        commandCombo->addItem("Stop", 'X');
        commandCombo->addItem("Forward", 'W');
        commandCombo->addItem("Backward", 'S');
        commandCombo->addItem("Left", 'A');
        commandCombo->addItem("Right", 'D');
        controlLayout->addWidget(commandCombo);
        
        QPushButton *sendButton = new QPushButton("Send Command", this);
        connect(sendButton, &QPushButton::clicked, this, [this]() {
            int index = commandCombo->currentIndex();
            QChar cmd = commandCombo->itemData(index).toChar();
            sendCommand(QString(cmd));
        });
        controlLayout->addWidget(sendButton);
        
        mainLayout->addLayout(controlLayout);
        
        // Кнопки управления
        QHBoxLayout *buttonLayout = new QHBoxLayout();
        
        QPushButton *forwardBtn = new QPushButton("W", this);
        forwardBtn->setFixedSize(60, 40);
        connect(forwardBtn, &QPushButton::clicked, this, [this]() { sendCommand("W"); });
        buttonLayout->addWidget(forwardBtn);
        
        QVBoxLayout *middleButtons = new QVBoxLayout();
        QPushButton *leftBtn = new QPushButton("A", this);
        leftBtn->setFixedSize(60, 40);
        connect(leftBtn, &QPushButton::clicked, this, [this]() { sendCommand("A"); });
        
        QPushButton *backBtn = new QPushButton("S", this);
        backBtn->setFixedSize(60, 40);
        connect(backBtn, &QPushButton::clicked, this, [this]() { sendCommand("S"); });
        
        QPushButton *rightBtn = new QPushButton("D", this);
        rightBtn->setFixedSize(60, 40);
        connect(rightBtn, &QPushButton::clicked, this, [this]() { sendCommand("D"); });
        
        middleButtons->addWidget(leftBtn);
        middleButtons->addWidget(backBtn);
        middleButtons->addWidget(rightBtn);
        
        QPushButton *stopBtn = new QPushButton("X", this);
        stopBtn->setFixedSize(60, 40);
        connect(stopBtn, &QPushButton::clicked, this, [this]() { sendCommand("X"); });
        
        buttonLayout->addWidget(forwardBtn);
        buttonLayout->addLayout(middleButtons);
        buttonLayout->addWidget(stopBtn);
        
        mainLayout->addLayout(buttonLayout);
    }
    
    void initializeGStreamerPipeline() {
        // Создаем пайплайн для отправки команд и получения видео через GStreamer
        pipeline = gst_pipeline_new("robot_control_pipeline");
        
        // AppSrc для отправки команд на сервер
        appsrc = gst_element_factory_make("appsrc", "command_source");
        g_object_set(GST_OBJECT(appsrc), "caps", 
                     gst_caps_new_simple("application/x-gst-control", 
                                         "format", G_TYPE_STRING, "text",
                                         NULL),
                     "stream-type", 0, // STREAM_TYPE_STREAM
                     "is-live", TRUE,
                     NULL);
        
        // TCP клиент для отправки команд
        tcp_client = gst_element_factory_make("tcpclientsink", "tcp_client");
        g_object_set(GST_OBJECT(tcp_client), 
                     "host", "192.168.31.251",
                     "port", 8601,
                     NULL);
        
        // Источник видео - UDP
        udpsrc = gst_element_factory_make("udpsrc", "udp_video_source");
        // Устанавливаем caps для правильного определения типа RTP
        GstCaps *udpcaps = gst_caps_from_string("application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96");
        g_object_set(GST_OBJECT(udpsrc), 
                     "port", 8600,
                     "caps", udpcaps,
                     "buffer-size", 2097152, // Увеличиваем размер буфера
                     NULL);
        gst_caps_unref(udpcaps);
        
        // Декапсуляция RTP
        rtp_depay = gst_element_factory_make("rtph264depay", "rtp_depayloader");
        
        // Парсер H.264
        h264_parse = gst_element_factory_make("h264parse", "h264_parser");
        
        // Декодер
        decoder = gst_element_factory_make("avdec_h264", "h264_decoder");
        // Настройка декодера для лучшей производительности
        g_object_set(decoder,
                     "max-threads", 4,
                     "skip-frame", 0,
                     "output-corrupt", FALSE,
                     NULL);
        
        // Конвертер видео
        video_convert = gst_element_factory_make("videoconvert", "video_converter");
        
        // AppSink для получения кадров
        appsink = gst_element_factory_make("appsink", "video_sink");
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                            "format", G_TYPE_STRING, "RGB",
                                            NULL);
        g_object_set(GST_OBJECT(appsink), 
                     "emit-signals", TRUE,
                     "caps", caps,
                     "sync", FALSE,
                     "max-lateness", -1,
                     "max-buffers", 5,
                     "drop", FALSE,
                     NULL);
        g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample_from_sink), this);
        
        // Добавляем элементы в пайплайн
        gst_bin_add_many(GST_BIN(pipeline), appsrc, tcp_client, udpsrc, rtp_depay, 
                         h264_parse, decoder, video_convert, appsink, NULL);
        
        // Связываем командный путь
        if (!gst_element_link(appsrc, tcp_client)) {
            std::cout << "Failed to link command source to TCP client" << std::endl;
        }
        
        // Связываем видео путь
        if (!gst_element_link_many(udpsrc, rtp_depay, h264_parse, decoder, video_convert, appsink, NULL)) {
            std::cout << "Failed to link video elements" << std::endl;
        }
        
        // Запускаем пайплайн
        GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cout << "Failed to start pipeline" << std::endl;
        }
    }
    
    void logCommand(const QString& cmd) {
        // Логирование команды в файл
        QFile logFile("client_commands.log");
        if (logFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
            QTextStream stream(&logFile);
            stream << QDateTime::currentDateTime().toString() << " - Command: " << cmd << "\n";
            logFile.close();
        }
    }
    
    GstElement *pipeline, *appsrc, *tcp_client, *udpsrc, *rtp_depay, *h264_parse, *decoder, *video_convert, *appsink;
    cv::Mat frame;
    QLabel *videoLabel;
    QComboBox *commandCombo;
    QTimer *videoTimer;
    VideoProcessor *processor;
};

#include "client.moc"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    RobotControlClient window;
    window.show();
    
    return app.exec();
}
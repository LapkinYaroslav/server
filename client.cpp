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
#include <QtNetwork/QTcpSocket>
#include <QtCore/QTimer>
#include <QtCore/QDateTime>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <iostream>
#include <vector>
#include <string>

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
        // Попробуем загрузить легковесную модель YOLO
        try {
            // Попробуем найти файлы модели YOLO
            std::vector<std::string> cfg_paths = {"yolov3-tiny.cfg", "/usr/local/share/opencv4/models/yolov3-tiny.cfg", "./models/yolov3-tiny.cfg"};
            std::vector<std::string> weights_paths = {"yolov3-tiny.weights", "/usr/local/share/opencv4/models/yolov3-tiny.weights", "./models/yolov3-tiny.weights"};
            
            for(const auto& cfg_path : cfg_paths) {
                for(const auto& weights_path : weights_paths) {
                    try {
                        net = cv::dnn::readNetFromDarknet(cfg_path, weights_path);
                        if(!net.empty()) {
                            modelLoaded = true;
                            std::cout << "YOLO model loaded successfully from " << cfg_path << " and " << weights_path << std::endl;
                            return;
                        }
                    } catch(...) {
                        continue; // Пробуем следующую комбинацию
                    }
                }
            }
            
            std::cout << "Warning: Could not load YOLO model, running without detection" << std::endl;
        } catch (...) {
            std::cout << "Warning: Could not load YOLO model, running without detection" << std::endl;
        }
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
        setupUI();
        connectToServer();
        setupVideoCapture();
        
        // Таймер для обновления видео
        videoTimer = new QTimer(this);
        connect(videoTimer, &QTimer::timeout, this, &RobotControlClient::updateVideoFrame);
        videoTimer->start(33); // ~30 FPS
        
        processor = new VideoProcessor();
    }

private slots:
    void sendCommand(const QString& cmd) {
        if (tcpSocket && tcpSocket->state() == QTcpSocket::ConnectedState) {
            tcpSocket->write(cmd.toUtf8());
            tcpSocket->flush();
            
            // Логирование команды
            logCommand(cmd);
        }
    }
    
    void updateVideoFrame() {
        if (cap.isOpened()) {
            cap >> frame;
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
    
    void connectToServer() {
        tcpSocket = new QTcpSocket(this);
        tcpSocket->connectToHost("192.168.31.251", 8601); // Адрес сервера
        
        if (tcpSocket->waitForConnected(5000)) {
            std::cout << "Connected to robot server" << std::endl;
        } else {
            std::cout << "Could not connect to robot server: " << tcpSocket->errorString().toStdString() << std::endl;
        }
    }
    
    void setupVideoCapture() {
        // Подключение к видеопотоку с Raspberry Pi
        cap.open("udpsrc port=8600 ! application/x-rtp, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink", cv::CAP_GSTREAMER);
        if (!cap.isOpened()) {
            std::cout << "Could not open video stream via GStreamer, trying direct UDP..." << std::endl;
            // Альтернативный способ открытия потока
            cap.open("http://192.168.31.251:8080/video_feed"); // Если используется HTTP поток
            if (!cap.isOpened()) {
                std::cout << "Could not open video stream" << std::endl;
            }
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
    
    QTcpSocket *tcpSocket;
    cv::VideoCapture cap;
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
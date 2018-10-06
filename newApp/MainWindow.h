#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/opencv.hpp>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QPushButton>
#include <QMainWindow>
#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QThread>
#include <QTime>
#include <QFileDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <string>
#include <iostream>
#include <fstream>

#include "uart.h"
#include "stm32.h"
#include "vp_vision.h"

using namespace std;
using namespace cv;

QImage cvMat2QImage(cv::Mat& mat);
Mat QImage2cvMat(QImage image);



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void safe_stop_ahead_back();//avoid damage electric machinery
    void safe_stop_up_down();//avoid damage electric machinery
    //RealSenseApp realsense;
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QTimer    *video_timer;
    QTimer    *camera_timer;
    QImage    camera_image;
    VideoCapture capture;
    VideoWriter createVideo;
    Mat camera_frame;
    bool camera_state;
    QSerialPort *my_serialport;
    string timeStr;
    QString stamp;
    int count;
    //STM32
    STM32 *Stm32;
    QThread Stm32_Thread;
    int Data_class;//use for commend input

    //// For color tracking
    Mat hsvFrame, threshLow, threshHigh, resultFrame_hsv;

    //// For the image-subtraction method
    cv::Mat colorFrame_current, colorFrame_previous, grayFrame_current, grayFrame_previous, diffFrame, resultFrame_imgsub;

    // For the background-subtraction method
    cv::Mat3f f_colorFrame, f_accumulatorFrame;
    //cv::Mat movingaverageFrame, graymovingaverageFrame;
    cv::Mat grayFrame, backgroundFrame, graybackgroundFrame, resultFrame_backsub;

    // For each tracking method
    cv::Mat resultFrame;


    // Mat data containers
    cv::Mat colorFrame;
    cv::Mat colorFrame_copy;
    cv::Mat depthFrame;


Q_SIGNALS:
    void emit_new_pwm();//emit the signals to change the underlying system pwm
    void emit_new_Stablilize_Yaw_PID();//emit the signals to change the underlying system pid
    void emit_new_Stablilize_Roll_PID();//emit the signals to change the underlying system pid
    void emit_new_Stablilize_Pitch_PID();//emit the signals to change the underlying system pid
    void emit_new_Stablilize_YawRate_PID();//emit the signals to change the underlying system pid
    void emit_new_Stablilize_RollRate_PID();//emit the signals to change the underlying system pid
    void emit_new_Stablilize_PitchRate_PID();//emit the signals to change the underlying system pid
    void emit_new_Depth_PID();
    void emit_new_Depth_Rate_PID();
    void emit_new_Target_Position();
    void new_Current_Position();
    void emit_new_Current_Position();
private Q_SLOTS:
    void timerUpdate();
    void on_open_camera_clicked();
    void readcamera_Frame();
    void writeVideo_Frame();
    void start_check_successful();
    void start_check_faild();
    void pwm_imu_dispay();
    void on_close_camera_clicked();
    void on_pic_camera_clicked();
    void on_shoot_video_clicked();
    void on_stop_video_clicked();
    void on_Tele_Ahead_clicked();
    void on_Tele_Back_clicked();
    void on_AUV_START_clicked();
    void on_AUV_STOP_clicked();
    void on_SendPort_Send_clicked();

    void on_Ahead_plus_clicked();
    void on_Ahead_reduce_clicked();
    void on_Back_plus_clicked();
    void on_Back_reduce_clicked();

    void on_Stablilize_Yaw_Send_clicked();
    void on_Stablilize_Roll_Send_clicked();
    void on_Stablilize_Pitch_Send_clicked();
    void on_Target_Position_Send_clicked();

    void on_Stablilize_YawRate_Send_clicked();
    void on_Stablilize_RollRate_Send_clicked();
    void on_Stablilize_PitchRate_Send_clicked();
};

#endif // MAINWINDOW_H

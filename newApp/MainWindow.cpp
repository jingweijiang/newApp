#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    camera_timer  = new QTimer(this);
    video_timer   = new QTimer(this);
    Stm32 = new STM32();
    Data_class=0;
    ui->textEdit->append(tr("Please push the Start Check button after this interface start 5 seconds"));
    ui->Telecontrol_Module->setEnabled(false);
    ui->videoRecordState->setStyleSheet("background:transparent");
    ui->videoRecordState->setVisible(false);
    ui->imuarea->setEnabled(false);
    ui->ultrasonicarea->setEnabled(false);
    ui->pidarea->setEnabled(false);
    ui->pwmarea->setEnabled(false);
    ui->Vision_area->setEnabled(false);
    //界面时间更新
    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timerUpdate()));
    timer->start(1000);
    //自动更新视频显示
    connect(camera_timer, SIGNAL(timeout()), this, SLOT(readcamera_Frame()));
    //自动写视频
    connect(video_timer, SIGNAL(timeout()), this, SLOT(writeVideo_Frame()));
    //new a STM32 thread
    Stm32_Thread.start(QThread::HighestPriority);
    Stm32->moveToThread(&Stm32_Thread);
    Stm32->Updata_timer->moveToThread(&Stm32_Thread);
    connect(ui->Start_Check,SIGNAL(clicked()),Stm32,SLOT(update_start()));
    //STM32 check
    connect(Stm32,SIGNAL(check_successful()),this,SLOT(start_check_successful()));
    connect(Stm32,SIGNAL(check_faild()),this,SLOT(start_check_faild()));
    //update IMU and PWM display
    connect(Stm32,SIGNAL(display_update()),this,SLOT(pwm_imu_dispay()));
    //manual input pwm
    connect(this,SIGNAL(emit_new_pwm()),Stm32,SLOT(change_pwm()));

    //manual Stabalize_Yaw_Pid_Confiig/////////////pengjun1.18/////////////////
    connect(this,SIGNAL(emit_new_Stablilize_Yaw_PID()),Stm32,SLOT(change_Stablilize_Yaw_PID()));
    connect(this,SIGNAL(emit_new_Stablilize_Roll_PID()),Stm32,SLOT(change_Stablilize_Roll_PID()));
    connect(this,SIGNAL(emit_new_Stablilize_Pitch_PID()),Stm32,SLOT(change_Stablilize_Pitch_PID()));
    connect(this,SIGNAL(emit_new_Target_Position()),Stm32,SLOT(change_Target_Position()));

    connect(this,SIGNAL(emit_new_Stablilize_YawRate_PID()),Stm32,SLOT(change_Stablilize_YawRate_PID()));
    connect(this,SIGNAL(emit_new_Stablilize_RollRate_PID()),Stm32,SLOT(change_Stablilize_RollRate_PID()));
    connect(this,SIGNAL(emit_new_Stablilize_PitchRate_PID()),Stm32,SLOT(change_Stablilize_PitchRate_PID()));

    //Remote control
    ui->Start_Check->setShortcut(Qt::Key_0);  //check
    ui->open_camera->setShortcut(Qt::Key_1);   //open camera
    ui->pic_camera->setShortcut(Qt::Key_F);    //take picture
    ui->shoot_video->setShortcut(Qt::Key_R);    //take video
    ui->stop_video->setShortcut(Qt::Key_C);     //stop video
    ui->AUV_START->setShortcut(Qt::Key_A);    //
    ui->IMU_Update_Sign->setShortcut(Qt::Key_I); // IMU_update_sign
    ui->AUV_STOP->setShortcut(Qt::Key_Space);   //AUV stop
    ui->Tele_Ahead->setShortcut(Qt::Key_Up);  //qian jin
    ui->Tele_Back->setShortcut(Qt::Key_Down);  //dao tui
    ui->Ahead_plus->setShortcut(Qt::Key_W);    //qian jin jia su
    ui->Ahead_reduce->setShortcut(Qt::Key_S);  //qian jin jian su
    ui->SendPort_Send->setShortcut(Qt::Key_Enter);
}
MainWindow::~MainWindow()
{
    capture.release();
    if(Stm32_Thread.isRunning())
    {
        Stm32_Thread.exit();
        Stm32_Thread.wait();
    }
    delete ui;
}
//update display
void MainWindow::pwm_imu_dispay()
{

    if(ui->IMU_Update_Sign->isChecked())
    {
        ui->IMU_A_Ang_X->setText(QString::number(IMU_A_Ang_c[0]));//(Data_vel[0]));角度
        ui->IMU_A_Ang_Y->setText(QString::number(IMU_A_Ang_c[1]));//(Data_vel[1]));
        ui->IMU_A_Ang_Z->setText(QString::number(IMU_A_Ang_c[2]));//(Data_vel[2]));
        ui->IMU_L_Acc_X->setText(QString::number(IMU_L_Acc_c[0]));//(Data_acc[0]));加速度
        ui->IMU_L_Acc_Y->setText(QString::number(IMU_L_Acc_c[1]));//(Data_acc[1]));
        ui->IMU_L_Acc_Z->setText(QString::number(IMU_L_Acc_c[2]));//(Data_acc[2]));
        ui->IMU_A_Vel_X->setText(QString::number(IMU_A_Vel_c[0]));//(Data_dis[0]));角速度
        ui->IMU_A_Vel_Y->setText(QString::number(IMU_A_Vel_c[1]));//(Data_dis[1]));
        ui->IMU_A_Vel_Z->setText(QString::number(IMU_A_Vel_c[2]));//(Data_dis[2]));


        ui->Motor_PWM_0->setText(QString::number( Motor_PWM_Show[0]));
        ui->Motor_PWM_1->setText(QString::number( Motor_PWM_Show[1]));
        ui->Motor_PWM_2->setText(QString::number( Motor_PWM_Show[2]));
        ui->Motor_PWM_3->setText(QString::number( Motor_PWM_Show[3]));

        ui->ULTRASONIC_FRONT->setText(QString::number( ULTRASONIC_WAVE[0]));
        ui->ULTRASONIC_BACK->setText(QString::number( ULTRASONIC_WAVE[1]));

        if(ui->IMU_Rec->isChecked())
        {
            int i;
            fstream imu_file("/home/ubuntu/Desktop/imu_rec.txt",ios::out|ios::app);
            if(!imu_file.is_open())
            {
                ui->textEdit->append("The imu_file open is failed");
            }
            for(i=0;i<3;i++)
            {
                imu_file<<IMU_A_Ang_c[i]<<" ";
                imu_file<<IMU_A_Vel_c[i]<<" ";
                imu_file<<IMU_L_Acc_c[i]<<" ";
            }
            for(i=0;i<4;i++)
            {
                imu_file<<Motor_PWM_Show[i]<<" ";
            }
            imu_file<<"\n";
            imu_file.close();
        }
    }
}
//打开摄像头
void MainWindow::on_open_camera_clicked()
{
//    cv::setUseOptimized(true);//CPU的硬件指令优化功能被开启
//    std::cout << "Terminate by pressing the q key\n";
//    //realsense.updateFrame();  //读取相机视频流，实现对其并转化成Mat类型
//    colorFrame_previous = realsense.GetcolorFrame().clone();
//    //colorFrame_previous = colorFrame.clone();

//    f_accumulatorFrame = cv::Mat::zeros(cv::Size(640, 480), CV_32FC3);
    capture.open(1);
    capture>>camera_frame;
    mark_point = vanish_point_detection(camera_frame, camera_frame);
    center_x_max = (1 + CENTER_RANGE) * mark_point.x;
    center_x_min = (1 - CENTER_RANGE) * mark_point.x;
    center_y_max = (1 + CENTER_RANGE) * mark_point.y;
    center_y_min = (1 - CENTER_RANGE) * mark_point.y;

    camera_timer->start(50);
    ui->shoot_video->setEnabled(true);
    ui->open_camera->setEnabled(false);
    ui->stop_video->setEnabled(false);
}
//读取摄像头并显示在窗口
void MainWindow::readcamera_Frame()
{
//    realsense.updateFrame();
//    colorFrame = realsense.GetcolorFrame();
//    colorFrame_copy = colorFrame.clone();
//    realsense.FPS();
//    depthFrame = realsense.GetDepthImage();

//    switch (realsense.GettrackingMethod())
//    {
//    case 0: // Color tracking (Detects reddish colors with the following parameters)
//        cv::cvtColor(colorFrame, hsvFrame, CV_BGR2HSV);
//        cv::inRange(hsvFrame, cv::Scalar(0, 155, 155), cv::Scalar(18, 255, 255), threshLow);
//        cv::inRange(hsvFrame, cv::Scalar(165, 155, 155), cv::Scalar(179, 255, 255), threshHigh);
//        cv::add(threshLow, threshHigh, resultFrame_hsv);

//        // Optional edits for reducing noise
//        cv::GaussianBlur(resultFrame_hsv, resultFrame_hsv, cv::Size(5, 5), 0);
//        ///形态学变换 膨胀、腐蚀
//        cv::dilate(resultFrame_hsv, resultFrame_hsv, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
//        cv::erode(resultFrame_hsv, resultFrame_hsv, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

//        //cv::imshow("Color Tracking", resultFrame_hsv);

//        resultFrame = resultFrame_hsv.clone();
//        break;
//    case 1: // Image subtraction
//        colorFrame_current = colorFrame.clone();
//        cv::cvtColor(colorFrame_current, grayFrame_current, CV_BGR2GRAY);
//        cv::cvtColor(colorFrame_previous, grayFrame_previous, CV_BGR2GRAY);
//        cv::GaussianBlur(grayFrame_current, grayFrame_current, cv::Size(5, 5), 0);
//        cv::GaussianBlur(grayFrame_previous, grayFrame_previous, cv::Size(5, 5), 0);
//        cv::absdiff(grayFrame_current, grayFrame_previous, diffFrame);             //计算两个数组差的绝对值函数
//        cv::threshold(diffFrame, resultFrame_imgsub, 30, 255.0, CV_THRESH_BINARY);
//        colorFrame_previous = colorFrame_current.clone();

//        // Optional edits for reducing noise
//        cv::dilate(resultFrame_imgsub, resultFrame_imgsub, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
//        cv::erode(resultFrame_imgsub, resultFrame_imgsub, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

//        cv::imshow("Image Subtraction", resultFrame_imgsub);

//        resultFrame = resultFrame_imgsub.clone();
//        break;
//    case 2: // Background subtraction
//        f_colorFrame = cv::Mat3f(colorFrame);
//        cv::accumulateWeighted(f_colorFrame, f_accumulatorFrame, 0.05);
//        cv::convertScaleAbs(f_accumulatorFrame, backgroundFrame);
//        cv::imshow("Background Frame", backgroundFrame);

//        cv::cvtColor(colorFrame, grayFrame, CV_BGR2GRAY);
//        cv::cvtColor(backgroundFrame, graybackgroundFrame, CV_BGR2GRAY);
//        cv::absdiff(grayFrame, graybackgroundFrame, diffFrame);
//        cv::threshold(diffFrame, resultFrame_backsub, 30, 255.0, CV_THRESH_BINARY);

//        // Optional edits for reducing noise
//        cv::dilate(resultFrame_backsub, resultFrame_backsub, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
//        cv::erode(resultFrame_backsub, resultFrame_backsub, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

//        cv::imshow("Background Subtraction", resultFrame_backsub);

//        resultFrame = resultFrame_backsub.clone();
//        break;
//    }

//    if (realsense.GettrackingMethod() < 3)
//        realsense.DetectCenterOfObject(resultFrame);

    if (colorFrame_copy.rows != 0 && colorFrame_copy.cols != 0)
        cv::imshow("Color Image", colorFrame_copy);
    if (depthFrame.rows != 0 && depthFrame.cols != 0)
        cv::imshow("Depth Image", depthFrame);


    if(!colorFrame.empty())
    {
        if(ui->colortracking->isChecked())
        {
            cv::imshow("color tracking",resultFrame);
            camera_image=cvMat2QImage(resultFrame);
            QImage camera_result=camera_image.scaled(ui->camera->width(),ui->camera->height(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation).rgbSwapped();
            ui->camera->setPixmap(QPixmap::fromImage(camera_result));
        }
        else if(ui->depthinfo->isChecked())
        {
            if (depthFrame.rows != 0 && depthFrame.cols != 0)
            {
                camera_image=cvMat2QImage(depthFrame);
                QImage camera_result=camera_image.scaled(ui->camera->width(),ui->camera->height(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation).rgbSwapped();
                ui->camera->setPixmap(QPixmap::fromImage(camera_result));
            }
            else
                cout<<"failed load depth image!"<<endl;
        }
        else
        {
            if (colorFrame_copy.rows != 0 && colorFrame_copy.cols != 0)
            {
                camera_image=cvMat2QImage(colorFrame_copy);
                QImage camera_result=camera_image.scaled(ui->camera->width(),ui->camera->height(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation).rgbSwapped();
                ui->camera->setPixmap(QPixmap::fromImage(camera_result));
            }
            else
                cout<<"failed load color image!"<<endl;
        }
    }



    capture>>camera_frame;
    if(!camera_frame.empty())
      {
        if(ui->vps->isChecked())
        {
            current_point = vanish_point_detection(camera_frame, camera_frame);
            if (current_point.x > center_x_max)
            {
                cout << "向左偏斜" << endl;
            }
            else if (current_point.x < center_x_min)
            {
                cout << "向右偏斜" << endl;
            }
            else
            {
                cout << "center" << endl;
            }

            line(camera_frame, Point(mark_point.x, 0), Point(mark_point.x, camera_frame.rows), Scalar(0, 0, 0), 1, CV_AA);
            camera_image=cvMat2QImage(camera_frame);
            QImage camera_result=camera_image.scaled(ui->camera->width(),ui->camera->height(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation).rgbSwapped();
            ui->camera->setPixmap(QPixmap::fromImage(camera_result));
        }
        else
        {
            camera_image=cvMat2QImage(camera_frame);
            QImage camera_result=camera_image.scaled(ui->camera->width(),ui->camera->height(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation).rgbSwapped();
            ui->camera->setPixmap(QPixmap::fromImage(camera_result));
        }
      }
}

void MainWindow::writeVideo_Frame()
{
    createVideo << camera_frame;
}
//开始录制视频
void MainWindow::on_shoot_video_clicked()
{
    string strN =".avi";
    string str = timeStr + strN;
    createVideo.open(str,CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(camera_frame.cols,camera_frame.rows));
    cout<<"record video"<<endl;
    ui->videoRecordState->setVisible(true);
    ui->shoot_video->setEnabled(false);
    ui->stop_video->setEnabled(true);
     ui->realtime->setVisible(false);
    video_timer->start(40);
}
//结束视频录制
void MainWindow::on_stop_video_clicked()
{
    video_timer->stop();
    ui->videoRecordState->setVisible(false);
    ui->shoot_video->setEnabled(true);
    ui->stop_video->setEnabled(false);

}
//关闭摄像头
void MainWindow::on_close_camera_clicked()
{
    camera_timer->stop();
    capture.release();
    ui->camera->clear();
    ui->realtime->setVisible(true);
    ui->videoRecordState->setVisible(false);
    ui->open_camera->setEnabled(true);
}
//采集图像
void MainWindow::on_pic_camera_clicked()
{
    QImage camera_result=camera_image.scaled(ui->camera->width(),ui->camera->height(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation).rgbSwapped();
    ui->camera->setPixmap(QPixmap::fromImage(camera_result));
    Mat frame_src = camera_frame;

    string strN ="_ori.jpg";
    string str = timeStr + strN;
    imwrite(str, frame_src);;

}
//实时时间显示
void MainWindow::timerUpdate()
{
    QDateTime time = QDateTime::currentDateTime();
    QString str = time.toString("yyyy-MM-dd hh:mm:ss");
    QString strForFile = time.toString("yyyy-MM-dd hh-mm-ss");
    stamp = time.toString("hh:mm:ss:");
    timeStr = strForFile.toStdString();
    ui->realtime->setText(str);
}
//Mat转换成Qimage
QImage cvMat2QImage(Mat& mat)
{
    QImage* qimg=new QImage;
    *qimg = QImage((unsigned char*)mat.data, // uchar* data
            mat.cols, mat.rows, // width height
            QImage::Format_RGB888); //format
    return *qimg;
}
//Qimage转换成Mat
Mat QImage2cvMat(QImage image)
{
    Mat mat;
    switch(image.format())
    {
        case QImage::Format_ARGB32:
        case QImage::Format_RGB32:
        case QImage::Format_ARGB32_Premultiplied:
            mat = Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
            break;
        case QImage::Format_RGB888:
            mat = Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
            cvtColor(mat, mat, CV_BGR2RGB);
            break;
        case QImage::Format_Indexed8:
            mat = Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
            break;
    }
    return mat;
}

void MainWindow::start_check_successful()
{
   ui->textEdit->append("the underlying system start successfully !");
   ui->Telecontrol_Module->setEnabled(true);
   ui->Start_Check->setEnabled(true);
   ui->imuarea->setEnabled(true);
   ui->ultrasonicarea->setEnabled(true);
   ui->pidarea->setEnabled(true);
   ui->pwmarea->setEnabled(true);
   ui->Vision_area->setEnabled(true);
}
void MainWindow::start_check_faild()
{
    ui->textEdit->append("the underlying system error!");
    ui->textEdit->append("error code :");
    ui->textEdit->append(QString::number(Check_Sign));
}

//avoid damage electric machinery
void MainWindow::safe_stop_ahead_back()
{
    Motor_PWM_Control[0]=725;
    Motor_PWM_Control[1]=725;
    Motor_PWM_Control[2]=725;
    Motor_PWM_Control[3]=725;
    Q_EMIT this->emit_new_pwm();
    usleep(2000000);//500ms
}

void MainWindow::on_AUV_START_clicked()
{
    Motor_PWM_Control[0]=725;
    Motor_PWM_Control[1]=725;
    Motor_PWM_Control[2]=725;
    Motor_PWM_Control[3]=725;
    Q_EMIT this->emit_new_pwm();
    ui->textEdit->append("ROBOT has been started,please select the control MODE");
}

void MainWindow::on_Tele_Ahead_clicked()
{
    safe_stop_ahead_back();
    Motor_PWM_Control[0]=737;
    Motor_PWM_Control[1]=736;
    Motor_PWM_Control[2]=735;
    Motor_PWM_Control[3]=735;
    Q_EMIT this->emit_new_pwm();
}

void MainWindow::on_Ahead_plus_clicked()
{
    Motor_PWM_Control[0]=Motor_PWM_Control[0]+Motor_PWM_ADD;
    Motor_PWM_Control[1]=Motor_PWM_Control[1]+Motor_PWM_ADD;
    Motor_PWM_Control[2]=Motor_PWM_Control[2]+Motor_PWM_ADD;
    Motor_PWM_Control[3]=Motor_PWM_Control[3]+Motor_PWM_ADD;
    Q_EMIT this->emit_new_pwm();
}

void MainWindow::on_Ahead_reduce_clicked()
{
    Motor_PWM_Control[0]=Motor_PWM_Control[0]-Motor_PWM_ADD;
    Motor_PWM_Control[1]=Motor_PWM_Control[1]-Motor_PWM_ADD;
    Motor_PWM_Control[2]=Motor_PWM_Control[2]-Motor_PWM_ADD;
    Motor_PWM_Control[3]=Motor_PWM_Control[3]-Motor_PWM_ADD;
    Q_EMIT this->emit_new_pwm();
}

void MainWindow::on_Tele_Back_clicked()
{
    safe_stop_ahead_back();
    Motor_PWM_Control[0]=713;
    Motor_PWM_Control[1]=715;
    Motor_PWM_Control[2]=711;
    Motor_PWM_Control[3]=708;
    Q_EMIT this->emit_new_pwm();
}

void MainWindow::on_Back_plus_clicked()
{
    Motor_PWM_Control[0]=Motor_PWM_Control[0]-Motor_PWM_ADD;
    Motor_PWM_Control[1]=Motor_PWM_Control[1]-Motor_PWM_ADD;
    Motor_PWM_Control[2]=Motor_PWM_Control[2]-Motor_PWM_ADD;
    Motor_PWM_Control[3]=Motor_PWM_Control[3]-Motor_PWM_ADD;
    Q_EMIT this->emit_new_pwm();
}

void MainWindow::on_Back_reduce_clicked()
{
    Motor_PWM_Control[0]=Motor_PWM_Control[0]+Motor_PWM_ADD;
    Motor_PWM_Control[1]=Motor_PWM_Control[1]+Motor_PWM_ADD;
    Motor_PWM_Control[2]=Motor_PWM_Control[2]+Motor_PWM_ADD;
    Motor_PWM_Control[3]=Motor_PWM_Control[3]+Motor_PWM_ADD;
    Q_EMIT this->emit_new_pwm();
}


void MainWindow::on_AUV_STOP_clicked()
{
    Motor_PWM_Control[0]=0;
    Motor_PWM_Control[1]=0;
    Motor_PWM_Control[2]=0;
    Motor_PWM_Control[3]=0;
    Q_EMIT this->emit_new_pwm();
    ui->textEdit->append("ROBOT has been stoped ");
}


void MainWindow::on_SendPort_Send_clicked()
{
    QString Command_Input =ui->Command_Input_Text->text();

    bool ok;
    char *Command_input_char=Command_Input.toLocal8Bit().data();
    int Data_Input = Command_Input.toInt(&ok,10);
    if(!ok)
    {
        switch(Command_input_char[0])
        {
            case 'l':
                ui->textEdit->append("Motor control command,please input the Motor_PWM_ADD");
                ui->Command_Input_Text->clear();
                Data_class=1;
                break;
            case 'a':
                ui->textEdit->append("Motor control command,please input the Motor_0 PWM ");
                ui->Command_Input_Text->clear();
                Data_class=2;
                break;
            case 'b':
                ui->textEdit->append("Motor control command,please input the Motor_1 PWM ");
                ui->Command_Input_Text->clear();
                Data_class=3;
                break;
            case 'c':
                ui->textEdit->append("Motor control command,please input the Motor_2 PWM ");
                ui->Command_Input_Text->clear();
                Data_class=4;
                break;
            case 'd':
                ui->textEdit->append("Motor control command,please input the Motor_3 PWM ");
                ui->Command_Input_Text->clear();
                Data_class=5;
                break;

            default :
                ui->textEdit->append("Invalid command,please enter again ");
                ui->Command_Input_Text->clear();
                break;
        }

    }
    else
    {
        if(Data_Input>=0&&Data_Input<1600)
        {
            switch(Data_class)
            {
            case 1:
                ui->textEdit->append("Motor_PWM_ADD: "+Command_Input);
                ui->Command_Input_Text->clear();
                Motor_PWM_ADD=Data_Input;
                Data_class=0;
                break;
            case 2:
                ui->textEdit->append("Motor_0 :" +Command_Input );
                ui->Command_Input_Text->clear();
                Motor_PWM_Control[0]=Data_Input;
                Q_EMIT this->emit_new_pwm();
                Data_class=0;
                break;
            case 3:
                ui->textEdit->append("Motor_1 :" +Command_Input);
                ui->Command_Input_Text->clear();
                Motor_PWM_Control[1]=Data_Input;
                Q_EMIT this->emit_new_pwm();
                Data_class=0;
                break;
            case 4:
                ui->textEdit->append("Motor_2 :" +Command_Input);
                ui->Command_Input_Text->clear();
                Motor_PWM_Control[2]=Data_Input;
                Q_EMIT this->emit_new_pwm();
                Data_class=0;
                break;
            case 5:
                ui->textEdit->append("Motor_3 :" +Command_Input);
                ui->Command_Input_Text->clear();
                Motor_PWM_Control[3]=Data_Input;
                Q_EMIT this->emit_new_pwm();
                Data_class=0;
                break;

            default :
                ui->textEdit->append("Invalid command,please enter again ");
                ui->Command_Input_Text->clear();
                Data_class=0;
                break;
            }

        }
        else
        {
            ui->textEdit->append("Invalid value,please enter again ");
            ui->Command_Input_Text->clear();
        }

    }
}
void MainWindow::on_Stablilize_Yaw_Send_clicked()
{
    QString Stablilize_Yaw_Kp_Input =ui->Stablilize_Yaw_Kp_Input->text();
    QString Stablilize_Yaw_Ki_Input =ui->Stablilize_Yaw_Ki_Input->text();
    QString Stablilize_Yaw_Kd_Input =ui->Stablilize_Yaw_Kd_Input->text();
    Stablilize_Yaw_PID_Input_Buff[0]=Stablilize_Yaw_Kp_Input.toFloat();/////将Stablilize_Yaw_Kp_Input中的值转化成float型
    Stablilize_Yaw_PID_Input_Buff[1]=Stablilize_Yaw_Ki_Input.toFloat();
    Stablilize_Yaw_PID_Input_Buff[2]=Stablilize_Yaw_Kd_Input.toFloat();
    Q_EMIT this->emit_new_Stablilize_Yaw_PID();
}

void MainWindow::on_Stablilize_Roll_Send_clicked()
{
    QString Stablilize_Roll_Kp_Input =ui->Stablilize_Roll_Kp_Input->text();
    QString Stablilize_Roll_Ki_Input =ui->Stablilize_Roll_Ki_Input->text();
    QString Stablilize_Roll_Kd_Input =ui->Stablilize_Roll_Kd_Input->text();
    Stablilize_Roll_PID_Input_Buff[0]=Stablilize_Roll_Kp_Input.toFloat();/////将Stablilize_Roll_Kp_Input中的值转化成float型
    Stablilize_Roll_PID_Input_Buff[1]=Stablilize_Roll_Ki_Input.toFloat();
    Stablilize_Roll_PID_Input_Buff[2]=Stablilize_Roll_Kd_Input.toFloat();
    Q_EMIT this->emit_new_Stablilize_Roll_PID();
}

void MainWindow::on_Stablilize_Pitch_Send_clicked()
{
    QString Stablilize_Pitch_Kp_Input =ui->Stablilize_Pitch_Kp_Input->text();
    QString Stablilize_Pitch_Ki_Input =ui->Stablilize_Pitch_Ki_Input->text();
    QString Stablilize_Pitch_Kd_Input =ui->Stablilize_Pitch_Kd_Input->text();
    Stablilize_Pitch_PID_Input_Buff[0]=Stablilize_Pitch_Kp_Input.toFloat();/////将Stablilize_Pitch_Kp_Input中的值转化成float型
    Stablilize_Pitch_PID_Input_Buff[1]=Stablilize_Pitch_Ki_Input.toFloat();
    Stablilize_Pitch_PID_Input_Buff[2]=Stablilize_Pitch_Kd_Input.toFloat();
    Q_EMIT this->emit_new_Stablilize_Pitch_PID();
}

void MainWindow::on_Target_Position_Send_clicked()
{
    QString Target_Position_Yaw_Input =ui->Target_Position_Yaw_Input->text();
    QString Target_Position_Roll_Input =ui->Target_Position_Roll_Input->text();
    QString Target_Position_Pitch_Input =ui->Target_Position_Pitch_Input->text();
    Target_Position_Buff[0]=Target_Position_Yaw_Input.toFloat();
    Target_Position_Buff[1]=Target_Position_Roll_Input.toFloat();
    Target_Position_Buff[2]=Target_Position_Pitch_Input.toFloat();

    Q_EMIT this->emit_new_Target_Position();
}


/////////////////////////////////////////////////////////////////////////
void MainWindow::on_Stablilize_YawRate_Send_clicked()
{
    QString Stablilize_YawRate_Kp_Input =ui->Stablilize_YawRate_Kp_Input->text();
    QString Stablilize_YawRate_Ki_Input =ui->Stablilize_YawRate_Ki_Input->text();
    QString Stablilize_YawRate_Kd_Input =ui->Stablilize_YawRate_Kd_Input->text();

    Stablilize_YawRate_PID_Input_Buff[0]=Stablilize_YawRate_Kp_Input.toFloat();/////将Stablilize_YawRate_Kp_Input中的值转化成float型
    Stablilize_YawRate_PID_Input_Buff[1]=Stablilize_YawRate_Ki_Input.toFloat();
    Stablilize_YawRate_PID_Input_Buff[2]=Stablilize_YawRate_Kd_Input.toFloat();
    Q_EMIT this->emit_new_Stablilize_YawRate_PID();
}

void MainWindow::on_Stablilize_RollRate_Send_clicked()
{
    QString Stablilize_RollRate_Kp_Input =ui->Stablilize_RollRate_Kp_Input->text();
    QString Stablilize_RollRate_Ki_Input =ui->Stablilize_RollRate_Ki_Input->text();
    QString Stablilize_RollRate_Kd_Input =ui->Stablilize_RollRate_Kd_Input->text();
    Stablilize_RollRate_PID_Input_Buff[0]=Stablilize_RollRate_Kp_Input.toFloat();/////将Stablilize_Roll_Kp_Input中的值转化成float型
    Stablilize_RollRate_PID_Input_Buff[1]=Stablilize_RollRate_Ki_Input.toFloat();
    Stablilize_RollRate_PID_Input_Buff[2]=Stablilize_RollRate_Kd_Input.toFloat();
    Q_EMIT this->emit_new_Stablilize_RollRate_PID();
}

void MainWindow::on_Stablilize_PitchRate_Send_clicked()
{
    QString Stablilize_PitchRate_Kp_Input =ui->Stablilize_PitchRate_Kp_Input->text();
    QString Stablilize_PitchRate_Ki_Input =ui->Stablilize_PitchRate_Ki_Input->text();
    QString Stablilize_PitchRate_Kd_Input =ui->Stablilize_PitchRate_Kd_Input->text();
    Stablilize_PitchRate_PID_Input_Buff[0]=Stablilize_PitchRate_Kp_Input.toFloat();/////将Stablilize_PitchRate_Kp_Input中的值转化成float型
    Stablilize_PitchRate_PID_Input_Buff[1]=Stablilize_PitchRate_Ki_Input.toFloat();
    Stablilize_PitchRate_PID_Input_Buff[2]=Stablilize_PitchRate_Kd_Input.toFloat();
    Q_EMIT this->emit_new_Stablilize_PitchRate_PID();
}



#ifndef DATALIB_H
#define DATALIB_H


#include <qdatetime.h>
#include <QDateTime>
#include <QTimer>
#include <QPoint>
#include <opencv2/core/core.hpp>



#define PI 3.1415926
#define CENTER_RANGE 0.1

extern int Check_Sign;
extern int Check_imu;

extern cv::Point mark_point, current_point;
extern int center_x_min, center_x_max, center_y_min, center_y_max;

extern float IMU_L_Acc[3];//linear acceleration
extern float IMU_L_Vel[3];//linear velocity
extern float IMU_L_Dis[3];//linear displacement
extern float IMU_A_Acc[3];
extern float IMU_A_Vel[3];
extern float IMU_A_Ang[3];

extern float IMU_L_Acc_c[3];//linear acceleration after correction
extern float IMU_L_Vel_c[3];//linear velocity after correction
extern float IMU_L_Dis_c[3];//linear displacement after correction
extern float IMU_A_Acc_c[3];
extern float IMU_A_Vel_c[3];
extern float IMU_A_Ang_c[3];

extern float IMU_L_Acc_Offset[3];
extern float IMU_A_Vel_Offset[3];
extern float IMU_A_Ang_Offset[3];
extern float IMU_DATA_PACKAGE[500][9];


extern float ULTRASONIC_WAVE[2];


extern double Data_acc[3];
extern double Data_vel[3];
extern double Data_dis[3];
extern double Data_dis_world[3];
extern double Data_eul[3];


extern int Motor_PWM_Show[4];
extern int Motor_PWM_Control[4];
extern int Motor_PWM_ADD;
extern int PWM[4];

//PICTURE PARAMETERES
extern int Code_ID;
extern int Taget_ID;

//TAGET PATAMETERS
extern double Taget_1[6];


extern double PID_ADD_0;
extern double PID_ADD_1;


extern float Stablilize_Yaw_PID_Input_Buff[3];
extern float Stablilize_Roll_PID_Input_Buff[3];
extern float Stablilize_Pitch_PID_Input_Buff[3];

extern float Stablilize_YawRate_PID_Input_Buff[3];
extern float Stablilize_RollRate_PID_Input_Buff[3];
extern float Stablilize_PitchRate_PID_Input_Buff[3];
extern float Target_Position_Rate_Buff[6];
extern float Target_Position_Buff[6];
extern float Current_Position_Buff[6];

void float_to_char(float f,unsigned char *s);

void int_to_char(int t,unsigned char *s);

void char_to_float(unsigned char *s, float* f);

void char_to_int(unsigned char *s,int *t);



#endif // DATALIB_H

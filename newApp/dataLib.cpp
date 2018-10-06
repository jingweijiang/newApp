#include "dataLib.h"
#include <qdatetime.h>
#include <QDateTime>
#include <QTimer>
#include <QPoint>

int Check_Sign=0;
int Check_imu=1;

cv::Point mark_point, current_point;
int center_x_min, center_x_max, center_y_min, center_y_max;

float IMU_L_Acc[3]={0};
float IMU_L_Vel[3]={0};
float IMU_L_Dis[3]={0};
float IMU_A_Acc[3]={0};
float IMU_A_Vel[3]={0};
float IMU_A_Ang[3]={0};


float IMU_L_Acc_c[3]={0};
float IMU_L_Vel_c[3]={0};
float IMU_L_Dis_c[3]={0};
float IMU_A_Acc_c[3]={0};
float IMU_A_Vel_c[3]={0};
float IMU_A_Ang_c[3]={0};

float IMU_L_Acc_Offset[3]={0};
float IMU_A_Vel_Offset[3]={0};
float IMU_A_Ang_Offset[3]={0};
float IMU_DATA_PACKAGE[500][9]={0};


float ULTRASONIC_WAVE[2];

double Data_acc[3]={0};
double Data_vel[3]={0};
double Data_dis[3] = {0};//the end of displacement data
double Data_eul[3] = {0};//tne end of eular angle data


int Motor_PWM_Show[4]={0};
int Motor_PWM_Control[4]={0};
int PWM[4];
int Motor_PWM_ADD=1;

int Code_ID=0;
int Taget_ID=0;

double Taget_1[6]={900,0,0,0,0,0};

double AUV_L1=0.071;
double AUV_L2=0.071;
double AUV_L3=0.150;
double AUV_L4=0.020;
double AUV_fG=140;
double AUV_fb=141;

double PID_ADD_0=0;
double PID_ADD_1=0;


/*////向底层设置PID的值////*/
float Stablilize_Yaw_PID_Input_Buff[3]={0};
float Stablilize_Roll_PID_Input_Buff[3]={0};
float Stablilize_Pitch_PID_Input_Buff[3]={0};

float Stablilize_YawRate_PID_Input_Buff[3]={0};
float Stablilize_RollRate_PID_Input_Buff[3]={0};
float Stablilize_PitchRate_PID_Input_Buff[3]={0};
float Target_Position_Rate_Buff[6]={0};
float Yaw_Rate_PID_Input_Buff[3]={0};
float Depth_PID_Input_Buff[3]={0};
float Depth_Rate_PID_Input_Buff[3]={0};
float Target_Position_Buff[6]={0};

void float_to_char(float f,unsigned char *s)
{
    union change
    {
        float d;
        unsigned char dat[4];
    }r1;
    r1.d = f;
    *s = r1.dat[0];
    *(s+1) = r1.dat[1];
    *(s+2) = r1.dat[2];
    *(s+3) = r1.dat[3];
}

void  int_to_char(int t,unsigned char *s)
{
    *s = t >>8;
    *(s+1) = t;
}

void char_to_float(unsigned char *s, float* f)
{
    union change
    {
        float d;
        unsigned char dat[4];
    }r1;
    r1.dat[0] = *s;
    r1.dat[1] = *(s+1);
    r1.dat[2] = *(s+2);
    r1.dat[3] = *(s+3);
    *f=r1.d;
}
void char_to_int(unsigned char *s,int *t)
{
    *t = *(s+1);
    *t |= *s <<8;
}

#include "stm32.h"
using namespace std;

static const double G_TO_MPSS = 9.80665;
unsigned char Commend_Buff[41]={0};//初始化数据包
unsigned char Receive_Buff[41]={0};//初始化数据包
int Receive_Buff_Valid_Pass=0;


STM32::STM32(QObject *parent):
   QObject(parent)
{
    Cor_count=0;
    //count=0;
    Uart_FD=0;//COM1
    Updata_timer = new QTimer(0);
    print_timer=new QTimer(0);
    Stm32_commend = 0;

    connect(Updata_timer,SIGNAL(timeout()),this,SLOT(updata_data()));

}
STM32::~STM32(void) {}
void STM32::update_start()
{
     if(Tx1UartInit_com2(Uart_FD)==0)
    {
        Check_Sign = 1;
    }
//    if((Check_Sign==0)&&(Check_imu==0))
     if(Check_Sign==0)
    {
        Q_EMIT this->check_successful();
        Updata_timer->start(100);
    }
    else
    {
        Q_EMIT this->check_faild();
    }
}

void STM32::update_stop()
{
    Updata_timer->stop();
}

void STM32::updata_data()
{
    int nread,nwrite;
    int roll=0;

    //call pwm data
    unsigned char pwm_buff[8]={0};
    Commend_Buff_Setting(STM32toTX1_PWM_DATA,&pwm_buff[0],0);
    nwrite= write(Uart_FD,Commend_Buff,41);
    while(roll<2000)
    {
       roll++;
    }
    nread=read(Uart_FD,Receive_Buff,41);
    roll=0;
    while(roll<2000)
    {
       roll++;
    }
    Receive_Buff_Valid();
    if(Receive_Buff_Valid_Pass)
    {
      for(int i=3;i<11;i=i+2)
       {
           int int_buff=0;
           char_to_int(&Receive_Buff[i],&int_buff);
           Motor_PWM_Show[(i-3)/2]=int_buff;
       }
    }

     //call imu data
     unsigned char imu_buff[36]={0};
     Commend_Buff_Setting(STM32toTX1_IMU_DATA,&imu_buff[0],0);
     nwrite= write(Uart_FD,Commend_Buff,41);
     roll=0;   //delay
     while(roll<2000)
     {
         roll++;
     }
     nread=read(Uart_FD,Receive_Buff,41);

     Receive_Buff_Valid();
     if(Receive_Buff_Valid_Pass)
     {
         float IMU_buff[3]={0.0};
         for(int i=0;i<12;i=i+4)// 加速度
         {
             float float_buff=0.0;
             char_to_float(&Receive_Buff[i+3],&float_buff);
             IMU_buff[i/4]=float_buff;
         }
         IMU_L_Acc[0]=IMU_buff[0]; IMU_L_Acc_c[0]=IMU_L_Acc[0]-IMU_L_Acc_Offset[0];
         IMU_L_Acc[1]=IMU_buff[1]; IMU_L_Acc_c[1]=IMU_L_Acc[1]-IMU_L_Acc_Offset[1];
         IMU_L_Acc[2]=IMU_buff[2]; IMU_L_Acc_c[2]=IMU_L_Acc[2]-IMU_L_Acc_Offset[2];

         IMU_buff[3]={0.0};
         for(int i=12;i<24;i=i+4)//角速度
         {
             float float_buff;
             char_to_float(&Receive_Buff[i+3],&float_buff);
             IMU_buff[(i-12)/4]=float_buff;
         }
         IMU_A_Vel[0]=IMU_buff[0]; IMU_A_Vel_c[0]=IMU_A_Vel[0]-IMU_A_Vel_Offset[0];
         IMU_A_Vel[1]=-IMU_buff[1]; IMU_A_Vel_c[1]=IMU_A_Vel[1]-IMU_A_Vel_Offset[1];
         IMU_A_Vel[2]=-IMU_buff[2]; IMU_A_Vel_c[2]=IMU_A_Vel[2]-IMU_A_Vel_Offset[2];

         IMU_buff[3]={0.0};
         for(int i=24;i<36;i=i+4)// 角度
         {
             float float_buff;
             char_to_float(&Receive_Buff[i+3],&float_buff);
             IMU_buff[(i-24)/4]=float_buff;
         }
         IMU_A_Ang[0]=IMU_buff[2]; IMU_A_Ang_c[0]=IMU_A_Ang[0]-IMU_A_Ang_Offset[0];
         IMU_A_Ang[1]=IMU_buff[1]; IMU_A_Ang_c[1]=IMU_A_Ang[1]-IMU_A_Ang_Offset[1];
         IMU_A_Ang[2]=IMU_buff[0]; IMU_A_Ang_c[2]=IMU_A_Ang[2]-IMU_A_Ang_Offset[2];

     }

     //call ultrasonicwave
     unsigned char ultrasoic_buff[4]={0};
     Commend_Buff_Setting(STM32toTX1_ULTRASONIC_DATA,&ultrasoic_buff[0],0);
     nwrite= write(Uart_FD,Commend_Buff,41);
     roll=0;   //delay
     while(roll<2000)
     {
         roll++;
     }
     nread=read(Uart_FD,Receive_Buff,41);
     Receive_Buff_Valid();
     if(Receive_Buff_Valid_Pass)
     {
         float Ultrasonic_buff1,Ultrasonic_buff2;
         char_to_float(&Receive_Buff[3],&Ultrasonic_buff1);
         ULTRASONIC_WAVE[0]=Ultrasonic_buff1;

         char_to_float(&Receive_Buff[7],&Ultrasonic_buff2);
         ULTRASONIC_WAVE[1]=Ultrasonic_buff2;
     }


     Q_EMIT this->display_update();
}
void STM32::change_pwm()
{
    int nwrite,nread;
    unsigned char commend = TX1toSTM32_PWM_DATA;
    unsigned char pwm_buff[8]={0};

    for(int i=0;i<4;i++)
    {
        int_to_char(Motor_PWM_Control[i],&pwm_buff[2*i]);
    }

    Commend_Buff_Setting(commend,&pwm_buff[0],8);
    nwrite= write(Uart_FD,Commend_Buff,41);
}

void STM32::change_Stablilize_Yaw_PID()
{
    int nwrite;
    unsigned char commend = TX1toSTM32_Stabilize_Yaw_PID_KpKiKd_DATA;
    unsigned char PID_buff[12]={0};
    float_to_char(Stablilize_Yaw_PID_Input_Buff[0],&PID_buff[0]);
    float_to_char(Stablilize_Yaw_PID_Input_Buff[1],&PID_buff[4]);
    float_to_char(Stablilize_Yaw_PID_Input_Buff[2],&PID_buff[8]);
    Commend_Buff_Setting(commend,&PID_buff[0],12);
    nwrite= write(Uart_FD,Commend_Buff,41);
}

void STM32::change_Stablilize_Roll_PID()
{
    int nwrite;
    unsigned char commend = TX1toSTM32_Stabilize_Roll_PID_KpKiKd_DATA;
    unsigned char PID_buff[12]={0};
    float_to_char(Stablilize_Roll_PID_Input_Buff[0],&PID_buff[0]);
    float_to_char(Stablilize_Roll_PID_Input_Buff[1],&PID_buff[4]);
    float_to_char(Stablilize_Roll_PID_Input_Buff[2],&PID_buff[8]);
    Commend_Buff_Setting(commend,&PID_buff[0],12);
    nwrite= write(Uart_FD,Commend_Buff,41);
}

void STM32::change_Stablilize_Pitch_PID()
{
    int nwrite;
    unsigned char commend = TX1toSTM32_Stabilize_Pitch_PID_KpKiKd_DATA;
    unsigned char PID_buff[12]={0};
    float_to_char(Stablilize_Pitch_PID_Input_Buff[0],&PID_buff[0]);
    float_to_char(Stablilize_Pitch_PID_Input_Buff[1],&PID_buff[4]);
    float_to_char(Stablilize_Pitch_PID_Input_Buff[2],&PID_buff[8]);
    Commend_Buff_Setting(commend,&PID_buff[0],12);
    nwrite= write(Uart_FD,Commend_Buff,41);
}


void STM32::change_Target_Position()
{
   int nwrite;
   u_int8_t commend = TX1toSTM32_Taeget_Positioin_DATA;
   unsigned char Position_buff[12]={0};
   float_to_char(Target_Position_Buff[0],&Position_buff[0]);//
   float_to_char(Target_Position_Buff[1],&Position_buff[4]);//
   float_to_char(Target_Position_Buff[2],&Position_buff[8]);//

   Commend_Buff_Setting(commend,&Position_buff[0],12);
   nwrite= write(Uart_FD,Commend_Buff,41);
}

////////////////////////////////////////////////////////////////////////
void STM32::change_Stablilize_YawRate_PID()
{
    int nwrite;
    unsigned char commend = TX1toSTM32_Stabilize_YawRate_PID_KpKiKd_DATA;
    unsigned char PID_buff[12]={0};
    float_to_char(Stablilize_YawRate_PID_Input_Buff[0],&PID_buff[0]);
    float_to_char(Stablilize_YawRate_PID_Input_Buff[1],&PID_buff[4]);
    float_to_char(Stablilize_YawRate_PID_Input_Buff[2],&PID_buff[8]);
    Commend_Buff_Setting(commend,&PID_buff[0],12);
    nwrite= write(Uart_FD,Commend_Buff,41);
}

void STM32::change_Stablilize_RollRate_PID()
{
    int nwrite;
    unsigned char commend = TX1toSTM32_Stabilize_RollRate_PID_KpKiKd_DATA;
    unsigned char PID_buff[12]={0};
    float_to_char(Stablilize_RollRate_PID_Input_Buff[0],&PID_buff[0]);
    float_to_char(Stablilize_RollRate_PID_Input_Buff[1],&PID_buff[4]);
    float_to_char(Stablilize_RollRate_PID_Input_Buff[2],&PID_buff[8]);
    Commend_Buff_Setting(commend,&PID_buff[0],12);
    nwrite= write(Uart_FD,Commend_Buff,41);
}

void STM32::change_Stablilize_PitchRate_PID()
{
    int nwrite;
    unsigned char commend = TX1toSTM32_Stabilize_PitchRate_PID_KpKiKd_DATA;
    unsigned char PID_buff[12]={0};
    float_to_char(Stablilize_PitchRate_PID_Input_Buff[0],&PID_buff[0]);
    float_to_char(Stablilize_PitchRate_PID_Input_Buff[1],&PID_buff[4]);
    float_to_char(Stablilize_PitchRate_PID_Input_Buff[2],&PID_buff[8]);
    Commend_Buff_Setting(commend,&PID_buff[0],12);
    nwrite= write(Uart_FD,Commend_Buff,41);
}


////////////////////////////////////////////////////////////////////





void STM32::Commend_Buff_Setting(unsigned char Send_Commend,unsigned char *Data,int num)
{
   Commend_Buff[0]=Data_Send_Start ;
   Commend_Buff[40]=Data_Send_Stop ;
   Commend_Buff[1]=Send_Commend ;
   int_to_char(num,&Commend_Buff[2]);
   unsigned char Data_Valid=NO_Data;
   if(num>0)
   {
       for(int i=0;i<num;i++)
       {
            Commend_Buff[i+3]=*(Data+i);
       }
   }
    Commend_Buff[39]=Data_Valid;
}

void STM32::Receive_Buff_Valid(void)
{
    Receive_Buff_Valid_Pass = 0;
    if((Receive_Buff[0]==Data_Receive_Start)&&(Receive_Buff[40]==Data_Receive_Stop))
    {
         Receive_Buff_Valid_Pass= 1;
    }
    else
    {
       Receive_Buff_Valid_Pass= 0;
    }
}

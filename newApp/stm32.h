#ifndef STM32_H
#define STM32_H

#include <QObject>
#include <QTimer>
#include "uart.h"
#include "dataLib.h"
#include "commandLib.h"
#include <iostream>
#include <string.h>

using namespace std;

class STM32 :public  QObject
{
    Q_OBJECT
public :
    explicit STM32 (QObject * parent = 0);
    ~STM32(void);

    QTimer * Updata_timer;
    QTimer * print_timer;
    unsigned char Stm32_commend;

private:
    int Cor_count;
    long long int count;
    int Uart_FD;
    int Uart_FC;
    void stm32_command_process(const unsigned char Stm32_commend);
Q_SIGNALS:
    void check_successful();
    void check_faild();
    void display_update();
    void filter_start();
public Q_SLOTS:
    void update_start(void);
    void update_stop(void);
    void updata_data(void);
    void change_pwm(void);
    void change_Stablilize_Yaw_PID();
    void change_Stablilize_Roll_PID();
    void change_Stablilize_Pitch_PID();
    void change_Stablilize_YawRate_PID();
    void change_Stablilize_RollRate_PID();
    void change_Stablilize_PitchRate_PID();
    void change_Target_Position();
    void Commend_Buff_Setting(unsigned char Send_Commend,unsigned char *s,int num);
    void Receive_Buff_Valid(void);
};


#endif // STM32_H

#ifndef COMMANDLIB_H
#define COMMANDLIB_H

#define STM32_CHECK                                         0x80
//call for data class
#define STM32toTX1_IMU_DATA                                 0x91
#define STM32toTX1_PWM_DATA                                 0x92
#define STM32toTX1_ULTRASONIC_DATA                          0x93


//send data class
#define TX1toSTM32_PWM_DATA                                 0xB0
#define TX1toSTM32_Stabilize_Yaw_PID_KpKiKd_DATA            0xB2
#define TX1toSTM32_Stabilize_Pitch_PID_KpKiKd_DATA          0xB3
#define TX1toSTM32_Stabilize_Roll_PID_KpKiKd_DATA           0xB4
#define TX1toSTM32_Taeget_Positioin_DATA                    0xB5

#define TX1toSTM32_Stabilize_YawRate_PID_KpKiKd_DATA        0xB6
#define TX1toSTM32_Stabilize_PitchRate_PID_KpKiKd_DATA      0xB7
#define TX1toSTM32_Stabilize_RollRate_PID_KpKiKd_DATA       0xB1




#define NO_Data                                              0xFF
#define Data_Send_Start                                      0xFE
#define Data_Receive_Start                                   0xFE
#define Data_Send_Stop                                       0xFD
#define Data_Receive_Stop                                    0xFD
#define INVALID_COMMAND                                      0xF0


#endif // COMMANDLIB_H

#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct
{
    float target_val;   //目标值
    float err;          //偏差值
    float err_last;     //上一个偏差值
    float Kp,Ki,Kd;     //比例、积分、微分系数
    float integral;     //积分值
    float output_val;   //输出值
    float last_error;
}PID;

extern PID vel_pid, ver_pid, turn_pid;

extern int16_t volacity_out, vertical_out, turn_out, Al_out, Motor_1, Motor_2, Can_out[4];

extern float pitch, roll, yaw;
extern float meg_angle;


extern int16_t target_speed, ture_speed, target_turn, gyro_x, gyro_y, gyro_z;
extern int8_t stop;

void pid_Init(PID * tag_pid, float  kp, float ki, float kd, float integral);

int16_t vertical(float med, float angle, float gyro_Y);

int16_t volacity(int16_t target, int16_t tspeed);

int16_t turn(float gyro_z, int16_t target_turn);

void contral(int16_t * volacity_out, int16_t * vertical_out, int16_t * turn_out);

#endif
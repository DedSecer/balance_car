#include "pid.h"
#include "main.h"

float meg_angle = 4.0;

// 初始化
void pid_Init(PID * tag_pid, float  kp, float ki, float kd, float integral)
{
    tag_pid->Kp = kp;
    tag_pid->Ki = ki;
    tag_pid->Kd = kd;
    tag_pid->integral = integral;
}

// 直立环
// 输入：期望角度，真实角度，角速度；
int16_t vertical(float med, float angle, float gyro_Y)
{
    int16_t temp = ver_pid.Kp * (angle - med) + ver_pid.Kd * gyro_Y;
    return temp;
}

// 速度环
// 输入：期望速度，真实速度；
int16_t volacity(int16_t target, int16_t tspeed)
{
    static float a = 0.6;
    vel_pid.err = tspeed-target;

    //低通滤波
    vel_pid.err = vel_pid.err*a + vel_pid.last_error*(0.7 - a);
    vel_pid.integral += vel_pid.err;

    vel_pid.integral = vel_pid.integral>2000?2000:(vel_pid.integral<-2000?(-2000):vel_pid.integral);

    if(stop == 1)
    {
        vel_pid.integral = 0;
    }

    int16_t temp = vel_pid.Kp * vel_pid.err + vel_pid.Ki * vel_pid.integral;
    vel_pid.last_error = vel_pid.err;
    return temp;
}

// 转向环
// 输入：角度，角速度；
int16_t turn(float gyro_z, int16_t target_turn)
{
    int16_t temp;
    temp = turn_pid.Kd * gyro_z + turn_pid.Kp * target_turn;
    return temp;
}

void contral(int16_t * volacity_out, int16_t * vertical_out, int16_t * turn_out)
{
    *volacity_out = volacity(target_speed, ture_speed);
    *vertical_out = vertical(*volacity_out + meg_angle, pitch, gyro_y);
    *turn_out = turn(gyro_z,target_turn);
}
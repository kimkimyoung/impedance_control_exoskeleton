/*
 * @Description: Impedance Controller
 * @Author: Yangjian
 * @Date: 2019-02-17 10:13:36
 * @LastEditTime: 2019-08-28 15:24:28
 * @LastEditors: Please set LastEditors
 */
#include "Impedance_Control.h"
#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS2:Keil RTX5
#include <stdio.h>
#include "..\interface\interface.h"
#include "math.h"

/*需要调参*/
#define KP 450.0f // PD controller parameters
#define KD 35.0f // PD controller parameters

#define MASS_KNEE 2.811f //kg
#define COM_KNEE_Y 0.02982 //m
#define MOI_KNEE 0.01348f //kg*m^2
#define GRAV_VAR 9.794f // 重力加速度
#define MD 1
#define K 635 // 机械腿弹簧系数
#define B 0.000317f // motor parameter

#define PI 3.14159f
#define DELTA_T 0.01f//s
#define ALFA 0.2f // filter parameter

#define GAMA 15.0f 
#define LAMBDA 19.2f // Kd = GAMA * LAMBDA

#define KZ 35.0f
#define KS 52.0f // parameters of terms of position control
#define K_THETA 0.15f // parameters of terms of velocity control
#define A_THETA 17.02f

static float T;
static float prev_q;
static float prev_q_dot;
static float prev_tau_l;
static float prev_qr_dot;
static float prev_qr_dot_dot;
static float prev_theta;
static float prev_theta_dot;
static float prev_theta_d;
static float prev_theta_d_dot;
static float prev_tau_l_dot;
static float prev_qd;

static int printf_cnt = 0;

float PD_main(knee_enc_abs_SEA_output, knee_enc_abs_SEA_input)
{
    // 变量声明
    float q;
    float theta;
    float qd; 
    float qd_dot;
    float q_dot;
    float delta_q_dot;
    float delta_q;
    float tau_sea;
    float tau_l;
    float z; // impedance vector
    float qr_dot; // reference vector
    float qr_dot_dot;
    float theta_d;
    float delta_theta;
    float s; // sliding vector
    float theta_r_dot;
    float theta_dot;
    float theta_d_dot;
    float tau;
    float G;
    float inertia;
    float moi;
    float current;
    float pwm;
    float q_err;
    float theta_err;
	float qd_dot_dot;
	
    // encoder2angle，编码值零点位置需要注意不要发生漂移(！以q的值为标准，q对零后再修改相对的theta值！)
	theta = 0 - (float)((knee_enc_abs_SEA_output - 40468)) / 567.978;
    q = 0 - (float)((knee_enc_abs_SEA_input - 25068)) / 567.978;

    // 选择弧度或者角度进行计算
    q = q * PI / 180;
    theta = theta * PI / 180;

    // 定义期望轨迹或者期望点
    // qd = -0.70;
	// qd_dot = 0;
    qd = -0.2 + 0.2 * sin(PI * T );
    qd_dot = 0.2 * PI * cos(PI * T);
	// qd_dot = (qd - prev_qd) / DELTA_T;
	// qd_dot_dot = -0.35 * PI * sin(PI * T);
	
    // q_dot = ALFA * (q - prev_q) / DELTA_T + (1-ALFA) * prev_q_dot;
    q_dot = (q - prev_q) / DELTA_T;
    q_dot = ALFA * q_dot + (1-ALFA) * prev_q_dot;
    delta_q = q - qd;
    delta_q_dot = q_dot - qd_dot;

    tau_sea = K * (theta - q); // 弹簧输出力矩
    tau_l = (tau_sea * DELTA_T / MD + prev_tau_l) / (1 + DELTA_T * GAMA); //交互力低通滤波
    // tau_l = (tau_sea * DELTA_T / (MD * ALFA) + prev_tau_l + (1-ALFA) * prev_tau_l_dot * DELTA_T) / (1+ DELTA_T * MATRIX_1);
    
    // 重力项
    G = - ( MASS_KNEE * GRAV_VAR * COM_KNEE_Y * sin(q) );
    inertia = MASS_KNEE * COM_KNEE_Y + MOI_KNEE;

    #if(0)
    /*------------------------------------PD Controller--------------------------------------------*/
    //  tau =  KP * (q_desired - q) + KD * q_dot ;// u = - Kp*(qd-q) - Kd*q_dot 
    tau = qd_dot_dot + KD * (qd_dot - q_dot) + KP * (qd - q);// u = qd_dot_dot + Kd * e_dot + Kp * e
    current = tau / 4.628; //ratio
	pwm = (float)(500 + current * 400 / 14 ); //百分之10 ~ 百分之90  ---> -14A ~ 14A

    #else
    /*---------------------------------IMPEDANCE Controller-------------------------------------------*/
    // impedance vector z
    qr_dot = qd_dot - LAMBDA * delta_q + tau_l;
    // qr_dot_dot = ALFA * (qr_dot - prev_qr_dot) / DELTA_T + (1-ALFA) * prev_qr_dot_dot;
    qr_dot_dot = (qr_dot - prev_qr_dot) / DELTA_T;
    z = q_dot - qr_dot;

    moi = inertia * qr_dot_dot;

    // desired input
    theta_d = q + (-KZ * z - tau_sea) / K;
    // theta_d = q + (-KZ * z - tau_sea + G + moi ) / K;
    
    delta_theta = theta - theta_d;
    // theta_dot = ALFA * (theta - prev_theta) / DELTA_T + (1-ALFA) * prev_theta_dot;
    // theta_d_dot = ALFA * (theta_d - prev_theta_d) / DELTA_T + (1-ALFA) * prev_theta_d_dot;
    theta_dot = (theta - prev_theta) / DELTA_T ;
    theta_d_dot = (theta_d - prev_theta_d) / DELTA_T;

    // sliding vector
    theta_r_dot = theta_d_dot - A_THETA * delta_theta;
    s = theta_dot - theta_r_dot;

    tau = tau_sea - KS * s - K_THETA * delta_theta;
    current = tau / 4.628; //ratio
	pwm = (float)(500 + current * 400 / 14 ); //百分之10 ~ 百分之90  ---> -14A ~ 14A

    #endif

    // error
	q_err = q - qd;
    theta_err = theta - theta_d;

    //  串口打印
    printf_cnt++;
    if(printf_cnt > 0)
    {
        printf_cnt = 0;
        // mprintf("z: %f, theta_err: %f ,theta: %f ,theta_d: %f, q_err: %f, q: %f ,qd: %f, %d %d\n", z, theta_err*180/PI, theta*180/PI, theta_d*180/PI, q_err*180/PI, q*180/PI, qd*180/PI, knee_enc_abs_SEA_output, knee_enc_abs_SEA_input);  
        mprintf("%f %f %f %f %f %f %f\n", z, theta_err*180/PI, theta*180/PI, theta_d*180/PI, q_err*180/PI, q*180/PI, qd*180/PI);
        // mprintf("%f %f %f %f\n", delta_q_dot, LAMBDA * delta_q , tau_l, delta_q );
        // mprintf("%f %f %f \n", theta_err, q_err, z );

    }

    /***************update*****************/
    T = T + DELTA_T;
    prev_q = q;
    prev_q_dot = q_dot;
    prev_tau_l = tau_l;
    prev_qr_dot = qr_dot;
    prev_qr_dot_dot = qr_dot_dot;
    prev_theta = theta;
    prev_theta_dot = theta_dot;
    prev_theta_d = theta_d;
    prev_theta_d_dot = theta_d_dot;
    prev_tau_l_dot = ALFA * (tau_l - prev_tau_l) / DELTA_T + (1-ALFA) * prev_tau_l_dot;
	prev_qd = qd;
	
    return pwm;

}
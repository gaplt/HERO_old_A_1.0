#ifndef CHASSIC_TASK_H
#define CHASSIC_TASK_H

#include "remote_control.h"
#include "CAN_receive.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "main.h"
#include "gimbal_task.h"
#include "arm_math.h"
#include "user_lib.h"

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//摇杆死区
#define CHASSIS_RC_DEADLINE 30

#define CHASSIS_VX_RC_SEN 0.004f
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.006f





#define s_right 0
#define s_left  1

#define chassic_follow_gim_yaw_mode      3
#define chassic_follow_chassic_yaw_mode  1


#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.05f     //姿态解算倍率
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.05f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define M3508_rpm_to_v 0.000415809748903494517209f //m3508转化成底盘速度(m/s)的比例，

//#define M3505_v_P      8000.0f   //10000.0f
//#define M3505_v_I      8.5f     //16.5f
//#define M3505_v_D      6.0f   //1200.0f
//#define M3508_out_max  16000.0f
//#define M3508_iout_max 2000.0f

#define M3505_v_P 10000.0f
#define M3505_v_I 10.0f
#define M3505_v_D 0.0f
#define M3508_out_max  16000.0f
#define M3508_iout_max 2000.0f
 
#define M3505_angle_P  8.0f  //10.0f   //底盘角度环   8
#define M3505_angle_I  0.0f
#define M3505_angle_D  0.2f  //30.0f
#define M3508_angle_out_max  50.50f
#define M3508_angle_iout_max 100.2f

#define M3505_speed_P 8.0f    //底盘速度环   3
#define M3505_speed_I 0.0f
#define M3505_speed_D 0.2f
#define M3508_speed_out_max  3.0f     //7
#define M3508_speed_iout_max 0.2f

#define spin_speed_P 10.0f     //小陀螺恒速旋转
#define spin_speed_I 0.0f
#define spin_speed_D 0.8f
#define spin_speed_out_max 100.0f
#define spin_speed_iout_max 100.0f

#define spin_wz 6.0f  //目标旋转速度

#define chassis_x_ch  1
#define chassis_y_ch  0
#define chassis_wz_ch 2

#define CHASSIS_WZ_SET_SCALE 0.1f

#define SUPER_CAP KEY_PRESSED_OFFSET_C  //超级电容

//旧小黑x 330       y 390
//新小黑x 400       y 375

#define MOTOR_DISTANCE_TO_CENTER 0.45673f  //底盘旋转速度倍率   （x/2+y/2）*0.001


#define CHASSIS_OPEN_RC_SCALE 10 // in CHASSIS_OPEN mode, multiply the value. 在chassis_open 模型下，遥控器乘以该比例发送到can上

//电机编码值转化成角度值

#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192


//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//小陀螺开启键可以左旋和右旋
#define TOP_LEFT_SPIN KEY_PRESSED_OFFSET_SHIFT

//键盘操作加速比例
#define CHASSISS_ACCELERATION 0.006

//55     /    60   1.3  1.1    /   65           
//70     /    90              /   120
//

#define power_limit_X_5 1.2f    //55
#define power_limit_Y_5 1.0f

#define power_limit_X_6 1.3f    //60
#define power_limit_Y_6 1.1f

#define power_limit_X_7 1.4f    //65
#define power_limit_Y_7 1.2f

#define power_limit_X_8 1.2f    //70
#define power_limit_Y_8 1.0f

#define power_limit_X_10 1.4f   //90
#define power_limit_Y_10 1.2f

#define power_limit_X_12 1.8f   //120
#define power_limit_Y_12 1.6f

#define power_limit_X_CP 2.5f   //unlimt
#define power_limit_Y_CP 2.2f

#define power_limit_X_FP 6.0f   //unlimt
#define power_limit_Y_FP 2.2f

typedef enum
{
	chassic_follow_gim_yaw_fanxiang,
  chassic_follow_gim_yaw,      //底盘跟随云台模式
	chassic_follow_gim_yaw_low,	
	
  chassic_follow_chassic_yaw,  //云台跟随底盘模式    
  
  chassic_zero,                //底盘停止模式
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
	first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
	chassis_mode_e chassis_mode;
	fp32 wheel_speed[4];                     //最后输出给4个电机对应的输出值
	chassis_motor_t chassis_motor[4];        //底盘电机数据
  const RC_ctrl_t *chassis_RC;             //遥控器值
  pid_type_def motor_speed_pid[4];         //四个电机的pid
	pid_type_def chassis_angle_pid;          //底盘跟随角度pid
	pid_type_def chassis_speed_pid;          //底盘跟随角度pid
	pid_type_def chassis_vx_speed_pid;       //底盘x速度环
	pid_type_def chassis_vy_speed_pid;      //底盘y速度环

	pid_type_def chassis_spin_wz;      //小陀螺自旋速度闭环 ，以yaw轴编码器旋转速度为参照
  fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
  fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
	fp32 vx;
	fp32 vy;
	fp32 wz;
	fp32 set_vx;
	fp32 set_vy;
	fp32 set_wz;
	fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
  fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
  fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度
	gimbal_motor_mode_e gimble_mode;
	uint16_t *power_limit;  //机器人底盘功率限制上限
	uint16_t *power_buffer;
	fp32 TOP_SPIN_SPEED;

	first_order_filter_type_t motor_speed_pid_slow[4];

} chassis;
extern chassis chassic_ctrl;//底盘所有参数存值结构体
extern uint16_t power_1  ;
extern void chassic_task(void const * pvParameters);

extern const chassis *get_chassic_date(void);

#endif




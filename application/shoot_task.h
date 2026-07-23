#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "MOTOR_ANGLE_PID.h"
#include "pid.h"
#include "main.h"
#include "remote_control.h"
#include "chassic_task.h"

#define OPEN_SERVO    2200
#define CLOSE_SERVO   645

#define cooling_heat_spend 100    //热量限制值   hero  100  infantry  10

#define Trigger_speed_PID_P 5.2f
#define Trigger_speed_PID_I 0.01f
#define Trigger_speed_PID_D 1.0f 

#define Trigger_angle_PID_P 30.0f
#define Trigger_angle_PID_I 0
#define Trigger_angle_PID_D 0.5f 

#define motor1_PID_P 14.0f         //摩擦轮
#define motor1_PID_I 0.001f
#define motor1_PID_D 1.2f 

#define motor2_PID_P 14.0f
#define motor2_PID_I 0.001f
#define motor2_PID_D 1.2f 

#define motor1_speed  6200  //5800          //摩擦轮转速6000
#define motor2_speed  6200  //5800



#define Trigger_speed_PID_MAX_OUT   6000
#define Trigger_speed_PID_MAX_I_OUT 3000

#define shooting_frequency 600   //拨弹频率设置  80
 
#define KEY_input 650             //按键高电平有效

#define division_angle   60     //一个拨弹盘分度角度   51绿步兵
#define tigger_direction 1     //拨盘旋转方向

#define set_angle_err  division_angle*2        //设置角度和实际角度差值允许值

#define tigger_blocked_count_judgment 10



typedef enum
{
	tigger_playing,       //拨弹正在转
	tigger_over,          //拨弹到达指定设定角度
	tigger_blocked        //拨弹盘堵转
	
}tigger_mode_en;
	
typedef enum
{
		shoot_down,           //停止拨弹
    shoot_start

}shoot_mode_en;

typedef struct
{
	uint8_t       Init_count;
	uint16_t      key_run_num;      //按键消抖计数值
	uint16_t      KEY_outcome;      //按键输出实际值
	float         set_angle  ;			//目标角度
	float         set_angle_old;    //上次设置的角度
	_PID          MOTOR_ANGLE;      
  _PID          MOTOR_SPEED;     
	pid_type_def  MOTOR_ANGLE1;     //拨盘电机角度环
	pid_type_def  MOTOR_SPEED1;     //拨盘电机速度环
	pid_type_def  motor1;           //摩擦轮速度环
	pid_type_def  motor2;
	CLASSICPID    MOTOR_ANGLE2;     //带死区pid
	shoot_mode_en shoot_mode;       //射击模式
	tigger_mode_en tigger_mode;      //拨弹模式
	const motor_measure_t *motor_data1;	//拨弹轮声明电机结构体指针
	const motor_measure_t *motor1_data;	//摩擦轮声明电机结构体指针
	const motor_measure_t *motor2_data;	//摩擦轮声明电机结构体指针
	int xLastWakeTime;
	uint32_t tigger_blocked_count;      //堵转判断计数值
	uint32_t last_count;                //堵转上次计数值
	const RC_ctrl_t *shoot_RC;             //遥控器值     
	float LAST_ANGLE;                   //上次角度存值
  int KEY;
	uint16_t *shoot_cooling_rate;        //射击冷却速度
	uint16_t *shoot_cooling_limit;       //射击冷却上限
	uint16_t *shoot_speed_limit;         //射速上限
	uint16_t *cooling_heat;              //实时枪口热量
	
}shoot_all;

extern  uint32_t xTaskGetTickCount(void);
extern void shoot_task(void const * pvParameters);

#endif


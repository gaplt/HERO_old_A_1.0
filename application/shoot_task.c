#include "shoot_task.h"
#include "FreeRTOS.h"
#include "MOTOR_ANGLE_PID.h"
#include "CAN_receive.h"
#include "referee.h"
#include "tim.h"
#include "robomaster_vcan.h"

shoot_all shoot;

/**
  * @brief         状态机模式设置          
  * @param[in]     null
	* @author        刘根 
  * @retval        null 
  */
void shoot_mode_set(void);

/**
  * @brief         状态机行为执行                
  * @param[in]     null 
	* @author        刘根 
  * @retval        null 
  */
void shoot_mode_behavior(void);

/**
  * @brief         按键消抖函数          
  * @param[in]     GPIOx
  * @param[in]     GPIO_Pin：对应引脚	
	* @author        刘根 
  * @retval        消抖后值，1为按键按下，0为没有按下 
  */
int key_detect(shoot_all *key_det);


/**
  * @brief         shoot任务初始化函数          
  * @param[in]     null 
	* @author        刘根 
  * @retval        null 
  */
void shoot_init(void);


/**
  * @brief         shoot任务调度函数          
  * @param[in]     pvParameters 
	* @author        刘根 
  * @retval        null 
  */
void shoot_task(void const * pvParameters)
{
	
	shoot_init();
	
	shoot.xLastWakeTime = xTaskGetTickCount();

  for(;;)
 {
	  shoot_mode_set();
	 
	  shoot_mode_behavior();
	  
    

    osDelay(1);
		
  }

}

void shoot_init()
{
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);

	  shoot.shoot_speed_limit=&ext_game_robot_status.shooter_id1_42mm_speed_limit;         //发射速度上限
	  shoot.shoot_cooling_rate=&ext_game_robot_status.shooter_id1_42mm_cooling_rate;    //枪口每秒冷却值
	  shoot.shoot_cooling_limit=&ext_game_robot_status.shooter_id1_42mm_cooling_limit;   //枪口热量上限
	  shoot.cooling_heat=&ext_power_heat_data.shooter_id1_42mm_cooling_heat;       //枪口热量
	
	  shoot.motor_data1=get_trigger2_motor_measure_point();       //拨弹电机
	  shoot.motor1_data=get_shoot_motor_measure_point(7);         //摩擦轮1
	  shoot.motor2_data=get_shoot_motor_measure_point(8);         //摩擦轮2
	
	  shoot.shoot_RC=get_remote_control_point(); //获取遥控器指针
	  const fp32 PID_angle[3]={Trigger_angle_PID_P,Trigger_angle_PID_I,Trigger_angle_PID_D};	//P,I,D    角度环
    const fp32 PID_speed[3]={Trigger_speed_PID_P,Trigger_speed_PID_I,Trigger_speed_PID_D};	//P,I,D    速度环
		
		const fp32 PID_motor1[3]={motor1_PID_P,motor1_PID_I,motor1_PID_D};	//P,I,D  
		const fp32 PID_motor2[3]={motor2_PID_P,motor2_PID_I,motor2_PID_D};	//P,I,D
		
		PID_init(&shoot.MOTOR_ANGLE1,PID_POSITION,PID_angle,16000,11000);
		PID_init(&shoot.MOTOR_SPEED1,PID_POSITION,PID_speed,16000,11000);
		
		PID_init(&shoot.motor1,PID_POSITION,PID_motor1,16000,11000);
		PID_init(&shoot.motor2,PID_POSITION,PID_motor2,16000,11000);
		
		shoot.set_angle = 0;
		shoot.KEY_outcome=0;
		shoot.key_run_num=0;
		shoot.Init_count =0;
    
		shoot.tigger_mode = tigger_over ;
    shoot.shoot_mode = shoot_down;           //停止拨弹

}	


/**
  * @brief         按键消抖函数          
  * @param[in]     GPIOx
  * @param[in]     GPIO_Pin：对应引脚	
	* @author        刘根 
  * @retval        消抖后值，1为按键按下，0为没有按下 
  */
int key_detect(shoot_all *key_det)
{	
	if( key_det->shoot_RC->rc.ch[4] >= KEY_input || key_det->shoot_RC->mouse.press_l == 1)		
	{
		
		shoot.key_run_num ++;	
		
			if( shoot.key_run_num >= shooting_frequency)
		{

			shoot.key_run_num = 0;
			
			return 1;
			
		}
	
	} 
	else
	{
		
		shoot.key_run_num=0;
		
		return 0;

	}
	return 0;
			
}


void shoot_mode_set(void)
{ 
  shoot.KEY_outcome = key_detect(&shoot);	
	if(shoot.shoot_RC->rc.s[s_left]==1 || shoot.shoot_RC->rc.s[s_left]==3)    //如果左拨杆到中间则打开摩擦轮和拨弹盘
	{
	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
		shoot.shoot_mode =shoot_start;
	
	}
	if(shoot.shoot_RC->rc.s[s_left]== 2|| (shoot.shoot_RC->rc.ch[4]<660) ||(*shoot.cooling_heat>=(*shoot.shoot_cooling_limit-cooling_heat_spend)) )   //当拨杆为最下或者按键没有按下时，停止拨弹
	{
		
	  shoot.shoot_mode =shoot_down;
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 2000);
		
	}

	//如果设置角度大于实际角度一定时间，且按键和滚轮没有触发，则为堵转
	if(fabs(shoot.set_angle-ABS_ANGLE)>(division_angle*3) && (shoot.shoot_RC->rc.ch[4]<660|| shoot.shoot_RC->mouse.press_l == 0) )  
	{
		
		shoot.tigger_blocked_count++;
		
	}
	
	if(shoot.tigger_blocked_count>tigger_blocked_count_judgment)  //当大于角度大于一定时间，则判断为堵转
	{
		
		shoot.tigger_mode = tigger_blocked;
	
	}
	else 
	{
		shoot.tigger_mode = tigger_playing;
	
	}

}

int iii;
void shoot_mode_behavior(void)
{

  if(shoot.shoot_mode == shoot_down)
	{
		shoot.set_angle = ABS_ANGLE;

		shoot.MOTOR_SPEED1.out = 0;		
	}
	else if(shoot.shoot_mode == shoot_start)
	{
		
		iii++;
		
		//在堵转之后，由于堵转判断时间过短，导致拨杆继续拨动后会继续增加拨盘角度目标值
    if(shoot.tigger_mode == tigger_blocked)
		{

			shoot.set_angle = ABS_ANGLE-10;
			iii=0;
//			 osDelay(500);
	//	  shoot.set_angle = ((int)(ABS_ANGLE/division_angle))*division_angle;      //如果判断为堵转则向相反方向旋转一个分度值
	//	  shoot.set_angle = ((int)(ABS_ANGLE/division_angle))*division_angle - tigger_direction*10;      //如果判断为堵转则向相反方向旋转一个分度值
//			if(shoot.tigger_blocked_count>=tigger_blocked_count_judgment*2)
//			{
//			if(shoot.tigger_blocked_count>1000)
//			{

			  shoot.tigger_blocked_count=0;
//			}
	  	shoot.shoot_mode = shoot_down;

//			}
			
		}		
		else if(shoot.KEY_outcome==1&&shoot.tigger_mode == tigger_playing)
		{
			
			shoot.set_angle=shoot.set_angle + tigger_direction*division_angle;       //每次拨盘旋转的角度、方向
		
		}
//		else 
//    {
//			
//			shoot.set_angle=ABS_ANGLE;
//		
//		}
		
		PID_calc(&shoot.MOTOR_ANGLE1,  ABS_ANGLE, shoot.set_angle);//角度环
		
		PID_calc(&shoot.MOTOR_SPEED1, shoot.motor_data1->speed_rpm, shoot.MOTOR_ANGLE1.out);//速度环

  
	
	}
	  if(shoot.shoot_RC->rc.s[s_left]==1 || shoot.shoot_RC->rc.s[s_left]==3)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,CLOSE_SERVO);  
			 PID_calc(&shoot.motor1, shoot.motor1_data->speed_rpm, -motor1_speed);//速度环
	     PID_calc(&shoot.motor2, shoot.motor2_data->speed_rpm, motor2_speed);//速度环	
		}
		else 
		{
	     shoot.motor1.out = 0 ;
	     shoot.motor2.out = 0 ;
		   shoot.MOTOR_SPEED1.out = 0;
		}
		wave_form_data [0] = shoot.motor1_data->speed_rpm ;
		wave_form_data [1] = shoot.motor2_data->speed_rpm ;
    shanwai_send_wave_form();
    CAN_cmd_shoot(0,shoot.MOTOR_SPEED1.out , shoot.motor1.out , shoot.motor2.out);	//发送控制电流
//		    CAN_cmd_shoot(0,shoot.MOTOR_SPEED1.out ,0 , 0);	//发送控制电流
}


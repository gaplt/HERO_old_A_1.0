
#include "chassic_task.h"
#include "bsp_visual.h"
#include "referee.h"
#include "remote_control.h"
#include "decoder.h"
#include "protocol.h"
#include "rng.h"


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


chassis chassic_ctrl;//底盘所有参数存值结构体
int8_t action_chassic = 1  ;
uint16_t power_1 = 0 ;

		
				
static void chassis_init(chassis *chassis_init);//初始化函数
static void chassis_ctrl_loop(chassis *chassic_ctrl_loop);//底盘电机输出循环
static void chassis_feedback_update(chassis *chassis_date_update);//底盘姿态更新
static void chassic_follow_gimbal(chassis *chassis_rc_to_vector);//底盘跟随云台角速度计算
static void chassis_vector_to_mecanum_wheel_speed(chassis *chassis_wheel_speed);
static void chassic_mode_set(chassis *mode);//状态机模式切换
static void chassic_mode_ctrl(chassis *mode_ctrl);//状态机模式对应执行

void chassic_task(void const * pvParameters)
{
	
	chassis_init(&chassic_ctrl); //初始化
	
	chassic_ctrl.chassis_RC = (RC_ctrl_t*)&hero_protocol.rm_control;
	for(;;)
	{
		chassic_ctrl.power_limit = &ext_game_robot_status.chassis_power_limit ;
		chassic_ctrl.power_buffer = &ext_power_heat_data.chassis_power_buffer ;
		if( (unsigned short)*chassic_ctrl.power_buffer >= 10.0f)
		{
		   power_1 = 10;
		}
		else 
		{
	     power_1 = 0 ;
		}
	  CAN_cmd_supercap((((unsigned short)*chassic_ctrl.power_limit)+power_1)*100);
//  CAN_cmd_supercap(((unsigned short)*chassic_ctrl.power_limit )*100,((unsigned short)* chassic_ctrl.power_buffer) * 100);
		chassis_feedback_update(&chassic_ctrl);  //底盘状态数据计算以及更新
	  chassic_mode_set(&chassic_ctrl);//状态机模式切换
	  chassic_mode_ctrl(&chassic_ctrl);//状态机模式对应执行
		if(chassic_ctrl.chassis_RC->key.v&KEY_PRESSED_OFFSET_Z  )
		{
		 action_chassic = !action_chassic ;
		}
		chassis_vector_to_mecanum_wheel_speed(&chassic_ctrl);  //运动分解
		chassis_ctrl_loop(&chassic_ctrl);        //底盘电机输出循环
    vTaskDelay(2);
		
	}

}

const static fp32 chassis_filter[1] = {0.0001};



const chassis *get_chassic_date(void)
{
	return &chassic_ctrl;
}

static void chassis_init(chassis *chassis_init)
{
	
	  const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};    //速度设置的一阶滤波参数
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	uint8_t i;
	chassis_init->chassis_motor[0].chassis_motor_measure = get_chassis_motor_measure_point(0); //获取底盘四个电机返回值
	chassis_init->chassis_motor[1].chassis_motor_measure = get_chassis_motor_measure_point(1);
	chassis_init->chassis_motor[2].chassis_motor_measure = get_chassis_motor_measure_point(2);
	chassis_init->chassis_motor[3].chassis_motor_measure = get_chassis_motor_measure_point(3);
	chassis_init->chassis_mode = chassic_zero;  //初始化关闭底盘
	 const static fp32 motor_speed_pid[3]   = {M3505_v_P, M3505_v_I, M3505_v_D};
	 const static fp32 chassis_angle_pid1[3] = {M3505_angle_P, M3505_angle_I, M3505_angle_D};
	 const static fp32 chassis_speed_pid1[3] = {M3505_speed_P, M3505_speed_I, M3505_speed_D};
	 
	 const static fp32 chassis_spin_pid1[3] = {spin_speed_P, spin_speed_I, spin_speed_D};    //自旋速度

	 for( i =0;i<4;i++ )//初始化底盘每个电机速度环的pid
	 {
		 
		 PID_init(&chassis_init->motor_speed_pid[i],PID_POSITION,motor_speed_pid,M3508_out_max,M3508_iout_max);
		 first_order_filter_init(&chassis_init->motor_speed_pid_slow[i],CHASSIS_CONTROL_TIME,chassis_filter);	   
	 }
	 PID_init(&chassis_init->chassis_angle_pid , PID_POSITION , chassis_angle_pid1 , M3508_angle_out_max,M3508_angle_iout_max);//
   PID_init(&chassis_init->chassis_speed_pid , PID_POSITION , chassis_speed_pid1 , M3508_speed_out_max,M3508_speed_iout_max);//
	 
	 PID_init(&chassis_init->chassis_spin_wz,PID_POSITION,chassis_spin_pid1,spin_speed_out_max,spin_speed_iout_max);  //自旋恒速pid
	 
	  //一阶滤波初始化
    first_order_filter_init(&chassis_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

}


fp32 speed_date[8][2]={{power_limit_X_5 ,power_limit_Y_5},
                       {power_limit_X_6 ,power_limit_Y_6},
											 {power_limit_X_7 ,power_limit_Y_7},
											 {power_limit_X_8 ,power_limit_Y_8},
											 {power_limit_X_10,power_limit_Y_10},
											 {power_limit_X_12,power_limit_Y_12},
                       {power_limit_X_CP,power_limit_Y_CP},
											 {power_limit_X_FP,power_limit_Y_FP}
                     };

//fp32 PIN_SPEED[7]={0.26,0.31,0.32,0.35,0.42,0.56,0.76};
										 
fp32 PIN_SPEED[8]={1.55,1.6,1.65,1.7,2.0,2.2,4,4};
/*55 60 65   //  70 90 120    */
fp32 spin_cp=4;


int  prower_bank;
int  date_middle;

int xjb_direction = 1;

//根据裁判系统返回的底盘功率上限，调整底盘不开电容的移动最大功率
fp32 *speed_update(void)
{
	if(chassic_ctrl.chassis_RC->key.v & SUPER_CAP )
	{
    date_middle=65535;
		chassic_ctrl.TOP_SPIN_SPEED=spin_cp;
	  
	}
	else 
	{
 		prower_bank = *chassic_ctrl.power_limit;
		date_middle =prower_bank;                    //功率限制，进入电管限幅
	}
	
	 if(chassic_ctrl.chassis_RC->key.v & KEY_PRESSED_OFFSET_E)  //E键飞坡，功率*6
	{
    date_middle=30000;
		chassic_ctrl.TOP_SPIN_SPEED=spin_cp;
	  
	}
	
	    switch(date_middle)   //狗都不用（刘根写的）
			{
				case 55:
				{
					chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[0];
					return speed_date[0];

					
					break;				
				}
				case 60:
				{
					chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[1];
					return speed_date[1];
				
					break;
				
				}
				
				case 65:
				{
					chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[2];
					return speed_date[2];
				
					break;
				
				}
				
				case 70:
				{
					chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[3];
					return speed_date[3];

					break;
				
				}
				case 100:
				{
					chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[4];
					return speed_date[4];

					break;
				
				}
				case 120:
				{
					chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[5];
					return speed_date[5];

					break;
				
				}
				case 65535:
				{
					chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[6];
          return speed_date[6];

					break;
				
				}
        case 30000 :     //飞坡功率  30000限制
				{
				 chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[7];	
          return speed_date[7];
					break;					
				}
				default :
				{
					chassic_ctrl.TOP_SPIN_SPEED=PIN_SPEED[4];
				  return speed_date[4];

				}break;
				
			}

}





//遥控器（处理后）值生成底盘速度
static void chassis_rc_to_control_vector(chassis *chassis_move_rc_to_vector)
{
	
	    //最大 最小速度X_speed_update(void)
    chassis_move_rc_to_vector->vx_max_speed =  speed_update()[0];
    chassis_move_rc_to_vector->vx_min_speed = -speed_update()[0];

    chassis_move_rc_to_vector->vy_max_speed =  speed_update()[1];
    chassis_move_rc_to_vector->vy_min_speed = -speed_update()[1];
	
	
    if (chassis_move_rc_to_vector == NULL )
    {
        return;
    }
    
		static double vector_acceleration_1 = 0.15;
		static double vector_acceleration_2 = 0.15;
		static double vector_acceleration_3 = 0.20;
		static double vector_acceleration_4 = 0.20;
//		static double vector_acceleration_5 = 0.1;
//		static double vector_acceleration_6 = 0.1;

		
		static double vector_acceleration_1_old = 0.6;
		static double vector_acceleration_2_old = 0.6;
		static double vector_acceleration_3_old = 0.6;
		static double vector_acceleration_4_old = 0.6;

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel *  CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    
		 //keyboard set speed set-point
    //键盘控制,更新为键盘矢量控制
		
	if(  !KEY_IS_DOWN(hero_protocol.rm_control,SHIFT))	
	{
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
			if(vector_acceleration_1 >= 1)
			{
				vector_acceleration_1 = 1;
			}
			else if(vector_acceleration_1 < 1)
			{
				vector_acceleration_1 = vector_acceleration_1 + CHASSISS_ACCELERATION;
			}
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed * vector_acceleration_1;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
			if(vector_acceleration_2 >= 1)
			{
				vector_acceleration_2 = 1;
			}
			else if(vector_acceleration_2 < 1)
			{
				vector_acceleration_2 = vector_acceleration_2 + CHASSISS_ACCELERATION;
			}
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed * vector_acceleration_2;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
			if(vector_acceleration_3 >= 1)
			{
				vector_acceleration_3 = 1;
			}
			else if(vector_acceleration_3 < 1)
			{
				vector_acceleration_3 = vector_acceleration_3 + CHASSISS_ACCELERATION;
			}
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed * vector_acceleration_3;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
			if(vector_acceleration_4 >= 1)
			{
				vector_acceleration_4 = 1;
			}
			else if(vector_acceleration_4 < 1)
			{
				vector_acceleration_4 = vector_acceleration_4 + CHASSISS_ACCELERATION;
			}
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed * vector_acceleration_4;
    }
		
//		else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY  &&  chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
//    {
//			if(vector_acceleration_5 >= 1)
//			{
//				vector_acceleration_5 = 1;
//			}
//			else if(vector_acceleration_5 < 1)
//			{
//				vector_acceleration_5 = vector_acceleration_5 + CHASSISS_ACCELERATION;
//			}
//       vx_set_channel = chassis_move_rc_to_vector->vx_max_speed * vector_acceleration_5;
//			vy_set_channel = chassis_move_rc_to_vector->vy_max_speed * vector_acceleration_5;
//    }
		
	}
		
	
		
	 if (chassis_move_rc_to_vector->chassis_RC->key.v & TOP_LEFT_SPIN)
	{
	   if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {

        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed * vector_acceleration_1_old;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {

        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed * vector_acceleration_2_old;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {

        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed * vector_acceleration_3_old;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {

        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed * vector_acceleration_4_old;
    }

	}
		
		
		
    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
				vector_acceleration_1 = 0.3;
				vector_acceleration_2 = 0.3;
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
				vector_acceleration_3 = 0.3;
				vector_acceleration_4 = 0.3;
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    chassis_move_rc_to_vector->set_vx = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    chassis_move_rc_to_vector->set_vy = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

float chassic_yaw_calc(void)//去零点
{
	float set;	
	set = hero_protocol.hero_gimbal_data.absolute_angle_set_communicate;
	float aset = (set +3.14)/3.14*2 * 360;
	float aget = (chassic_ctrl.chassis_yaw+3.14) / 3.14*2 * 360;

	float res = 0.0f;
		if(aset >= aget) {
		if((aset - aget) > 180){	//aset = 350 aget = 10
			res = aset - 360 - aget;
		} else {					//aset = 20 aget = 10
			res = aset - aget;
		}
	} else {
		if((aset - aget) < -180){ 	//aset = 10 aget = 350
			res = 360 + aset - aget;
		} else {					//aset = 10 aget = 20
			res = aset - aget;
		}
	}
	
	return PID_calc(&chassic_ctrl.chassis_angle_pid,aget, aget + res);//角度环
}
	

//底盘跟随云台计算角速度函数
static void chassic_follow_gimbal(chassis *chassis_rc_to_vector)
{
	
	PID_calc(&chassis_rc_to_vector->chassis_angle_pid,-(hero_protocol.hero_gimbal_data.relative_angle_communicate),0);//底盘跟随底盘角度环
	PID_calc(&chassis_rc_to_vector->chassis_speed_pid,-chassis_rc_to_vector->wz,chassis_rc_to_vector->chassis_angle_pid.out);//速度环


	chassis_rc_to_vector->set_wz =  chassis_rc_to_vector->chassis_speed_pid.out*PITCH_DIRECTION;
  
}

static void chassis_vector_to_mecanum_wheel_speed(chassis *chassis_wheel_speed)
{
// 麦轮

	 if(hero_protocol.hero_gimbal_data.action_chassic == 0)
	 {
	  chassis_wheel_speed->wheel_speed[0] = (-chassis_wheel_speed->set_vx - chassis_wheel_speed->set_vy + ( - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_wheel_speed->set_wz)  ;
    chassis_wheel_speed->wheel_speed[1] = ( chassis_wheel_speed->set_vx - chassis_wheel_speed->set_vy + ( - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_wheel_speed->set_wz)  ;
    chassis_wheel_speed->wheel_speed[2] = ( chassis_wheel_speed->set_vx + chassis_wheel_speed->set_vy + ( - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_wheel_speed->set_wz)  ;
    chassis_wheel_speed->wheel_speed[3] = (-chassis_wheel_speed->set_vx + chassis_wheel_speed->set_vy + ( - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_wheel_speed->set_wz)  ;

	 }
	 else if (hero_protocol.hero_gimbal_data.action_chassic ==1 )
	 {
	  chassis_wheel_speed->wheel_speed[0] = (chassis_wheel_speed->set_vx   + chassis_wheel_speed->set_vy + ( - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_wheel_speed->set_wz)  ;
    chassis_wheel_speed->wheel_speed[1] = ( -chassis_wheel_speed->set_vx + chassis_wheel_speed->set_vy + ( - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_wheel_speed->set_wz)  ;
    chassis_wheel_speed->wheel_speed[2] = ( -chassis_wheel_speed->set_vx - chassis_wheel_speed->set_vy + ( - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_wheel_speed->set_wz)  ;
    chassis_wheel_speed->wheel_speed[3] = (chassis_wheel_speed->set_vx - chassis_wheel_speed->set_vy + ( - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_wheel_speed->set_wz)  ;
	 }
}

static void chassis_feedback_update(chassis *chassis_date_update)
{
	if (chassis_date_update == NULL)
    {
        return;
    }
	
	uint8_t i;
	for(i=0;i<4;i++)
	{
		//计算获得的电机速度并存入speed
		chassis_date_update->chassis_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN*chassis_date_update->chassis_motor[i].chassis_motor_measure->speed_rpm;
		

	}
	
	//空转下，由轮子速度计算车世界坐标系下的速度
	chassis_date_update->vx = ( -chassis_date_update->chassis_motor[0].speed + chassis_date_update->chassis_motor[1].speed + chassis_date_update->chassis_motor[2].speed - chassis_date_update->chassis_motor[3].speed)*MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_date_update->vy = ( -chassis_date_update->chassis_motor[0].speed - chassis_date_update->chassis_motor[1].speed + chassis_date_update->chassis_motor[2].speed + chassis_date_update->chassis_motor[3].speed)*MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	chassis_date_update->wz = ( -chassis_date_update->chassis_motor[0].speed - chassis_date_update->chassis_motor[1].speed - chassis_date_update->chassis_motor[2].speed - chassis_date_update->chassis_motor[3].speed)*MOTOR_SPEED_TO_CHASSIS_SPEED_WZ;
	
	//计算车世界坐标系下的yaw轴角度
  chassis_date_update->chassis_yaw =  hero_protocol.hero_gimbal_data.chassis_yaw_communicate;
}
 
static void chassic_mode_set(chassis *mode)
{
	
  if(switch_is_down(mode->chassis_RC->rc.s[s_right]))//右下则全停机
	{
		mode->chassis_mode = chassic_zero;
		gimbal_ctrl.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_ZERO;
	
	}

  else if(switch_is_mid(mode->chassis_RC->rc.s[s_right])||(mode->chassis_RC->key.v&KEY_PRESSED_OFFSET_CTRL))//右中只开云台
	{
	   mode->chassis_mode = chassic_zero;      
		 gimbal_ctrl.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
			
	}
	
	else if(switch_is_up(mode->chassis_RC->rc.s[s_right])) //右上全开
	{		
		 gimbal_ctrl.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;     
//		 mode->chassis_mode = chassic_follow_chassic_yaw;
		 mode->chassis_mode = chassic_zero;    
//		gimbal_ctrl.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_VISUAL; 
		
	}
	 
	if(switch_is_mid(mode->chassis_RC->rc.s[s_left]) || gimbal_ctrl.gimbal_rc->mouse.press_r == 1)   //左中或者鼠标右键则开启视觉模式
	{
		gimbal_ctrl.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_VISUAL; 

	}
	
  if((mode->chassis_RC->key.v& KEY_PRESSED_OFFSET_SHIFT)||switch_is_up(mode->chassis_RC->rc.s[s_left])) //左上或者shift键为小陀螺
	{
		gimbal_ctrl.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_VISUAL; 
	 // mode->chassis_mode = chassic_follow_gim_yaw;
		mode->chassis_mode = chassic_follow_gim_yaw_fanxiang;
//			mode->chassis_mode = chassic_zero;
	}
// if(switch_is_mid(mode->chassis_RC->rc.s[s_right]) && switch_is_down(mode->chassis_RC->rc.s[s_left]) && mode->chassis_RC->rc.ch[4]>=660)
// {
//	  mode->chassis_mode = chassic_follow_gim_yaw_fanxiang;
// }
}

pid_type_def spin_pid;

static void chassic_mode_ctrl(chassis *mode_ctrl)
{
	
	if(mode_ctrl->chassis_mode == chassic_follow_chassic_yaw)   //底盘跟随云台
	{
		
		chassis_rc_to_control_vector(mode_ctrl);  //遥控器（处理后）值生成底盘速度
		
		chassic_follow_gimbal(mode_ctrl); //底盘跟随云台计算角速度函数
		
	}
	else if(mode_ctrl->chassis_mode == chassic_follow_gim_yaw)   //底盘跟随云台
	{
    	 

			const static fp32 wz_pid[3] = {5, 0 , 0.5};  //??ì¨ò??ˉμ×?ì2?±??ù
		  PID_init(&spin_pid,PID_POSITION,wz_pid,100,50);
		  static fp32 delta = 0.0f;
	   	static fp32 angle_delta = 0.0f;
			static fp32 absolute_wz =0.0f;
			float vx,vy,wz;
			chassis_rc_to_control_vector(mode_ctrl);//ò￡???÷￡¨′|àíoó￡??μéú3éμ×?ì?ù?è
  		delta = -(hero_protocol.hero_gimbal_data.relative_angle_communicate) /** PITCH_DIRECTION*/ ;
		    float sin_delta = arm_sin_f32(delta);
				float cos_delta = arm_cos_f32(delta);
				vx = mode_ctrl->set_vx;
				vy = mode_ctrl->set_vy;
				mode_ctrl->set_vx = vx*cos_delta + vy*sin_delta;
        mode_ctrl->set_vy = -vx*sin_delta + vy*cos_delta;
        mode_ctrl->set_wz = 2.0;
	}
	else if(mode_ctrl->chassis_mode == chassic_follow_gim_yaw_fanxiang)   //D?íó?Y
	{
    	 

			const static fp32 wz_pid[3] = {5, 0 , 0.5};  //??ì¨ò??ˉμ×?ì2?±??ù
		  PID_init(&spin_pid,PID_POSITION,wz_pid,100,50);
		  static fp32 delta = 0.0f;
	   	static fp32 angle_delta = 0.0f;
			static fp32 absolute_wz =0.0f;
			float vx,vy,wz;
			chassis_rc_to_control_vector(mode_ctrl);//ò￡???÷￡¨′|àíoó￡??μéú3éμ×?ì?ù?è
  		delta = -(hero_protocol.hero_gimbal_data.chassis_yaw_communicate) /** PITCH_DIRECTION*/ ;
		    float sin_delta = arm_sin_f32(delta);
				float cos_delta = arm_cos_f32(delta);
				vx = mode_ctrl->set_vx;
				vy = mode_ctrl->set_vy;
				mode_ctrl->set_vx = vx*cos_delta + vy*sin_delta;//vx*cos_delta + vy*sin_delta;
        mode_ctrl->set_vy = -vx*sin_delta +  vy*cos_delta;//-vx*sin_delta + vy*cos_delta;
        mode_ctrl->set_wz = -2.5;
	}

	else if(mode_ctrl->chassis_mode == chassic_zero)
	{
		mode_ctrl->set_vx = 0;
	  mode_ctrl->set_vy = 0;
	  mode_ctrl->set_wz = 0;

	}
	
	
}

static void chassis_ctrl_loop(chassis *chassic_ctrl_loop)
{
//	if(chassic_ctrl_loop->chassis_mode == chassic_zero || ext_game_robot_status.mains_power_chassis_output == 0)
	if(chassic_ctrl_loop->chassis_mode == chassic_zero )
	{
      chassic_ctrl_loop->motor_speed_pid[0].out=0;
			chassic_ctrl_loop->motor_speed_pid[1].out=0;
			chassic_ctrl_loop->motor_speed_pid[2].out=0;
			chassic_ctrl_loop->motor_speed_pid[3].out=0;
		
				chassic_ctrl_loop->motor_speed_pid_slow[0].out=0;
				chassic_ctrl_loop->motor_speed_pid_slow[1].out=0;
				chassic_ctrl_loop->motor_speed_pid_slow[2].out=0;
        chassic_ctrl_loop->motor_speed_pid_slow[3].out=0;
			
	}
	else
	{
		 uint8_t i;
		 for(i=0;i<4;i++)
			{
				 PID_calc(&chassic_ctrl_loop->motor_speed_pid[i],
				           chassic_ctrl_loop->chassis_motor[i].speed,      
									 chassic_ctrl_loop->wheel_speed[i]);//对每个轮子进行速度设置
					first_order_filter_cali(&chassic_ctrl_loop->motor_speed_pid_slow[i], chassic_ctrl_loop->motor_speed_pid[i].out);
	
			}
	
	}
		CAN_cmd_chassis(chassic_ctrl_loop->motor_speed_pid[0].out,
										chassic_ctrl_loop->motor_speed_pid[1].out,
										chassic_ctrl_loop->motor_speed_pid[2].out,
										chassic_ctrl_loop->motor_speed_pid[3].out
		             );
}



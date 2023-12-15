/*
**********Shoot_task射击任务**********
包含对两个摩擦轮以及拨盘的控制
拨盘为3508，ID = 5，CAN1控制, motor_can1[4]
摩擦轮分别为3508，ID = 6（left）和 ID = 7（right），CAN2 控制
遥控器右边拨杆控制，拨到最上面启动 (从上到下分别为132)
*/

#include "Shoot_task.h"
#include "pid.h"
#include "cmsis_os.h"

shoot_t shoot_motor[3]; //摩擦轮can2，id = 67； 拨盘can1，id = 5
motor_info_t motor_can2[3]; //[2]:pitch
extern RC_ctrl_t rc_ctrl;
extern motor_info_t motor_can1[6];


void Shoot_task(void const * argument)
{
  shoot_loop_init();

  for(;;)
  {
    //遥控器右边拨到上，电机启动
    if(rc_ctrl.rc.s[0] == 1){
      shoot_start();
    }
    else{
      shoot_stop();
    }

    // shoot_stop();

    // //遥控器左边拨到下，弹仓盖打开
    // if(rc_ctrl.rc.s[1] == 2){
    //   shoot_lid_open();
    // }
    // else{
    //   shoot_lid_close();
    // }

    shoot_current_give();
    osDelay(1);
  }
}


/***************初始化***************/
void shoot_loop_init()
{
  // friction_left
  shoot_motor[0].pid_value[0] = 10;
  shoot_motor[0].pid_value[1] = 0;
  shoot_motor[0].pid_value[2] = 0;

  // friction_right
  shoot_motor[1].pid_value[0] = 10;
  shoot_motor[1].pid_value[1] = 0;
  shoot_motor[1].pid_value[2] = 0;

  // trigger
  shoot_motor[2].pid_value[0] = 10;
  shoot_motor[2].pid_value[1] = 0;
  shoot_motor[2].pid_value[2] = 0;

  // 初始化目标速度
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;
  shoot_motor[2].target_speed = 0;

  // 初始化PID
  pid_init(&shoot_motor[0].pid, shoot_motor[0].pid_value, 1000, 5000); // friction_right
  pid_init(&shoot_motor[1].pid, shoot_motor[1].pid_value, 1000, 5000); // friction_left
  pid_init(&shoot_motor[2].pid, shoot_motor[2].pid_value, 1000, 2000); // trigger
}

/***************射击模式*****************/
void shoot_start()
{
  shoot_motor[0].target_speed = 5000;
  shoot_motor[1].target_speed = 5000;
  shoot_motor[2].target_speed = 1000;
}

/***************停止射击模式**************/
void shoot_stop()
{
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;
  shoot_motor[2].target_speed = 0;
}


/********************************摩擦轮can2发送电流***************************/
void shoot_can2_cmd(int16_t v1, int16_t v2)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x1FF;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
	
  tx_header.DLC   = 8;		//发送数据长度（字节）

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] =         NULL;
  tx_data[5] =         NULL;
  tx_data[6] =         NULL;
  tx_data[7] =         NULL;

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}


/********************************拨盘can1发送电流***************************/
void trigger_can1_cmd(int16_t v1)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x1FF;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
	
  tx_header.DLC   = 8;		//发送数据长度（字节）

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] =         NULL;
  tx_data[3] =         NULL;
  tx_data[4] =         NULL;
  tx_data[5] =         NULL;
  tx_data[6] =         NULL;
  tx_data[7] =         NULL;

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}

/********************************PID计算速度并发送电流****************************/
void shoot_current_give()
{

    motor_can2[0].set_current = pid_calc(&shoot_motor[0].pid, shoot_motor[0].target_speed, motor_can2[0].rotor_speed);
    motor_can2[1].set_current = pid_calc(&shoot_motor[1].pid, shoot_motor[1].target_speed, -motor_can2[1].rotor_speed);

    // trigger
    motor_can1[4].set_current = pid_calc(&shoot_motor[2].pid, shoot_motor[2].target_speed, -motor_can1[4].rotor_speed);

    shoot_can2_cmd(motor_can2[0].set_current, -motor_can2[1].set_current);
    trigger_can1_cmd(-motor_can1[4].set_current);
    //set_motor_current_can2(1, motor_can2[0].set_current, motor_can2[1].set_current, motor_can2[2].set_current, motor_can2[3].set_current);
}


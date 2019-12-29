/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "main.h"
#include "can.h"
#include "board.h"
#include "motor.h"
#include "dbus.h"
#include "detect.h"
#include "test.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

#include "chassis_task.h"
#include "gimbal_task.h"
#include "timer_task.h"
#include "shoot_task.h"
#include "communicate.h"
#include "infantry_cmd.h"
#include "init.h"

#include "protocol.h"
#include "ulog.h"
#include "param.h"
#include "offline_check.h"
#include "referee_system.h"
/* by rzf  祖师爷 之后的所有底盘信息都是从从这里开始的 内存是在这里分配的 全局变量区域  */
struct chassis chassis;

struct gimbal gimbal;
struct shoot shoot;
static struct rc_device rc_dev;

static uint8_t glb_sys_cfg;

extern int ulog_console_backend_init(void);

void system_config(void)
{
  glb_sys_cfg = HAL_GPIO_ReadPin(SYS_CFG_Port, SYS_CFG_Pin);
}

uint8_t get_sys_cfg(void)
{
  return glb_sys_cfg;
}

void hw_init(void)
{
	/* by rzf   云台校准 */
  cali_param_init();
	/* by rzf 板载资源初始化   */
  board_config();
  test_init();
  system_config();
  ulog_init();
  ulog_console_backend_init();
  /* by rzf 裁判系统参数初始化   */
  referee_param_init();
	/* by rzf  裁判系统信号的中断处理函数定义  */
  usart3_rx_callback_register(referee_uart_rx_data_handle);
	/* by rzf   发送给裁判系统的 发送函数 ，，发送啥呀*/
  referee_send_data_register(usart3_transmit);

  //if(glb_sys_cfg == CHASSIS_APP)
	if(1)
  {
    rc_device_register(&rc_dev, "uart_rc", 0);
    dr16_forword_callback_register(rc_data_forword_by_can);
	/* by rzf   pid 计算回调函数 */
    chassis_pid_register(&chassis, "chassis", DEVICE_CAN1);
		/* by rzf 四个电机控制使能   */
    chassis_disable(&chassis);
  }
  else
  {
    rc_device_register(&rc_dev, "can_rc", 0);
    gimbal_cascade_register(&gimbal, "gimbal", DEVICE_CAN1);

    shoot_pid_register(&shoot, "shoot", DEVICE_CAN1);

    gimbal_yaw_disable(&gimbal);
    gimbal_pitch_disable(&gimbal);
    shoot_disable(&shoot);
  }
	// 其实这个掉线检测用的好的话 还是有用的
  offline_init();
}

osThreadId timer_task_t;
osThreadId chassis_task_t;
osThreadId gimbal_task_t;
osThreadId communicate_task_t;
osThreadId cmd_task_t;
osThreadId shoot_task_t;

void task_init(void)
{
  uint8_t app;
  app = get_sys_cfg();

  osThreadDef(TIMER_1MS, timer_task, osPriorityHigh, 0, 512);
  timer_task_t = osThreadCreate(osThread(TIMER_1MS), NULL);

  osThreadDef(COMMUNICATE_TASK, communicate_task, osPriorityHigh, 0, 4096);
  communicate_task_t = osThreadCreate(osThread(COMMUNICATE_TASK), NULL);

  osThreadDef(CMD_TASK, infantry_cmd_task, osPriorityNormal, 0, 4096);
  cmd_task_t = osThreadCreate(osThread(CMD_TASK), NULL);
  
  //if (app == CHASSIS_APP)
	if(1)
  {
    osThreadDef(CHASSIS_TASK, chassis_task, osPriorityRealtime, 0, 512);
    chassis_task_t = osThreadCreate(osThread(CHASSIS_TASK), NULL);
  }
  else
  {
    osThreadDef(GIMBAL_TASK, gimbal_task, osPriorityRealtime, 0, 512);
    gimbal_task_t = osThreadCreate(osThread(GIMBAL_TASK), NULL);

    osThreadDef(SHOOT_TASK, shoot_task, osPriorityNormal, 0, 512);
    shoot_task_t = osThreadCreate(osThread(SHOOT_TASK), NULL);
  }
}

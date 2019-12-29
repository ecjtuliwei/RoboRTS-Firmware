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

#ifndef __MECANUM_H__
#define __MECANUM_H__

#ifdef MECANUM_H_GLOBAL
#define MECANUM_H_EXTERN
#else
#define MECANUM_H_EXTERN extern
#endif

/************************ chassis parameter ****************************/
/* the radius of wheel(mm) 轮子半径*/
#define RADIUS 76
/* the perimeter of wheel(mm) 轮子周长 */
#define PERIMETER 478
/* wheel track distance(mm) 轮距 */
#define WHEELTRACK 394
/* wheelbase distance(mm) 轴距 */
#define WHEELBASE 415

/* gimbal is relative to chassis center x axis offset(mm) 坐标变换信息 云台相对底盘中心的x轴向距离*/
#define ROTATE_X_OFFSET 7
/* gimbal is relative to chassis center y axis offset(mm) 坐标变换信息 云台相对底盘中心的y轴向距离 */
#define ROTATE_Y_OFFSET 0

/* chassis motor use 3508 强大的3508 */
/* the deceleration ratio of chassis motor 底盘电机的减速比 */
#define MOTOR_DECELE_RATIO (1.0f / 19.0f)
/* single 3508 motor maximum speed, unit is rpm  3508单电机最大速度，单位为rpm  也就是说最大是8347rpm 3.5m/s DJI NB*/
#define MAX_WHEEL_RPM 8500 //8347rpm = 3500mm/s
/* chassis maximum translation speed, unit is mm/s  底盘最大平移速度，单位为mm / s vx vy都是一样的*/
#define MAX_CHASSIS_VX_SPEED 3300 //8000rpm 
#define MAX_CHASSIS_VY_SPEED 3300
/* chassis maximum rotation speed, unit is degree/s  底盘最大转速，单位为度/秒 不是全向轮的留下了眼泪*/
#define MAX_CHASSIS_VW_SPEED 300 //5000rpm

#define MOTOR_ENCODER_ACCURACY 8192.0f // 电机编码器精度

/** 
  * @brief  infantry structure configuration information 
	*  步兵结构配置信息 我是一个没的感情的google 翻译搬砖工人
  */
struct mecanum_structure
{
  float wheel_perimeter; /* the perimeter(mm) of wheel 轮子的周长（mm） */
  float wheeltrack;      /* wheel track distance(mm) 轮距（mm） */
  float wheelbase;       /* wheelbase distance(mm) 轴距（mm） */
  float rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center 云台偏移量x */
  float rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center 云台偏移量y */
};
// 麦克纳姆位置 没注释看个鬼呀
struct mecanum_position
{
  float v_x_mm;
  float v_y_mm;
  float rate_deg;
  float position_x_mm;
  float position_y_mm;
  float angle_deg;
};
// 麦克纳姆 速度信息 难道是每隔轮子都有一个速度信息吗  emmm 好像是要这么干
struct mecanum_speed
{
  float vx; // forward/back
  float vy; // left/right
  float vw; // anticlockwise/clockwise
};
// 麦克纳姆陀螺 what is this???
struct mecanum_gyro
{
  float yaw_gyro_angle;
  float yaw_gyro_rate;
};
// bos 出马 把上面的所有结构体 一共四个加入到了自己家里来 
// 还邀请了wheel_rpm这个float[4]数组 记录每个轮子的转动速度

struct mecanum
{
  struct mecanum_structure param;
  struct mecanum_speed speed;
  struct mecanum_position position;
  struct mecanum_gyro gyro;
  float  wheel_rpm[4];
};
// 不明觉厉 这是啥
/* by rzf    */
struct mecanum_motor_fdb
{
  float total_ecd;
  float speed_rpm;
};
/* by rzf 麦克纳姆计算 应该是为上面的bos 结构体填充数据   */
void mecanum_calculate(struct mecanum *mec);
void mecanum_position_measure(struct mecanum *mec, struct mecanum_motor_fdb wheel_fdb[]);

#endif // __MECANUM_H__

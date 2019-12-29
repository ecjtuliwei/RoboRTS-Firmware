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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mecanum.h"

#ifndef RADIAN_COEF
  #define RADIAN_COEF 57.3f
#endif

#define MEC_VAL_LIMIT(val, min, max) \
  do                                 \
  {                                  \
    if ((val) <= (min))              \
    {                                \
      (val) = (min);                 \
    }                                \
    else if ((val) >= (max))         \
    {                                \
      (val) = (max);                 \
    }                                \
  } while (0)

/**
  * @brief mecanum glb_chassis velocity decomposition.F:forword; B:backword; L:left; R:right
	* by rzf  麦克纳姆氏菌底盘速度分解。 V：向后； L：左； R：对 
	* 输入为 ccx，ccy，ccw x y w 的加速度 输出是四个轮子 的 rpm 有点像运动学解算了
  * @param input : ccx=+vx(mm/s)  ccy=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR 定义四个轮子的位置代码
  */
void mecanum_calculate(struct mecanum *mec)
{
	/* by rzf    static 变量一直存在静态数据区域 不随函数调用生命周期结束而销毁（不在函数调用栈上）*/
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float wheel_rpm_ratio;
	/* by rzf 速度和加速度都是基于底盘的中心  解算速度的时候要把速度分解到四个电机上面 所以要指导四个电机对于底盘中心 的偏移量  */
  rotate_ratio_fr = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f - mec->param.rotate_x_offset + mec->param.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_fl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f - mec->param.rotate_x_offset - mec->param.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_bl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f + mec->param.rotate_x_offset - mec->param.rotate_y_offset) / RADIAN_COEF;
  rotate_ratio_br = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f + mec->param.rotate_x_offset + mec->param.rotate_y_offset) / RADIAN_COEF;
	/* by rzf  车轮转速比 电机的rpm 转换成轮子的转动速度  */
  wheel_rpm_ratio = 60.0f / (mec->param.wheel_perimeter * MOTOR_DECELE_RATIO);
	/* by rzf  先判断下速度不要超过  不要超过最大速度 */
  MEC_VAL_LIMIT(mec->speed.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); //mm/s
  MEC_VAL_LIMIT(mec->speed.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); //mm/s
  MEC_VAL_LIMIT(mec->speed.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED); //deg/s

  float wheel_rpm[4];
  float max = 0;
	/* by rzf    1=FR 2=FL 3=BL 4=BR 定义四个轮子的位置代码 */
  wheel_rpm[0] = (-mec->speed.vx - mec->speed.vy - mec->speed.vw * rotate_ratio_fr) * wheel_rpm_ratio;
  wheel_rpm[1] = (mec->speed.vx - mec->speed.vy - mec->speed.vw * rotate_ratio_fl) * wheel_rpm_ratio;
  wheel_rpm[2] = (mec->speed.vx + mec->speed.vy - mec->speed.vw * rotate_ratio_bl) * wheel_rpm_ratio;
  wheel_rpm[3] = (-mec->speed.vx + mec->speed.vy - mec->speed.vw * rotate_ratio_br) * wheel_rpm_ratio;

  //find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (fabs(wheel_rpm[i]) > max)
      max = fabs(wheel_rpm[i]);
  }
/* by rzf   找到最大值后 如果最大速度大于 MAX_WHEEL_RPM 计算比例系数 让四个电机的转速成比例的增加 等比例 */
  //equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
	/* by rzf   内存值拷贝函数 */
  memcpy(mec->wheel_rpm, wheel_rpm, 4 * sizeof(float));
}
/* by rzf 给底盘四个电机 上电之后调用这个函数来更新电机的编码器的测量信息  */
void mecanum_position_measure(struct mecanum *mec, struct mecanum_motor_fdb wheel_fdb[])
{
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float rpm_ratio;
  static float ecd_ratio;
  static double mecanum_angle;
  static double last_d_x, last_d_y, last_d_w, d_x, d_y, d_w, diff_d_x, diff_d_y, diff_d_w;
  static double position_x, position_y, angle_w;
  static double v_x, v_y, v_w;

  rotate_ratio_fr = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f -
                     mec->param.rotate_x_offset + mec->param.rotate_y_offset);
  rotate_ratio_fl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f -
                     mec->param.rotate_x_offset - mec->param.rotate_y_offset);
  rotate_ratio_bl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f +
                     mec->param.rotate_x_offset - mec->param.rotate_y_offset);
  rotate_ratio_br = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f +
                     mec->param.rotate_x_offset + mec->param.rotate_y_offset);
  rpm_ratio = mec->param.wheel_perimeter * MOTOR_DECELE_RATIO / (4 * 60.0f);
  ecd_ratio = mec->param.wheel_perimeter * MOTOR_DECELE_RATIO / (4 * MOTOR_ENCODER_ACCURACY);

  last_d_x = d_x;
  last_d_y = d_y;
  last_d_w = d_w;
	/* by rzf   四个轮子的编码器数值合成 底盘中心的速度 */
  d_x = ecd_ratio * (-wheel_fdb[0].total_ecd + wheel_fdb[1].total_ecd + wheel_fdb[2].total_ecd - wheel_fdb[3].total_ecd);
  d_y = ecd_ratio * (-wheel_fdb[0].total_ecd - wheel_fdb[1].total_ecd + wheel_fdb[2].total_ecd + wheel_fdb[3].total_ecd);
  d_w = ecd_ratio * (-wheel_fdb[0].total_ecd / rotate_ratio_fr - wheel_fdb[1].total_ecd / rotate_ratio_fl - wheel_fdb[2].total_ecd / rotate_ratio_bl - wheel_fdb[3].total_ecd / rotate_ratio_br);

  diff_d_x = d_x - last_d_x;
  diff_d_y = d_y - last_d_y;
  diff_d_w = d_w - last_d_w;

  /* use glb_chassis gyro angle data */
  mecanum_angle = mec->gyro.yaw_gyro_angle / RADIAN_COEF;

  position_x += diff_d_x * cos(mecanum_angle) - diff_d_y * sin(mecanum_angle);
  position_y += diff_d_x * sin(mecanum_angle) + diff_d_y * cos(mecanum_angle);

  angle_w += diff_d_w;

  mec->position.position_x_mm = position_x;        //mm
  mec->position.position_y_mm = position_y;        //mm
  mec->position.angle_deg = angle_w * RADIAN_COEF; //degree

  v_x = rpm_ratio * (-wheel_fdb[0].speed_rpm + wheel_fdb[1].speed_rpm + wheel_fdb[2].speed_rpm - wheel_fdb[3].speed_rpm);
  v_y = rpm_ratio * (-wheel_fdb[0].speed_rpm - wheel_fdb[1].speed_rpm + wheel_fdb[2].speed_rpm + wheel_fdb[3].speed_rpm);
  v_w = rpm_ratio * (-wheel_fdb[0].speed_rpm / rotate_ratio_fr - wheel_fdb[1].speed_rpm / rotate_ratio_fl - wheel_fdb[2].speed_rpm / rotate_ratio_bl - wheel_fdb[3].speed_rpm / rotate_ratio_br);

  mec->position.v_x_mm = v_x;                 //mm/s
  mec->position.v_y_mm = v_y;                 //mm/s
  mec->position.rate_deg = v_w * RADIAN_COEF; //degree/s
}

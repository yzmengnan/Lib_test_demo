//
// Created by 91418 on 24-4-10.
//

#ifndef FINAL_INC_TASK_H
#define FINAL_INC_TASK_H

class TASK
{
	/**
	 * \brief 初始化，应该包括机械臂接口driver，末端操作器接口
	 * 	使用指针调用
	 */
	TASK();

	/**
	 * \brief task A 执行定位移动任务，从A点直线运动到B点，参考坐标系为世界坐标系
	 */
	void TASK_A();
	/**
	 * \brief task B 执行定位移动任务，从A点直线运动到B点，参考坐标系为末端坐标系
	 */
	void TASK_B();
	/**
 	 * \brief task C 执行定位任务，使用视觉伺服
 	 */
	void TASK_C();
};

#endif// FINAL_INC_TASK_H

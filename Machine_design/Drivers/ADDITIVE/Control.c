#include "Control.h"
#include "tim.h"
#include "cmsis_os.h"
#include "dac.h"
#include "main.h"
#include <stdio.h>
/********************************************************************结构体变量*******************************************************************************/
PID x_dir; // 二维云台X方向
PID y_dir; // 二维云台Y方向
uint8_t zz_state=0; // 种子状态
uint8_t zz_cnt=1; //运行次数
uint8_t send_state=0; //发送串口状态
uint8_t start = 0;
//=====================================================常规函数区=====================================================
//-------------------------------------------------------------------------------------------------------------------
// @brief       SE_init()
// @param       void
// @return      void
// @function    外设初始化（舵机、电机、步进电机、推杆）
//-------------------------------------------------------------------------------------------------------------------
void SE_init(void)
{
	// 舵机初始化
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // TIM1通道1PWM初始化（后面上层舵机）
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // TIM1通道2PWM初始化（前面下层舵机）
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // TIM1通道3PWM初始化（后面下层舵机）
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // TIM1通道4PWM初始化（前面上层舵机）
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // TIM2通道1PWM初始化(推盘子落地的丝杆步进电机PWM)
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // TIM2通道2PWM初始化(推盘子落地的丝杆步进电机方向)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // TIM3通道1PWM初始化（第一个刮土舵机）
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // TIM3通道2PWM初始化（第二个刮土舵机）
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // TIM3通道3PWM初始化（顶落种子舵机1）
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // TIM3通道4PWM初始化（顶落种子舵机2）
	// 步进电机初始化
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // TIM4通道2PWM初始化（右传送带）
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // TIM4通道4PWM初始化（左传送带）
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // TIM8通道2PWM初始化（第一个播种传送带）
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // TIM8通道4PWM初始化（第二个播种传送带）
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // TIM9通道1PWM初始化（第一个平台步进电机）
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); // TIM9通道2PWM初始化（第二个平台步进电机）
	//推杆初始化
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // TIM4通道3PWM初始化 (推杆dir1)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // TIM2通道4PWM初始化 (推杆dir2)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // TIM2通道3PWM初始化 (底座推杆1)
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // TIM12通道1PWM初始化 (底座推杆2)
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // TIM4通道1PWM初始化 (盘子推杆dir1)
	// 电机初始化
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // TIM8通道1PWM初始化（第二个落土电机）
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // TIM12通道2PWM初始化（第一个落土电机）
	// 落盘舵机归位
	FU_DJPWM(0);
	FD_DJPWM(0);
	BU_DJPWM(0);
	BD_DJPWM(0);
	// 落种子舵机归位
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 130);//初始位置（80-开 140-关）
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 190);//初始位置（240-开 180-关）
	//刮土舵机归位
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 175);//刮土装置1
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 200);//刮土装置2（193-闭 150-开）
	//轮子初始化
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       LP_start(void)
// @param       void
// @return      void
// @function    实现第一次落盘
//-------------------------------------------------------------------------------------------------------------------
void LP_start(void)
{
	// 落盘上层舵机打开
	FU_DJPWM(1);
	BU_DJPWM(1);
	osDelay(1000);
	// 落盘上层舵机关闭
	FU_DJPWM(0);
	BU_DJPWM(0);
	osDelay(1000);
	// 落盘下层舵机打开
	FD_DJPWM(1);
	BD_DJPWM(1);
	osDelay(1000);
	// 落盘下层舵机关闭
	FD_DJPWM(0);
	BD_DJPWM(0);
	osDelay(1000);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       CSD_Move(uint8_t state)
// @param       state 0:停止状态 1:运动状态
// @return      void
// @function    实现传送带运动
//-------------------------------------------------------------------------------------------------------------------
void CSD_Move(uint8_t state)
{
	switch(state)
	{
	    case 0:	
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0); 
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); 
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 50); 
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 50);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       Platform_Move(uint8_t state)
// @param       state 0:停止状态 1:前进状态 2:后退状态
// @return      void
// @function    实现平台传送带运动
//-------------------------------------------------------------------------------------------------------------------
void Platform_Move(uint8_t state)
{
	switch(state)
	{
	    case 0:	
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0); 
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0); 
			break;
		case 1:
			HAL_GPIO_WritePin(platform_dir2_GPIO_Port, platform_dir2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(platform_dir1_GPIO_Port, platform_dir1_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 50); 
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 50);
			break;
		case 2:
			HAL_GPIO_WritePin(platform_dir2_GPIO_Port, platform_dir2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(platform_dir1_GPIO_Port, platform_dir1_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 50); 
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 50);
		break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       Platform_UD(uint8_t state)
// @param       state 0:停止状态 1:上升状态 2:下降状态
// @return      void
// @function    运动实现平台上下运动
//-------------------------------------------------------------------------------------------------------------------
void Platform_UD(uint8_t state)
{
	switch(state)
	{
	    case 0:	
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); 
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); 
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); 
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 2000);
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100); 
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       Ground_UD(uint8_t state)
// @param       state 0:停止状态 1:上升状态 2:下降状态
// @return      void
// @function    运动实现底座上下运动
//-------------------------------------------------------------------------------------------------------------------
void Ground_UD(uint8_t state)
{
	switch(state)
	{
	    case 0:	
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); 
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0); 
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); 
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 100);
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000); 
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
		break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       PANZI(uint8_t state)
// @param       state 0:停止状态 1:伸长状态 2:收缩状态
// @return      void
// @function    运动实现推动盘子运动
//-------------------------------------------------------------------------------------------------------------------
void PANZI(uint8_t state)
{
	switch(state)
	{
	    case 0:	
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // TIM4通道1PWM初始化 (盘子推杆dir1)
			HAL_GPIO_WritePin(panzi_dir_GPIO_Port, panzi_dir_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100); 
			HAL_GPIO_WritePin(panzi_dir_GPIO_Port, panzi_dir_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); 
			HAL_GPIO_WritePin(panzi_dir_GPIO_Port, panzi_dir_Pin, GPIO_PIN_SET);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       ZZ_start(void)
// @param       void
// @return      void
// @function    实现种子下落
//-------------------------------------------------------------------------------------------------------------------
void ZZ_start(void)
{
	static uint8_t zz_dir = 1;//种子方向
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 140);//初始位置（80-开 140-关）
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 180);//初始位置（240-开 180-关）
	while(zz_state != zz_cnt)
	{
		ZZ_PWM(zz_dir);
		osDelay(5600);
		ZZ_PWM(0);
		if(zz_dir == 1)zz_dir = 2;
		else zz_dir = 1;
		zz_state++;
		osDelay(250);
	}
	zz_state = 0;
	//先开
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 80);//初始位置（80-开 140-关）
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 240);//初始位置（240-开 180-关）
	osDelay(750);
	//再关闭
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 140);//初始位置（80-开 140-关）
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 180);//初始位置（240-开 180-关）
	osDelay(750);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       FU_DJPWM(uint8_t state)
// @param       state 0:关闭状态  1：打开状态
// @return      void
// @function    落盘前面上层舵机pwm输出
//-------------------------------------------------------------------------------------------------------------------
void FU_DJPWM(uint8_t state)
{
	switch (state)
	{
		case 0:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 120);
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 160);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       FD_DJPWM(uint8_t state)
// @param       state 0:关闭状态  1：打开状态
// @return      void
// @function    落盘前面下层舵机pwm输出
//-------------------------------------------------------------------------------------------------------------------
void FD_DJPWM(uint8_t state)
{
	switch (state)
	{
		case 0:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 130);
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 90);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       BU_DJPWM(uint8_t state)
// @param       state 0:关闭状态  1：打开状态
// @return      void
// @function    落盘后面上层舵机pwm输出
//-------------------------------------------------------------------------------------------------------------------
void BU_DJPWM(uint8_t state)
{
	switch (state)
	{
		case 0:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 170);
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 210);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       BD_DJPWM(uint8_t state)
// @param       state 0:关闭状态  1：打开状态
// @return      void
// @function    落盘后面下层舵机pwm输出
//-------------------------------------------------------------------------------------------------------------------
void BD_DJPWM(uint8_t state)
{
	switch (state)
	{
		case 0:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 200);
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 165);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       ZZ_PWM(uint8_t state)
// @param       state  0: 停止状态 1:后退状态  2：前进状态
// @return      void
// @function    落盘pwm输出
//-------------------------------------------------------------------------------------------------------------------
void ZZ_PWM(uint8_t state)
{
	switch (state)
	{
		case 0:
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 50);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 50);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 50);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 50);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       Push_to_ground_PWM(uint8_t state)
// @param       state  0: 停止状态 1:前进状态  2：后退状态
// @return      void
// @function    推盘子到地上pwm输出
//-------------------------------------------------------------------------------------------------------------------
void Push_to_ground_PWM(uint8_t state)
{
	switch (state)
	{
		case 0:
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(GROUND_DIR_GPIO_Port, GROUND_DIR_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
			break;
		case 2:
			HAL_GPIO_WritePin(GROUND_DIR_GPIO_Port, GROUND_DIR_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       run()
// @param       void
// @return      void
// @function    机器整体移动
//-------------------------------------------------------------------------------------------------------------------
void run(uint8_t state)
{
	switch(state)
	{
	    case 0:
			HAL_GPIO_WritePin(PT_LUNZI_GPIO_Port, PT_LUNZI_Pin, GPIO_PIN_RESET);
			osDelay(5);
			HAL_GPIO_WritePin(LP_LUNZI_GPIO_Port, LP_LUNZI_Pin, GPIO_PIN_RESET);//落盘平台附近轮子
			break;
		case 1:
			HAL_GPIO_WritePin(LP_LUNZI_GPIO_Port, LP_LUNZI_Pin, GPIO_PIN_SET);//落盘平台附近轮子
			osDelay(50);
			HAL_GPIO_WritePin(PT_LUNZI_GPIO_Port, PT_LUNZI_Pin, GPIO_PIN_SET);//升降平台附近轮子
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       pwm()
// @param       void
// @return      void
// @function    二维云台舵机的pwm脉冲信号输出
//-------------------------------------------------------------------------------------------------------------------
void pwm(int xpwm, int ypwm)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, xpwm);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ypwm);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       PID_init()
// @param       void
// @return      void
// @function    云台舵机的参数初始化
//-------------------------------------------------------------------------------------------------------------------
void PID_init(PID *x, float P, float I, float D)
{
	x->now_error = 0;
	x->pre_error = 0;
	x->d_error = 0;
	x->sum_error = 0;
	x->P = P;
	x->I = I;
	x->D = D;
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       PID_init()
// @param       void
// @return      void
// @function    云台舵机的pwm输出
//-------------------------------------------------------------------------------------------------------------------
float Position_pid(float target, float reality, PID *pid) // 位置式PID
{
	pid->now_error = target - reality; // 谁减谁需要调整
	pid->d_error = pid->now_error - pid->pre_error;
	pid->sum_error += pid->now_error;

	pid->output = (pid->P * pid->now_error) + (pid->D * pid->d_error) + (pid->I * pid->sum_error);

	pid->pre_error = pid->now_error;

	return pid->output;
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       con1_half()
// @param       void
// @return      void
// @function    第一个盘子共同前半部分运动
//-------------------------------------------------------------------------------------------------------------------
void con1_half(void)
{
	LP_start();//启动落盘
	CSD_Move(1);//启动传�?�带
	osDelay(1700);//1550
	CSD_Move(0);//停止传�?�带
	osDelay(1000);

	send_state = (5 * (start-1)) + 1;//发送第一/二个盘子第一次覆土状态

	__HAL_TIM_SET_PRESCALER(&htim4,1260-1);//传�?�带降�??
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,100);//落土滚筒1启动
	osDelay(1400);
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);//落土滚筒1关闭
	osDelay(1000);
	CSD_Move(1);//启动传�?�带
	osDelay(1000);

	CSD_Move(0);//停止传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 240);//刮土装置1打开
	CSD_Move(1);//启动传�?�带
	osDelay(1100);
	CSD_Move(0);//停止传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 175);//刮土装置1关闭
	osDelay(500);
	CSD_Move(1);//启动传�?�带
	osDelay(3750);
	CSD_Move(0);//停止传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 240);//刮土装置1打开
	osDelay(500);
	CSD_Move(1);//启动传�?�带
	osDelay(1200);
    CSD_Move(0);//停止传�?�带
	send_state = (5 * (start-1)) + 2;//发送第一/二个盘子播种状态
	ZZ_start();//落种子程序启�??????????
	send_state = (5 * (start-1)) + 3;//发送第一/二个盘子完成播种率状态

	CSD_Move(1);//启动传�?�带
	osDelay(2250);
	CSD_Move(0);//停止传�?�带

	send_state = (5 * (start-1)) + 4;//发送第一/二个盘子第二次覆土状态
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,100);//落土滚筒2启动
	osDelay(1000);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//落土滚筒2关闭
	osDelay(500);

	CSD_Move(1);//启动传�?�带
	osDelay(1100);
	CSD_Move(0);//停止传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 145);//刮土装置2打开
	CSD_Move(1);//启动传�?�带
	Platform_Move(1);
	osDelay(950);
	CSD_Move(0);//停止传�?�带
	Platform_Move(0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 200);//刮土装置2关闭
	osDelay(550);
	Platform_Move(1);
	CSD_Move(1);//启动传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 175);//刮土装置1关闭
	osDelay(4500);//4500

	CSD_Move(0);
	Platform_Move(0);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       con_half()
// @param       void
// @return      void
// @function    第二个盘子共同前半部分运动
//-------------------------------------------------------------------------------------------------------------------
void con2_half(void)
{
	LP_start();//启动落盘
	CSD_Move(1);//启动传�?�带
	osDelay(2500);//1550
	CSD_Move(0);//停止传�?�带
	osDelay(1000);

	send_state = (5 * (start-1)) + 1;//发送第一/二个盘子第一次覆土状态

	__HAL_TIM_SET_PRESCALER(&htim4,1260-1);//传�?�带降�??
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,100);//落土滚筒1启动
	osDelay(2000);
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);//落土滚筒1关闭
	osDelay(1000);
	CSD_Move(1);//启动传�?�带
	osDelay(1000);

	CSD_Move(0);//停止传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 240);//刮土装置1打开
	CSD_Move(1);//启动传�?�带
	osDelay(1200);
	CSD_Move(0);//停止传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 175);//刮土装置1关闭
	osDelay(500);
	CSD_Move(1);//启动传�?�带
	osDelay(3750);
	CSD_Move(0);//停止传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 240);//刮土装置1打开
	osDelay(500);
	CSD_Move(1);//启动传�?�带
	osDelay(1200);
    CSD_Move(0);//停止传�?�带
	send_state = (5 * (start-1)) + 2;//发送第一/二个盘子播种状态
	ZZ_start();//落种子程序启�??????????
	send_state = (5 * (start-1)) + 3;//发送第一/二个盘子完成播种率状态

	CSD_Move(1);//启动传�?�带
	osDelay(2250);
	CSD_Move(0);//停止传�?�带

	send_state = (5 * (start-1)) + 4;//发送第一/二个盘子第二次覆土状态
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,100);//落土滚筒2启动
	osDelay(1200);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);//落土滚筒2关闭
	osDelay(500);

	CSD_Move(1);//启动传�?�带
	osDelay(1100);
	CSD_Move(0);//停止传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 145);//刮土装置2打开
	CSD_Move(1);//启动传�?�带
	Platform_Move(1);
	osDelay(950);
	CSD_Move(0);//停止传�?�带
	Platform_Move(0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 200);//刮土装置2关闭
	osDelay(550);
	Platform_Move(1);
	CSD_Move(1);//启动传�?�带
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 175);//刮土装置1关闭
	osDelay(4500);//4500

	CSD_Move(0);
	Platform_Move(0);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       first_half()
// @param       void
// @return      void
// @function    第一个盘子后半部分运动
//-------------------------------------------------------------------------------------------------------------------
void first_half(void)
{
	Platform_UD(2);
	osDelay(8500);//9000
	Platform_UD(0);

	Platform_Move(2);
	osDelay(2000);//3000
	Platform_UD(1);
	Platform_Move(2);
	osDelay(8500);//10000
	Platform_UD(0);
	Platform_Move(0);


	send_state=5;//发送第一个盘子排布状态
	PANZI(1);
	osDelay(8000);
	PANZI(0);
	osDelay(500);
	PANZI(2);
	osDelay(8500);
	PANZI(0);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       second_half()
// @param       void
// @return      void
// @function    第二个盘子后半部分运动
//-------------------------------------------------------------------------------------------------------------------
void second_half(void)
{
	Platform_UD(2);
	osDelay(8500);//9000
	Platform_UD(0);

	Platform_Move(2);
	osDelay(2000);//3000
	Platform_UD(1);
	Platform_Move(2);
	osDelay(8500);//10000
	Platform_UD(0);
	Platform_Move(0);
	
	Ground_UD(1);
	osDelay(8000);
	Ground_UD(0);
	send_state=10;//发送第二个盘子排布状态
	Push_to_ground_PWM(1);
	osDelay(14000);//14000
	Push_to_ground_PWM(0);	
	send_state=10;//发送两个盘子排布状态
	osDelay(10000);
	Push_to_ground_PWM(2);
	Ground_UD(2);
	osDelay(8000);
	Ground_UD(0);
	osDelay(6000);//14000-8000
	Push_to_ground_PWM(0);
}

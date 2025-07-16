#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "stdint.h"
//=====================================================常规函数区=====================================================
typedef struct{
	  float now_error;		//这次的差值
	  float pre_error;		//上次的差值
	  float d_error;      //上次误差和这次误差的变化值
    float sum_error;		//累积的差值
			
    float P;						//比例系数
    float I;						//积分系数
    float D;						//微分系数   
		float output;       //输出值
}PID;

extern PID x_dir;//二维云台X方向
extern PID y_dir;//二维云台Y方向
extern uint8_t start;
extern uint8_t send_state; //发送串口状态

void SE_init(void);
void LP_start(void);
void CSD_Move(uint8_t state);
void Platform_Move(uint8_t state);
void Platform_UD(uint8_t state);
void Ground_UD(uint8_t state);
void PANZI(uint8_t state);
void ZZ_start(void);
void FU_DJPWM(uint8_t state);
void FD_DJPWM(uint8_t state);
void BU_DJPWM(uint8_t state);
void BD_DJPWM(uint8_t state);
void ZZ_PWM(uint8_t state);
void Push_to_ground_PWM(uint8_t state);
void run(uint8_t state);
void pwm(int xpwm, int ypwm);
void PID_init(PID *x, float P, float I, float D);
float Position_pid(float target,float reality,PID *pid);
void con1_half(void);
void con2_half(void);
void first_half(void);
void second_half(void);
#endif /* __CONTROL_H__ */
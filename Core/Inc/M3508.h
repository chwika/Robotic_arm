#ifndef __M3508_H
#define __M3508_H
#include "fdcan.h"
//8192  19
extern volatile int16_t vx;
extern volatile int16_t vy;
extern volatile int16_t w;
extern volatile int16_t v[4];
extern int16_t arm_speed_P[2];
extern int16_t arm_speed_S[2];

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_TxFrame_TypeDef;



typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_RxFrame_TypeDef;


extern FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
extern volatile int32_t arm_angle[2];
extern int32_t arm_angle_init[2];

typedef struct{
float KP;
float KI;
float KD;
	
float error;
float lasterror;
float lastlasterror;
	
float measure;
float target;
	
float p_out;
float i_out;
float d_out;
float Output;
	
float lastv;
float lastlastv;
	

}PID_STU;


typedef struct{
float KP;
float KI;
float KD;
	
float error;
float lasterror;
float sum;
	
float measure;
float target;
	

float Output;
	
float lasta;	

}PID_POS;

typedef struct 
{ int16_t angle;
int16_t before_angle;
	int32_t angle_sum;
	int16_t v;
PID_STU M3508_Speed_PID;
PID_POS M3508_Angle_PID;
	int32_t output;

}M3508_STU ;

extern M3508_STU m3508[2];
extern M3508_STU M3508_LEFT[2];
extern M3508_STU M3508_RIGHT[2];
void M3508_Arm_Process(FDCAN_RxHeaderTypeDef* RxHeader,uint8_t Data[]);
void M3508_Left_Process(FDCAN_RxHeaderTypeDef* RxHeader,uint8_t Data[]);
void M3508_Right_Process(FDCAN_RxHeaderTypeDef* RxHeader,uint8_t Data[]);
void M3508_Speed_Mode(M3508_STU *M3508,volatile int16_t *speed);
void M3508_Position_Mode(M3508_STU *M3508,volatile int32_t *angle,int16_t *speed);
void Chassis_Calculation(int16_t x,int16_t y,int16_t w,volatile int16_t* v);



void PID_Calculate_Incremental(PID_STU*PID,float measure,float target);
void PID_Calculate_Positional(PID_POS*PID,float measure,float target);
void PID_Arm_Speed_Init(M3508_STU*m3508);
void PID_Chassis_Speed_Init(M3508_STU*m3508);
void PID_Position_Init(M3508_STU*m3508);

#endif

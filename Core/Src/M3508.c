#include "M3508.h"
#include "fdcan.h"
#include <stdlib.h>
#include <math.h>

volatile int16_t vx;
volatile int16_t vy;
volatile int16_t w;
volatile int16_t v[4];


int8_t init[2]={0};

volatile int32_t arm_angle[2];
int32_t arm_angle_init[2];
int16_t arm_speed_P[2];
int16_t arm_speed_S[2];
M3508_STU m3508[2];
M3508_STU M3508_LEFT[2];
M3508_STU M3508_RIGHT[2];
#define r 0.5;




void M3508_Arm_Process(FDCAN_RxHeaderTypeDef* RxHeader,uint8_t Data[]){
	uint8_t num=(RxHeader->Identifier&0x000F)-1;
	if(num==0||num==1){
	m3508[num].angle=(Data[0]<<8)|(Data[1]);
	m3508[num].v=(Data[2]<<8)|(Data[3]);
		if(init[num]==0){
	
	
	
	arm_angle[num]=m3508[num].angle;
		
	m3508[num].before_angle=m3508[num].angle;
	m3508[num].angle_sum=m3508[num].angle;
		
	arm_angle_init[num]=arm_angle[num];
		
	init[num]=1;}
	
	
	int16_t delta=m3508[num].angle-m3508[num].before_angle;
	if((delta)<-4096){
		  m3508[num].angle_sum+=delta+8192;}
		  else if((delta)>4096){
			m3508[num].angle_sum+=delta-8192;}
			else{
		  m3508[num].angle_sum+=delta;}
	
	m3508[num].before_angle=m3508[num].angle;}}

	
void M3508_Left_Process(FDCAN_RxHeaderTypeDef* RxHeader,uint8_t Data[]){
	
	uint8_t num=(RxHeader->Identifier&0x000F)-1;
	if(num==0||num==1){
	
	M3508_LEFT[num].v=(Data[2]<<8)|(Data[3]);
	}}

	
void M3508_Right_Process(FDCAN_RxHeaderTypeDef* RxHeader,uint8_t Data[]){
	
	uint8_t num=(RxHeader->Identifier&0x000F)-1;
	if(num==0||num==1){
	
	M3508_RIGHT[num].v=(Data[2]<<8)|(Data[3]);
	}}

	
void M3508_Speed_Mode(M3508_STU *M3508,volatile int16_t *Speed){
	PID_Calculate_Incremental(&(M3508->M3508_Speed_PID), M3508->v,*Speed);
	if(M3508->M3508_Speed_PID.Output>10000)M3508->M3508_Speed_PID.Output=10000;
	if(M3508->M3508_Speed_PID.Output<-10000)M3508->M3508_Speed_PID.Output=-10000;
	//if(fabs(M3508->M3508_Speed_PID.Output)<50)M3508->M3508_Speed_PID.Output=0;
	
	M3508->output=M3508->M3508_Speed_PID.Output;
}

void M3508_Position_Mode(M3508_STU *M3508,volatile int32_t *angle,int16_t *speed){
	PID_Calculate_Positional(&(M3508->M3508_Angle_PID),M3508->angle_sum,*angle);
	*speed=M3508->M3508_Angle_PID.Output;}

	
void Chassis_Calculation(int16_t x,int16_t y,int16_t w,volatile int16_t* v){
	v[0]=x*0.70710678+y*0.70710678+w*r;
	v[1]=-x*0.70710678+y*0.70710678+w*r;
	v[2]=-x*0.70710678-y*0.70710678+w*r;
	v[3]=x*0.70710678-y*0.70710678+w*r;

}



void PID_Calculate_Incremental(PID_STU*PID,float measure,float target){
	PID->measure=measure;
	PID->target=target;
	PID->error=target-measure;
	
	PID->p_out=PID->KP*(PID->error-PID->lasterror);
	PID->i_out=PID->KI*PID->error;
	PID->d_out=PID->KD*(PID->error-PID->lasterror-(PID->lasterror-PID->lastlasterror));
	
	
	
	PID->Output+=PID->p_out+PID->i_out+PID->d_out;
	PID->lastlasterror=PID->lasterror;
	PID->lasterror=PID->error;
	
	
}
void PID_Calculate_Positional(PID_POS*PID,float measure,float target){
	PID->measure=measure;
	PID->target=target;
	PID->error=target-measure;
	
	PID->sum+=PID->error;
	if(PID->sum>1200)PID->sum=1200;
	if(PID->sum<-1200)PID->sum=-1200;
	
	
	
	PID->Output=PID->KP*PID->error+PID->KI*PID->sum+PID->KD*(PID->error-PID->lasterror);
	if(PID->Output>2000)PID->Output=2000;
	if(PID->Output<-2000)PID->Output=-2000;
	PID->lasterror=PID->error;
	//if((fabs(PID->Output)<=250)&&(fabs(PID->error)<=250)){
	//PID->Output=0;}
	
}

void PID_Arm_Speed_Init(M3508_STU*m3508){
	
	m3508->M3508_Speed_PID.KP=3.7;
	m3508->M3508_Speed_PID.KI=0.1;
	m3508->M3508_Speed_PID.KD=0;
	m3508->M3508_Speed_PID.lasterror=0;
	m3508->M3508_Speed_PID.lastlasterror=0;
		m3508->M3508_Speed_PID.lastv=0;
		m3508->M3508_Speed_PID.lastlastv=0;
}


void PID_Chassis_Speed_Init(M3508_STU*m3508){
	
	m3508->M3508_Speed_PID.KP=3.7;
	m3508->M3508_Speed_PID.KI=0.1;
	m3508->M3508_Speed_PID.KD=0;
	m3508->M3508_Speed_PID.lasterror=0;
	m3508->M3508_Speed_PID.lastlasterror=0;
		m3508->M3508_Speed_PID.lastv=0;
		m3508->M3508_Speed_PID.lastlastv=0;
}


void PID_Position_Init(M3508_STU*m3508){
	
	m3508->M3508_Angle_PID.KP=0.5;
	m3508->M3508_Angle_PID.KI=0;
	m3508->M3508_Angle_PID.KD=0;
		m3508->M3508_Angle_PID.sum=0;
		m3508->M3508_Angle_PID.lasterror=0;
		m3508->M3508_Angle_PID.lasta=0;
}

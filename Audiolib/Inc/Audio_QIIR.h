/*
 * Audio_IIR.h
 *
 *  Created on: Dec 11, 2024
 *      Author: Garin Anggara
 */

#ifndef AUDIO_QIIR_H
#define AUDIO_QIIR_H

#include "main.h"
#include "Audio_IO.h"
#include "math.h"

#define PIE2	(2.0f*3.14159265359f)

typedef enum qiir_siwtch {QIIR_OFF, QIIR_ON} QIIR_Switch;
typedef enum qiir_type {HIGHPASS, LOWPASS, BELL, HI_SHELF, LO_SHELF} QIIR_Type;

typedef struct{
	float 		*Control_Signal;	//Array of knob/potentiometer value scaled 0 to 1
	uint8_t 	Current_Knob_ID;	//Current knob/potentiometer assignment
	Run_Status  Curr_Run_Sts;
	Run_Status  Prv_Run_State;
	Run_Status  CR_Status;
	Enable_Ctrl Enable_Control;
}QIIR_Ctrl_Signal;

typedef struct{
	float 		a[3];
	float 		b[3];
	float 		Inp_Buff[3];
	float 		Out_Buff[3];
	float 		Lvl_Inp;
	float 		Lvl_Out;
	float 		Gain;
	float		Gain_Backup;
	float 		F0;
	float		F0_Backup;
	float 		Q;
	float		Q_Backup;
	float 		w0;
	float 		Alpha;
	float 		Fs;
	float 		Ts;
	QIIR_Switch Switch;
	QIIR_Type	Filter_Type;
	uint8_t		Process_ID;
	QIIR_Ctrl_Signal Control;

}QIIR_Struct;

float 	Process_QIIR_Filter(QIIR_Struct *QIIRobj, float Inp);
void 	Update_QIIR_Filter(QIIR_Struct *QIIRobj);
void 	Init_QIIR_Filter(QIIR_Struct *QIIRobj, QIIR_Type Set_Filter_Type, float Set_F0,
						float Set_Q, float Set_Gain, float Set_Fs, QIIR_Switch Set_Enable, uint8_t Set_Process_ID, Enable_Ctrl Set_Enable_Control);


#endif /* AUDIO_QIIR_H */

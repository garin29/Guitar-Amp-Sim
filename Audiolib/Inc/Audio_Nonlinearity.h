/*
 * Audio_Dist.h
 *
 *  Created on: Dec 11, 2024
 *      Author: Garin Anggara
 */

#ifndef AUDIO_NONLINEARITY_H
#define AUDIO_NONLINEARITY_H

#include "main.h"
#include "math.h"
#include "Audio_IO.h"

typedef enum nlin_type {DRIVE, COMPRESSOR, LIMITTER} NLIN_Type;
typedef enum nlin_switch {NLIN_OFF, NLIN_ON} NLIN_Switch;
typedef enum drive_switch {DRIVE_OFF, DRIVE_ON} Drive_Switch;

typedef struct{
	float 		*Control_Signal;	//Array of knob/potentiometer value scaled 0 to 1
	uint8_t 	Current_Knob_ID;	//Current knob/potentiometer assignment
	Run_Status  Curr_Run_Sts;
	Run_Status  Prv_Run_State;
	Run_Status  CR_Status;
	Enable_Ctrl Enable_Control;
}Nlin_Ctrl_Signal;


typedef struct{
	float 		Inp_Lvl;
	float		Inp_Lvl_Backup;
	float 		Out_Lvl;
	float 		Pos_Slope;
	float 		Neg_Slope;
	float 		Pos_Lim;
	float 		Neg_Lim;
	Drive_Switch Switch;
	NLIN_Type 	Type;
	uint8_t		Process_ID;
	Nlin_Ctrl_Signal Control;
}Drive_Struct;

float Process_Drive(Drive_Struct *Drvobj, float Inp);
void Update_Drive(Drive_Struct *Drvobj);
void Init_Drive(Drive_Struct *Drvobj, float Set_Inp_Lvl, float Set_Out_Lvl, float Set_Pos_Lim,
		        float Set_Neg_Lim, float Set_Pos_Slope, float Set_Neg_Slope, Drive_Switch Set_Enable, uint8_t Set_Process_ID, Enable_Ctrl Set_Enable_Control);

#endif /* AUDIO_NONLINEARITY_H */

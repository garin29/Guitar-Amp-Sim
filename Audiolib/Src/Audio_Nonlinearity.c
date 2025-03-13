/*
 * Audio_Nonlinearity.c
 *
 *  Created on: Dec 13, 2024
 *      Author: Garin Anggara
 */


#include "Audio_Nonlinearity.h"

void Update_Drive(Drive_Struct *Drvobj){
	if ((Drvobj->Switch == DRIVE_ON) && (Drvobj->Control.Enable_Control == CTRL_ON )){
			if ((Drvobj->Control.Current_Knob_ID ==  Drvobj->Process_ID) && (Drvobj->Control.Curr_Run_Sts == ON_EDIT)){
				Drvobj->Inp_Lvl = 5.0f + (25* Drvobj->Control.Control_Signal[0]);
			}

			if ((Drvobj->Control.Current_Knob_ID !=  Drvobj->Process_ID) || (Drvobj->Control.Curr_Run_Sts == ON_RUN)){
				if (Drvobj->Control.Prv_Run_State == ON_RUN){
				Drvobj->Inp_Lvl_Backup = Drvobj->Inp_Lvl;
				Drvobj->Control.Curr_Run_Sts = Drvobj->Control.Curr_Run_Sts;
				Drvobj->Control.CR_Status = ON_RUN;
				}
			}

			if ((Drvobj->Control.Curr_Run_Sts == CANCEL_CHANGES)){
				if (Drvobj->Control.Prv_Run_State == ON_EDIT){
				Drvobj->Inp_Lvl = Drvobj->Inp_Lvl_Backup;
				Drvobj->Control.CR_Status = DONE_CANCEL;
				}
			}
			if ((Drvobj->Control.Current_Knob_ID ==  Drvobj->Process_ID) && (Drvobj->Control.Curr_Run_Sts == APPLY_CHANGES)){
				Drvobj->Inp_Lvl_Backup = Drvobj->Inp_Lvl;
				Drvobj->Control.CR_Status = DONE_APPLY;
			}
			Drvobj->Control.Prv_Run_State = Drvobj->Control.Curr_Run_Sts;
	}
}

float Process_Drive(Drive_Struct *Drvobj, float Inp){
	float Out = 0.0f;
	float Scaled_Inp = 0.0f;

	if (Drvobj->Switch == DRIVE_ON){
		Scaled_Inp = Drvobj->Inp_Lvl * Inp;
		if (Inp >= 0.0f){
			Out =    Drvobj->Pos_Lim - (Drvobj->Pos_Lim* exp(-1.0f*Drvobj->Pos_Slope * Scaled_Inp));
		}
		else{
			Out = -1.0f*Drvobj->Pos_Lim + (Drvobj->Pos_Lim* exp(Drvobj->Neg_Slope * Scaled_Inp));
		}
		Out = Out * Drvobj->Out_Lvl;
	}
	else{
		Out = Inp;
	}

	return Out;
}

void Init_Drive(Drive_Struct *Drvobj, float Set_Inp_Lvl, float Set_Out_Lvl, float Set_Pos_Lim,
		        float Set_Neg_Lim, float Set_Pos_Slope, float Set_Neg_Slope, Drive_Switch Set_Enable, uint8_t Set_Process_ID, Enable_Ctrl Set_Enable_Control){
	Drvobj->Inp_Lvl = Set_Inp_Lvl;
	Drvobj->Inp_Lvl_Backup = Set_Inp_Lvl;
	Drvobj->Out_Lvl = Set_Out_Lvl;
	Drvobj->Pos_Lim = Set_Pos_Lim;
	Drvobj->Neg_Lim = Set_Neg_Lim;
	Drvobj->Pos_Slope = Set_Pos_Slope;
	Drvobj->Neg_Slope = Set_Neg_Slope;
	Drvobj->Switch = Set_Enable;
	Drvobj->Type = DRIVE;
	Drvobj->Process_ID = Set_Process_ID;
	Drvobj->Control.Enable_Control = Set_Enable_Control;

	for (uint8_t i = 1; i < 4; i++){
		Drvobj->Control.Control_Signal[i] = 0;
	}
};

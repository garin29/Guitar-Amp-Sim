/*
 * Audio_FIR.c
 *
 *  Created on: Dec 11, 2024
 *      Author: Garin Anggara
 */


#include "Audio_FIR.h"

void Init_FIR_Filter(FIR_Struct *FIRobj, uint16_t Set_Coef_Size, float *pSet_Coefs_Address, float *pSet_Buffer_Address, FIR_Switch Set_Enable, uint8_t Set_Process_ID){
	FIRobj->Coef_Size 	= Set_Coef_Size;		//Size of filter coefficient
	FIRobj->pCoefs 		= pSet_Coefs_Address;	//Address to 1st member of filter coefficient
	FIRobj->pBuffers	= pSet_Buffer_Address;	//Address to 1st member of filter buffer
	FIRobj->Lvl_Inp		= 1;					//Default initialization to 1
	FIRobj->Lvl_Out		= 1;					//Default initialization to 1
	FIRobj->Buf_Idx		= 0;					//Default initialization to 1
	FIRobj->Switch		= Set_Enable;			//ENABLE (1) or DISABLE (0)
	FIRobj->Process_ID 	= Set_Process_ID;

	//Init FIR buffer to 0
	for (uint16_t n = 0; n < Set_Coef_Size; n++){
		pSet_Buffer_Address[n] = 0;
	}
}

float Process_FIR_Filter(FIR_Struct *FIRobj, float Inp){
    static uint16_t Convo_Idx = 0;
    static float Out = 0.0f;

	Out = 0.0f;
	if (FIRobj->Switch == FIR_ON){
		FIRobj->pBuffers[FIRobj->Buf_Idx] = Inp;
		FIRobj->Buf_Idx++;
		if (FIRobj->Buf_Idx == FIRobj->Coef_Size){
			FIRobj->Buf_Idx = 0;
		}
		Convo_Idx = FIRobj->Buf_Idx;
		for (uint16_t n = 0; n < FIRobj->Coef_Size; n++){
			if (Convo_Idx> 0){
				Convo_Idx--;
			}
			else{
				Convo_Idx = FIRobj->Coef_Size - 1;
			}
			Out = Out + (FIRobj->pCoefs[n] * FIRobj->pBuffers[Convo_Idx]);
			//Out = FIRobj->Lvl_Out;
		}
	}
	else{
		Out = Inp;
	}

	return Out;
}

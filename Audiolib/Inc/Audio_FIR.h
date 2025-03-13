/*
 * Audio_FIR.h
 *
 *  Created on: Dec 11, 2024
 *      Author: Garin Anggara
 */

#ifndef AUDIO_FIR_H
#define AUDIO_FIR_H

#include "main.h"

typedef enum fir_switch {FIR_OFF, FIR_ON} FIR_Switch;

typedef struct {
	uint16_t		Coef_Size;
	float			*pCoefs;
	float			*pBuffers;
	float			Lvl_Inp;
	float			Lvl_Out;
	uint16_t		Buf_Idx;
	FIR_Switch		Switch;
	uint8_t			Process_ID;
}FIR_Struct;

float Process_FIR_Filter(FIR_Struct *FIRobj, float Inp);
void Init_FIR_Filter(FIR_Struct *FIRobj, uint16_t Set_Coef_Size, float *pSet_Coefs_Address, float *pSet_Buffer_Address, FIR_Switch Set_Enable, uint8_t Set_Process_ID);

#endif /* AUDIO_FIR */

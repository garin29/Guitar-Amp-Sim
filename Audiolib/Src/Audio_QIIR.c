/*
 * Audio_QIIR.c
 *
 *  Created on: Dec 12, 2024
 *      Author: Garin Anggara
 */


#include "Audio_QIIR.h"


float Process_QIIR_Filter(QIIR_Struct *QIIRobj, float Inp){
	float Out = 0.0f;

	if (QIIRobj->Switch == QIIR_ON){
		QIIRobj->Inp_Buff[2] = QIIRobj->Inp_Buff[1];
		QIIRobj->Inp_Buff[1] = QIIRobj->Inp_Buff[0];
		QIIRobj->Inp_Buff[0] = Inp;

		QIIRobj->Out_Buff[2] = QIIRobj->Out_Buff[1];
		QIIRobj->Out_Buff[1] = QIIRobj->Out_Buff[0];
		QIIRobj->Out_Buff[0] = QIIRobj->a[0] * ((QIIRobj->b[0] * QIIRobj->Inp_Buff[0]) +
												(QIIRobj->b[1] * QIIRobj->Inp_Buff[1]) +
												(QIIRobj->b[2] * QIIRobj->Inp_Buff[2]) -
												(QIIRobj->a[1] * QIIRobj->Out_Buff[1]) -
												(QIIRobj->a[2] * QIIRobj->Out_Buff[2]) );
		Out = QIIRobj->Out_Buff[0];
	}
	else{
		Out = Inp;
	}

	return Out;
}


void Update_QIIR_Filter(QIIR_Struct *QIIRobj){
	if ((QIIRobj->Switch == QIIR_ON) ){
		if (QIIRobj->Control.Enable_Control == CTRL_ON){
			if ((QIIRobj->Control.Current_Knob_ID ==  QIIRobj->Process_ID) && (QIIRobj->Control.Curr_Run_Sts == ON_EDIT)){
				QIIRobj->Gain = 0.1f + (5* QIIRobj->Control.Control_Signal[0]);
				QIIRobj->F0 = 30.0f + (18000.0f * QIIRobj->Control.Control_Signal[1]);
				QIIRobj->Q  = 0.1f + (5.0f * QIIRobj->Control.Control_Signal[2]);
			}

			if ((QIIRobj->Control.Current_Knob_ID !=  QIIRobj->Process_ID) || (QIIRobj->Control.Curr_Run_Sts == ON_RUN)){
				if (QIIRobj->Control.Prv_Run_State == ON_RUN){
				QIIRobj->Gain_Backup = QIIRobj->Gain;
				QIIRobj->F0_Backup = QIIRobj->F0;
				QIIRobj->Q_Backup = QIIRobj->Q;
				QIIRobj->Control.Curr_Run_Sts = QIIRobj->Control.Curr_Run_Sts;
				QIIRobj->Control.CR_Status = ON_RUN;
				}
			}

			if ((QIIRobj->Control.Curr_Run_Sts == CANCEL_CHANGES)){
				if (QIIRobj->Control.Prv_Run_State == ON_EDIT){
				QIIRobj->Gain = QIIRobj->Gain_Backup;
				QIIRobj->F0 = QIIRobj->F0_Backup;
				QIIRobj->Q = QIIRobj->Q_Backup;
				QIIRobj->Control.CR_Status = DONE_CANCEL;
				}
			}
			if ((QIIRobj->Control.Current_Knob_ID ==  QIIRobj->Process_ID) && (QIIRobj->Control.Curr_Run_Sts == APPLY_CHANGES)){
				QIIRobj->Gain_Backup = QIIRobj->Gain;
				QIIRobj->F0_Backup = QIIRobj->F0;
				QIIRobj->Q_Backup = QIIRobj->Q;
				QIIRobj->Control.CR_Status = DONE_APPLY;
			}
			QIIRobj->Control.Prv_Run_State = QIIRobj->Control.Curr_Run_Sts;
		}
		switch (QIIRobj->Filter_Type){
		case HIGHPASS:
			QIIRobj->w0 	= (PIE2*QIIRobj->F0)/QIIRobj->Fs;
			QIIRobj->Alpha 	= (sin(QIIRobj->w0))/(2*QIIRobj->Q);
			QIIRobj->b[0] 	= 0.5f * (1+cos(QIIRobj->w0));
			QIIRobj->b[1] 	= -2.0f*QIIRobj->b[0];
			QIIRobj->b[2]	= QIIRobj->b[0];
			QIIRobj->a[0]	= 1.0f/(1+QIIRobj->Alpha);
			QIIRobj->a[1]	= -2.0f*cos(QIIRobj->w0);
			QIIRobj->a[2]	= 1.0f - QIIRobj->Alpha;
			break;

		case LOWPASS:
			QIIRobj->w0 	= (PIE2*QIIRobj->F0)/QIIRobj->Fs;
			QIIRobj->Alpha 	= (sin(QIIRobj->w0))/(2*QIIRobj->Q);
			QIIRobj->b[0] 	= 0.5f * (1-cos(QIIRobj->w0));
			QIIRobj->b[1] 	= 2.0f*QIIRobj->b[0];
			QIIRobj->b[2]	= QIIRobj->b[0];
			QIIRobj->a[0]	= 1.0f/(1+QIIRobj->Alpha);
			QIIRobj->a[1]	= -2.0f*cos(QIIRobj->w0);
			QIIRobj->a[2]	= 1.0f - QIIRobj->Alpha;
			break;

		case BELL:
			QIIRobj->w0 	= (PIE2*QIIRobj->F0)/QIIRobj->Fs;
			QIIRobj->Alpha 	= (sin(QIIRobj->w0))/(2*QIIRobj->Q);
			QIIRobj->b[0] 	= 1.0f + (QIIRobj->Alpha * QIIRobj->Gain);
			QIIRobj->b[1] 	= -2.0f*(cos(QIIRobj->w0));
			QIIRobj->b[2]	= 1.0f - (QIIRobj->Alpha * QIIRobj->Gain);
			QIIRobj->a[0]	= 1.0f / (1+ (QIIRobj->Alpha/QIIRobj->Gain));
			QIIRobj->a[1]	= QIIRobj->b[1];
			QIIRobj->a[2]	= 1.0f - (QIIRobj->Alpha/QIIRobj->Gain);
			break;

		case HI_SHELF:
			QIIRobj->w0 	= (PIE2*QIIRobj->F0)/QIIRobj->Fs;
			QIIRobj->Alpha 	= (sin(QIIRobj->w0))/(2*QIIRobj->Q);
			QIIRobj->b[0] 	=    QIIRobj->Gain * ((QIIRobj->Gain+1) + ((QIIRobj->Gain-1)*cos(QIIRobj->w0)) + (QIIRobj->Alpha*2*sqrt(QIIRobj->Gain)) ) ;
			QIIRobj->b[1] 	= -2*QIIRobj->Gain * ((QIIRobj->Gain-1) + ((QIIRobj->Gain+1)*cos(QIIRobj->w0)) ) ;
			QIIRobj->b[2]	=	 QIIRobj->Gain * ((QIIRobj->Gain+1) + ((QIIRobj->Gain-1)*cos(QIIRobj->w0)) - (QIIRobj->Alpha*2*sqrt(QIIRobj->Gain)) ) ;
			QIIRobj->a[0]	=  1/                 (QIIRobj->Gain+1) - ((QIIRobj->Gain-1)*cos(QIIRobj->w0)) + (QIIRobj->Alpha*2*sqrt(QIIRobj->Gain));
			QIIRobj->a[1]	=  2*                ((QIIRobj->Gain-1) - ((QIIRobj->Gain+1)*cos(QIIRobj->w0)) ) ;
			QIIRobj->a[2]	= 					  (QIIRobj->Gain+1) - ((QIIRobj->Gain-1)*cos(QIIRobj->w0)) - (QIIRobj->Alpha*2*sqrt(QIIRobj->Gain));
			break;

		case LO_SHELF:
			QIIRobj->w0 	= (PIE2*QIIRobj->F0)/QIIRobj->Fs;
			QIIRobj->Alpha 	= (sin(QIIRobj->w0))/(2*QIIRobj->Q);
			QIIRobj->b[0] 	=    QIIRobj->Gain * ((QIIRobj->Gain+1) - ((QIIRobj->Gain-1)*cos(QIIRobj->w0)) + (QIIRobj->Alpha*2*sqrt(QIIRobj->Gain)) ) ;
			QIIRobj->b[1] 	=  2*QIIRobj->Gain * ((QIIRobj->Gain-1) - ((QIIRobj->Gain+1)*cos(QIIRobj->w0)) ) ;
			QIIRobj->b[2]	=	 QIIRobj->Gain * ((QIIRobj->Gain+1) - ((QIIRobj->Gain-1)*cos(QIIRobj->w0)) - (QIIRobj->Alpha*2*sqrt(QIIRobj->Gain)) ) ;
			QIIRobj->a[0]	=  1/                 (QIIRobj->Gain+1) + ((QIIRobj->Gain-1)*cos(QIIRobj->w0)) + (QIIRobj->Alpha*2*sqrt(QIIRobj->Gain));
			QIIRobj->a[1]	= -2*                ((QIIRobj->Gain-1) + ((QIIRobj->Gain+1)*cos(QIIRobj->w0)) ) ;
			QIIRobj->a[2]	= 					  (QIIRobj->Gain+1) + ((QIIRobj->Gain-1)*cos(QIIRobj->w0)) - (QIIRobj->Alpha*2*sqrt(QIIRobj->Gain));
			break;

		}
	}
}


void Init_QIIR_Filter(QIIR_Struct *QIIRobj, QIIR_Type Set_Filter_Type, float Set_F0, float Set_Q, float Set_Gain, float Set_Fs, QIIR_Switch Set_Enable, uint8_t Set_Process_ID, Enable_Ctrl Set_Enable_Control){

	for (uint8_t i = 0; i < 3; i++){
		QIIRobj->a[i] = 0.0f;
		QIIRobj->b[i] = 0.0f;
		QIIRobj->Inp_Buff[i] = 0.0f;
		QIIRobj->Out_Buff[i] = 0.0f;

	}
	QIIRobj->Lvl_Inp = 1.0f;
	QIIRobj->Lvl_Out = 1.0f;

	QIIRobj->F0 = Set_F0;
	QIIRobj->F0_Backup = Set_F0;
	QIIRobj->Q	= Set_Q;
	QIIRobj->Q_Backup	= Set_Q;
	QIIRobj->Gain = Set_Gain;
	QIIRobj->Gain_Backup = Set_Gain;
	QIIRobj->Fs = Set_Fs;
	QIIRobj->Filter_Type = Set_Filter_Type;
	QIIRobj->Switch = Set_Enable;
	QIIRobj->Process_ID = Set_Process_ID;
	QIIRobj->Control.Current_Knob_ID = 0;
	QIIRobj->Control.Curr_Run_Sts = ON_RUN;
	QIIRobj->Control.CR_Status = ON_RUN;
	QIIRobj->Control.Prv_Run_State = ON_RUN;
	QIIRobj->Control.Enable_Control = Set_Enable_Control;

	for (uint8_t i = 1; i < 4; i++){
		QIIRobj->Control.Control_Signal[i] = 0.0f;
	}
}

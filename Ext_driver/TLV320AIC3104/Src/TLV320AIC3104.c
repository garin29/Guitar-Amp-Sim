/*
 * TLV320AIC3104.c
 *
 *  Created on: Dec 7, 2024
 *      Author: Garin Anggara
 */

#include "TLV320AIC3104.h"

uint8_t AIC320_Init(AIC320_CTL_OBJ *codec){
	HAL_StatusTypeDef status = 0;
	uint8_t init_sts = 0;

	//1. Hardware reset
	AIC320_Reset();

	//2. Software reset
	status = AIC320_RegWrite(codec, AIC320_REG_SW_RESET, 0x80);
	if (status != HAL_OK){
		init_sts = 1;
	}

	//3. Determine fs
	status = AIC320_RegWrite(codec, AIC320_REG_SAMPLE_RATE, 0x00);
	if (status != HAL_OK){
		init_sts = 2;
	}

	status = AIC320_RegWrite(codec, AIC320_REG_PLL_QP, 0x10 );
	if (status != HAL_OK){
		init_sts = 3;
	}

	status = AIC320_RegWrite(codec, AIC320_REG_DATA_PATH, 0x08);
	if (status != HAL_OK){
		init_sts = 7;
	}
	status = AIC320_RegWrite(codec, AIC320_REG_AUDIO_DATA_INTERFACE_CTL2, 0x20 );
	if (status != HAL_OK){
		init_sts = 9;
	}

	status = AIC320_RegWrite(codec, AIC320_REG_AUDIO_DATA_INTERFACE_CTL3, 0x00 );
		if (status != HAL_OK){
			init_sts = 9;
		}

	status = AIC320_RegWrite(codec, AIC320_ADC_PGA_LEFT, 0x00 );
	if (status != HAL_OK){
		init_sts = 15;
	}

	status = AIC320_RegWrite(codec, AIC320_ADC_PGA_RIGHT, 0x80 );
	if (status != HAL_OK){
		init_sts = 16;
	}

	status = AIC320_RegWrite(codec, AIC320_MIC2LR_TO_LEFT_ADC, 0x0F);
	if (status != HAL_OK){
		init_sts = 17;
	}

	status = AIC320_RegWrite(codec, AIC320_MIC1LP_LINE1LP_TO_LEFT_ADC, 0x7C);
	if (status != HAL_OK){
		init_sts = 19;
	}

	status = AIC320_RegWrite(codec, AIC320_DAC_PWR, 0x80);
	if (status != HAL_OK){
		init_sts = 37;
	}

	status = AIC320_RegWrite(codec, AIC320_HI_POWER_OUTPUT_STAGE, 0x80);
	if (status != HAL_OK){
		init_sts = 40 ;
	}

	status = AIC320_RegWrite(codec, AIC320_LEFT_DAC_VOL, 0x00);
	if (status != HAL_OK){
		init_sts = 49;
	}

	status = AIC320_RegWrite(codec, AIC320_DAC_L1_TO_LEFTLOPM_VOL, 0x80);
	if (status != HAL_OK){
		init_sts = 82;
	}

	status = AIC320_RegWrite(codec, AIC320_LEFTLOPM_LEVEL, 0x09);
	if (status != HAL_OK){
		init_sts = 86;
	}

	status = AIC320_RegWrite(codec, AIC320_CLOCK_SELECT, 0x01)	;
	if (status != HAL_OK){
		init_sts = 101;
	}


	return init_sts;

}


HAL_StatusTypeDef AIC320_RegWrite(AIC320_CTL_OBJ *codec, uint8_t RegAddr, uint8_t RegData){
	return HAL_I2C_Mem_Write(codec->I2C_Handler, AIC320_DEVICE_ADDRESS, RegAddr, 1,
			         	 	 &RegData, AIC320_I2C_REG_DATA_SIZE, HAL_MAX_DELAY);
}


HAL_StatusTypeDef AIC320_RegRead(AIC320_CTL_OBJ *codec, uint8_t RegAddr, uint8_t *RegData){
	return HAL_I2C_Mem_Read(codec->I2C_Handler, AIC320_DEVICE_ADDRESS, RegAddr, 1,
			                RegData, AIC320_I2C_REG_DATA_SIZE, HAL_MAX_DELAY);
}


void AIC320_Reset(){
	HAL_GPIO_WritePin(CODEC_NRST_GPIO_Port, CODEC_NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(CODEC_NRST_GPIO_Port, CODEC_NRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(CODEC_NRST_GPIO_Port, CODEC_NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
}

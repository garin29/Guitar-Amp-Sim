/*
 * TLV320AIC3104.h
 *
 *  Created on: Dec 7, 2024
 *      Author: Garin Anggara
 */



#ifndef TLV320AIC3104_H
#define TLV320AIC3104_H

#include "main.h"
#include "stm32h7xx_hal.h"

#define AIC320_DEVICE_ADDRESS					(0x18<<1) //7-bit device address is 001 1000 --> 1 shift to the left make it 8 bit of 0011 0000
#define AIC320_NRST_PIN							CODEC_NRST_Pin
#define AIC320_NRST_PORT 						CODEC_NRST_GPIO_Port
#define AIC320_I2C_REG_ADDR_SIZE				8
#define AIC320_I2C_REG_DATA_SIZE				1
#define AIC320_I2C_TIMEOUT						100

#define AIC320_REG_PAGE_SELECT					0x00
#define AIC320_REG_SW_RESET						0x01

#define AIC320_REG_SAMPLE_RATE 					0x02
#define AIC320_REG_PLL_QP						0x03
#define AIC320_REG_PLL_J						0x04
#define AIC320_REG_PLL_D_MSB					0x05
#define AIC320_REG_PLL_D_LSB					0x06

#define AIC320_REG_DATA_PATH					0x07
#define AIC320_REG_AUDIO_DATA_INTERFACE_CTL1	0x08
#define AIC320_REG_AUDIO_DATA_INTERFACE_CTL2	0x09
#define AIC320_REG_AUDIO_DATA_INTERFACE_CTL3	0x0A
#define AIC320_REG_OVF_FLAG_PLL_R  				0x0B
#define AIC320_DIGITAL_FILTER_CTL				0x0C
#define AIC320_HEADSET_DETECT_1  				0x0D
#define AIC320_HEADSET_DETECT_2					0x0E
#define AIC320_ADC_PGA_LEFT						0x0F
#define AIC320_ADC_PGA_RIGHT					0x10
#define AIC320_MIC2LR_TO_LEFT_ADC				0x11
#define	AIC320_MIC2_LINE2_TO_RIGHT_ADC			0x12
#define	AIC320_MIC1LP_LINE1LP_TO_LEFT_ADC		0x13
#define AIC320_MIC1RP_LINE1RP_TO_LEFT_ADC		0x15
#define AIC320_MIC1RP_LINE1RP_TO_RIGHT_ADC		0x16
#define AIC320_MIC1LP_LINE1LP_TO_RIGHT_ADC		0x18
#define AIC320_MICBIAS							0x19
#define AIC320_LEFT_AGC_CTL1					0x1A
#define AIC320_LEFT_AGC_CTL2					0x1B
#define AIC320_LEFT_AGC_CTL3					0x1C
#define AIC320_RIGHT_AGC_CTL1					0x1D
#define AIC320_RIGHT_AGC_CTL2					0x1E
#define AIC320_RIGHT_AGC_CTL3					0x1F
#define AIC320_LEFT_AGC_GAIN					0x20
#define AIC320_RIGHT_AGC_GAIN					0x21
#define AIC320_ADC_FLAG							0x24
#define	AIC320_DAC_PWR 							0x25
#define AIC320_HI_POWER_OUTPUT_DRIVER			0x26
#define AIC320_HI_POWER_OUTPUT_STAGE			0x28
#define	AIC320_DAC_OUT_SWITCH					0x29
#define	AIC320_OUT_DRV_POP						0x2A
#define	AIC320_LEFT_DAC_VOL						0x2B
#define AIC320_RIGHT_DAC_VOL					0x2C

//HPLOUT Configuration Address
#define AIC320_PGA_L_TO_HPLOUT_VOL				0x2E
#define	AIC320_DAC_L1_TO_HPLOUT_VOL				0x2F
#define AIC320_PGA_R_TO_HPLOUT_VOL				0x31
#define	AIC320_DAC_R1_TO_HPLOUT_VOL				0x32
#define	AIC320_HPLOUT_LEVEL						0x33
#define AIC320_PGA_L_TO_HPLCOM_VOL				0x35
#define	AIC320_DAC_L1_TO_HPLCOM_VOL				0x36
#define AIC320_PGA_R_TO_HPLCOM_VOL				0x38
#define AIC320_DAC_R1_TO_HPLCOM_VOl				0x39
#define AIC320_HPLCOM_LEVEL						0x3A

//HPROUT COnfiguration Address
#define AIC320_PGA_L_TO_HPROUT_VOL				0x3C
#define	AIC320_DAC_L1_TO_HPROUT_VOL				0x3D
#define AIC320_PGA_R_TO_HPROUT_VOL				0x3F
#define	AIC320_DAC_R1_TO_HPROUT_VOL				0x40
#define	AIC320_HPROUT_LEVEL						0x41
#define AIC320_PGA_L_TO_HPRCOM_VOL				0x43
#define	AIC320_DAC_L1_TO_HPRCOM_VOL				0x44
#define AIC320_PGA_R_TO_HPRCOM_VOL				0x46
#define AIC320_DAC_R1_TO_HPRCOM_VOl				0x47
#define AIC320_HPRCOM_LEVEL						0x48

//LOP/M Configuration Address
#define AIC320_PGA_L_TO_LEFTLOPM_VOL			0x51
#define	AIC320_DAC_L1_TO_LEFTLOPM_VOL			0x52
#define AIC320_PGA_R_TO_LEFTLOPM_VOL			0x54
#define	AIC320_DAC_R1_TO_LEFTLOPM_VOL			0x55
#define	AIC320_LEFTLOPM_LEVEL					0x56
#define AIC320_PGA_L_TO_RIGHTLOPM_VOL			0x58
#define	AIC320_DAC_L1_TO_RIGHTLOPM_VOL			0x59
#define AIC320_PGA_R_TO_RIGHTLOPM_VOL			0x5B
#define AIC320_DAC_R1_TO_RIGHTLOPM_VOl			0x5C
#define AIC320_RIGHTLOPM_LEVEL					0x5D


#define AIC320_MODULE_POWER_STATUS				0x5E
#define AIC320_OUTDRV_SHORT_DETECT				0x5F
#define AIC320_STICKY_INTERRUPT_FLG				0x60
#define	AIC320_REALT_TIME_INTERRUPT_FLAG		0x61
#define AIC320_CLOCK_SELECT						0x65
#define AIC320_CLOCK_GEN_CTL					0x66


typedef struct{
	I2C_HandleTypeDef 	*I2C_Handler;
	//uint8_t 			*Left_ADC_PWR_Status;
	//uint8_t				*Right_ADC_PWR_Status;
	//uint8_t				*Left_DAC_PWR_Status;
	//uint8_t				*Right_DAC_PWR_Status;
} AIC320_CTL_OBJ;

uint8_t AIC320_Init(AIC320_CTL_OBJ *codec);
void 	AIC320_Reset();

HAL_StatusTypeDef AIC320_RegWrite(AIC320_CTL_OBJ *codec, uint8_t RegAddr, uint8_t RegData);
HAL_StatusTypeDef AIC320_RegRead(AIC320_CTL_OBJ *codec, uint8_t RegAddr, uint8_t *RegData);

#endif /* TLV320AIC3104_H*/

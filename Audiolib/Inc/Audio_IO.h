/*
 * Audio_IO.h
 *
 *  Created on: Dec 11, 2024
 *      Author: Garin Anggara
 */

#ifndef AUDIO_IO_H
#define AUDIO_IO_H

#include "main.h"

#define	ENABLE			1
#define DISABLE			0

typedef enum btn_action {BTN_IDLE, BTN_SELECT, BTN_ENTER, BTN_CANCEL} Btn_Action;
typedef enum run_status {ON_RUN, ON_EDIT, ON_SELECTION, APPLY_CHANGES, DONE_APPLY, CANCEL_CHANGES, DONE_CANCEL} Run_Status;
typedef enum enable_ctrl {CTRL_OFF, CTRL_ON} Enable_Ctrl;

#endif /* AUDIO_IO_H */

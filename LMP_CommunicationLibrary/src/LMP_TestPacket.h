#pragma once
/**
  ******************************************************************************
  * @file    GKV_TestPacket.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 structures module for request/response packets of device parameters
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Laboratory of Microdevices, Ltd. </center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Laboratory of Microdevices nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifndef __GKV_TEST_PACKET_H__
#define __GKV_TEST_PACKET_H__



/** @defgroup ID_PACKET_CODES
  * @brief    packet codes for "type" field of ID request/response packets
  * @{
  */ 
#define GKV_DEV_ID_REQUEST 								0x04								/*	type of empty packet (with length = 0) to request device status packet	*/
#define GKV_DEV_ID_PACKET								0x05								/*	type of packet with device status	*/
/**
  * @}
  */



/** @defgroup RUNNING_PROGRAM_STATUS
  * @brief 	device mode for "mode" field of response packet 0x05 (identification device parameters)
  * @{
  */ 
#define GKV_MODE_BOOTLOADER_ERR 							0x00                                /*  bootloader mode error (no working firmware)    */
#define GKV_MODE_BOOTLOADER_OK 							    0x01                                /*  bootloader mode ok (no working firmware)    */
#define GKV_MODE_WORK 								        0x02                                /*  working firmware mode   */
#define GKV_MODE_INAPP_BOOTLOADER 							0x03                                /*  bootloader mode with working firmware   */
/**
  * @}
  */



#ifndef __GKV_DEVICE_STATUS__
#define __GKV_DEVICE_STATUS__
/** @defgroup DEVICE_STATUS
  * @brief 	status_bits for "status" field of each response packet	
  * @{
  */ 
#define GKV_GNSS_TIMESTAMP 									1<<11								/* synchronization bit for GNSS receiver	*/
#define GKV_DEV_SETTING_FINISHED 							1<<10								/* set as 1 after defice selfconfiguration finish (in orientation or navigation algorithms)	*/
#define GKV_AZ_ERROR 										1<<9								/*  accelerometer X axis hardware error  */
#define GKV_AY_ERROR 										1<<8								/*  accelerometer Y axis hardware error  */
#define GKV_AX_ERROR 										1<<7                                /*  accelerometer Z axis hardware error  */
#define GKV_WZ_ERROR 										1<<6                                /*  rate sensor X axis hardware error  */
#define GKV_WY_ERROR 										1<<5                                /*  rate sensor Y axis hardware error  */
#define GKV_WX_ERROR 										1<<4                                /*  rate sensors Z axis hardware error  */
#define GKV_ADC_ERROR 							    		1<<3                                /*  ADC hardware error  */
#define GKV_ADC_LOST_DATA 									1<<2                                /*  ADC software error  */
#define GKV_BUFFER_OVERRUN 									1<<1                                /*  Data Submission Queue Overflow  */
#define GKV_SYNC_INPUT 										1									/*	1 - sync pin voltage > 2,5V, 0 - sync pin voltage < 0.5 V	*/
/**
  * @}
  */
#endif


/** 
  * @brief  Response packet 0x05 with main id parameters of device (receive only)
  */
typedef struct __GKV_ID
{

	uint16_t bootloader_version;                            /*	version of embended software	*/
	uint16_t firmware_version;                              /*	version of changeable software	*/
	uint32_t production_date;                               /*	date of device manufacturing	*/
	char serial_id[16];                                     /*	serial number of device	*/
	char description[16];                                   /*	device code	in ASCII */
	uint8_t mode;                                           /*	used in the  manufacturing phase	*/
	uint16_t status;                                        /*	field for detecting errors, sync and algorithm state	*/
}GKV_ID;


#endif
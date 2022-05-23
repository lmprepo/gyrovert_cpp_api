#pragma once
/**
  ******************************************************************************
  * @file    GKV_BasePacket.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   Definition of template structure of each request/response packet for GKV-10 and structure of testing connection packet "data" field
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
#ifndef __GKV_BASE_PACKET_H__
#define __GKV_BASE_PACKET_H__


/** @defgroup MAIN PACKET PARAMETERS
  * @brief    parameters of each packet (preamble, device address, max length of 'data' field)
  * @{
  */ 
#define GKV_DATA_LENGTH  255                /*  max length of field "data" of each packet */
#define GKV_PREAMBLE_VALUE  255             /*  beginning of each packet */
#define GKV_DEFAULT_ADDRESS  1              /*  default address of each sending device */

#include <stdint.h>

/** 
  * @brief  Template for each request (to gkv)/response (from gkv) packet 
  */
typedef struct __GKV_PacketBase
{
	uint8_t preamble;                   /*	always 255	*/
	uint8_t address;                    /*	address of sending	device  */
	uint8_t type;                       /*	type of the packet	*/
	uint8_t length;                     /*	length of data fields (without checksum)	*/
	uint8_t data[GKV_DATA_LENGTH + 4];      /*	all data that packet's containing including checksum	*/
}GKV_PacketBase;


/*---------------------------------------------------------------------------------------------------------------*/

/** @defgroup TEST_PACKET_CODES
  * @brief    packet codes for "type" field of 'Nop' test connection packet
  * @{
  */ 

#define GKV_CHECK_PACKET								0x00								/*	type of empty packet (with length = 0) to check connection (send to GKV) (send to GKV) */
#define GKV_CONFIRM_PACKET							 	0x00								/*	type of empty packet (with length = 0) to confirm request receiving	(receive from GKV) */
#define GKV_RESET_PACKET							 	0x01								/*	type of empty packet (with length = 0) to reset GKV */

#endif

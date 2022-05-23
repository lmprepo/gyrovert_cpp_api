#pragma once
/**
  ******************************************************************************
  * @file    GKV_GyroOffsetPacket.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 structures module for packets with algorithm (Gyrovert, BINS, GNSS) parameters and results
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
#ifndef __GKV_GYRO_OFFSET_PACKET_H__
#define __GKV_GYRO_OFFSET_PACKET_H__

/** @defgroup alg_PARAM_PACKET_CODES
  * @brief    packet codes for "type" field of algorithm parameters packet
  * @{
  */ 
#define GKV_GYRO_OFFSET_CALC_REQUEST			        0x1C								/*	type of packet (with length = 4) to request calculation of gyro offset parameters using N "samples"	*/
#define GKV_GYRO_OFFSET_REQUEST						    0x1D								/*	type of empty packet (with length = 0) to request  gyro offset */ 
#define GKV_GYRO_OFFSET_PACKET					    	0x1E								/*	type of packet with gyro offset coefficients	(send/receive) */
/**
  * @}
  */




/** 
  * @brief  Packet 0x1C is used as command to calculate compensation coefficients of gyro axis using N samples (field "samples") (send only)
  */
typedef struct __GKV_GyroOffsetCalc
{
	uint32_t samples; /*	number of samples for calculating gyro offset	*/
}GKV_GyroOffsetCalc;



/** 
  * @brief  Packet 0x1E with compensation coefficients of gyro axis (send/receive)
  */
typedef struct __GKV_GyroOffset
{
	int32_t x_900; /*	custom (send) or current (receive)	gyro offset for X axis in 24-bit ADC codes */
	int32_t y_900; /*	custom (send) or current (receive)	gyro offset for Y axis in 24-bit ADC codes */
	int32_t z_900; /*	custom (send) or current (receive)	gyro offset for Z axis in 24-bit ADC codes */
	int32_t reserved[9];
}GKV_GyroOffset;




#endif
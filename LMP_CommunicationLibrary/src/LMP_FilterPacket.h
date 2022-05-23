#pragma once
/**
  ******************************************************************************
  * @file    GKV_FilterPacket.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 structures module for packets with filter parameters (FIR, IIR, and moving average)
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
#ifndef __GKV_FILTER_PACKET_H__
#define __GKV_FILTER_PACKET_H__

/** @defgroup FILTER_PACKET_CODES
  * @brief    packet codes for "type" field of filter packet
  * @{
  */ 
#define GKV_FILTER_PARAM_REQUEST 					    0x1F								/*	type of empty packet (with length = 0) to request filter parameters packet	*/
#define GKV_FILTER_PARAM_PACKET				            0x20								/*	type of packet with FIR filter settings	(send/receive) */
/**
  * @}
  */


/** @defgroup FIR_FILTER_PARAMETERS
  * @brief 		settings for "type" field of request/response packet 0x20 (FIR filter settings)
  * @{
  */
#define GKV_TYPE_DOWNSAMPLE 												        0x00				/*	set output data as last data from 24 KHz */
#define GKV_TYPE_DOWN_OVERSAMPLE 												    0x01				/*	??? */
#define GKV_TYPE_AVE 						                                        0x02				/*	set output data as an average of adc data */
#define GKV_TYPE_FIR             						                            0x03				/*	turn on IIR Filter */
#define GKV_TYPE_IIR             						                            0x04				/*	turn on IIR Filter */
#define GKV_TYPE_MULTISTAGE_FIR_1K 										            0x05				/*	Fd=Fs/K0, Fs=24 KHz, K0=24, Fpass=250 Hz*/
#define GKV_TYPE_MULTISTAGE_FIR_2K 										            0x06				/*	Fd=Fs/K0, Fs=24 KHz, K0=12, Fpass=250 Hz*/
#define GKV_TYPE_MULTISTAGE_FIR_4K 										            0x07				/*	Fd=Fs/K0, Fs=24 KHz, K0=6, Fpass=250 Hz*/
#define GKV_TYPE_MULTISTAGE_FIR_8K 										            0x08				/*	Fd=Fs/K0, Fs=24 KHz, K0=3, Fpass=250 Hz*/
/**
  * @}
  */


/** 
  * @brief  Packet 0x20 is used to get or set parameters of FIR filter (send/receive)
  */
typedef struct __GKV_Filter
{
	uint8_t type;							                    /*	cut frequency of FIR filter or selecting IIR/AWE/Downsample. Sampling Frequency = 24 KHz. Cut frequency of inertial sensors = 250 Hz	*/
	uint8_t fir_len;                                            /*	data length for filtering when FIR filter is on	*/
	uint8_t iir_len;                                            /*	data length for filtering when IIR filter is on	*/
	uint16_t movave_len;					                    /*	simple moving average length for simple filtering when FIR and IIR filter is off	*/
}GKV_Filter;

#endif
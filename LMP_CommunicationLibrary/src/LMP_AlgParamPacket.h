#pragma once
/**
  ******************************************************************************
  * @file    GKV_AlgParamPacket.h
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
#ifndef __GKV_ALG_PARAM_PACKET_H__
#define __GKV_ALG_PARAM_PACKET_H__

/** @defgroup alg_PARAM_PACKET_CODES
  * @brief    packet codes for "type" field of algorithm parameters packet
  * @{
  */ 
#define GKV_ALG_PARAM_REQUEST 						    0x23								/*	type of empty packet (with length = 0) to request filter parameters packet	*/
#define GKV_ALG_PARAM_PACKET							0x24								/*	type of packet with settings of selected parameter and general algorithm parameters	*/
/**
  * @}
  */


/** @defgroup KALMAN_FILTER_ALGORITHM_PARAMETERS
  * @brief 		values for "i" field of packets 0x23 and 0x24 if algorithm set as Kalman Filtering
  * @{
  */
#define GKV_KALMAN_RESET															0x01				/*	If 1 set algorithm reset. After reset processing value returns to 0 */
#define GKV_KALMAN_IDLE_TIME													    0x02				/*	Number of samples to get device ready. Default value = 10000	*/
#define GKV_KALMAN_STEADY_TIME												        0x03				/*	Number of samples to get orientation correction from accelerometers	*/
#define GKV_KALMAN_A_THRESHOLD												        0x04				/*	Deviation from 1g acceleration vector in tolerance of which correction is used	(default 0.003)*/
#define GKV_KALMAN_W_THRESHOLD												        0x05				/*	The threshold for determining statis by the angular velocity vector	(default 0,0005)	*/
/**
  * @}
  */
	

/** @defgroup BINS_ALGORITHM_PARAMETERS
  * @brief 		values for "parameter_index" field of packets 0x23 and 0x24 if algorithm set as BINS
  * @{
  */
#define GKV_BINS_RESET															0x01				/*	If 1 set then algorithm reset. After reset processing value returns to 0 */
#define GKV_BINS_IDLE_TIME													    0x02				/*	Number of samples to get device ready. Default value = 10000	*/
/**
  * @}
  */

	
/** @defgroup BINS_GNSS_ALGORITHM_PARAMETERS
  * @brief 		values for "parameter_index" field of packets 0x23 and 0x24 if algorithm set as BINS and sattelite correction is on
  * @{
  */
#define GKV_RESET															0x01				/*	If 1 set algorithm reset. After reset processing value returns to 0 */
#define GKV_IDLE_TIME													    0x02				/*	Number of samples to get device ready. Default value = 10000	*/
#define GKV_FILT_YAW_TIME											        0x03                /*  Course data filtering time to find the true heading when speeding vel_threshold   */
#define GKV_VEL_THRESHOLD											        0x04				/*	the linear speed threshold from GNSS receiver after which it is possible to determine the true heading	*/
#define GKV_STEADY_TIME												        0x05                /*  statis time setting in samples. Default = 500*/
#define GKV_IDLE_CORRECTOR										            0x06				/*	Corrector of a statis mode. 1 - correction from accelerometers,	2 - from calculating speed	*/
#define GKV_A_THRESHOLD												        0x07				/*	Deviation from 1g acceleration vector in tolerance of which correction is used	(default 0.003)*/
#define GKV_W_THRESHOLD												        0x08				/*	The threshold for determining statis by the angular velocity vector	(default 0,0005)	*/
#define GKV_TA_THRESHOLD											        0x09                /*  The threshold for determining statis by the apparent linear vector acceleration for corrector (default  0.055 */
#define GKV_USE_ONEPPS												        0x0A				/*	Using sync signal from GNSS. 1 - 1PPS, 0 - OFF*/
#define GKV_USE_LEVER_ARM											        0x0B                /*  Using the distance parameters from the inertial module to the GNSS antenna. (default 0)    */
#define GKV_USE_PHASE_CORR										            0x0C                /*  Using calculation of navigation data without phase delay of data from GNSS receiver. “1” - phase correlation. "0"- correction is disabled (default  0)*/
#define GKV_START_YAW													    0x0D                /*  Start azimuth angle (default 0) */
#define GKV_LEVER_ARM_X												        0x0E                /*  the X axis distance from the inertial module to the GNSS antenna    */
#define GKV_LEVER_ARM_Y												        0x0F                /*  the Y axis distance from the inertial module to the GNSS antenna    */
#define GKV_LEVER_ARM_Z												        0x10                /*  the Z axis distance from the inertial module to the GNSS antenna    */
/**
  * @}
  */



/** 
  * @brief  Packet 0x23 is used to request parameters of algorithm (send only)
  */
typedef struct __GKV_GetAlgParam
{
	uint32_t i;		                                            /*	index from Kalman Filter/BINS/GNSS algorithm parameters list	*/
}GKV_GetAlgParam;

/** 
  * @brief  Packet 0x24 is used to set value of algorithm parameter  or receive it from GKV(send/receive)
  */
typedef struct __GKV_AlgParam
{
	uint32_t i;					                                /*	index from Kalman Filter/BINS/GNSS algorithm parameters list	*/
	float val;						                            /*	value for selected by 'parameter_index' algorithm parameter	*/
	uint32_t count;				                                /* number of algorithm parameters (receive only) */
	char name[32]; 					                            /* ASCII algorithm parameter name (receive only) */	
	uint8_t save_to_flash;						                /* write parameter value to ROM (send only)	*/
}GKV_AlgParam;

#endif
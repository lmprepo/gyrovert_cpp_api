#pragma once
/**
  ******************************************************************************
  * @file    GKV_AlgorithmPacket.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 structures module for request/response packets of device changeable settings
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
#ifndef __GKV_ALGORITHM_PACKET_H__
#define __GKV_ALGORITHM_PACKET_H__

/** @defgroup ALGORITHMS_PACKETS
  * @brief    Packet templates for different data processing algo
  * @{
  */

#ifndef __GKV_DEVICE_STATUS__
#define __GKV_DEVICE_STATUS__
/** @defgroup DEVICE_STATUS
  * @brief 	status_bits for "status" field of each response packet	
  * @{
  */ 
#define GKV_GNSS_TIMESTAMP 									1<<12								/* synchronization bit for GNSS receiver	*/
#define GKV_DEV_SETTING_FINISHED 							1<<11								/* set as 1 after defice selfconfiguration finish (in orientation or navigation algorithms)	*/
#define GKV_SYNC_SIGNAL_LEVEL								1<<10								/* set as 1 if sync signal input level more than 2.5 v, set as 0 if sync signal input level less than 0.5 v */
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
/*-------------------------------------------------DATA_REQUEST---------------------------------------------------------------------*/

#ifndef  __GET_DATA_PACKET__
#define __GET_DATA_PACKET__

/** @defgroup REQUEST_PACKET_CODE
  * @brief    packet codes for "type" field of request data packet
  * @{
  */ 
#define GKV_DATA_REQUEST 								0x17								/*	type of empty packet (with length = 0) to request data packet in "by request" mode	*/  /*!!!!!!!!!*/
/**
  * @}
  */

#endif




/*-----------------------------------------------ADC_CODES_ALOGORITHM---------------------------------------------------------------*/
   
/** 
  * @brief  ADC codes algorithm. Packet 0x0A with non-calibrated sensors data (receive only)        !!!
  */


/** @defgroup ADC_PACKET_CODE
  * @brief    packet code for "type" field of received packet
  * @{
  */ 
#define GKV_ADC_CODES_PACKET 							0x0A								/*	type of data packet with non-calibrated sensors data */
/**
  * @}
  */



/** 
  * @brief  ADC codes algorithm. Packet 0x0A with non-calibrated sensors data (receive only)
  */
typedef struct __GKV_ADCData
{
	uint16_t sample_cnt;										/*	0-65535 counter to detect number of lost packets	*/
	uint16_t status;											/*	field for detecting errors, sync and algorithm state	*/
	int32_t a[3];												/*	accelerometer non-calibrated X Y Z axis data in 24 bit ADC codes	*/
	int32_t w[3];												/*	rate sensor non-calibrated	X Y Z axis data in 24 bit ADC codes	*/
	int32_t t[4];												/*	X Y Z axis and CPU temperature data in 12 bit ADC	*/
}GKV_ADCData;


/*-----------------------------------------------RAW_DATA_ALOGORITHM----------------------------------------------------------------*/


/** @defgroup RAW_PACKET_CODE
  * @brief    packet code for "type" field of received packet
  * @{
  */
#define GKV_RAW_DATA_PACKET 			                0x0B								/*	type of data packet with calibrated sensors data */
/**
  * @}
  */


/** 
  * @brief  Sensors data algorithm. Packet 0x0B with calibrated data of gyroscope, accelerometer and temperature sensors for every axis and cpu (receive only)
  */
typedef struct __GKV_RawData
{
	uint16_t sample_cnt;										/*	0-65535 counter to detect number of lost packets	*/
	uint16_t status;											/*	field for detecting errors, sync and algorithm state	*/
	float a[3];													/*	accelerometer calibrated	X Y Z axis data in g or m/s2	*/
	float w[3];													/*	rate sensor calibrated	X Y Z axis data in deg/s or rad/s	*/
	float t[4];													/*	X Y Z axis and CPU temperature data in Celsius degrees 	*/
}GKV_RawData;



/*-----------------------------------------------GYROVERT_ALGORITHM------------------------------------------------------------*/


/** @defgroup   GYROVERT_PACKET_CODE
  * @brief    packet code for "type" field of received packet
  * @{
  */
#define GKV_EULER_ANGLES_PACKET 					    0x0C								/*	type of data packet with calculated by Kalman filtering Euler angles */
/**
  * @}
  */


/** 
  * @brief  Kalman filter algorithm. Packet 0x0C with basic orientation data (euler angles) calculated using Kalman/Mahony filter (receive only)
  */

typedef struct __GKV_GyrovertData
{
	uint16_t sample_cnt;                                        /*	0-65535 counter to detect number of lost packets	*/
	uint16_t status;						                    /*	field for detecting errors, sync and algorithm state	*/
	float pitch;								                /*	pitch Euler angle	*/
	float roll;									                /*	roll Euler angle	*/
	float yaw;                                                  /*	yaw Euler angle	*/
}GKV_GyrovertData;



/*-----------------------------------------------INCLINOMETER_ALGORITHM------------------------------------------------------------*/

/** @defgroup   INCLINOMETER_PACKET_CODE
  * @brief    packet code for "type" field of received packet
  * @{
  */
#define GKV_INCLINOMETER_PACKET 					    0x0D								/*	type of data packet with inclinometer angles */
/**
  * @}
  */


/** 
  * @brief  Inclinometer algorithm. Packet 0x0D with inclinometer angles (receive only)
  */
typedef struct __GKV_InclinometerData
{
	uint16_t sample_cnt; /*	0-65535 counter to detect number of lost packets	*/
	uint16_t status; /*	field for detecting errors, sync and algorithm state	*/
	float alfa; /*	inclinometer angle alfa	(XZ)    */
	float beta; /*	inclinometer angle beta (YZ)    */
}GKV_InclinometerData;



/*-----------------------------------------------BINS_ALGORITHM------------------------------------------------------------*/

/** @defgroup   BINS_PACKET_CODE
  * @brief    packet code for "type" field of received packet
  * @{
  */
#define GKV_BINS_PACKET 								0x12								/*	type of data packet with navigation and orientation data */
/**
  * @}
  */

/** 
  * @brief  BINS algorithm. Packet 0x12 with navigation data as position (x,y,z), Euler angles, inclinometer angles and orientation quaternion (receive only)   !!!!
  */
typedef struct __GKV_BINSData
{
	uint16_t sample_cnt;                                        /*	0-65535 counter to detect number of lost packets	*/
	uint16_t status;						                    /*	field for detecting errors, sync and algorithm state	*/
	float x;									                /*	x axis position	*/
	float y;									                /*	y axis position	*/
	float z;									                /*	z axis position	*/
	float pitch;								                /*	pitch Euler angle	*/
	float roll;									                /*	roll Euler angle	*/
	float yaw;									                /*	yaw Euler angle	*/
	float alfa;								                    /*	inclinometer angle alfa	XZ  */
	float beta;									                /*	inclinometer angle beta	YZ  */
	float q[4];				                                    /*	orientation quaternion q3 q2 q1 q0	*/
}GKV_BINSData;


/*-----------------------------------------------GNSS_NAVIGATION_ALGORITHM------------------------------------------------------------*/


/** @defgroup   GNSS_PACKET_CODE
  * @brief    packet code for "type" field of received packet
  * @{
  */
#define GKV_GNSS_PACKET 								0x0E								/*	type of data packet with standard GNSS data */
/**
  * @}
  */

#pragma pack(push, 1)
/** 
  * @brief  If GNSS receiver is connected GKV can send packet 0x0E with GNSS data without correction from inertial system   !!!!
  */
typedef struct __GKV_GpsData
{
	uint32_t time;								                    /*	Coordinated Universal Time	(UTC)*/
	double latitude;								                /*	latitude from GNSS	*/
	double longitude;							                /*	longitude from GNSS	*/
	double altitude;								                /*	altitude from GNSS	*/
	uint32_t state_status;				    	                /*	state of GNSS receiver	*/
	float TDOP;									                /*	geometry factor of GNSS receiver	*/
	float HDOP;									                /*	geometry factor of GNSS receiver	*/
	float VDOP;									                /*	geometry factor of GNSS receiver	*/
	float velocity;     						                /*	horizontal speed */
	float yaw;  								                /*	azimuth angle from GNSS */
	float alt_velocity;                                         /*	vertical speed */
}GKV_GpsData;
#pragma pack(pop)



/** @defgroup   NV08C_CSM_STATE
  * @brief    bits of state field of GNSS when NV08C_CSM receiver connected

  * @{
  */
#define GKV_NV_DIFF_CORR_MODE_ON 							1<<5								/*	differential corrections mode on */
#define GKV_NV_RAIM_CHECK           						1<<4								/*	data RAIM check (autonomous GPS system integrity monitoring)*/
#define GKV_NV_DIFF_CORR_IN_SLN 							1<<3								/*	solution calculated using differential corrections */
#define GKV_NV_2D_SLN            							1<<1								/*	got 2d-solution on current step */
#define GKV_NV_GOT_SLN             							1   								/*	got GNSS solution on current step */
#define GKV_NV_NO_SLN             							0&(0xFFFFFFFF))   					/*	no GNSS solution on current step */
  /**
    * @}
    */



/** @defgroup   MNP_STATE
  * @brief    bits of state field of GNSS when MNP receiver connected

  * @{
  */
#define GKV_MNP_GLONASS_TIME     	    					(3<<13)								/*	global time is GLONASS */
#define GKV_MNP_GPS_TIME     	        					(2<<13)								/*	global time is GPS */
#define GKV_MNP_UTC_SU_TIME     	    					(1<<13)								/*	global time is UTS(SU) */
#define GKV_MNP_UTC_USNO_TIME     	    					(0<<13)								/*	global time is UTS(USNO) */
#define GKV_MNP_KRAS_ELL            						(2<<11)								/*	Krasovsky ellipsoid */
#define GKV_MNP_PZ_90_02_ELL         						(1<<11)								/*	ellipsoid type is from PZ_90_02 */
#define GKV_MNP_WGS_84_ELLIPSE     							(0<<11)								/*	ellipsoid type is from WGS-84 */
#define GKV_MNP_SK_95_COORD     							(3<<8)								/*	coordinates system is SK-95 */
#define GKV_MNP_SK_42_COORD     							(2<<8)								/*	coordinates system is SK-42 */
#define GKV_MNP_PZ_90_02_COORD     							(1<<8)								/*	coordinates system is from PZ_90_02 */
#define GKV_MNP_WGS_84_COORD       							(0<<8)								/*	coordinates system is from WGS-84 */
#define GKV_MNP_TIME_OK           							1<<1								/*	time is correct */
#define GKV_MNP_SLN_OK            							1   								/*	solution is correct */

  /**
    * @}
    */



/** @defgroup   ZED_9FP_STATE
  * @brief    bits of state field of GNSS when ZED_9FP receiver connected

  * @{
  */
#define GKV_ZED_RTK_TIME_OK   						    	1<<31  								/*	RTK date is correct */
#define GKV_ZED_RTK_DATE_OK   							    1<<30  								/*	RTK time is correct */
#define GKV_ZED_RTK_DATE_TIME_OK   							1<<29  								/*	RTK date and time is correct */
#define GKV_ZED_RTK_FIX_SLN        							2<<22								/*	fixed RTK solution */
#define GKV_ZED_RTK_FLOAT_SLN      							1<<22								/*	floationg RTK solution */
#define GKV_ZED_RTK_NO_SLN         							0<<22								/*	no RTK solution */
#define GKV_ZED_DIFF_CORR_USED     							1<<17								/*	solution uses differential corrections  */
#define GKV_ZED_COORD_DOP_OK           						1<<16								/*  coordinates, DOP and precision are correct  */
#define GKV_ZED_TIME_SLN           							5<<8								/*	got time solution  */
#define GKV_ZED_3D_SLN           							3<<8								/*	got 3d-solution */
#define GKV_ZED_2D_SLN           							2<<8								/*	got 2d-solution */
#define GKV_ZED_NO_SLN           							0<<8								/*	no solution */
#define GKV_ZED_GOT_TIME_CNF_SLN   							1<<2								/*	time ambiguity resolved */
#define GKV_ZED_TIME_OK           							1<<1								/*	time is correct */
#define GKV_ZED_DATE_OK           							1   								/*	date is correct */

  /**
    * @}
    */


/*-----------------------------------------------EXTENDED_GNSS_NAVIGATION_ALGORITHM------------------------------------------------------------*/


/** @defgroup   EXTENDED_GNSS_PACKET_CODE
  * @brief    packet code for "type" field of received packet
  * @{
  */
#define GKV_EXTENDED_GNSS_PACKET 					    0x0F								/*	type of data packet with extended GNSS data */
/**
  * @}
  */

#pragma pack(push, 1)
/** 
  * @brief  If GNSS receiver is connected GKV can send packet 0x0F with extended GNSS data          !!!!
  */
typedef struct __GKV_GpsDataExt
{
	double vlat;     							                /*	velocity on latitude	*/
	double vlon;         						                /*	velocity on longitude	*/
	float sig_lat;							                    /*	STD of latitude data	*/
	float sig_lon;						                        /*	STD of longtitude data	*/
	float sig_alt;							                    /*	STD of altitude data	*/
	float sig_vlat;						                        /*	STD of velocity on latitude	*/
	float sig_vlon;					                            /*	STD of velocity on longitude	*/
	float sig_valt;						                        /*	STD of velocity on altitude	*/
	uint16_t num_ss;		                                    /*	number of sattelites used in calculation of GNSS data*/
	uint16_t reserved;
}GKV_GpsDataExt;
/**
  * @}
  */
#pragma pack(pop)



#endif
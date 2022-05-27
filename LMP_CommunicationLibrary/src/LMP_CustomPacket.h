#pragma once
/**
  ******************************************************************************
  * @file    GKV_CustomPacket.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 structures module for packets WITH CUSTOM PARAMETERS OF EACH ALGORITHM
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
#ifndef __GKV_CUSTOM_PACKET_H__
#define __GKV_CUSTOM_PACKET_H__

/** @defgroup CUSTOM_PACKET_CODES
  * @brief    packet codes for "type" field of custom data/parameters packet
  * @{
  */ 
#define GKV_CUSTOM_PACKET 								0x13								/*	type of data packet custom data	*/
#define GKV_CUSTOM_PACKET_PARAM_REQUEST 	            0x26								/*	type of empty packet (with length = 0) to request parameters of "custom data packet"	*/
#define GKV_CUSTOM_DATA_PARAM_PACKET		 	        0x27								/*	type of packet with current parameters list of "custom data packet"	*/
/**
  * @}
  */


/** 
  * @brief  Any algorithm. Packet 0x13 with custom parameters that can be selected instead standart packet of each algorithm (receive only).
  */
#pragma pack(push, 1)
typedef struct __GKV_CustomData
{
	//uint8_t number_of_parameters;           /*	number of parameters that device sends in custom packet	*/
    float parameter[63];                    /*	value of parameter N from list of 'custom_data_parameters'	*/
}GKV_CustomData;
#pragma pack(pop)


#define GKV_CUSTOM_PACKET_PARAMS_QUANTITY                                   0X6D
/** @defgroup CUSTOM_PACKET_PARAMETERS
  * @brief 	LIST OF PARAMETER CODES FOR CUSTOM RESPONSE PACKET
  * @{
  */
#define GKV_STATUS 														    0x00					/* device status */
#define GKV_SAMPLE_COUNTER 										            0x01					/*	16-bit sample counter to control lost packets (changes from 0 to 65535)	*/
#define GKV_NAX 															0x02					/*	non-calibrated data from X axis of accelerometer 	*/
#define GKV_NAY 															0x03					/*	non-calibrated data from Y axis of accelerometer 	*/
#define GKV_NAZ 															0x04					/*	non-calibrated data from Z axis of accelerometer 	*/
#define GKV_NWX 															0x05					/*	non-calibrated data from X axis of gyroscope 	*/
#define GKV_NWY 															0x06					/*	non-calibrated data from Y axis of gyroscope 	*/
#define GKV_NWZ 															0x07					/*	non-calibrated data from Z axis of gyroscope 	*/
#define GKV_NT0 															0x08					/*	non-calibrated data from 0 channel of 12-bit adc of MCU (temperature 0)	*/
#define GKV_NT1 															0x09					/*	non-calibrated data from 1 channel of 12-bit adc of MCU (temperature 1)	*/
#define GKV_NT2 															0x0A					/*	non-calibrated data from 2 channel of 12-bit adc of MCU (temperature 2)	*/
#define GKV_NT3 															0x0B					/*	non-calibrated data from 3 channel of 12-bit adc of MCU (temperature 3)	*/
#define GKV_NT4 															0x0C					/*	non-calibrated data from 4 channel of 12-bit adc of MCU (temperature 4)	*/
#define GKV_NT5 															0x0D					/*	non-calibrated data from 5 channel of 12-bit adc of MCU (temperature 5)	*/
#define GKV_NT6 															0x0E					/*	non-calibrated data from 6 channel of 12-bit adc of MCU (temperature 6)	*/
#define GKV_NT7 															0x0F					/*	non-calibrated data from 7 channel of 12-bit adc of MCU (temperature 7)	*/
#define GKV_NT8 															0x10					/*	non-calibrated data from 8 channel of 12-bit adc of MCU (temperature 8)	*/
#define GKV_NT9 															0x11					/*	non-calibrated data from 9 channel of 12-bit adc of MCU (temperature 9)	*/
#define GKV_AX 																0x12					/*	calibrated data from X axis of accelerometer 	*/
#define GKV_AY 																0x13					/*	calibrated data from Y axis of accelerometer 	*/
#define GKV_AZ 																0x14					/*	calibrated data from Z axis of accelerometer 	*/
#define GKV_WX 																0x15					/*	calibrated data from X axis of gyroscope 	*/
#define GKV_WY 																0x16					/*	calibrated data from X axis of gyroscope	*/
#define GKV_WZ 																0x17					/*	calibrated data from X axis of gyroscope	*/
#define GKV_T0 																0x18					/*	calibrated data from 0 channel of 12-bit adc of MCU (temperature 0)	*/
#define GKV_T1 																0x19					/*	calibrated data from 1 channel of 12-bit adc of MCU (temperature 1)	*/
#define GKV_T2																0x1A					/*	calibrated data from 2 channel of 12-bit adc of MCU (temperature 2)	*/
#define GKV_T3 																0x1B					/*	calibrated data from 3 channel of 12-bit adc of MCU (temperature 3)	*/
#define GKV_T4 																0x1C					/*	calibrated data from 4 channel of 12-bit adc of MCU (temperature 4)	*/
#define GKV_T5 																0x1D					/*	calibrated data from 5 channel of 12-bit adc of MCU (temperature 5)	*/
#define GKV_T6 																0x1E					/*	calibrated data from 6 channel of 12-bit adc of MCU (temperature 6)	*/
#define GKV_T7 																0x1F					/*	calibrated data from 7 channel of 12-bit adc of MCU (temperature 7)	*/
#define GKV_T8 																0x20					/*	calibrated data from 8 channel of 12-bit adc of MCU (temperature 8)	*/
#define GKV_T9 																0x21					/*	calibrated data from 9 channel of 12-bit adc of MCU (temperature 9)	*/
#define GKV_ALPHA 														    0x22					/*	alpha angle of inclinometer	*/
#define GKV_BETA 															0x23					/*	beta angle of inclinometer	*/
#define GKV_PITCH 														    0x24					/*	pitch angle of euler orientation system	*/
#define GKV_ROLL 															0x25					/*	roll angle of euler orientation system	*/
#define GKV_YAW 															0x26					/*	yaw angle of euler orientation system	*/
#define GKV_Q0 																0x27					/*	scalar part of orientation quaternion	*/
#define GKV_Q1 																0x28					/*	x part of orientation quaternion	*/
#define GKV_Q2 																0x29					/*	y part of orientation quaternion	*/
#define GKV_Q3 																0x2A					/*	z part of orientation quaternion	*/
#define GKV_X 																0x2B					/*	position x	*/
#define GKV_Y 																0x2C					/*	position y	*/
#define GKV_Z 																0x2D					/*	position z	*/
#define GKV_VX 																0x2E					/*	linear velocity x	*/
#define GKV_VY 																0x2F					/*	linear velocity y	*/
#define GKV_VZ 																0x30					/*	linear velocity z	*/
#define GKV_IWX 															0x31					/*	integrated angle from rate of x axis of gyro	*/
#define GKV_IWY 															0x32					/*	integrated angle from rate of y axis of gyro	*/
#define GKV_IWZ 															0x33					/*	integrated angle from rate of z axis of gyro	*/
#define GKV_YAW_NOPH													    0x34					/*	yaw angle without phase delay (when use_phase_corr = 1)*/
#define GKV_PITCH_NOPH												        0x35					/*	pitch angle without phase delay (when use_phase_corr = 1)*/
#define GKV_ROLL_NOPH													    0x36					/*	roll angle without phase delay (when use_phase_corr = 1)*/
#define GKV_ALG_INT_LAT_NOPH									            0x37					/*	latitude calculated using BINS in codes (to convert into radians multiply 2*pi/(2^32))  (when use_phase_corr = 1)*/
#define GKV_ALG_INT_LON_NOPH									            0x38					/*	longitude calculated using BINS in codes (to convert into radians multiply 2*pi/(2^32))  (when use_phase_corr = 1)*/
#define GKV_ALG_INT_ALT_NOPH									            0x39					/*	altitude calculated using BINS in m (when use_phase_corr = 1)*/
/* parameter codes 0x3A:0x3F are reserved*/
#define GKV_LAX 															0x40					/*	linear acceleration x	*/
#define GKV_LAY 															0x41					/*	linear acceleration y	*/
#define GKV_LAZ 															0x42					/*	linear acceleration z	*/
#define GKV_AZIMUTH 												    	0x43					/*	azimuth angle (when using GNSS)	*/
#define GKV_UTC_TIME 												    	0x44					/*	Coordinated Universal Time (when using GNSS)	*/
#define GKV_LAT 															0x45					/*	latitude (when using GNSS)	*/
#define GKV_LON 															0x46					/*	longitude (when using GNSS)	*/
#define GKV_ALT 															0x47					/*	altitude (when using GNSS)	*/
#define GKV_GNSS_STATUS 											        0x48					/*	state of GNSS receiver	*/
#define GKV_GNSS_TDOP 												        0x49					/*	geometry factor of GNSS receiver	*/
#define GKV_GNSS_HDOP 												        0x4A					/*	geometry factor of GNSS receiver	*/
#define GKV_GNSS_VDOP 												        0x4B					/*	geometry factor of GNSS receiver	*/
#define GKV_GNSS_VEL 												    	0x4C					/*	horizontal velocity calculated using GNSS	*/
#define GKV_GNSS_YAW 												    	0x4D					/*	yaw angle calculated using GNSS	*/
#define GKV_GNSS_ALT_VEL 										        	0x4E					/*	verical velocity calculated using GNSS	*/
#define GKV_GNSS_SAT_NUM 										            0x4F					/*	number of sattelites using to calculate GNSS parameters	*/
#define GKV_MX	 															0x50					/*	magnetometer data x	*/
#define GKV_MY 																0x51					/*	magnetometer data y	*/
#define GKV_MZ 																0x52					/*	magnetometer data z	*/
#define GKV_GNSS_LAT_VEL 											        0x53					/*	velocity on latitude (using GNSS)	*/
#define GKV_GNSS_LON_VEL 											        0x54					/*	velocity on longitude (using GNSS)	*/
#define GKV_GNSS_SIG_LAT 											        0x55					/*	STD of latitude data	*/
#define GKV_GNSS_SIG_LON 											        0x56					/*	STD of longitude data	*/
#define GKV_GNSS_SIG_ALT 										        	0x57					/*	STD of altitude data	*/
#define GKV_GNSS_SIG_VLAT 										            0x58					/*	STD of velocity on latitude	*/
#define GKV_GNSS_SIG_VLON 										            0x59					/*	STD of velocity on longitude	*/
#define GKV_GNSS_SIG_VALT 										            0x5A					/*	STD of velocity on altitude	*/
#define GKV_ALG_INT_LAT												        0X5B					/*	latitude, calculated by BINS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define GKV_ALG_INT_LON												        0X5C					/*	longitude, calculated by BINS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define GKV_ALG_ALT														    0X5D					/*	altitude, calculated by BINS in float32	*/
#define GKV_GNSS_INT_LAT											        0X5E					/*	latitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define GKV_GNSS_INT_LON											        0X5F					/*	longitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define GKV_ALG_STATE_STATUS										        0X60					/*	altitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
#define GKV_BAROMETER_ADC											        0X61					/*	Barometer data in codes	*/
#define GKV_ALG_VAR_X													    0X62					/*	variance of position error of X axis in m2	*/
#define GKV_ALG_VAR_Y													    0X63					/*	variance of position error of Y axis in m2	*/
#define GKV_ALG_VAR_Z													    0X64					/*	variance of position error of Z axis in m2	*/
#define GKV_ALG_VAR_VX												        0X65					/*	variance of velocity error of X axis in (m/s)^2	*/
#define GKV_ALG_VAR_VY												        0X66					/*	variance of velocity error of Y axis in (m/s)^2	*/
#define GKV_ALG_VAR_VZ												        0X67					/*	variance of velocity error of Z axis in (m/s)^2	*/
#define GKV_ALG_VAR_PSI												        0X68					/*	variance of orientation error of yaw axis in rad^2	*/
#define GKV_ALG_VAR_THETA											        0X69					/*	variance of orientation error of pitch axis in rad^2	*/
#define GKV_ALG_VAR_PHI												        0X6A					/*	variance of orientation error of roll axis in rad^2	*/
#define GKV_GPS_INT_X												        0X6B					/*	X axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
#define GKV_GPS_INT_Y												        0X6C					/*	Y axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
#define GKV_GPS_INT_Z												        0X6D					/*	Z axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
/**
  * @}
  */

/**
  * @brief  Structure with all custom parameters.
  */
typedef struct __GKV_AllParameters
{
    float    STATUS; 													    					/* device status */
    float    SAMPLE_COUNTER; 									            					/*	16-bit sample counter to control lost packets (changes from 0 to 65535)	*/
    float    NAX; 																				/*	non-calibrated data from X axis of accelerometer 	*/
    float    NAY; 																				/*	non-calibrated data from Y axis of accelerometer 	*/
    float    NAZ; 																				/*	non-calibrated data from Z axis of accelerometer 	*/
    float    NWX;																				/*	non-calibrated data from X axis of gyroscope 	*/
    float    NWY;																				/*	non-calibrated data from Y axis of gyroscope 	*/
    float    NWZ;																				/*	non-calibrated data from Z axis of gyroscope 	*/
    float    NT0;																				/*	non-calibrated data from 0 channel of 12-bit adc of MCU (temperature 0)	*/
    float    NT1;																				/*	non-calibrated data from 1 channel of 12-bit adc of MCU (temperature 1)	*/
    float    NT2;																				/*	non-calibrated data from 2 channel of 12-bit adc of MCU (temperature 2)	*/
    float    NT3;																				/*	non-calibrated data from 3 channel of 12-bit adc of MCU (temperature 3)	*/
    float    NT4;																				/*	non-calibrated data from 4 channel of 12-bit adc of MCU (temperature 4)	*/
    float    NT5;																				/*	non-calibrated data from 5 channel of 12-bit adc of MCU (temperature 5)	*/
    float    NT6;																				/*	non-calibrated data from 6 channel of 12-bit adc of MCU (temperature 6)	*/
    float    NT7;																				/*	non-calibrated data from 7 channel of 12-bit adc of MCU (temperature 7)	*/
    float    NT8;																				/*	non-calibrated data from 8 channel of 12-bit adc of MCU (temperature 8)	*/
    float    NT9;																				/*	non-calibrated data from 9 channel of 12-bit adc of MCU (temperature 9)	*/
    float    AX;																				/*	calibrated data from X axis of accelerometer 	*/
    float    AY;																				/*	calibrated data from Y axis of accelerometer 	*/
    float    AZ;																				/*	calibrated data from Z axis of accelerometer 	*/
    float    WX;																				/*	calibrated data from X axis of gyroscope 	*/
    float    WY;																				/*	calibrated data from X axis of gyroscope	*/
    float    WZ;																				/*	calibrated data from X axis of gyroscope	*/
    float    T0;																				/*	calibrated data from 0 channel of 12-bit adc of MCU (temperature 0)	*/
    float    T1;																				/*	calibrated data from 1 channel of 12-bit adc of MCU (temperature 1)	*/
    float    T2;																				/*	calibrated data from 2 channel of 12-bit adc of MCU (temperature 2)	*/
    float    T3;																				/*	calibrated data from 3 channel of 12-bit adc of MCU (temperature 3)	*/
    float    T4;																				/*	calibrated data from 4 channel of 12-bit adc of MCU (temperature 4)	*/
    float    T5;																				/*	calibrated data from 5 channel of 12-bit adc of MCU (temperature 5)	*/
    float    T6;																				/*	calibrated data from 6 channel of 12-bit adc of MCU (temperature 6)	*/
    float    T7;																				/*	calibrated data from 7 channel of 12-bit adc of MCU (temperature 7)	*/
    float    T8;																				/*	calibrated data from 8 channel of 12-bit adc of MCU (temperature 8)	*/
    float    T9;																				/*	calibrated data from 9 channel of 12-bit adc of MCU (temperature 9)	*/
    float    ALPHA; 														    				/*	alpha angle of inclinometer	*/
    float    BETA;																				/*	beta angle of inclinometer	*/
    float    PITCH;														    					/*	pitch angle of euler orientation system	*/
    float    ROLL;                                                                              /*	roll angle of euler orientation system	*/
    float    YAW;                                                                               /*	yaw angle of euler orientation system	*/
    float    Q0;                                                                                /*	scalar part of orientation quaternion	*/
    float    Q1;                                                                                /*	x part of orientation quaternion	*/
    float    Q2;                                                                                /*	y part of orientation quaternion	*/
    float    Q3;                                                                                /*	z part of orientation quaternion	*/
    float    X;                                                                                 /*	position x	*/
    float    Y;                                                                                 /*	position y	*/
    float    Z;                                                                                 /*	position z	*/
    float    VX;                                                                                /*	linear velocity x	*/
    float    VY;                                                                                /*	linear velocity y	*/
    float    VZ;                                                                                /*	linear velocity z	*/
    float    IWX;                                                                               /*	integrated angle from rate of x axis of gyro	*/
    float    IWY;                                                                               /*	integrated angle from rate of y axis of gyro	*/
    float    IWZ;                                                                               /*	integrated angle from rate of z axis of gyro	*/
    float    YAW_NOPH;													    					/*	yaw angle without phase delay (when use_phase_corr = 1)*/
    float    PITCH_NOPH;												        				/*	pitch angle without phase delay (when use_phase_corr = 1)*/
    float    ROLL_NOPH;                                                                         /*	roll angle without phase delay (when use_phase_corr = 1)*/
    float    ALG_INT_LAT_NOPH;									            					/*	latitude calculated using BINS in codes (to convert into radians multiply 2*pi/(2^32))  (when use_phase_corr = 1)*/
    float    ALG_INT_LON_NOPH;									            					/*	longitude calculated using BINS in codes (to convert into radians multiply 2*pi/(2^32))  (when use_phase_corr = 1)*/
    float    ALG_INT_ALT_NOPH;									            					/*	altitude calculated using BINS in m (when use_phase_corr = 1)*/
    float       reserved[6];    /* parameter codes 0x3A:0x3F are reserved*/
    float    LAX; 																				/*	linear acceleration x	*/
    float    LAY;																				/*	linear acceleration y	*/
    float    LAZ;																				/*	linear acceleration z	*/
    float    AZIMUTH; 												    						/*	azimuth angle (when using GNSS)	*/
    uint32_t UTC_TIME;												    						/*	Coordinated Universal Time (when using GNSS)	*/
    float    LAT;                                                                               /*	latitude (when using GNSS)	*/
    float    LON;                                                                               /*	longitude (when using GNSS)	*/
    float    ALT;                                                                               /*	altitude (when using GNSS)	*/
    uint32_t GNSS_STATUS;											        					/*	state of GNSS receiver	*/
    float    GNSS_TDOP;                                                                         /*	geometry factor of GNSS receiver	*/
    float    GNSS_HDOP;                                                                         /*	geometry factor of GNSS receiver	*/
    float    GNSS_VDOP;                                                                         /*	geometry factor of GNSS receiver	*/
    float    GNSS_VEL;                                                                          /*	horizontal velocity calculated using GNSS	*/
    float    GNSS_YAW;                                                                          /*	yaw angle calculated using GNSS	*/
    float    GNSS_ALT_VEL;										        						/*	verical velocity calculated using GNSS	*/
    float    GNSS_SAT_NUM;										            					/*	number of sattelites using to calculate GNSS parameters	*/
    float    MX;                                                                                /*	magnetometer data x	*/
    float    MY;                                                                                /*	magnetometer data y	*/
    float    MZ;                                                                                /*	magnetometer data z	*/
    float    GNSS_LAT_VEL;                                                                      /*	velocity on latitude (using GNSS)	*/
    float    GNSS_LON_VEL;                                                                      /*	velocity on longitude (using GNSS)	*/
    float    GNSS_SIG_LAT;                                                                      /*	STD of latitude data	*/
    float    GNSS_SIG_LON;                                                                      /*	STD of longitude data	*/
    float    GNSS_SIG_ALT;                                                                      /*	STD of altitude data	*/
    float    GNSS_SIG_VLAT;                                                                     /*	STD of velocity on latitude	*/
    float    GNSS_SIG_VLON;                                                                     /*	STD of velocity on longitude	*/
    float    GNSS_SIG_VALT;                                                                     /*	STD of velocity on altitude	*/
    uint32_t ALG_INT_LAT;                                                                       /*	latitude, calculated by BINS in codes (to convert into radians multiply 2*pi/(2^32))	*/
    uint32_t ALG_INT_LON;                                                                       /*	longitude, calculated by BINS in codes (to convert into radians multiply 2*pi/(2^32))	*/
    float    ALG_ALT;                                                                           /*	altitude, calculated by BINS in float32	*/
    uint32_t GNSS_INT_LAT;                                                                      /*	latitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
    uint32_t GNSS_INT_LON;                                                                      /*	longitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
    uint32_t ALG_STATE_STATUS;										        					/*	altitude from GNSS in codes (to convert into radians multiply 2*pi/(2^32))	*/
    float    BAROMETER_ADC;                                                                     /*	Barometer data in codes	*/
    float    ALG_VAR_X;                                                                         /*	variance of position error of X axis in m2	*/
    float    ALG_VAR_Y;                                                                         /*	variance of position error of Y axis in m2	*/
    float    ALG_VAR_Z;                                                                         /*	variance of position error of Z axis in m2	*/
    float    ALG_VAR_VX;										        					/*	variance of velocity error of X axis in (m/s)^2	*/
    float    ALG_VAR_VY;										        					/*	variance of velocity error of Y axis in (m/s)^2	*/
    float    ALG_VAR_VZ;										        					/*	variance of velocity error of Z axis in (m/s)^2	*/
    float    ALG_VAR_PSI;										        					/*	variance of orientation error of yaw axis in rad^2	*/
    float    ALG_VAR_THETA;										        					/*	variance of orientation error of pitch axis in rad^2	*/
    float    ALG_VAR_PHI;										        					/*	variance of orientation error of roll axis in rad^2	*/
    uint32_t GPS_INT_X;										        					/*	X axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
    uint32_t GPS_INT_Y;										        					/*	Y axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
    uint32_t GPS_INT_Z;										        					/*	Z axis in ECEF coordinates in integer value (to convert into cm multiply 2*pi/(2^32))	*/
}GKV_AllParameters;

/** 
  * @brief  Packet 0x27 with list of parameters of custom packet (send/receive) 
  */
typedef struct __GKV_CustomDataParam
{
	uint8_t num;                            /*	number of parameters that device sends in custom packet	*/
	uint8_t param[63];                      /*	type of parameter N from list of 'custom_data_parameters'	*/
}GKV_CustomDataParam;

enum ParameterType
{
    FloatParameter,
    Int32Parameter,
    Uint32Parameter
};
/*----------------------------------------------------------------------------DATA_REQUEST-------------------------------------------------------------------------------------*/

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



/** 
  * @brief  Packet 0x17 is used to request data (send only)
  */
typedef struct __GetData {}GetData;





#endif


#endif

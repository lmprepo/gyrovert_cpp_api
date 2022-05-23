#pragma once
/**
  ******************************************************************************
  * @file    GKV_SettingsPacket.h
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
#ifndef __GKV_SETTINGS_PACKET_H__
#define __GKV_SETTINGS_PACKET_H__




/** @defgroup SETTINGS_PACKET_CODES
  * @brief    packet codes for "type" field of settings packet
  * @{
  */ 
#define GKV_DEV_SETTINGS_REQUEST 					    0x06								/*	type of empty packet (with length = 0) to request device parameters packet	*/
#define GKV_DEV_SETTINGS_PACKET 					    0x07								/*	type of packet with device main settings	*/
/**
  * @}
  */




/** @defgroup DATA_FORMAT_SETTINGS
  * @brief 		data_format_bits for "mode" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_CUSTOM_PACK_LEN_CHANGEABLE  					1<<14                               /*  Length of custom paket may changes while parameters are updating */
#define GKV_YAW_0_360     									1<<13                               /*  Course angle only positive (range from 0 to 360) */
#define GKV_YAW_m180_180 									((0<<13)&(0xFFFF))                  /*  Course angle range from -180 to 180)   */
#define GKV_ADC_SYNC_OFF 									1<<12                               /*  Send data after processing (calculation time may vary)  */
#define GKV_ADC_SYNC_ON 									((0<<12)&(0xFFFF))                  /*  Set signal sync pin when ADC  data capture   */
#define GKV_BASE_FREQ_24_KHZ 								1<<11                               /*  ADC  data capture  frequency = 24 kHz */
#define GKV_BASE_FREQ_1_KHZ 								((0<<11)&(0xFFFF))                  /*  ADC  data capture  frequency = 1 kHz */
#define GKV_SET_CUSTOM_PACKET 								1<<10                               /*  sending custom data packet 0x13 with parameters set in packet 0x27 */
#define GKV_SET_DEFAULT_ALGORITHM_PACKET 			        ((0<<10)&(0xFFFF))                  /*  sending data packets according to the algorithm  */

/* adc data sync parameters	*/

#define GKV_SYNC_ADC_PULSE   								1<<9                                /*  Pulse signal sync pin when ADC  data capture  */
#define GKV_SYNC_ADC_TOGGLE 								((0<<9)&(0xFFFF))                   /*  Toggle signal sync pin when ADC  data capture  */

/* axis direction parameters	*/

#define GKV_Z_INVERTED 										1<<8                                /*  Z coordinate axis inverted  */
#define GKV_Y_INVERTED 										1<<7                                /*  Y coordinate axis inverted  */
#define GKV_X_INVERTED 										1<<6                                /*  X coordinate axis inverted  */

/* coordinates parameters	*/

#define GKV_YZX_COORDINATES 								1<<3                                /*  setting YZX coordinate system */
#define GKV_ZXY_COORDINATES 								2<<3                                /*  setting ZXY coordinate system */
#define GKV_XZY_COORDINATES 								3<<3                                /*  setting XZY coordinate system */
#define GKV_YXZ_COORDINATES 								4<<3                                /*  setting YXZ coordinate system */
#define GKV_ZYX_COORDINATES 								5<<3                                /*  setting ZYX coordinate system */

/* angle calculation mode parameters	*/

#define GKV_RADIANS_ANGLE_MODE 								1<<2                                /*  calculate angles as radians */
#define GKV_DEGREES_ANGLE_MODE 								((0<<2)&(0xFFFF))                   /*  calculate angles as degrees */

/* angular rate mode parameters	*/

#define GKV_RADIANS_RATE_MODE 								1<<1                                /*  calculate rates as radians  per second */
#define GKV_DEGREES_RATE_MODE 								((0<<1)&(0xFFFF))                   /*  calculate rates as degrees  per second */

/* acceleration mode parameters	*/

#define GKV_MS2_ACCEL_MODE 									1                                   /*  calculate acceleration as m/s^2 */
#define GKV_G_ACCEL_MODE 									((0)&&(0xFFFF))                     /*  calculate acceleration as g */
/**
  * @}
  */



/** @defgroup DATA_FORMAT_MASK
  * @brief 		allow parameters for "mode_mask" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_ALLOW_CHANGE_SYNC_DATA_MODE 					1<<12                           /*  when set - ADC_SYNC field of "data_format" can be changed  */
#define GKV_ALLOW_CHANGE_OUTPUT_FREQ 					    1<<11                           /*  when set - ADC_FREQ field of "data_format" can be changed  */
#define GKV_ALLOW_CHANGE_SELECTED_PACKET 			        1<<10                           /*  when set - custom packet/default algorithm packet mode of "data_format" can be changed  */
#define GKV_ALLOW_CHANGE_OUTPUT_SYNC 					    1<<9                            /*  when set - output synchrosignal mode of "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_Z_MODE 							1<<8                            /*  when set - Z_INVERTED field of "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_Y_MODE 							1<<7                            /*  when set - Y_INVERTED field of "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_X_MODE 							1<<6                            /*  when set - X_INVERTED field of "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_COORDINATES_HIGH_BIT               1<<5                            /*  when set - high bit of coordinates transformation part of " "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_COORDINATES_MID_BIT 	            1<<4                            /*  when set - mid bit of coordinates transformation part of " "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_COORDINATES_LOW_BIT 	            1<<3                            /*  when set - low bit of coordinates transformation part of "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_ANGLE_MODE  					    1<<2                            /*  when set - ANGLE_MODE  field of "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_RATE_MODE 						    1<<1                            /*  when set - RATE_MODE  field of "data_format" can be changed  */    
#define GKV_ALLOW_CHANGE_ACCEL_MODE 					    1                               /*  when set - ACCEL_MODE field of "data_format" can be changed  */    
/**
  * @}
  */


/** @defgroup DATA_PARAMETERS
  * @brief 		change parameters of "param_mask" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_SKIP_PACKETS 								    1<<9                                /*  Setting skip output data packets*/
#define GKV_CHANGE_SECOND_RS485_MODE		                1<<8                                /*  Setting of equipment type on optional RS-485    */
#define GKV_DCM_ROTATE 								        1<<7                                /*  Setting the DCM rotation matrix (guide cosines)  */
#define GKV_CHANGE_OUTPUT_SYNC_FREQ 		                1<<6                                /*  Setting the output sync prescaler  */
#define GKV_CHANGE_ACCEL_RANGE					      	    1<<5                                /*  Change accelerometers range */
#define GKV_CHANGE_GYRO_RANGE					      	    1<<4                                /*  Change rate sensors range   */
#define GKV_CHANGE_ALGORITHM					      	    1<<3                                /*  Change calculation algorithm    */
#define GKV_CHANGE_BASE_FREQ 					      	    1<<2                                /*  Change output data frequency    */
#define GKV_CHANGE_DEV_ADDRESS 				    	        1<<1                                /*  Change device address   */
#define GKV_CHANGE_BAUDRATE 					    	    1                                   /*  Change baudrate of main RS-485*/
/**
  * @}
  */




/** @defgroup BAUDRATE
  * @brief 		device main RS-485 birate for "uart_baud_rate" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_BAUDRATE_921600 								0x00
#define GKV_BAUDRATE_460800 								0x01
#define GKV_BAUDRATE_230400 								0x02
#define GKV_BAUDRATE_115200 								0x03
#define GKV_BAUDRATE_1000000 								0x04
#define GKV_BAUDRATE_2000000 								0x05
#define GKV_BAUDRATE_3000000 								0x06
/**
  * @}
  */




/** @defgroup CURRENT_ALGORITHM
  * @brief 		type of data processing for "algorithm" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_ADC_CODES_ALGORITHM 							0x00                               /*  send raw data from sensors  */
#define GKV_SENSORS_DATA_ALGORITHM 						    0x01                               /*  send calibrated data from sensors  */
#define GKV_ORIENTATION_KALMAN_ALGORITHM 			        0x02                               /*  send Euler angles calculated using Kalman Filter (also calculate quaternions) */
#define GKV_INCLINOMETER_ALGORITHM 					    	0x04                               /*  send inclinometer angles */
#define GKV_ORIENTATION_MAHONY_ALGORITHM 			        0x05                               /*  send Euler angles calculated using Mahony Filter (also calculate quaternions) */
#define GKV_BINS_NAVIGATON_ALGORITHM 				    	0x06                               /*  send navigation data calculated without sattelite correction */
#define GKV_CUSTOM_ALGORITHM 								0x07                               /*  set algorithm developed specially for customer */
#define GKV_KALMAN_GNSS_NAVIGATON_ALGORITHM 	            0x08                               /*  send navigation data calculated with sattelite correction */
#define GKV_ESKF5_NAVIGATON_ALGORITHM 	                    0x09                               /*  send navigation data calculated with sattelite correction by algorithm ESKF5*/
/**
  * @}
  */



/** @defgroup TYPE_OF_SECOND_RS-485
  * @brief 		type of connection protocol for "aux_485_type" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_SECOND_RS_485_OFF		 						0x00                               /*  Second RS-485  off  */
#define GKV_SECOND_RS_485_ASCII_NMEA				    	0x01                               /*  Receiving ASCII Messages via NMEA   */                                
#define GKV_SECOND_RS_485_GNSS_IRZ_MNP_X	 	        	0x02                               /*  GNSS Binary Protocol Receiver IRZ MNP-X */
#define GKV_SECOND_RS_485_GEOS_3M 				    		0x04                               /*  Binary protocol GEOS-3M */
#define GKV_SECOND_RS_485_NV08C_CSM				    		0x05                               /*  Binary Protocol NV08C-CSM   */
#define GKV_SECOND_RS_485_CUSTOM		 					0x06                               /*  set protocol by order of customer  */
#define GKV_SECOND_RS_485_ORIENT_SYSTEMS		           	0x07                               /*  Orient Systems Protocol (only on order with receiver)*/
#define GKV_SECOND_RS_485_NOVATEL_OEM_7			        	0x08                               /*  Protocol of Novatel OEM6    */
#define GKV_SECOND_RS_485_UBLOX								0x09                               /*  Ublox (NAV-PVT, NAV-HHREFLLA, NAVPOSNED)  */
#define GKV_SECOND_RS_485_PASSTHROUGH						0x0A                               /*  Passthrow (data received on second RS-485 will be sent to main port in 0x42 packet)  */

/**
  * @}
  */



/** @defgroup BAUDRATE
  * @brief 		device second RS-485 birate for "aux_485_baudrate" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_2BAUDRATE_921600 								0x00
#define GKV_2BAUDRATE_460800 								0x01
#define GKV_2BAUDRATE_230400 								0x02
#define GKV_2BAUDRATE_115200 								0x03
#define GKV_2BAUDRATE_1000000 								0x04
#define GKV_2BAUDRATE_2000000 								0x05
#define GKV_2BAUDRATE_3000000 								0x06
#define GKV_2BAUDRATE_4000000 								0x07
#define GKV_2BAUDRATE_500000 								0x08
#define GKV_2BAUDRATE_57600									0x09
#define GKV_2BAUDRATE_38400									0x0A
#define GKV_2BAUDRATE_19200									0x0B
#define GKV_2BAUDRATE_9600		 							0x0C
/**
  * @}
  */




/** @defgroup MAGNETOMETER_RANGE
  * @brief 		magnetometer range for "magnetometer_range" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_MAGN_RANGE_0_4_mTL 								0x00                            /*  Set magnetometer range as 0.4   mTl*/
#define GKV_MAGN_RANGE_0_8_mTL 								0x01                            /*  Set magnetometer range as 0.8   mTl*/
#define GKV_MAGN_RANGE_0_12_mTL 							0x02                            /*  Set magnetometer range as 0.12   mTl*/
#define GKV_MAGN_RANGE_0_16_mTL 							0x03                            /*  Set magnetometer range as 0.16   mTl*/

/**
  * @}
  */




/** @defgroup EXT_SYNC_MODE
  * @brief 		external sync mode for "ext_sync_mode" field of response packet 0x07	(device output settings)
  * @{
  */ 
#define GKV_EXT_SYNC_OFF 									0x00                            /*  External sync pin off   */
#define GKV_EXT_SYNC_1PPS_GNSS 								0x01                            /*  External sync pin connected to GNSS receiver with 1 pps sync  */
#define GKV_EXT_SYNC_DATA_BY_PULSE						    0x02                            /*  Send data when pulse  */
/**
  * @}
  */




/** 
  * @brief  Packet 0x07 with gkv device settings (send/receive)
  */
#pragma pack(push, 1)
typedef struct __GKV_Settings
{
	uint32_t mode_mask;                     /*	field to allow change data format parameters	*/
	uint32_t mode;                          /*	field to change data format parameters when	it is allowed	*/
	uint32_t param_mask;                    /*	field to allow change settings of data processing and sending	*/
	uint8_t uart_baud_rate;                 /*	baudrate of main RS-485 can be changed from 115200 bit/s to 3 MBit/s	*/
	uint8_t uart_address;                   /*	address of device */
	uint16_t rate_prescaler;                /*	basic freq of data packets for GKV = 1000 Hz */
	uint8_t algorithm;                      /*	type of sensors and GNSS data processing	*/
	uint8_t gyro_range;
	uint8_t acc_range;
	uint16_t sync_out_prescaler;            /*	changes the speed of data output by averaging	*/
	float dcm[9];                           /*	rotation matrix (3x3) of the measured data	*/
	uint8_t aux_485_type;                   /*	when external device (as GNSS receiver or custom device) is connected, user can receive and process	extended data	*/
	uint8_t data_out_skip;                  /*	number of skipping output packets	when output (custom) packet is too long to send it with default freq 1000 Hz*/
	uint8_t aux_485_baudrate;               /*	baudrate of additional RS-485 can be changed from 9600 bit/s to 3 MBit/s	*/
	uint8_t magnetometer_range;             /*	range of three axis magnetic sensor */
	uint8_t ext_sync_mode;                  /*	??????????	*/
}GKV_Settings;
#pragma pack(pop)


#endif
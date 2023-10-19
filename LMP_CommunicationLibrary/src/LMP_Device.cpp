/**
  ******************************************************************************
  * @file    LMP_Device.cpp
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 class module for data receiving and transmitting using selected serial device
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
#include "LMP_Device.h"
namespace Gyrovert
{
    /**
      * @name	LMP_Device
      * @brief  Default Constructor
      * @retval no return value.
      */
    LMP_Device::LMP_Device()
    {
        memset(CurrentReceivedPacket, 0, sizeof(GKV_PacketBase));
    }

    /**
      * @name	~LMP_Device
      * @brief  Default Destructor
      * @retval no return value.
      */
    LMP_Device::~LMP_Device()
    {
        delete(Output_Packet);
        delete(CurrentReceivedPacket);
    }



    /**
      * @name	SetSendDataFunction
      * @brief  Function sets pointer on user function for seril data transmition from PC to LMP Device
      * @param  ptrSendPacketFun - pointer on void-type callback function that gets pointer on PacketBase structure and sends "length" fields + 8 bytes
      * @retval no return value.
      */
    void  LMP_Device::SetSendDataFunction(std::function<void(GKV_PacketBase *)>ptrSendPacketFun)
    {
        ptrSendFun = ptrSendPacketFun;
    }

    /**
      * @name	SetReceiveDataFunction
      * @brief  Function sets pointer on user function for serial data receive to PC
      * @param  ptrRecPacketFun - pointer on char-type callback function that returns received byte from serial port
      * @retval no return value.
      */
    //void  LMP_Device::SetReceiveDataFunction(std::function<char*()>ptrRecPacketFun)
    //{
    //    ptrRecFcn = ptrRecPacketFun;
    //}

    /**
      * @name	SetReceivedPacketCallback
      * @brief  Function sets pointer on user function for processing every received and parsed packet from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received PacketBase structure
      * @retval no return value.
      */
    void  LMP_Device::SetReceivedPacketCallback(std::function<void(LMP_Device *, GKV_PacketBase *)> ptrReceivedPacketProcessingFun)
    {
        GKV_PacketProcessingCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetSettingsReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed settings packet (type 0x07) from LMP Device
      * @param  ptrSendPacketFun - pointer on void-type user callback function that gets pointer on received and parsed Settings structure
      * @retval no return value.
      */
    void LMP_Device::SetSettingsReceivedCallback(std::function<void(LMP_Device*, GKV_Settings *)> ptrReceivedPacketProcessingFun)
    {
        ptrSettingsPacketCallback = ptrReceivedPacketProcessingFun;
    }

     /**
    * @name	SetIDReceivedCallback
    * @brief  Function sets pointer on user function for processing received and parsed id packet (type 0x05) from LMP Device
    * @param  ptrSendPacketFun - pointer on void-type user callback function that gets pointer on received and parsed Settings structure
    * @retval no return value.
    */
    void LMP_Device::SetIDReceivedCallback(std::function<void(LMP_Device*, GKV_ID*)> ptrReceivedPacketProcessingFun)
    {
        ptrDeviceIDCallback = ptrReceivedPacketProcessingFun;
    }
    /**
      * @name	SetCustomPacketParamReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed custom parameters packet (type 0x27) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed custom data parameters structure
      * @retval no return value.
      */
    void LMP_Device::SetCustomPacketParamReceivedCallback(std::function<void(LMP_Device*, GKV_CustomDataParam *)> ptrReceivedPacketProcessingFun)
    {
        ptrCustomPacketParamCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetCustomPacketParamReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed custom packet (type 0x13) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed custom packet structure
      * @retval no return value.
      */
    void LMP_Device::SetCustomPacketReceivedCallback(std::function<void(LMP_Device*, GKV_CustomData *)>ptrReceivedPacketProcessingFun)
    {
        ptrCustomDataPacketCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetADCDataReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed ADC Codes packet (type 0x0A) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed ADC Codes structure
      * @retval no return value.
      */
    void LMP_Device::SetADCDataReceivedCallback(std::function<void(LMP_Device*, GKV_ADCData *)> ptrReceivedPacketProcessingFun)
    {
        ptrADCPacketCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetRawDataReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed Calibrated Sensors Data packet (type 0x0B) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Calibrated Sensors Data structure
      * @retval no return value.
      */
    void LMP_Device::SetRawDataReceivedCallback(std::function<void(LMP_Device*, GKV_RawData *)> ptrReceivedPacketProcessingFun)
    {
        ptrRawDataPacketCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetGyrovertDataReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed Orientation Data packet (type 0x0C) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Orientation Data structure
      * @retval no return value.
      */
    void LMP_Device::SetGyrovertDataReceivedCallback(std::function<void(LMP_Device*, GKV_GyrovertData *)> ptrReceivedPacketProcessingFun)
    {
        ptrGyrovertDataPacketCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetInclinometerDataReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed Inclinometer Data packet (type 0x0D) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Inclinometer Data structure
      * @retval no return value.
      */
    void LMP_Device::SetInclinometerDataReceivedCallback(std::function<void(LMP_Device*, GKV_InclinometerData *)>ptrReceivedPacketProcessingFun)
    {
        ptrInclinometerDataPacketCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetBINSDataReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed BINS Data packet (type 0x12) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed BINS Data structure
      * @retval no return value.
      */
    void LMP_Device::SetBINSDataReceivedCallback(std::function<void(LMP_Device*, GKV_BINSData *)> ptrReceivedPacketProcessingFun)
    {
        ptrBINSDataPacketCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetGNSSDataReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed GNSS Data packet (type 0x0E) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed GNSS Data structure
      * @retval no return value.
      */
    void LMP_Device::SetGNSSDataReceivedCallback(std::function<void(LMP_Device*, GKV_GpsData *)>ptrReceivedPacketProcessingFun)
    {
        ptrGNSSDataPacketCallback = ptrReceivedPacketProcessingFun;
    }

    /**
      * @name	SetExtGNSSDataReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed Extended GNSS Data packet (type 0x0F) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed Extended GNSS Data structure
      * @retval no return value.
      */
    void LMP_Device::SetExtGNSSDataReceivedCallback(std::function<void(LMP_Device*, GKV_GpsDataExt *)>ptrReceivedPacketProcessingFun)
    {
        ptrExtGNSSDataPacketCallback = ptrReceivedPacketProcessingFun;
    }


    /**
      * @name	SetConfirmPacketReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed request confirmation packet (type 0x00) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on device object
      * @retval no return value.
      */
    void LMP_Device::SetConfirmPacketReceivedCallback(std::function<void(LMP_Device*)> ptrReceivedPacketProcessingFun)
    {
        ptrConfirmPacketCallback = ptrReceivedPacketProcessingFun;
    }


    /**
      * @name	SetGyroOffsetsWrittenCallback
      * @brief  Function sets pointer on user function for processing received and parsed request confirmation packet of gyro offsets writing (type 0x1D) from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on device object
      * @retval no return value.
      */
    void LMP_Device::SetGyroOffsetsWrittenCallback(std::function<void(LMP_Device*)> ptrReceivedPacketProcessingFun)
    {
        ptrGyroOffsetsWrittenCallback = ptrReceivedPacketProcessingFun;
    }

    /**
          * @name	SetGyroOffsetsReceivedCallback
          * @brief  Function sets pointer on user function for processing received and parsed packet with gyro offsets (type 0x1E) from LMP Device
          * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on device object
          * @retval no return value.
          */
    void LMP_Device::SetGyroOffsetsReceivedCallback(std::function<void(LMP_Device*, GKV_GyroOffset*)>ptrReceivedPacketProcessingFun)
    {
        ptrGyroOffsetsReceivedCallback = ptrReceivedPacketProcessingFun;
    }


    /**
      * @name	SetIfProtoCommandResponseReceivedCallback
      * @brief  Function sets pointer on user function for processing received and parsed response for IfProtoCommand from LMP Device
      * @param  ptrReceivedPacketProcessingFun - pointer on void-type user callback function that gets pointer on received and parsed ifProto Response structure
      * @retval no return value.
      */
    void LMP_Device::SetIfProtoCommandResponseReceivedCallback(std::function<void(LMP_Device*, IfProtoConfig*)>ptrReceivedPacketProcessingFun)
    {
        ptrIfProtoPacketCallback = ptrReceivedPacketProcessingFun;
    }


    /**
      * @name	Configure_Output_Packet
      * @brief  Function inserts selected packet structure into base packet structure, sets values for basic fields and computes crc32.
      * @param  type - unsigned char value for type of transmitting packet
      * @param  data_ptr - pointer on beginning of packet structure that should be inserted into "data" field of transmitting packet. Packet can be empty
      * @param	size - length of data that should be copied from "data_ptr" into "data" field of transmitting packet
      * @retval no return value.
      */
    void LMP_Device::Configure_Output_Packet(uint8_t type, void* data_ptr, uint8_t size)
    {
        Output_Packet->preamble = 0xFF;
        Output_Packet->address = device_address;
        Output_Packet->type = type;
        Output_Packet->length = size;
        if (size)
        {
            memcpy(Output_Packet->data, data_ptr, size);
        }
        *((uint32_t*)&Output_Packet->data[size]) = crc32_compute(Output_Packet, Output_Packet->length + 4);
    }

    /**
      * @name	Send_Data
      * @brief  Function run void callback function that sending data to serial interface connected to GKV
      * @retval no return value.
      */
    void LMP_Device::Send_Data()
    {
        if (ptrSendFun)
        {
            ptrSendFun(Output_Packet);
        }
    }

    /**
      * @name	SendEmptyPacket
      * @brief  Function Configures and Sends Packet with length = 0 for different types of requests
      * @param  type - type of empty packet
      * @retval no return value.
      */
    void LMP_Device::SendEmptyPacket(uint8_t type)
    {
        Configure_Output_Packet(type, 0, 0);
        Send_Data();
    }

    /**
      * @name	SetAlgorithm
      * @brief  Function configures and sends Settings Packet (type=0x07) with selected algorithm number
      * @param  algorithm_register_value - number of selected algorithm
      * @retval no return value.
      */
    void LMP_Device::SetAlgorithm(uint8_t algorithm_register_value)
    {
        GKV_Settings GKV_Settings;
        memset(&GKV_Settings, 0, sizeof(GKV_Settings));
        uint8_t type = GKV_DEV_SETTINGS_PACKET;

        if (algorithm_register_value <= GKV_ESKF5_NAVIGATON_ALGORITHM)
        {
            GKV_Settings.param_mask |= GKV_CHANGE_ALGORITHM;
            GKV_Settings.algorithm = algorithm_register_value;
        }
        Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
        Send_Data();
    }

    /**
        *@name	SetDataRatePrescaler
        * @brief  Function configures and sends Settings Packet(type = 0x07) with selected data rate prescaler from default value (1000 Hz)
        * @param  rate_prescaler - quantity of packets filtered
        * @retval no return value.
        */
    void LMP_Device::SetDataRatePrescaler(uint16_t rate_prescaler)
    {
        GKV_Settings GKV_Settings;
        memset(&GKV_Settings, 0, sizeof(GKV_Settings));
        uint8_t type = GKV_DEV_SETTINGS_PACKET;
        if (rate_prescaler <= 1000)
        {
            GKV_Settings.param_mask |= GKV_CHANGE_BASE_FREQ;
            GKV_Settings.rate_prescaler = rate_prescaler;
        }
        Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
        Send_Data();
    }


    /**
    *@name	SetSkipOutputPackets
    * @brief  Function configures and sends Settings Packet(type = 0x07) with selected data packet skip number from current frequency selected by SetDataRatePrescaler().
              Recommended for navigation data, not for raw sensor data.
    * @param  data_out_skip - quantity of packets skipped
    * @retval no return value.
    */
    void LMP_Device::SetSkipOutputPackets(uint8_t data_out_skip)
    {
        GKV_Settings GKV_Settings;
        memset(&GKV_Settings, 0, sizeof(GKV_Settings));
        uint8_t type = GKV_DEV_SETTINGS_PACKET;

        if (data_out_skip <= 255)
        {
            GKV_Settings.param_mask |= GKV_SKIP_PACKETS;
            GKV_Settings.data_out_skip = data_out_skip;
        }
        Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
        Send_Data();
    }

    /**
    *@name	SetSecondRS485PortMode
    * @brief  Function configures and sends Settings Packet(type = 0x07) with selected mode of second RS-485
    * @param  mode - selected type of GNSS receiver for second RS-485 or passthrough mode
    * @retval no return value.
    */
    void LMP_Device::SetSecondRS485PortMode(uint8_t mode)
    {
        GKV_Settings GKV_Settings;
        memset(&GKV_Settings, 0, sizeof(GKV_Settings));
        uint8_t type = GKV_DEV_SETTINGS_PACKET;
        if (mode <= GKV_SECOND_RS_485_PASSTHROUGH)
        {
            GKV_Settings.param_mask |= GKV_CHANGE_SECOND_RS485_MODE;
            GKV_Settings.aux_485_type = mode;
        }
        Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
        Send_Data();
    }

    /**
      * @name	SetSyncOutType
      * @brief  Function configures and sends Settings Packet (type=0x07) with type of output synchrosignal (pulse or toggle).
      * @retval no return value.
      */
    void LMP_Device::SetSyncOutType(uint16_t sync_type)
    {
        GKV_Settings GKV_Settings;
        memset(&GKV_Settings, 0, sizeof(GKV_Settings));
        uint8_t type = GKV_DEV_SETTINGS_PACKET;
        GKV_Settings.mode = sync_type;
        GKV_Settings.mode_mask = GKV_ALLOW_CHANGE_OUTPUT_SYNC;
        Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
        Send_Data();
    }

    /**
      *  *@name	SetAUXPortMode
        * @brief  Function configures and sends IfProto Packet(type = 0x30) with selected mode of second RS-485 using IfProto Settings.
        * @retval no return value.
        */
    void LMP_Device::SetAUXPortMode(uint8_t port, uint16_t protocol, uint16_t protocol_param, uint16_t baudrate, uint16_t mode)
    {
        struct IfProtoConfig IFProtpDataField;
        struct IfUartConfig UartConfigField;
        IFProtpDataField.iface = port;
        IFProtpDataField.cmd = IfProtoConfig::CMD_CONFIG;
        IFProtpDataField.index = 0;
        IFProtpDataField.reserved = 0;
        UartConfigField.baudrate = baudrate;
        UartConfigField.proto = protocol;
        UartConfigField.proto_param = protocol_param;
        UartConfigField.mode = mode;
        memcpy(&(IFProtpDataField.payload), &UartConfigField, sizeof(UartConfigField));
        Configure_Output_Packet(IfProtoConfig::type, &IFProtpDataField, sizeof(UartConfigField) + 4);
        Send_Data();
    }

  /**
  *  *@name	ForwardAUXPDataToMain
    * @brief  Function configures and sends IfProto Packet(type = 0x30) with selected mode of second RS-485 using IfProto Settings.
    * @retval no return value.
    */
    void LMP_Device::SetAUXPPassthroughParams(uint8_t port, uint16_t buf_size, uint8_t target_iface)
    {
        struct IfProtoConfig IFProtpDataField;
        struct IfUartForward UartConfigField;
        IFProtpDataField.iface = port;
        IFProtpDataField.cmd = IfProtoConfig::CMD_FORWARD;
        IFProtpDataField.index = 0;
        IFProtpDataField.reserved = 0;
        UartConfigField.buf_size = buf_size;
        UartConfigField.iface = target_iface;
        memcpy(&(IFProtpDataField.payload), &UartConfigField, sizeof(UartConfigField));
        Configure_Output_Packet(IfProtoConfig::type, &IFProtpDataField, sizeof(UartConfigField) + 4);
        Send_Data();
    }

    /**
      *  *@name	SetCANPortMode
        * @brief  Function configures and sends IfProto Packet(type = 0x30) with selected mode of CAN interface using IfProto Settings.
        * @retval no return value.
        */
    void LMP_Device::SetCANPortMode(uint8_t port, uint16_t protocol, uint16_t baudrate)
    {
        struct IfProtoConfig IFProtpDataField;
        struct IfCanConfig CANConfigField;
        IFProtpDataField.iface = port;
        IFProtpDataField.cmd = IfProtoConfig::CMD_CONFIG;
        IFProtpDataField.index = 0;
        IFProtpDataField.reserved = 0;
        CANConfigField.baudrate_k = baudrate;
        CANConfigField.proto = protocol;
        memcpy(&(IFProtpDataField.payload), &CANConfigField, sizeof(CANConfigField));
        Configure_Output_Packet(IfProtoConfig::type, &IFProtpDataField, sizeof(CANConfigField) + 4);
        Send_Data();
    }


    /**
      *  *@name	SetCANPortMsg
        * @brief  Function configures and sends IfProto Packet(type = 0x30) with selected message that will be sent by CAN interface.
        * @retval no return value.
        */
    void LMP_Device::SetCANPortMsg(uint8_t port, uint8_t index, uint16_t prescaler, CanId id, float limit)
    {
        IfProtoConfig IFProtpDataField;
        IfCanMessage CANMesField;
        IFProtpDataField.iface = port;
        IFProtpDataField.cmd = IfProtoConfig::CMD_MESSAGE;
        IFProtpDataField.index = index;
        IFProtpDataField.reserved = 0;
        CANMesField.prescaler = prescaler;
        CANMesField.id = id;
        CANMesField.limit = limit;
        memcpy(&(IFProtpDataField.payload), &CANMesField, sizeof(CANMesField));
        Configure_Output_Packet(IfProtoConfig::type, &IFProtpDataField, sizeof(CANMesField) + 4);
        Send_Data();
    }
    /**
      * @name	SetBaudrate
      * @brief  Function configures and sends Settings Packet (type=0x07) with selected baudrate number
      * @param  baudrate_register_value - number of selected baudrate of main RS-422 interface
      * @retval no return value.
      */
    void LMP_Device::SetBaudrate(uint8_t baudrate_register_value)
    {
        GKV_Settings GKV_Settings;
        memset(&GKV_Settings, 0, sizeof(GKV_Settings));
        uint8_t type = GKV_DEV_SETTINGS_PACKET;

        if (baudrate_register_value <= GKV_BAUDRATE_3000000)
        {
            GKV_Settings.param_mask |= GKV_CHANGE_BAUDRATE;
            GKV_Settings.uart_baud_rate = baudrate_register_value;
        }
        Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
        Send_Data();
    }

    /**
      * @name	SetDefaultAlgorithmPacket
      * @brief  Function configures and sends Settings Packet (type=0x07) with set sending mode as default packet for current algorithm.
      * @retval no return value.
      */
    void LMP_Device::SetDefaultAlgorithmPacket()
    {
        GKV_Settings GKV_Settings;
        memset(&GKV_Settings, 0, sizeof(GKV_Settings));
        uint8_t type = GKV_DEV_SETTINGS_PACKET;

        GKV_Settings.mode = GKV_SET_DEFAULT_ALGORITHM_PACKET;
        GKV_Settings.mode_mask = GKV_ALLOW_CHANGE_SELECTED_PACKET;
        Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
        Send_Data();
    }

    /**
      * @name	SetCustomAlgorithmPacket
      * @brief  Function configures and sends Settings Packet (type=0x07) with set sending mode as custom packet for current algorithm.
      * @retval no return value.
      */
    void LMP_Device::SetCustomAlgorithmPacket()
    {
        GKV_Settings GKV_Settings;
        memset(&GKV_Settings, 0, sizeof(GKV_Settings));
        uint8_t type = GKV_DEV_SETTINGS_PACKET;
        GKV_Settings.mode = GKV_SET_CUSTOM_PACKET;
        GKV_Settings.mode_mask = GKV_ALLOW_CHANGE_SELECTED_PACKET;
        Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
        Send_Data();
    }

    /**
      * @name	SetAccelerationUnits
      * @brief  Function configures and sends Settings Packet (type=0x07) with selected units of linear acceleration.
      * @retval no return value.
      */
    void LMP_Device::SetAccelerationUnits(uint8_t units)
    {
      GKV_Settings GKV_Settings;
      memset(&GKV_Settings, 0, sizeof(GKV_Settings));
      uint8_t type = GKV_DEV_SETTINGS_PACKET;
      if (units==GKV_MS2)
      {
        GKV_Settings.mode = GKV_MS2_ACCEL_MODE;
      }
      else {
        GKV_Settings.mode = GKV_G_ACCEL_MODE;

      }
      GKV_Settings.mode_mask = GKV_ALLOW_CHANGE_ACCEL_MODE;
      Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
      Send_Data();
    }


    /**
      * @name	SetAngularRateUnits
      * @brief  Function configures and sends Settings Packet (type=0x07) with selected units of angular velocity.
      * @retval no return value.
      */
    void LMP_Device::SetAngularRateUnits(uint8_t units)
    {
      GKV_Settings GKV_Settings;
      memset(&GKV_Settings, 0, sizeof(GKV_Settings));
      uint8_t type = GKV_DEV_SETTINGS_PACKET;
      if (units==GKV_DEGREES_PER_SECOND)
      {
        GKV_Settings.mode = GKV_DEGREES_RATE_MODE;
      }
      else {
        GKV_Settings.mode = GKV_RADIANS_RATE_MODE;

      }
      GKV_Settings.mode_mask = GKV_ALLOW_CHANGE_RATE_MODE;
      Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
      Send_Data();
    }

    /**
      * @name	SetAngleUnits
      * @brief  Function configures and sends Settings Packet (type=0x07) with selected units of angular velocity.
      * @retval no return value.
      */
    void LMP_Device::SetAngleUnits(uint8_t units)
    {
      GKV_Settings GKV_Settings;
      memset(&GKV_Settings, 0, sizeof(GKV_Settings));
      uint8_t type = GKV_DEV_SETTINGS_PACKET;
      if (units==GKV_DEGREES)
      {
        GKV_Settings.mode = GKV_DEGREES_ANGLE_MODE;
      }
      else
      {
        GKV_Settings.mode = GKV_RADIANS_ANGLE_MODE;
      }
      GKV_Settings.mode_mask = GKV_ALLOW_CHANGE_ANGLE_MODE;
      Configure_Output_Packet(type, &GKV_Settings, sizeof(GKV_Settings));
      Send_Data();
    }
    /**
      * @name	SetCustomPacketParam
      * @brief  Function configures and sends Custom Parameters Packet (type=0x27) with selected quantity and numbers of selected parameters
      * @param  param_array_ptr - pointer on array of bytes with numbers of custom data parameters
      * @param  quantity_of_params - quantity of selected parameters in byte array
      * @retval no return value.
      */
    void LMP_Device::SetCustomPacketParam(uint8_t* param_array_ptr, uint8_t quantity_of_params)
    {
        GKV_CustomDataParam GKV_CustomDataParam;
        memset(&GKV_CustomDataParam, 0, sizeof(GKV_CustomDataParam));
        uint8_t type = GKV_CUSTOM_DATA_PARAM_PACKET;
        
        for (uint8_t i = 0; i < quantity_of_params; i++)
        {
            if (*(param_array_ptr + i) > GKV_CUSTOM_PACKET_PARAMS_QUANTITY)
            {
                memmove((param_array_ptr + i), (param_array_ptr + i+1), (quantity_of_params-i-1));
                quantity_of_params--;
            }
        }
        GKV_CustomDataParam.num = quantity_of_params;
        memcpy(&(GKV_CustomDataParam.param), param_array_ptr, quantity_of_params);
        Configure_Output_Packet(type, &GKV_CustomDataParam, sizeof(GKV_CustomDataParam));
        Send_Data();
    }

    /**
      * @name	RequestSettings
      * @brief  Function Configures and sends Empty Packet with Settings Request Type (type=0x06)
      * @retval no return value.
      */
    void LMP_Device::RequestSettings()
    {
        SendEmptyPacket(GKV_DEV_SETTINGS_REQUEST);
    }

    /**
      * @name	RequestDeviceID
      * @brief  Function Configures and sends Empty Packet with ID Request Type (type=0x04)
      * @retval no return value.
      */
    void LMP_Device::RequestDeviceID()
    {
        SendEmptyPacket(GKV_DEV_ID_REQUEST);
    }

    /**
      * @name	RequestData
      * @brief  Function Configures and sends Empty Packet with Data Request Type (type=0x17)
      * @retval no return value.
      */
    void LMP_Device::RequestData()
    {
        SendEmptyPacket(GKV_DATA_REQUEST);
    }


    /**
      * @name	CheckConnection
      * @brief  Function Configures and sends Empty Packet with Check Connection Type (type=0x00)
      * @retval no return value.
      */
    void LMP_Device::CheckConnection()
    {
        SendEmptyPacket(GKV_CHECK_PACKET);
    }

    /**
      * @name	RequestCustomPacketParams
      * @brief  Function Configures and sends Empty Packet with Request of Custom Parameters List Type (type=0x26)
      * @retval no return value.
      */
    void LMP_Device::RequestCustomPacketParams()
    {
        SendEmptyPacket(GKV_CUSTOM_PACKET_PARAM_REQUEST);
    }

    /**
      * @name	CalculateGyroOffsets
      * @brief  Function Configures and sends Packet with number of samples for gyro offset calculation (type=0x1C)
      * @retval no return value.
      */
    void LMP_Device::CalculateGyroOffsets(uint32_t samples)
    {
        GKV_GyroOffsetCalc gyroOffsetRequest;
        gyroOffsetRequest.samples = samples;
        Configure_Output_Packet(GKV_GYRO_OFFSET_CALC_REQUEST, &gyroOffsetRequest, sizeof(gyroOffsetRequest));
        Send_Data();
    }

    /**
      * @name	RequestGyroOffsets
      * @brief  Function Configures and sends Empty Packet with Request of Gyro offsets for selected samples numbers (type=0x1D)
      * @retval no return value.
      */
    void LMP_Device::RequestGyroOffsets()
    {
        SendEmptyPacket(GKV_GYRO_OFFSET_REQUEST);
    }
    /**
      * @name	SetGyroOffsets
      * @brief  Function Configures and sends Packet with Gyro offsets in 24bit ADC-codes (type=0x1E)
      * @retval no return value.
      */
    void LMP_Device::SetGyroOffsets(int32_t offset_x, int32_t offset_y, int32_t offset_z)
    {
        GKV_GyroOffset gyroOffsets;
        gyroOffsets.x_900 = offset_x;
        gyroOffsets.y_900 = offset_y;
        gyroOffsets.z_900 = offset_z;
        Configure_Output_Packet(GKV_GYRO_OFFSET_PACKET, &gyroOffsets, sizeof(gyroOffsets));
        Send_Data();
    }


    /**
      * @name	ResetDevice
      * @brief  Function Configures and sends Empty Packet with Request of Custom Parameters List Type (type=0x26)
      * @retval no return value.
      */
    void LMP_Device::ResetDevice()
    {
        SendEmptyPacket(GKV_RESET_PACKET);
    }

    /**
      * @name	crc32_compute
      * @brief  CRC32 checksum calculation
      * @param  buf - pointer on beginning of packet
      * @param  size - length of packet without checksum
      * @retval function returns result crc32 calculation.
      */
    uint32_t LMP_Device::crc32_compute(const void* buf, unsigned long size)
    {
        uint32_t crc = 0;
        const uint8_t* p = (const uint8_t*)buf;
        crc = crc ^ 0xFFFFFFFFUL;

        while (size--)
            crc = crc32_tabl[(crc ^ *p++) & 0xFF] ^ (crc >> 8);

        return crc ^ 0xFFFFFFFFUL;
    }

    /**
      * @name	check
      * @brief  chcecking input packet buffer for crc32
      * @param  pack - pointer on beginning of input packet buffer
      * @retval function returns result of checking crc32. 0x00 - checksum is incorrect, 0x01 - checksum is correct
      */
    uint8_t LMP_Device::check(GKV_PacketBase* pack)
    {
        if (pack->preamble != 0xFF)
            return false;
        if (pack->length > 255)
            return false;

        uint32_t crc = crc32_compute(pack, pack->length + 4);
        uint8_t* p = &pack->data[pack->length];
        uint8_t* t = (uint8_t*)&crc;
        if (*p++ != *t++) return false;
        if (*p++ != *t++) return false;
        if (*p++ != *t++) return false;
        if (*p++ != *t++) return false;

        return true;
        return false;


    }

    /**
      * @name	put
      * @brief  Function checks current received byte, searching preamble and after finding it puts it into input packet buffer and increments counter
      * @param  b - byte received from serial port connected to GKV
      * @retval function returns result of searching preamble and returns zero until it found.
      */
    uint8_t LMP_Device::put(uint8_t b)//???????? ?? ?????????
    {
        //if (DataWritingEnabled)
        //{
        //    if (WritingMode == 0)
        //    {
        //        buffer_1[buffer_point_1] = b;
        //        buffer_point_1++;
        //        if (buffer_point_1 > (100000 - 0xFFFF))
        //        {
        //            WritingMode = 1;
        //            WriteBuffer1Flag = true;
        //        }
        //    }
        //    else
        //    {
        //        buffer_2[buffer_point_2] = b;
        //        buffer_point_2++;
        //        if (buffer_point_2 > (100000 - 0xFFFF))
        //        {
        //            WritingMode = 0;
        //            WriteBuffer2Flag = true;
        //        }
        //    }
        //}
        if (CTR == 0)
        {
            if (b != 0xFF)
            {
                return 0;
            }
        }
        *((uint8_t*)(&InputPacket) + CTR) = b;
        CTR++;
        return 1;
    }

    /**
      * @name	Receive_Process
      * @brief  Main function of received data processing. It can be inserted into main cycle and calls when byte received. function forms packet with received bytes and runs callback fucntion when it formed.
      * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
      */
    //uint8_t LMP_Device::Receive_Process()
    //{
    //    char buffer_byte = 0;
    //    if (ptrRecFcn)
    //    {
    //        buffer_byte = ptrRecFcn();
    //    }
    //    else
    //    {
    //        buffer_byte = ReadDataFromGKV();
    //    }
    //    if (put(buffer_byte))
    //    {
    //        return parseCycle();
    //    }
    //    return 0;
    //}

    uint8_t LMP_Device::Receive_Process(char* data_ptr, uint16_t buffer_size)
    {
        uint8_t result = 0;
            for (uint16_t i = 0; i < buffer_size; i++)
            {
                if (put(*(data_ptr + i)))
                {
                    result = parseCycle();
                }
            }
        /*}*/
        return result;
    }


    /**
      * @name	parseCycle
      * @brief  Parcing cycle function. When new byte added to input packet buffer. Function checks number of received bytes and checksum result.
      * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
      */
    uint8_t LMP_Device::parseCycle()
    {
        uint8_t status = 0;
        while (1)
        {
            status = parse();
            if (status == NOT_ENOUGH)
            {
                break;
            }
            else if (status == REFIND_PREAMBLE)
            {
                if (!refind_preamble(1))
                    break;
            }
            else if (status == CHECK_OK)
            {
                memcpy(CurrentReceivedPacket, &InputPacket, (((GKV_PacketBase*)&InputPacket)->length + 8));
                if (GKV_PacketProcessingCallback)
                {
                    GKV_PacketProcessingCallback(this,CurrentReceivedPacket);
                }
                RecognisePacket(CurrentReceivedPacket);
                if (!refind_preamble(((GKV_PacketBase*)&InputPacket)->length + 8))
                    break;
            }
        }
        if (status < CHECK_OK)
        {
            status = 0;
        }
        return status;
    }



    /**
      * @name	refind_preamble
      * @brief  Moving memory function when checksum is incorrect to check next 0xFF as preamble or moving memory when correct packet processed.
      * @param  start - byte with number of start byte to move memory when checksum is incorrect (1) or when received packet is correct (buf->length) + 8 .
      * @retval Function returns 1 when 0xFF found after memory moving and 0, when it wasn't found.
      */
    uint8_t LMP_Device::refind_preamble(int start)
    {
        uint8_t* in_buf = (uint8_t*)(InputPacket);
        for (int i = start; i < CTR; i++)
        {
            if (*(in_buf + i) == 0xFF)
            {
                CTR -= i;
                memmove(in_buf, (in_buf + i), CTR);
                return 1;
            }
        }
        CTR = 0;
        return 0;
    }



    /**
      * @name	parse
      * @brief  Parcing step function. Trying to find correct packet in current quantity of received bytes
      * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
      */
    uint8_t LMP_Device::parse()
    {
        if (CTR >= 4)
        {
            if (((GKV_PacketBase*)&InputPacket)->length > GKV_DATA_LENGTH)
            {
                return REFIND_PREAMBLE;
            }
        }
        if (CTR < (((GKV_PacketBase*)&InputPacket)->length) + 8)
        {
            return NOT_ENOUGH;
        }
        if (!check((GKV_PacketBase*)&InputPacket))
        {
            return REFIND_PREAMBLE;
        }
        return CHECK_OK;
    }

    /**
      * @name	GetInputPacketType
      * @brief  Function sets pointer on user function for processing received and parsed custom parameters packet (type 0x27) from LMP Device
      * @retval function returns value of last received packet type.
      */
    uint8_t LMP_Device::GetInputPacketType()
    {
        return (CurrentReceivedPacket->type);
    }

    /**
      * @name	RecognisePacket
      * @brief  Default callback for every received packet
      * @param  buf - pointer on received packet
      * @retval no return value.
      */
    void LMP_Device::RecognisePacket(GKV_PacketBase* buf)
    {
        switch (buf->type)
        {
        case GKV_ADC_CODES_PACKET:
        {
            GKV_ADCData data;
            memcpy(&(data), &(buf->data), sizeof(data));
            if (ptrADCPacketCallback)
            {
                ptrADCPacketCallback(this, &data);
            }
            break;
        }
        case GKV_RAW_DATA_PACKET:
        {
            GKV_RawData data;
            memcpy(&(data), &(buf->data), sizeof(data));
            if (ptrRawDataPacketCallback)
            {
                ptrRawDataPacketCallback(this, &data);
            }
            break;
        }
        case GKV_EULER_ANGLES_PACKET:
        {
            GKV_GyrovertData data;
            memcpy(&(data), &(buf->data), sizeof(data));
            if (ptrGyrovertDataPacketCallback)
            {
                ptrGyrovertDataPacketCallback(this, &data);
            }
            break;
        }
        case GKV_INCLINOMETER_PACKET:
        {
            GKV_InclinometerData data;
            memcpy(&(data), &(buf->data), sizeof(data));
            if (ptrInclinometerDataPacketCallback)
            {
                ptrInclinometerDataPacketCallback(this, &data);
            }
            break;
        }
        case GKV_BINS_PACKET:
        {
            GKV_BINSData data;
            memcpy(&(data), &(buf->data), sizeof(data));
            if (ptrBINSDataPacketCallback)
            {
                ptrBINSDataPacketCallback(this, &data);
            }
            break;
        }
        case GKV_GNSS_PACKET:
        {
            GKV_GpsData data;
            memcpy(&(data), &(buf->data), sizeof(data));

            if (ptrGNSSDataPacketCallback)
            {
                ptrGNSSDataPacketCallback(this, &data);
            }
            break;
        }
        case GKV_CUSTOM_PACKET:
        {
            GKV_CustomData data;
            memcpy(&(data), &(buf->data), buf->length);
            if (ptrCustomDataPacketCallback)
            {
                ptrCustomDataPacketCallback(this, &data);
            }
            break;
        }
        case GKV_DEV_ID_PACKET:
        {
            memcpy(&(DeviceState.GeneralDeviceParameters), &(buf->data), sizeof(GKV_ID));
            device_address=buf->address;
            DeviceIDRequestedFlag = false;
            if (ptrDeviceIDCallback)
            {
                ptrDeviceIDCallback(this, &(DeviceState.GeneralDeviceParameters));
            }
            break;
        }
        case GKV_DEV_SETTINGS_PACKET:
        {
            memcpy(&(DeviceState.CurrentSettings), &(buf->data), sizeof(GKV_Settings));
            SettingsRequestedFlag = false;
            if (ptrSettingsPacketCallback)
            {
                ptrSettingsPacketCallback(this, &(DeviceState.CurrentSettings));
            }
            break;
        }
        case GKV_CUSTOM_DATA_PARAM_PACKET:
        {
            memcpy(&(DeviceState.CurrentCustomPacketParameters), &(buf->data), sizeof(GKV_CustomDataParam));
            CustomPacketParamRequestedFlag = false;
            CustomPacketParamReceivedFlag = true;
            if (ptrCustomPacketParamCallback)
            {
                ptrCustomPacketParamCallback(this, &(DeviceState.CurrentCustomPacketParameters));
            }
            break;
        }
        case GKV_CONFIRM_PACKET:
        {
            if (ptrConfirmPacketCallback)
            {
                ptrConfirmPacketCallback(this);
            }
            break;
        }
        case GKV_GYRO_OFFSET_REQUEST:
        {
            if (ptrGyroOffsetsWrittenCallback)
            {
                ptrGyroOffsetsWrittenCallback(this);
            }
            break;
        }
        case GKV_GYRO_OFFSET_PACKET:
        {
            GKV_GyroOffset data;
            memcpy(&(data), &(buf->data), buf->length);
            if (ptrGyroOffsetsReceivedCallback)
            {
                ptrGyroOffsetsReceivedCallback(this, &data);
            }
            break;
        }
        case IfProtoConfig::type:
        {
            IfProtoConfig data;
            memcpy(&(data), &(buf->data), buf->length);
            if (ptrIfProtoPacketCallback)
            {
                ptrIfProtoPacketCallback(this, &data);
            }
            break;
        }
        }
    }

 
}

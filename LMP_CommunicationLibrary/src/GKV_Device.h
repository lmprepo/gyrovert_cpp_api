/**
  ******************************************************************************
  * @file    GKV_Device.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 class module that inherits LMP_Device class and includes functions 
  * for data transmitting and receiving using standard serial libraries for Linux 
  * and Windows. Added in library since 1.1 version
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

#ifndef GKV_DEVICE_H
#define GKV_DEVICE_H

#include <LMP_Device.h>
#include <vector>
namespace Gyrovert
{
#define GKV_SELECT_DEFAULT_ALGORITHM_PACKET 0
#define GKV_SELECT_CUSTOM_PACKET 1
// windows class for GKV serial port comm
#ifdef _WIN32
#include <windows.h>
    class GKV_Device : public LMP_Device
    {
    public:
        HANDLE hSerial;
        GKV_Device(std::string serial_port, uint32_t baudrate) : LMP_Device()
        {
            SerialInitialized=InitSerialPort(serial_port, baudrate);
        }
        ~GKV_Device() {}

        bool InitSerialPort(std::string port_name, uint32_t baudrate)
        {
            port_name = "\\\\.\\" + port_name;
            hSerial = CreateFileA( port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
            if (hSerial == INVALID_HANDLE_VALUE)
            {
                if (GetLastError() == ERROR_FILE_NOT_FOUND)
                {
                    return 0;
                }
                return 0;
            }
            DCB dcbSerialParams = { 0 };
            dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
            if (!GetCommState(hSerial, &dcbSerialParams))
            {
                return 0;
            }
            dcbSerialParams.BaudRate = baudrate;
            dcbSerialParams.ByteSize = 8;
            dcbSerialParams.StopBits = ONESTOPBIT;
            dcbSerialParams.Parity = NOPARITY;
            if (!SetCommState(hSerial, &dcbSerialParams))
            {
                return 0;
            }
            return 1;
        }

        void WriteDataToGKV(GKV_PacketBase* data) override
        {
            if (SerialInitialized)
            {
                DWORD dwBytesWritten;
                char iRet = WriteFile(hSerial, data, data->length + 8, &dwBytesWritten, NULL);
                Sleep(1);
            }
        }

        char* ReadDataFromGKV() override
        {
            if (SerialInitialized)
            {
                DWORD iSize;
                char sReceivedChar;
                char iRet = 0;
                while (true)
                {
                    iRet = ReadFile(hSerial, &inBuffer, 2048, &iSize, 0);
                    if (iRet)
                    {
                        if (iSize > 0)
                        {
                            SetReceiveBufferSize(iSize);
                            return inBuffer;
                        }
                    }
                    _sleep(10);
                }
            }
            return 0;
        }

        bool GetSerialConnectionState()
        {
            return SerialInitialized;
        }
    private:
        char inBuffer[2048] = { 0 };
        bool SerialInitialized = false;
    };
#else
#ifdef __linux
#include <stdlib.h>
#include <unistd.h>     
#include <fcntl.h>      
#include <termios.h>   

    class GKV_Device : public LMP_Device
    {
    public:
        int SerialPortHandle;
        GKV_Device(std::string serial_port, uint32_t baudrate) : LMP_Device()
        {
            SerialInitialized = InitSerialPort(serial_port, baudrate);
        }
        ~GKV_Device() {}

        bool InitSerialPort(std::string port_name, uint32_t baudrate)
        {
            SerialPortHandle = open(port_name.c_str(), O_RDWR | O_NOCTTY);
            if (SerialPortHandle < 0) {
                return 0;
            }
            struct termios tty;
            struct termios tty_old;
            memset(&tty, 0, sizeof tty);
            /* Error Handling */
            if (tcgetattr(SerialPortHandle, &tty) != 0) {
                return 0;
            }
            /* Save old tty parameters */
            tty_old = tty;
            /* Set Baud Rate */
            cfsetospeed(&tty, (speed_t)baudrate);
            cfsetispeed(&tty, (speed_t)baudrate);
            /* Setting other Port Stuff */
            tty.c_cflag &= ~PARENB;            // Make 8n1
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;

            tty.c_cflag &= ~CRTSCTS;           // no flow control
            tty.c_cc[VMIN] = 1;                  // read doesn't block
            tty.c_cc[VTIME] = 5;                  // 0.5 seconds read timeout
            tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
            /* Make raw */
            cfmakeraw(&tty);
            /* Flush Port, then applies attributes */
            tcflush(SerialPortHandle, TCIFLUSH);
            if (tcsetattr(SerialPortHandle, TCSANOW, &tty) != 0) {
                return 0;
            }
            return 1;
        }

        void WriteDataToGKV(GKV_PacketBase* data) override
        {
            if (SerialInitialized)
            {
                int iOut = write(SerialPortHandle, data, data->length + 8);
                usleep(1000);
            }
        }
        char* ReadDataFromGKV() override
        {
            if (SerialInitialized)
            {
                int iOut;
                char sReceivedChar;
                while (true)
                {
                    iOut = read(SerialPortHandle, &inBuffer, 2048);
                    SetReceiveBufferSize(iOut);
                    return inBuffer;
                }
            }
            return 0;
        }
        bool GetSerialConnectionState()
        {
            return SerialInitialized;
        }
    private:
        char inBuffer[2048] = { 0 };
        bool SerialInitialized = false;
};
#endif // __linux
#endif

}
#endif

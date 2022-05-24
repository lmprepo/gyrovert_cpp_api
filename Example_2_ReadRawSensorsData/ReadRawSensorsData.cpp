#ifdef _WIN32
#include <windows.h>
#define SerialPortHandle HANDLE
#define GKV_BAUDRATE_921600 921600
#endif
#ifdef __linux
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#define SerialPortHandle int
#define GKV_BAUDRATE_921600 B921600
#endif

#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace Gyrovert;
using namespace std;

SerialPortHandle hSerial;
uint8_t algorithm_selected = 0;

bool InitSerialPort(string port_name, int32_t baudrate);
void ReadGkvData(LMP_Device* dev);
void WriteCOM(GKV_PacketBase* buf);
void ShowPacketData(LMP_Device* GKV, GKV_RawData* buf);

int main()
{
    /* Select Serial Port */
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    //Create LMP Device Object GKV
    LMP_Device* GKV = new LMP_Device();
    //Serial Port Settings For Windows
    #ifdef _WIN32
    com_port = "\\\\.\\" + com_port;
    #endif
    if (!(InitSerialPort(com_port, GKV_BAUDRATE_921600))) return 1;
    // GKV Settings
    GKV->SetSendDataFunction(WriteCOM);/*Set User Function That Sends Data to Serial Port connected to GKV*/
    GKV->SetRawDataReceivedCallback(ShowPacketData);/*Set User Callback for Parsed Sensors Data Packet*/

    cout << "#start main loop\n";
    while (1)
    {
        if (!(algorithm_selected))
        {
            GKV->SetDefaultAlgorithmPacket();
            GKV->SetAlgorithm(GKV_SENSORS_DATA_ALGORITHM);
        }
        ReadGkvData(GKV);
    }
    return 0;
}



/*Sensors Data Packet Receive Callback*/
void ShowPacketData(LMP_Device* GKV, GKV_RawData* packet)
{
    string output_str;
    output_str.append("Sensors Data Packet: ");
    output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
    output_str.append(" ax = " + std::to_string(packet->a[0]));
    output_str.append(" ay = " + std::to_string(packet->a[1]));
    output_str.append(" az = " + std::to_string(packet->a[2]));
    output_str.append(" wx = " + std::to_string(packet->w[0]));
    output_str.append(" wy = " + std::to_string(packet->w[1]));
    output_str.append(" wz = " + std::to_string(packet->w[2]));
    cout << output_str << endl;
    algorithm_selected = 1;
}

/* Write data to serial port Using LMP_Device Commands*/

void WriteCOM(GKV_PacketBase* buf)
{
    #ifdef _WIN32
    DWORD dwBytesWritten;
    char iRet = WriteFile(hSerial, buf, buf->length + 8, &dwBytesWritten, NULL);
    _sleep(1);
    #endif
    #ifdef __linux
    int iOut = write(hSerial, buf, buf->length + 8);
    usleep(1000);
    #endif

}

void ReadGkvData(LMP_Device* dev)
{
    static char ReceivedData[2048] = { 0 };
    ssize_t iSize;
    ssize_t iRet = 0;
    #ifdef _WIN32
    iRet = ReadFile(hSerial, &ReceivedData, sizeof(ReceivedData), &iSize, 0);
    #endif
    #ifdef __linux
    iRet = read(hSerial, &ReceivedData, sizeof(ReceivedData));
    iSize=iRet;
    #endif
    if (iRet)
    {
        if (iSize > 0)
        {
            dev->Receive_Process(&ReceivedData[0], iSize);
        }
    }
}

bool InitSerialPort(string port_name, int32_t baudrate)
{
    #ifdef _WIN32
    hSerial = ::CreateFileA(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            cout << "serial port does not exist.\n";
            return 0;
        }
        cout << "some other error occurred.\n";
        return 0;
    }
    cout << "#connect ok\n";
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        cout << "getting state error\n";
        return 0;
    }
    cout << "#get state ok\n";
    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        cout << "error setting serial port state\n";
        return 0;
    }
    #endif
    #ifdef __linux
    hSerial = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (hSerial < 0) {
        printf("Error opening port\n");
        return 0;
    }
    struct termios tty;
    struct termios tty_old;
    memset(&tty, 0, sizeof tty);
    /* Error Handling */
    if (tcgetattr(hSerial, &tty) != 0) {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
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
    tcflush(hSerial, TCIFLUSH);
    if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
        cout << "Error " << errno << " from tcsetattr" << endl;
        return 0;
    }
    #endif
    return 1;
}

#ifdef _WIN32
#include <windows.h>
#define SerialPortHandle HANDLE
#define GKV_BDRT921600 921600
#endif
#ifdef __linux
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#define SerialPortHandle int
#define GKV_BDRT921600 B921600
#endif

#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"

using namespace Gyrovert;
using namespace std;

SerialPortHandle hSerial;

bool InitSerialPort(string port_name, int32_t baudrate);
void ReadGkvData(LMP_Device* dev);
void WriteCOM(GKV_PacketBase* buf);
void ShowPacketData(LMP_Device * GKV,GKV_PacketBase* buf);

int main()
{
    /* Select Serial Port */
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    /* Create LMP Device Object GKV */
    LMP_Device* GKV = new LMP_Device();
    /* Serial Port Settings For Windows */
    #ifdef _WIN32
    com_port = "\\\\.\\" + com_port;
    #endif
    if (!(InitSerialPort(com_port, GKV_BDRT921600))) return 1;
    /* GKV Settings */
    GKV->SetReceivedPacketCallback(ShowPacketData);//Set User Callback for Each Parsed GKV Packet
    GKV->SetSendDataFunction(WriteCOM);//Set User Function That Sends Data to Serial Port connected to GKV
    cout << "#start main loop\n";
    while (1)
    {
        ReadGkvData(GKV);
    }
    return 0;
}

void ShowPacketData(LMP_Device *GKV, GKV_PacketBase* buf)
{
std::string output_str;
switch (buf->type)
    {
    case GKV_ADC_CODES_PACKET:
    {
        GKV_ADCData* packet;
        packet = (GKV_ADCData*)&buf->data;
        output_str.append("ADC Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" ax = " + std::to_string(packet->a[0]));
        output_str.append(" ay = " + std::to_string(packet->a[1]));
        output_str.append(" az = " + std::to_string(packet->a[2]));
        output_str.append(" wx = " + std::to_string(packet->w[0]));
        output_str.append(" wy = " + std::to_string(packet->w[1]));
        output_str.append(" wz = " + std::to_string(packet->w[2]));
        cout << output_str << endl;
        break;
    }
    case GKV_RAW_DATA_PACKET:
    {
        GKV_RawData* packet;
        packet = (GKV_RawData*)&buf->data;
        output_str.append( "Raw Sensors Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" ax = " + std::to_string(packet->a[0]));
        output_str.append(" ay = " + std::to_string(packet->a[1]));
        output_str.append(" az = " + std::to_string(packet->a[2]));
        output_str.append(" wx = " + std::to_string(packet->w[0]));
        output_str.append(" wy = " + std::to_string(packet->w[1]));
        output_str.append(" wz = " + std::to_string(packet->w[2]));
        cout   << output_str << endl;
        break;
    }
    case GKV_EULER_ANGLES_PACKET:
    {
        GKV_GyrovertData* packet;
        packet = (GKV_GyrovertData*)&buf->data;
        output_str.append("Gyrovert Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" yaw = " + std::to_string(packet->yaw));
        output_str.append(" pitch = " + std::to_string(packet->pitch));
        output_str.append(" roll = " + std::to_string(packet->roll));
        cout << output_str << endl;
        break;
    }
    case GKV_INCLINOMETER_PACKET:
    {
        GKV_InclinometerData* packet;
        packet = (GKV_InclinometerData*)&buf->data;
        output_str.append("Inclinometer Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" alfa = " + std::to_string(packet->alfa));
        output_str.append(" beta = " + std::to_string(packet->beta));
        cout << output_str << endl;
        break;
    }
    case GKV_BINS_PACKET:
    {
        GKV_BINSData* packet;
        packet = (GKV_BINSData*)&buf->data;
        output_str.append("BINS Data Packet: ");
        output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
        output_str.append(" x = " + std::to_string(packet->x));
        output_str.append(" y = " + std::to_string(packet->y));
        output_str.append(" z = " + std::to_string(packet->z));
        output_str.append(" alfa = " + std::to_string(packet->alfa));
        output_str.append(" beta = " + std::to_string(packet->beta));
        output_str.append(" q0 = " + std::to_string(packet->q[0]));
        output_str.append(" q1 = " + std::to_string(packet->q[1]));
        output_str.append(" q2 = " + std::to_string(packet->q[2]));
        output_str.append(" q3 = " + std::to_string(packet->q[3]));
        output_str.append(" yaw = " + std::to_string(packet->yaw));
        output_str.append(" pitch = " + std::to_string(packet->pitch));
        output_str.append(" roll = " + std::to_string(packet->roll));
        cout << output_str << endl;
        break;
    }
    case GKV_GNSS_PACKET:
    {
        GKV_GpsData* packet;
        packet = (GKV_GpsData*)&buf->data;
        output_str.append("GNSS Data Packet: ");
        output_str.append("time = " + std::to_string(packet->time));
        output_str.append(" latitude = " + std::to_string(packet->latitude));
        output_str.append(" longitude = " + std::to_string(packet->longitude));
        output_str.append(" altitude = " + std::to_string(packet->altitude));
        output_str.append(" state_status = " + std::to_string(packet->state_status));
        output_str.append(" TDOP = " + std::to_string(packet->TDOP));
        output_str.append(" HDOP = " + std::to_string(packet->HDOP));
        output_str.append(" VDOP = " + std::to_string(packet->VDOP));
        cout << output_str << endl;
        break;
    }
    case GKV_EXTENDED_GNSS_PACKET:
    {
        GKV_GpsDataExt* packet;
        packet = (GKV_GpsDataExt*)&buf->data;
        output_str.append("Extended GNSS Data Packet: ");
        output_str.append("vlat = " + std::to_string(packet->vlat));
        output_str.append(" vlon = " + std::to_string(packet->vlon));
        output_str.append(" sig_lat = " + std::to_string(packet->sig_lat));
        output_str.append(" sig_lon = " + std::to_string(packet->sig_lon));
        output_str.append(" sig_alt = " + std::to_string(packet->sig_alt));
        output_str.append(" sig_vlat = " + std::to_string(packet->sig_vlat));
        output_str.append(" sig_vlon = " + std::to_string(packet->sig_vlon));
        output_str.append(" sig_valt = " + std::to_string(packet->sig_valt));
        cout << output_str << endl;
        break;
    }
    case GKV_CUSTOM_PACKET:
    {
        GKV_CustomData* packet;
        packet = (GKV_CustomData*)&buf->data;
        output_str.append("CustomPacket:");
        for (uint8_t i = 0; i < ((buf->length) / 4); i++)
        {
            output_str.append(" param = " + std::to_string(packet->parameter[i]));
        }
        cout << output_str << endl;
        /*Примечание: в данном примере вывод значений некоторых параметров наборного пакета с пометкой int будет некорректен, поскольку данная программа
        не посылает запроса на получение номеров парамеров наборного пакета и выводит все параметры, как float.*/
        break;
    }

    }
}


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
    uint32_t iRet = 0;
#ifdef _WIN32
    DWORD iSize;
    iRet = ReadFile(hSerial, &ReceivedData, sizeof(ReceivedData), (LPDWORD)&iSize, 0);
#endif
#ifdef __linux
    uint16_t iSize;
    iRet = read(hSerial, &ReceivedData, sizeof(ReceivedData));
    iSize = iRet;
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

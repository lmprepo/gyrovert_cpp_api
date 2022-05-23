#ifdef _WIN32

#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace Gyrovert;
using namespace std;

HANDLE hSerial;
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
    com_port = "\\\\.\\" + com_port;
    if (!(InitSerialPort(com_port, 921600))) return 1;
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

/* Write data to serial port Using LMP_Device Commands*/
void WriteCOM(GKV_PacketBase* buf)
{
    DWORD dwBytesWritten;
    char iRet = WriteFile(hSerial, buf, buf->length + 8, &dwBytesWritten, NULL);/* Write to serial Port Command/Request Pakcet Data with header (4 bytes) and CRC32 (4 bytes) */
    _sleep(1);
}

/* Read Serial Port Data and Process it with LMP_Device ReceiveProcess Function */
void ReadGkvData(LMP_Device* dev)
{
    static char ReceivedData[2048] = { 0 };
    DWORD iSize;
    char iRet = 0;
    iRet = ReadFile(hSerial, &ReceivedData, sizeof(ReceivedData), &iSize, 0);
    if (iRet)
    {
        if (iSize > 0)
        {
            dev->Receive_Process(&ReceivedData[0], iSize);
        }
    }
}

/*Sensors Data Packet Receive Callback*/
void ShowPacketData(LMP_Device* GKV, GKV_RawData* packet)
{
    string output_str;
    output_str.append("Sensors Data Packet: ");
    output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
    output_str.append(" ax = " + std::to_string((float)packet->a[0]));
    output_str.append(" ay = " + std::to_string((float)packet->a[1]));
    output_str.append(" az = " + std::to_string((float)packet->a[2]));
    output_str.append(" wx = " + std::to_string((float)packet->w[0]));
    output_str.append(" wy = " + std::to_string((float)packet->w[1]));
    output_str.append(" wz = " + std::to_string((float)packet->w[2]));
    cout << output_str << endl;
    algorithm_selected = 1;
}

/*Serial Port initialization Function for Windows and Linux*/
bool InitSerialPort(string port_name, int32_t baudrate)
{
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
    return 1;
}
#else
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"
using namespace Gyrovert;
using namespace std;

int SerialPortHandle;
uint8_t algorithm_selected = 0;
LMP_Device* GKV;
bool InitSerialPort(string port_name, int32_t baudrate);
char* ReadCOM();
void WriteCOM(GKV_PacketBase* buf);
void ShowPacketData(LMP_Device* GKV, GKV_ADCData* buf);

int main()
{
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    /* Create LMP Device Object GKV */
    GKV = new LMP_Device();
    /* Serial Port Settings For Linux */
    if (!(InitSerialPort(com_port, B921600))) return 1;
    /* GKV Settings */
    GKV->SetSendDataFunction(WriteCOM);//Set User Function That Sends Data to Serial Port connected to GKV
    GKV->SetReceiveDataFunction(ReadCOM);//Set User Function That Receives Data From Serial Port And Returns Received Byte
    GKV->SetADCDataReceivedCallback(ShowPacketData);//Set User Callback for Parsed ADC GKV Packet
    GKV->RunDevice();//Run Thread For Receiving Data From GKV
    while (!(algorithm_selected))
    {
        GKV->SetDefaultAlgorithmPacket();
        GKV->SetAlgorithm(GKV_ADC_CODES_ALGORITHM);
    }
    cout << "#start main loop\n";
    while (1)
    {
        //do something
    }
    return 0;
}

bool InitSerialPort(string port_name, int32_t baudrate)
{
    SerialPortHandle = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (SerialPortHandle < 0) {
        printf("Error opening port\n");
        return 0;
    }
    struct termios tty;
    struct termios tty_old;
    memset(&tty, 0, sizeof tty);
    /* Error Handling */
    if (tcgetattr(SerialPortHandle, &tty) != 0) {
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
    tcflush(SerialPortHandle, TCIFLUSH);
    if (tcsetattr(SerialPortHandle, TCSANOW, &tty) != 0) {
        cout << "Error " << errno << " from tcsetattr" << endl;
        return 0;
    }
    return 1;
}

void WriteCOM(GKV_PacketBase* buf)
{
    int iOut = write(SerialPortHandle, buf, buf->length + 8);
    usleep(1000);
}

char* ReadCOM()
{
    static char ReceivedData[2048] = { 0 };
    while (true)
    {
        int iOut = read(SerialPortHandle, &ReceivedData, 2048);
        GKV->SetReceiveBufferSize(iOut);
        return &ReceivedData;
    }
    return 0;
}

void ShowPacketData(LMP_Device* GKV, GKV_ADCData* packet)
{
    char str[30];
    sprintf(str, "%d", packet->sample_cnt);
    cout << "Sample Counter = " << str << ' ';
    sprintf(str, "%d", packet->a[0]);
    cout << "ax = " << str << ' ';
    sprintf(str, "%d", packet->a[1]);
    cout << "ay = " << str << ' ';
    sprintf(str, "%d", packet->a[2]);
    cout << "az = " << str << ' ';
    sprintf(str, "%d", packet->w[0]);
    cout << "wx = " << str << ' ';
    sprintf(str, "%d", packet->w[1]);
    cout << "wy = " << str << ' ';
    sprintf(str, "%d", packet->w[2]);
    cout << "wz = " << str << endl;
    algorithm_selected = 1;
}
#endif

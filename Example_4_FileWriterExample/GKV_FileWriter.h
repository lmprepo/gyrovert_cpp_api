#ifndef GKV_FILE_WRITER_H
#define GKV_FILE_WRITER_H
#include "LMP_Device.h"

#include <fstream>

#ifdef _WIN32
#include <windows.h>
#define SerialPortHandle HANDLE
#define GKV_BDRT921600 921600
#endif
#include <thread>
#ifdef __linux
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#define SerialPortHandle int
#define GKV_BDRT921600 B921600
#endif

class GKV_FileWriter : public Gyrovert::LMP_Device
{
public:
	GKV_FileWriter(std::string port_name, int32_t baudrate);
	~GKV_FileWriter();
    void StartWriteBinaryData();
    void StopWriteBinaryData() { DataWritingEnabled = false; Logger.join();}
private:
    void dataNewThreadReceiveFcn();
    void dataNewThreadWriteFcn();
    void RunDevice();
    void WriteCOM(Gyrovert::GKV_PacketBase* buf);
    void ReadGkvData();
    bool InitSerialPort(std::string port_name, int32_t baudrate);
    void AppendDataToFileWritingBuf(char* data_ptr, uint16_t buffer_size);

private:
    SerialPortHandle hSerial;
    bool gkv_open = true;
    bool DataWritingEnabled = false;

	std::thread Receiver;
	//Data Logger Parameters
	std::thread Logger;
    char* buffer_1;
    char* buffer_2;
    uint32_t buffer_point_1 = 0;
    uint32_t buffer_point_2 = 0;
    uint8_t WritingMode = 0;
    bool WriteBuffer1Flag = false;
    bool WriteBuffer2Flag = false;
    std::string filePath;
    std::ofstream outfile;
};
#endif

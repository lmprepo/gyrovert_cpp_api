#include "GKV_FileWriter.h"


GKV_FileWriter::GKV_FileWriter(std::string port_name, int32_t baudrate) : LMP_Device()
{
//    memset(CurrentReceivedPacket, 0, sizeof(GKV_PacketBase));
    filePath = "LogData.bin";// Default name of LogFile (can be changed with date-time)
    buffer_1 = new char[100000];
    buffer_2 = new char[100000];
    InitSerialPort(port_name, baudrate);
    RunDevice();
}

/**
  * @name	~LMP_Device
  * @brief  Default Destructor
  * @retval no return value.
  */
GKV_FileWriter::~GKV_FileWriter()
{
    delete[] buffer_1;
    delete[] buffer_2;
    gkv_open = false;
    Receiver.join();
}

/**
  * @name	StartWriteBinaryData
  * @brief  Function sets flag for writing data to log file
  * @param  no parameters
  * @retval no return value.
  */
void GKV_FileWriter::StartWriteBinaryData() {
    time_t t = time(0);
    struct tm* now = localtime(&t);
    char buffer[80];
    strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", now);
    std::string str(buffer);
    filePath = "LogData_" + str + ".bin";
    DataWritingEnabled = true;
    Logger = std::move(std::thread(&GKV_FileWriter::dataNewThreadWriteFcn, this));
}

/**
   * @name	dataNewThreadWriteFcn
   * @brief  Function of GKV data writing thread main cycle
   * @retval no return value.
   */
void GKV_FileWriter::dataNewThreadWriteFcn()
{
    while (DataWritingEnabled)
    {
        if (WriteBuffer1Flag == true)
        {
            outfile.open(filePath, std::ios::binary | std::ios::app);
            outfile.write(buffer_1, buffer_point_1);
            buffer_point_1 = 0;
            WriteBuffer1Flag = false;
            outfile.close();
        }
        if (WriteBuffer2Flag == true)
        {
            outfile.open(filePath, std::ios::binary | std::ios::app);
            outfile.write(buffer_2, buffer_point_2);
            buffer_point_2 = 0;
            WriteBuffer2Flag = false;
            outfile.close();
        }
    }
}

/**
  * @name	RunDevice
  * @brief  Function creates and runs new thread for receiving and parsing GKV Data
  * @retval no return value.
  */
  void GKV_FileWriter::RunDevice()
  {
      //Receive_Process();
      Receiver = std::move(std::thread(&GKV_FileWriter::dataNewThreadReceiveFcn, this));
  }

  /**
  * @name	dataNewThreadReceiveFcn
  * @brief  Function of GKV data receiving thread main cycle
  * @retval no return value.
  */
void GKV_FileWriter::dataNewThreadReceiveFcn()
{
    while (gkv_open)
    {
        ReadGkvData();
        #ifdef _WIN32
            _sleep(10);
        #endif
        #ifdef __linux
            usleep(10000);
        #endif
    }
}

void GKV_FileWriter::WriteCOM(Gyrovert::GKV_PacketBase* buf)
{
#ifdef _WIN32
    DWORD dwBytesWritten;
    WriteFile(hSerial, buf, buf->length + 8, &dwBytesWritten, NULL);
    _sleep(1);
#endif
#ifdef __linux
    write(hSerial, buf, buf->length + 8);
    usleep(1000);
#endif

}

void GKV_FileWriter::ReadGkvData()
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
            Receive_Process(&ReceivedData[0], iSize);
            AppendDataToFileWritingBuf(&ReceivedData[0], iSize);
        }
    }
}


void GKV_FileWriter::AppendDataToFileWritingBuf(char* data_ptr, uint16_t buffer_size)
{
    if (DataWritingEnabled)
    {
        if (WritingMode == 0)
        {
            memcpy(&(buffer_1[buffer_point_1]), data_ptr, buffer_size);
            buffer_point_1 += buffer_size;
            if (buffer_point_1 > (100000 - 0xFFFF))
            {
                WritingMode = 1;
                WriteBuffer1Flag = true;
            }
        }
        else
        {
            memcpy(&(buffer_2[buffer_point_2]), data_ptr, buffer_size);
            buffer_point_2 += buffer_size;
            if (buffer_point_2 > (100000 - 0xFFFF))
            {
                WritingMode = 0;
                WriteBuffer2Flag = true;
            }
        }
    }
}

bool GKV_FileWriter::InitSerialPort(std::string port_name, int32_t baudrate)
{
    using namespace std;
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

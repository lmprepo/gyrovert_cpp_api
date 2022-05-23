#ifdef _WIN32
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include "LMP_Device.h"

using namespace Gyrovert;
using namespace std;

HANDLE hSerial;
uint8_t algorithm = GKV_ADC_CODES_ALGORITHM;
uint8_t algorithm_packet = 0;
uint8_t algorithm_selected = 0;

bool InitSerialPort(string port_name, int32_t baudrate);
void WriteCOM(GKV_PacketBase* buf);
void ReadGkvData(LMP_Device* dev);
void RecognisePacket(LMP_Device * GKV,GKV_PacketBase* buf);
uint8_t check_input(string str);
uint8_t SelectAlgorithmPacket(uint8_t algorithm);

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
    GKV->SetReceivedPacketCallback(RecognisePacket);/*Set User Callback for Parsed Sensors Data Packet*/

    cout<< "Choose GKV algorithm:\n";
    cout << "0 - ADC Codes Data from Sensors\n";
    cout << "1 - Calibrated Raw Sensor Data\n";
    cout << "2 - Orientation Kalman Filtered Data\n";
    cout << "4 - Inclinometer Data\n";
    cout << "5 - Orientation Mahony Filtered Data\n";
    cout << "6 - BINS Navigation Data\n";
    cout << "7 - Custom Algorithm\n";
    cout << "8 - Navigation Data GNSS+BINS\n";
    cout << "9 - Navigation Data GNSS+BINS type 2\n";
    /* Select Algorithm Number and Check It */
    string input;
    cout << "Selected Algorithm = ";
    do
    {
        cin >> input;
        if (check_input(input))
        {
            algorithm = atoi(input.c_str());
        }
        else
        {
            cout << "Wrong value. Try Again. Selected Algorithm = ";
            algorithm = 255;
        }
    } while (!(check_input(input)));
    algorithm_packet = SelectAlgorithmPacket(algorithm);
    cout << "#start main loop\n";
    while (1)
    {
        if (!(algorithm_selected))
        {
            GKV->SetDefaultAlgorithmPacket(); //selection of standard packet for selected algorithm
            GKV->SetAlgorithm(algorithm);
        }
        ReadGkvData(GKV);
    }
    return 0;
}

/*------------------------------------------------------------------------------------------------------------------*/

uint8_t check_input(string str)
{
    if (!(str.length() == 1))
    {
        return 0;
    }
    char char_array[2];
    strcpy(char_array, str.c_str());
    if (!(isdigit(char_array[0])))
    {
        return 0;
    }
    if ((atoi(str.c_str()) > 9) || (atoi(str.c_str()) == 3))
    {
        return 0;
    }
    return 1;
}

/*------------------------------------------------------------------------------------------------------------------*/
/* Packet Check for selected algorithm*/
uint8_t SelectAlgorithmPacket(uint8_t algorithm)
{
    switch (algorithm)
    {
        case GKV_ADC_CODES_ALGORITHM:               {   return GKV_ADC_CODES_PACKET;    }
        case GKV_SENSORS_DATA_ALGORITHM:            {   return GKV_RAW_DATA_PACKET;     }
        case GKV_ORIENTATION_KALMAN_ALGORITHM:      {   return GKV_EULER_ANGLES_PACKET; }
        case GKV_INCLINOMETER_ALGORITHM:            {   return GKV_INCLINOMETER_PACKET; }
        case GKV_ORIENTATION_MAHONY_ALGORITHM:      {   return GKV_EULER_ANGLES_PACKET; }
        case GKV_BINS_NAVIGATON_ALGORITHM:          {   return GKV_BINS_PACKET;         }
        case GKV_CUSTOM_ALGORITHM:                  {   return GKV_ADC_CODES_PACKET;    }
        case GKV_KALMAN_GNSS_NAVIGATON_ALGORITHM:   {   return GKV_GNSS_PACKET;         }
        case GKV_ESKF5_NAVIGATON_ALGORITHM:         {   return GKV_GNSS_PACKET;         }
        default:                                        return GKV_ADC_CODES_PACKET;
    }
}

/*------------------------------------------------------------------------------------------------------------------*/

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

void RecognisePacket(LMP_Device* GKV, GKV_PacketBase* buf)
{
    std::string output_str;
    if (algorithm_selected)
    {
        switch (buf->type)
        {
        case GKV_ADC_CODES_PACKET:
        {
            GKV_ADCData* packet;
            packet = (GKV_ADCData*)&buf->data;
            output_str.append("ADC Data Packet: ");
            output_str.append("Sample Counter = " + std::to_string(packet->sample_cnt));
            output_str.append(" ax = " + std::to_string((int32_t)packet->a[0]));
            output_str.append(" ay = " + std::to_string((int32_t)packet->a[1]));
            output_str.append(" az = " + std::to_string((int32_t)packet->a[2]));
            output_str.append(" wx = " + std::to_string((int32_t)packet->w[0]));
            output_str.append(" wy = " + std::to_string((int32_t)packet->w[1]));
            output_str.append(" wz = " + std::to_string((int32_t)packet->w[2]));
            cout << output_str << endl;
            break;
        }
        case GKV_RAW_DATA_PACKET:
        {
            GKV_RawData* packet;
            packet = (GKV_RawData*)&buf->data;
            output_str.append("Raw Sensors Data Packet: ");
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
            break;
        }
        }
    }
    else
    {
        if (buf->type == algorithm_packet) algorithm_selected = 1;// check for algorithm selection with default packet 
    }
}

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
#include <iostream>
#include <stdio.h>
#include "GKV_Device.h"

using namespace Gyrovert;
using namespace std;

string input;
uint8_t algorithm = GKV_ADC_CODES_ALGORITHM;
uint8_t algorithm_packet = 0;
uint8_t algorithm_selected = 0;

void RecognisePacket(LMP_Device* GKV, GKV_PacketBase* buf);
uint8_t check_input(string str);
uint8_t ChooseAlgorithmPacket(uint8_t algorithm);

int main()
{
    /* Select Serial Port */
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    /* Create LMP Device Object GKV */
    GKV_Device* GKV = new GKV_Device(com_port, B921600);
    if (!(GKV->GetSerialConnectionState())) return 0;
    /* GKV Settings */
    GKV->SetReceivedPacketCallback(RecognisePacket);//Set User Callback for Each Parsed GKV Packet
    GKV->RunDevice();//Run Thread For Receiving Data From GKV  

    printf("Choose GKV algorithm:\n");
    printf("0 - ADC Codes Data from Sensors\n");
    printf("1 - Calibrated Raw Sensor Data\n");
    printf("2 - Orientation Kalman Filtered Data\n");
    printf("4 - Inclinometer Data\n");
    printf("5 - Orientation Mahony Filtered Data\n");
    printf("6 - BINS Navigation Data\n");
    printf("7 - Custom Algorithm\n");
    printf("8 - Navigation Data GNSS+BINS\n");
    printf("9 - Navigation Data GNSS+BINS type 2\n");
    /* Select Algorithm Number and Check It */
    cout << "Selected Algorithm = ";
    do
    {
        cin >> input;
        if (check_input(input))
        {
            cout << "Selected Algorithm = " << input;
            algorithm = atoi(input.c_str());
        }
        else
        {
            printf("Wrong value. Try Again. Selected Algorithm = ");
            algorithm = 255;
        }
    } while (!(check_input(input)));
    algorithm_packet = ChooseAlgorithmPacket(algorithm);

    while (!(algorithm_selected))
    {
        GKV->SetDefaultAlgorithmPacket(); //selection of standard packet for selected algorithm
        //GKV->SetCustomAlgorithmPacket(); //selection of custom data processed using selected algorithm. Uncomment to select this packet type (with string in RecognisePacket function)
        GKV->SetAlgorithm(algorithm);
    }
    GKV->RequestDeviceID();

    while (1)
    {
        //do something
    }
    return 0;
}

uint8_t check_input(string str)
{
    if (!(str.length() == 1))
    {
        return 0;
    }
    char char_array[2];
    strcpy(char_array, str.c_str());
    if (!(isdigit(char_array[0])))
    {
        return 0;
    }
    if ((atoi(str.c_str()) > 9) || (atoi(str.c_str()) == 3))
    {
        return 0;
    }
    return 1;
}

uint8_t ChooseAlgorithmPacket(uint8_t algorithm)
{
    switch (algorithm)
    {
    case GKV_ADC_CODES_ALGORITHM:
    {
        return GKV_ADC_CODES_PACKET;
        break;
    }
    case GKV_SENSORS_DATA_ALGORITHM:
    {
        return GKV_RAW_DATA_PACKET;
        break;
    }
    case GKV_ORIENTATION_KALMAN_ALGORITHM:
    {
        return GKV_EULER_ANGLES_PACKET;
        break;
    }
    case GKV_INCLINOMETER_ALGORITHM:
    {
        return GKV_INCLINOMETER_PACKET;
        break;
    }
    case GKV_ORIENTATION_MAHONY_ALGORITHM:
    {
        return GKV_EULER_ANGLES_PACKET;
        break;
    }
    case GKV_BINS_NAVIGATON_ALGORITHM:
    {
        return GKV_BINS_PACKET;
        break;
    }
    case GKV_CUSTOM_ALGORITHM:
    {
        return GKV_ADC_CODES_PACKET;
        break;
    }
    case GKV_KALMAN_GNSS_NAVIGATON_ALGORITHM:
    {
        return GKV_GNSS_PACKET;
        break;
    }
    case GKV_ESKF5_NAVIGATON_ALGORITHM:
    {
        return GKV_GNSS_PACKET;
        break;
    }
    default:
        return GKV_ADC_CODES_PACKET;
        break;
    }
}

void RecognisePacket(LMP_Device* GKV, GKV_PacketBase* buf)
{
    char str[30];
    if (algorithm_selected)
    {
        switch (buf->type)
        {
        case GKV_ADC_CODES_PACKET:
        {
            GKV_ADCData* packet;
            packet = (GKV_ADCData*)&buf->data;
            cout << "ADC Data Packet: ";
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
            break;
        }
        case GKV_RAW_DATA_PACKET:
        {
            GKV_RawData* packet;
            packet = (GKV_RawData*)&buf->data;
            cout << "Raw Sensors Data Packet: ";
            sprintf(str, "%d", packet->sample_cnt);
            cout << "Sample Counter = " << str << ' ';
            sprintf(str, "%f", packet->a[0]);
            cout << "ax = " << str << ' ';
            sprintf(str, "%f", packet->a[1]);
            cout << "ay = " << str << ' ';
            sprintf(str, "%f", packet->a[2]);
            cout << "az = " << str << ' ';
            sprintf(str, "%f", packet->w[0]);
            cout << "wx = " << str << ' ';
            sprintf(str, "%f", packet->w[1]);
            cout << "wy = " << str << ' ';
            sprintf(str, "%f", packet->w[2]);
            cout << "wz = " << str << endl;
            break;
        }
        case GKV_EULER_ANGLES_PACKET:
        {
            GKV_GyrovertData* packet;
            packet = (GKV_GyrovertData*)&buf->data;
            cout << "Gyrovert Data Packet: ";
            sprintf(str, "%d", packet->sample_cnt);
            cout << "Sample Counter = " << str << ' ';
            sprintf(str, "%f", packet->yaw);
            cout << "yaw = " << str << ' ';
            sprintf(str, "%f", packet->pitch);
            cout << "pitch = " << str << ' ';
            sprintf(str, "%f", packet->roll);
            cout << "roll = " << str << endl;
            break;
        }
        case GKV_INCLINOMETER_PACKET:
        {
            GKV_InclinometerData* packet;
            packet = (GKV_InclinometerData*)&buf->data;
            sprintf(str, "%d", packet->sample_cnt);
            cout << "Sample Counter = " << str << ' ';
            sprintf(str, "%f", packet->alfa);
            cout << "alfa = " << str << ' ';
            sprintf(str, "%f", packet->beta);
            cout << "beta = " << str << endl;
            break;
        }
        case GKV_BINS_PACKET:
        {
            GKV_BINSData* packet;
            packet = (GKV_BINSData*)&buf->data;
            cout << "BINS Data Packet: ";
            sprintf(str, "%d", packet->sample_cnt);
            cout << "Sample Counter = " << str << ' ';
            sprintf(str, "%f", packet->x);
            cout << "x = " << str << ' ';
            sprintf(str, "%f", packet->y);
            cout << "y = " << str << ' ';
            sprintf(str, "%f", packet->z);
            cout << "z = " << str << ' ';
            sprintf(str, "%f", packet->alfa);
            cout << "alfa = " << str << ' ';
            sprintf(str, "%f", packet->beta);
            cout << "beta = " << str << ' ';
            sprintf(str, "%f", packet->q[0]);
            cout << "q0 = " << str << ' ';
            sprintf(str, "%f", packet->q[1]);
            cout << "q1 = " << str << ' ';
            sprintf(str, "%f", packet->q[2]);
            cout << "q2 = " << str << ' ';
            sprintf(str, "%f", packet->q[3]);
            cout << "q3 = " << str << ' ';
            sprintf(str, "%f", packet->yaw);
            cout << "yaw = " << str << ' ';
            sprintf(str, "%f", packet->pitch);
            cout << "pitch = " << str << ' ';
            sprintf(str, "%f", packet->roll);
            cout << "roll = " << str << endl;
            break;
        }
        case GKV_GNSS_PACKET:
        {
            GKV_GpsData* packet;
            packet = (GKV_GpsData*)&buf->data;
            cout << "GNSS Data Packet: ";
            sprintf(str, "%d", packet->time);
            cout << "time = " << str << ' ';
            sprintf(str, "%f", packet->latitude);
            cout << "latitude = " << str << ' ';
            sprintf(str, "%f", packet->longitude);
            cout << "longitude = " << str << ' ';
            sprintf(str, "%f", packet->altitude);
            cout << "altitude = " << str << ' ';
            sprintf(str, "%d", packet->state_status);
            cout << "state_status = " << str << ' ';
            sprintf(str, "%f", packet->TDOP);
            cout << "TDOP = " << str << ' ';
            sprintf(str, "%f", packet->HDOP);
            cout << "HDOP = " << str << ' ';
            sprintf(str, "%f", packet->VDOP);
            cout << "VDOP = " << str << endl;
            break;
        }
        case GKV_EXTENDED_GNSS_PACKET:
        {
            GKV_GpsDataExt* packet;
            packet = (GKV_GpsDataExt*)&buf->data;
            cout << "Extended GNSS Data Packet: ";
            sprintf(str, "%d", packet->vlat);
            cout << "vlat = " << str << ' ';
            sprintf(str, "%f", packet->vlon);
            cout << "vlon = " << str << ' ';
            sprintf(str, "%f", packet->sig_lat);
            cout << "sig_lat = " << str << ' ';
            sprintf(str, "%f", packet->sig_lon);
            cout << "sig_lon = " << str << ' ';
            sprintf(str, "%f", packet->sig_alt);
            cout << "sig_alt = " << str << ' ';
            sprintf(str, "%f", packet->sig_vlat);
            cout << "sig_vlat = " << str << ' ';
            sprintf(str, "%f", packet->sig_vlon);
            cout << "sig_vlon = " << str << ' ';
            sprintf(str, "%f", packet->sig_valt);
            cout << "sig_valt = " << str << endl;
            break;
        }
        case GKV_CUSTOM_PACKET:
        {
            GKV_CustomData* packet;
            packet = (GKV_CustomData*)&buf->data;
            cout << "CustomPacket: ";
            for (uint8_t i = 0; i < ((buf->length) / 4); i++)
            {
                if (GKV->IsCustomPacketParamReceived())//if custom parameters list received
                {
                    uint8_t CurrentParameterType = FloatParameter;
                    for (uint8_t j = 0; j < sizeof(GKV->INT_PARAM_NUMBERS); j++)
                    {
                        if (GKV->DeviceState.CurrentCustomPacketParameters.param[i] == GKV->INT_PARAM_NUMBERS[j])
                        {
                            CurrentParameterType = Int32Parameter;
                            break;
                        }
                    }
                    for (uint8_t j = 0; j < sizeof(GKV->UINT_PARAM_NUMBERS); j++)
                    {
                        if (GKV->DeviceState.CurrentCustomPacketParameters.param[i] == GKV->UINT_PARAM_NUMBERS[j])
                        {
                            CurrentParameterType = Uint32Parameter;
                            break;
                        }
                    }
                    if (CurrentParameterType == FloatParameter)
                    {
                        cout << "param = " << (packet->parameter[i]) << ' ';
                    }
                    else if (CurrentParameterType == Int32Parameter)
                    {
                        cout << "param = " << *(int32_t*)&(packet->parameter[i]) << ' ';
                    }
                    else
                    {
                        cout << "param = " << *(uint32_t*)&(packet->parameter[i]) << ' ';
                    }
                }
            }
            cout << endl;
            break;
        }
        case GKV_DEV_ID_PACKET:
        {
            cout << "ID Packet: " << endl;
            break;
        }
        }
    }
    else
    {
        //if (buf->type == GKV_CUSTOM_PACKET) algorithm_selected = 1;// check for custom  data stream. Uncomment to use this type (with custom packet selection in main)
        if (buf->type == algorithm_packet) algorithm_selected = 1;// check for algorithm selection with default packet 
    }
}
#endif

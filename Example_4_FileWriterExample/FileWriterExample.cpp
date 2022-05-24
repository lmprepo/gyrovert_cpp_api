
#include <string>
#include <iostream>
#include <stdio.h>
#ifdef _WIN32
#include <direct.h>
#endif
#include "GKV_FileWriter.h"
using namespace Gyrovert;
using namespace std;

string GetCurrentPath();

int main()
{
    string com_port;
    cout << "Set Serial Port:";
    cin >> com_port;
    cout << "#start connecting to " << com_port << "\n";
    /* Create GKV Device Object GKV */
    GKV_FileWriter GKV(com_port, GKV_BDRT921600);
    /* Show current folder */
    cout << "Writing data to " << GetCurrentPath() << '\n';
    /* GKV Settings */
    GKV.StartWriteBinaryData();
    cout << "#start main loop\n";
    while (1)
    {
        //do something
    }
    GKV.StopWriteBinaryData();
    return 0;
}

string GetCurrentPath() {
    char buff[FILENAME_MAX];
    getcwd(buff, FILENAME_MAX);
    string current_working_dir(buff);
    return current_working_dir;
}

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <iostream>
#include "system.h"
#include "logger.h"

LOG_DEFINE(loggerSystem , "system");

int msleep(long ms)
{
    struct timespec t;

    if (ms < 1000) {   
        t.tv_sec = 0;
        t.tv_nsec = ms * 1000000;
    } else {   
        t.tv_sec = (int)(ms / 1000);
        t.tv_nsec = (ms - ((long)t.tv_sec * 1000)) * 1000000;
    }   

    return nanosleep(&t, NULL);
}

#ifdef ISTRO_SERIAL_TERMIOS

#include <termios.h>

/***********************************************************************************************************************
int openSerialPort(char* portName)
attempt to open a serial port with the given name, returning -1 on failure
***********************************************************************************************************************/
int openSerialPort(const char* portName)
{
    // store the file descriptor for the serial port
    int fd;

    // attempt to open the port
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    // return -1 if we are unable to open the port
    if(fd == -1)
    {
        return -1;
    }
    else
    {
        // create a structure to store the port settings
        struct termios port_settings;

        // get the current port settings

        tcgetattr(fd, &port_settings);

        // set the baud rates
// fixme
if (portName[5]=='a') {
        cfsetispeed(&port_settings, B38400);
        cfsetospeed(&port_settings, B38400);
LOGM_WARN(loggerSystem, "openSerialPort", "port=\"" << portName << "\", baudrate=38400");
} else {
        cfsetispeed(&port_settings, B115200);
        cfsetospeed(&port_settings, B115200);

LOGM_WARN(loggerSystem, "openSerialPort", "port=\"" << portName << "\", baudrate=115200");
}
        // set 8 bits, no parity, no stop bits
        port_settings.c_cflag &= ~PARENB;
        port_settings.c_cflag &= ~CSTOPB;
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= CS8;

        // set raw input mode
        port_settings.c_lflag |= ~ICANON;

        // apply the settings to the port
        tcsetattr(fd, TCSANOW, &port_settings);

        // set the non blocking functionality
        fcntl(fd, F_SETFL, O_NONBLOCK);

        // return the file descriptor
        return(fd);
    }
}

/***********************************************************************************************************************
void closeSerialPort(int serialPort)
close the given serial port
***********************************************************************************************************************/

void closeSerialPort(int serialPort)
{
    tcflush(serialPort, TCIOFLUSH);
    close(serialPort);
}

int readSerialPort(int serialPort, void *buf, int count)
{
    return read(serialPort, buf, count);
}

int writeSerialPort(int serialPort, const void *buf, int count)
{
    return write(serialPort, buf, count);
}

#else

#include <string.h>

int system_sp = 1;
    
int openSerialPort(const char* portName) { 
    LOGM_WARN(loggerSystem, "init", "MOCK SERIAL PORT \"" << portName << "\" implementation, DEBUG ONLY!!");
    return system_sp++; 
}

void closeSerialPort(int serialPort) {}

int writeSerialPort(int serialPort, const void *buf, int count) 
{
    char ss[1024];
    int ll = strlen((const char *)buf);
    if (ll < sizeof(ss)) {
        strcpy(ss, (const char *)buf);
        if ((ll > 0) && (ss[ll - 1] == '\n')) ss[--ll] = 0; 
        if ((ll > 0) && (ss[ll - 1] == '\r')) ss[--ll] = 0; 
        LOGM_TRACE(loggerSystem, "writeSerialPort", "port=" << serialPort << ", data=\"" << ss << "\"");
    } else {
        LOGM_TRACE(loggerSystem, "writeSerialPort", "port=" << serialPort << ", data=\"" << (char *)buf << "\"");
    }
    return count; 
}

const char data1[] = "{\"BNOEVC\":[10.123,-100.456,+135.789,1,2,3]}\n";

int system_readsp = 0;

int readSerialPort(int serialPort, void *buf, int count) 
{ 
    if (count < strlen(data1)) {
        LOGM_WARN(loggerSystem, "readSerialPort", "port=" << serialPort << ", res=0");
        return 0;
    }

    if (!system_readsp) {
        system_readsp = 1;
        strcpy((char *)buf, data1);
        return strlen(data1); 
    } else {
        system_readsp = 0;
        return 0;
   }
}

#endif

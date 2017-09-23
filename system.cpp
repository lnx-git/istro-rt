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
int openSerialPort(const char* portName, int speed)
{
    // store the file descriptor for the serial port
    int fd;

    speed_t tspeed;

    switch (speed) {
    case   1200: 
        tspeed =   B1200;  
        break;
    case   1800: 
        tspeed =   B1800;  
        break;
    case   2400: 
        tspeed =   B2400;  
        break;
    case   4800: 
        tspeed =   B4800;  
        break;
    case   9600: 
        tspeed =   B9600;  
        break;
    case  19200: 
        tspeed =  B19200;  
        break;
    case  38400: 
        tspeed =  B38400;  
        break;
    case  57600: 
        tspeed =  B57600;  
        break;
    case 115200: 
        tspeed = B115200;  
        break;
    case 230400: 
        tspeed = B230400;  
        break;
    default : 
        tspeed = B115200;
    }

    // attempt to open the port
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    LOGM_INFO(loggerSystem, "openSerialPort", "port=\"" << portName << "\", baudrate=" << speed << ", fd=" << fd);

    // return -1 if we are unable to open the port
    if(fd == -1) {
        LOGM_ERROR(loggerSystem, "openSerialPort", "unable to open port \"" << portName << "\"!");
        return -1;
    }

    // create a structure to store the port settings
    struct termios port_settings;

    // get the current port settings
    tcgetattr(fd, &port_settings);

    // set the baud rates
    cfsetispeed(&port_settings, tspeed);
    cfsetospeed(&port_settings, tspeed);

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
#include "mtime.h"

int system_sp = 1;
    
int openSerialPort(const char* portName, int speed) { 
    LOGM_WARN(loggerSystem, "init", "MOCK SERIAL PORT \"" << portName << "\" implementation, DEBUG ONLY!!");
    return system_sp++; 
}

void closeSerialPort(int serialPort) {}

int writeSerialPort(int serialPort, const void *buf, int count) 
{
    char ss[1024];
    int ll = strlen((const char *)buf);
    if (ll < (int)sizeof(ss)) {
        strcpy(ss, (const char *)buf);
        if ((ll > 0) && (ss[ll - 1] == '\n')) ss[--ll] = 0; 
        if ((ll > 0) && (ss[ll - 1] == '\r')) ss[--ll] = 0; 
        LOGM_TRACE(loggerSystem, "writeSerialPort", "port=" << serialPort << ", data=\"" << ss << "\"");
    } else {
        LOGM_TRACE(loggerSystem, "writeSerialPort", "port=" << serialPort << ", data=\"" << (char *)buf << "\"");
    }
    return count; 
}

/*
  R - ako robot
  1 - stav robota 1-STOP 2-FWD 5-OBST z ultrazvuku
 67 - lavy ultrazvuk v cm
289 - predny ultrazvuk v cm
  9 - pravy ultrazvu
243.83 - heading z kompasu
  0 - inkrement z IRC v casovom intervale, moze poslat 1 a viacej impulzov
 92 - aktualne nastavenie serva riadenia
 90 - aktualne nastavenie motora - 90= stop 
*/

//const char data1[] = "R:2:67:289:9:243.83:0:92:90\n\r\nR:2:69:291:9:243.92:2:96:91\r";
//const char data1a[] = "R:2:67:289:9:243.83:2:92:90\n\r";
//const char data1b[] = "R:2:69:291:9:243.92:2:96:91\r";
const char data1a[] = "R:2:243.83:2:92:90:0:100:67:289:9:90:130\n\r";
const char data1b[] = "R:2:243.92:3:96:91:1:120\r";
const char data2[] = "{\"BNOEVC\":[166.523,-100.456,+135.789,1,2,3]}\n";

double system_readsp1 = -1;
int    system_readsp1n = 0;
double system_readsp2 = -1;

int readSerialPort(int serialPort, void *buf, int count) 
{ 
    if (serialPort == 1) {
        if (count < (int)strlen(data1a)) {
            LOGM_WARN(loggerSystem, "readSerialPort", "msg=\"buffer too small!\", port=" << serialPort << ", res=0");
            return 0;
        }
    
        if ((system_readsp1 < 0) || (timeDelta(system_readsp1) >= 100)) {
            system_readsp1 = timeBegin();
            if (system_readsp1n) {
                system_readsp1n = 0;
                strcpy((char *)buf, data1b);
                return strlen(data1b); 
            } else {
                system_readsp1n = 1;
                strcpy((char *)buf, data1a);
                return strlen(data1a); 
            }
        } else {
            return 0;
        }
    }
    if (serialPort == 2) {
        if (count < (int)strlen(data2)) {
            LOGM_WARN(loggerSystem, "readSerialPort", "msg=\"buffer too small!\", port=" << serialPort << ", res=0");
            return 0;
        }
    
        if ((system_readsp2 < 0) || (timeDelta(system_readsp2) >= 20)) {
            system_readsp2 = timeBegin();
            strcpy((char *)buf, data2);
            return strlen(data2); 
        } else {
            return 0;
        }
    }
    return -1;
}

#endif

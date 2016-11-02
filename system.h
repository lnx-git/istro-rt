#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#ifndef WIN32

#define ISTRO_LOGGER_LOG4CXX
#define ISTRO_SERIAL_TERMIOS
#define ISTRO_OPENCV_CAMERA
#define ISTRO_RPLIDAR
#define ISTRO_GPSD_CLIENT
#define ISTRO_GEOGRAPHIC_LIB
//#define ISTRO_MYAHRS_PLUS
//#define ISTRO_GUI

#else

//#define ISTRO_LOGGER_LOG4CXX
//#define ISTRO_SERIAL_TERMIOS
//#define ISTRO_OPENCV_CAMERA
//#define ISTRO_RPLIDAR
//#define ISTRO_GPSD_CLIENT
//#define ISTRO_GEOGRAPHIC_LIB
//#define ISTRO_MYAHRS_PLUS
//#define ISTRO_GUI

#endif

int msleep(long ms);

int openSerialPort(const char* portName);
void closeSerialPort(int serialPort);

int writeSerialPort(int serialPort, const void *buf, int count);
int readSerialPort(int serialPort, void *buf, int count);

#endif

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <errno.h>  //fixme
#include "ctrlboard.h"
#include "config.h"
#include "dataset.h"
#include "threads.h"
#include "system.h"
#include "logger.h"

LOG_DEFINE(loggerCtrlBoard, "ControlBoard");

// fixme
extern Threads threads;

ControlBoard::ControlBoard(void)
{
    serialPort = -1;
    serialPort2 = -1;
    rxlen = 0;
} 

int ControlBoard::init(const char *portName, const char *portName2) 
{
    serialPort = openSerialPort(portName);
    
    if (serialPort == -1) {
        LOGM_ERROR(loggerCtrlBoard, "init", "unable to open serial port1 \"" << portName << "\"!");
        return -1;
    }
    // sleep(3);
    LOGM_INFO(loggerCtrlBoard, "init", "serial port1 \"" << portName << "\" opened successfully!");

    if (portName2 == NULL) {
        serialPort2 = -1;
        LOGM_WARN(loggerCtrlBoard, "init", "port2 not initialized!");
        return 0;
    }

    serialPort2 = openSerialPort(portName2);
    
    if (serialPort2 == -1) {
        LOGM_ERROR(loggerCtrlBoard, "init", "unable to open serial port2 \"" << portName2 << "\"!");
        return -2;
    }
    // sleep(3);
    LOGM_INFO(loggerCtrlBoard, "init", "serial port2 \"" << portName2 << "\" opened successfully!");

    return 0;
}

void ControlBoard::close(void)
{
    closeSerialPort(serialPort);
    serialPort = -1;

    if (serialPort2 >= 0) {
        closeSerialPort(serialPort2);
        serialPort2 = -1;
    }
} 

int ControlBoard::writef(int fd, const void *buf, int count)
{
    DataSet *pdata;
    
    pdata = (DataSet *)threads.getData(THDATA_STATE_CTRLBOARD, THDATA_STATE_CTRLBOARD_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    int res = writeSerialPort(fd, buf, count);

    threads.setData(pdata, THDATA_STATE_CTRLBOARD, 1);

    return res;
}

int ControlBoard::readf(int fd, void *buf, int count)
{
    DataSet *pdata;
    
    pdata = (DataSet *)threads.getData(THDATA_STATE_CTRLBOARD, THDATA_STATE_CTRLBOARD_LOCK);
    if (pdata == NULL) {
        return -1;
    }

    int res = readSerialPort(fd, buf, count);

    threads.setData(pdata, THDATA_STATE_CTRLBOARD, 1);

    return res;
}

void ControlBoard::start(void)
{
    int result = writef(serialPort, "START\n", 6);
    LOGM_INFO(loggerCtrlBoard, "start", "result=" << result);
}

void ControlBoard::setSpeed(int speed)
{
    char ss[30] = "";

    if(speed < 90) speed = 90;
    if(speed > 100) speed = 100;

    sprintf(ss, "S2%03d\n", speed);
    writef(serialPort, ss, 6);
    ss[5] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}

void ControlBoard::setSteeringAngle(int angle)
{
    char ss[30] = "";
    
    if(angle < SA_MIN) angle = SA_MIN;
    if(angle > SA_MAX) angle = SA_MAX;
    
    sprintf(ss, "S1%03d\n", angle);
    writef(serialPort, ss, 6);
    ss[5] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}

const unsigned char RX_FRAME_BEGIN = '{';
const unsigned char RX_FRAME_END   = '}';

int ControlBoard::readFrame(int serialPort, unsigned char *buf, int count)
{
    //rxbuff[rxlen] = '\0';
    //printf("rxbuff(%d): \"%s\"\n", rxlen, rxbuff);
    int ll = readf(serialPort2, (void *)&rxbuff[rxlen], RXBUFF_LENGTH - rxlen);
    if (ll < 0) {
        if (errno != EAGAIN) {
            LOGM_ERROR(loggerCtrlBoard, "readData", "unable to read from serial port (" << ll << ", " << errno << ", " << strerror(errno) << ")!");
            
            /* check "sudo stty -a -F /dev/ttyUSB0" */
            return -1;
        }
        ll = 0;    // ignore EAGAIN
    }
    rxlen += ll;
    //rxbuff[rxlen] = '\0';
    //printf("rxbuff(%d, %d): \"%s\"\n", rxlen, ll, rxbuff);

    // find start of frame
    int i = 0;
    while ((i < rxlen) && (rxbuff[i] != RX_FRAME_BEGIN)) {
        i++;
    }
    // printf("frame_begin: %d\n", i);
    if (i >= rxlen) {
        // frame begin not found -> discard all characters
        // printf("readData: begin not found!\n");
        rxlen = 0;
        return 0;
    }
    // ignore characters before FRAME_BEGIN
    memcpy((void *)rxbuff, (void *)&rxbuff[i], rxlen - i);
    rxlen -= i;

    // find end of frame
    i = 0;
    while ((i < rxlen) && (rxbuff[i] != RX_FRAME_END)) {
        i++;
    }
    // printf("frame_end: %d\n", i);
    if (i >= rxlen) {
        // frame begin not found 
        if (rxlen >= RXBUFF_LENGTH) {
            // buffer full, no end found -> discard all characters (message is larger than than our buffer - ignore it)
            // printf("readData: message too long!\n");
            rxlen = 0;            
            return 0;
        }
        // partial frame read -> wait
        // printf("readData: partial frame read...\n");
        return 0;
    }

    // return complete frame
    memcpy((void *)buf, (void *)&rxbuff[0], i + 1);
    // remove frame characters from the buffer
    if (i + 1 < rxlen) {
        memcpy((void *)rxbuff, (void *)&rxbuff[i + 1], rxlen - i - 1);
        rxlen -= i + 1;
    } else {
        rxlen = 0;
    }
    // return frame length (always >= 2)
    return (i + 1);
}

const char RX_FRAME_HEADER[]   = "{\"BNOEVC\":[";
const int  RX_FRAME_HEADER_LEN =  11;
const char RX_PARAM_DELIMITER  = ',';
const int  RX_PARAM_LEN        = 64;    // maximum parameter length

int ControlBoard::parseInt(unsigned char *buf, int count, int &idx, int &x)
{
    char pp[RX_PARAM_LEN + 1];

    int ll = 0;
    while ((idx < count) && (buf[idx] != RX_PARAM_DELIMITER) && (ll < RX_PARAM_LEN)) {
        pp[ll++] = (char)buf[idx++];
    }
    if ((idx < count) && (buf[idx] == RX_PARAM_DELIMITER)) {
        idx++;
    }

    pp[ll] = '\0';
    // printf("parseInt: \"%s\"\n", pp);
    if (sscanf(pp, "%d", &x) < 1) { 
        return -1;
    }

    return 0;
}

int ControlBoard::parseDouble(unsigned char *buf, int count, int &idx, double &x)
{
    char pp[RX_PARAM_LEN + 1];

    int ll = 0;
    while ((idx < count) && (buf[idx] != RX_PARAM_DELIMITER) && (ll < RX_PARAM_LEN)) {
        pp[ll++] = (char)buf[idx++];
    }
    if ((idx < count) && (buf[idx] == RX_PARAM_DELIMITER)) {
        idx++;
    }

    pp[ll] = '\0';
    // printf("parseDouble: \"%s\"\n", pp);
    if (sscanf(pp, "%lf", &x) < 1) { 
        return -1;
    }

    return 0;
}

                                                  
int ControlBoard::parseFrameBnoEVC(unsigned char *buf, int count, double &euler_x, double &euler_y, double &euler_z, int &calib_gyro, int &calib_accel, int &calib_mag)
{
    euler_x = euler_y = euler_z = 0; 
    calib_gyro = calib_accel = calib_mag = 0;

    if (count < RX_FRAME_HEADER_LEN) {
        return -1;  // frame header not found
    }

    if (memcmp(buf, RX_FRAME_HEADER, RX_FRAME_HEADER_LEN) != 0) {
        return -2;  // frame header does not match
    }

    int idx = RX_FRAME_HEADER_LEN;
    if (parseDouble(buf, count, idx, euler_x) < 0) {    
        return -3;  // parameter parsing failed
    }

    if (parseDouble(buf, count, idx, euler_y) < 0) {    
        return -4;  // parameter parsing failed
    }

    if (parseDouble(buf, count, idx, euler_z) < 0) {    
        return -5;  // parameter parsing failed
    }

    if (parseInt(buf, count, idx, calib_gyro) < 0) {    
        return -6;  // parameter parsing failed
    }

    if (parseInt(buf, count, idx, calib_accel) < 0) {    
        return -7;  // parameter parsing failed
    }

    if (parseInt(buf, count, idx, calib_mag) < 0) {    
        return -8;  // parameter parsing failed
    }

    // printf("parseFrame: result OK!\n");
    return 0;
}

int ControlBoard::getImuData(double &euler_x, double &euler_y, double &euler_z, int &calib_gyro, int &calib_accel, int &calib_mag)
{
    unsigned char buf[RXBUFF_LENGTH+1];

    euler_x = euler_y = euler_z = ANGLE_NONE; 
    calib_gyro = calib_accel = calib_mag = -1;

    int ll = readFrame(serialPort, buf, RXBUFF_LENGTH);
    if (ll < 0) {
        LOGM_ERROR(loggerCtrlBoard, "getImuData", "readFrame failed!");
        return -1;
    }
    if (ll == 0) {
        LOGM_TRACE(loggerCtrlBoard, "getImuData", "no frame!");
        return 0;
    }

    buf[ll] = '\0';
    LOGM_TRACE(loggerCtrlBoard, "getImuData", "frame=\"" << buf << "\"");

    int res = parseFrameBnoEVC(buf, ll, euler_x, euler_y, euler_z, calib_gyro, calib_accel, calib_mag);
    LOGM_DEBUG(loggerCtrlBoard, "getImuData", "res=" << res << ", euler_x=" << ioff(euler_x, 2) << ", euler_y=" << ioff(euler_y, 2) << ", euler_z=" << ioff(euler_z, 2)
        << ", calib_gyro=" << calib_gyro << ", calib_accel=" << calib_accel << ", calib_mag=" << calib_mag);

    if (res < 0) {
        LOGM_ERROR(loggerCtrlBoard, "getImuData", "parseFrame failed!");
        return 0;
    }

    return 1;
}

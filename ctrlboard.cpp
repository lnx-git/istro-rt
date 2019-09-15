#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <errno.h>  //fixme
#include "threads.h"
#include "ctrlboard.h"
#include "config.h"
#include "dataset.h"
#include "system.h"
#include "logger.h"

const int CTRLBOARD_PORT1_SPEED = 115200;  // 38400
const int CTRLBOARD_PORT2_SPEED = 115200;
const double CTRLBOARD_ZERO_EPS = 0.000001;  // precision calculation - epsilon: 1ms

LOG_DEFINE(loggerCtrlBoard, "ControlBoard");

void ControlBoard::setXX4(void)
{
    char ss[30] = "";
    
    sprintf(ss, "U4%03d\n", 3);
    writef(serialPort, ss, 6);
    ss[5] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}
void ControlBoard::setXX5(void)
{
    char ss[30] = "";
    
    sprintf(ss, "U5%03d\n", 3);
    writef(serialPort, ss, 6);
    ss[5] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}


void ircth_init(ircth_t &ircth)
{
    ircth.cnt = 0;
    ircth.time_start0 = -1;
    ircth.time_last = -1;
    for(int i = 0; i < IRCTH_LENGTH; i++) {
        ircth.ircv[i] = 0;
    }
}

double ircth_start0 = -1;

int ircth_process(ircth_t &ircth, int ircv, double time)
{
    if ((ircth.time_last < 0) || (time < ircth.time_last)) {
        ircth.time_last = time;
        ircth.cnt = 1;
        ircth.ircv[0] = 0;
        ircth.time_start0 = time;
        ircth_start0 = time;
        return 0;
    } 

    double dt = timeDelta2(ircth.time_last, time);  // always: dt >= 0
    if (dt > IRCTH_MAX_DTIME) {
        LOGM_ERROR(loggerCtrlBoard, "ircth_process", "msg=\"max dtime exceeded!\"");
        ircth_init(ircth);
        return -1;
    }
    
    double dt0 = timeDelta2(ircth.time_start0, ircth.time_last);  // dt0: from start of current interval to last time 
    if (dt0 < 0) dt0 = 0;  // dt0 >= 0
    double dt1 = IRCTH_TIME_INTERVAL - dt0;  // dt1: to the end of current interval
    //LOGM_TRACE(loggerCtrlBoard, "ircth_process", "dt=" << ioff(dt, 1) << ", dt0=" << ioff(dt0, 1) << ", dt1=" << ioff(dt1, 1));
    if (dt <= dt1 + CTRLBOARD_ZERO_EPS) {
        ircth.ircv[0] += ircv;
        ircth.time_last = time;
        return 1;
    }

    double dt_total = dt;
    while (dt >= CTRLBOARD_ZERO_EPS) {  // dt >= dt1
        if (dt1 >= CTRLBOARD_ZERO_EPS) {
            ircth.ircv[0] += ((double)ircv) * dt1 / dt_total;
            dt -= dt1;
        }
        /*LOGM_TRACE(loggerCtrlBoard, "ircth_process.while", "dt1=" << ioff(dt1, 1) << ", dt=" << ioff(dt, 1) << ", cnt=" << (int)ircth.cnt
        << ", time_start0=" << ioff((ircth.time_start0 < 0)?(-1):(timeDelta2(ircth_start0, ircth.time_start0)), 1)
        << ", time_last="   << ioff((ircth.time_last < 0)?(-1):(timeDelta2(ircth_start0, ircth.time_last)), 1)
        << ", i" << 0 << "=" << ioff(ircth.ircv[0], 1)
        << ", i" << 1 << "=" << ioff(ircth.ircv[1], 1)
        << ", i" << 2 << "=" << ioff(ircth.ircv[2], 1)
        << ", i" << 3 << "=" << ioff(ircth.ircv[3], 1)
        << ", i" << 4 << "=" << ioff(ircth.ircv[4], 1));*/
        // dont add new element if the loop will not continue
        if (dt < CTRLBOARD_ZERO_EPS) {
            break;
        }
        // next interval length
        dt1 = dt;
        if (dt1 > IRCTH_TIME_INTERVAL) {
            dt1 = IRCTH_TIME_INTERVAL;
        }
        // add new element
        if (ircth.cnt < IRCTH_LENGTH) {
            ircth.cnt++;
        }
        // move old elements
        for(int i = ircth.cnt - 1; i > 0; i--) {
            ircth.ircv[i] = ircth.ircv[i - 1];
        }
        // init new element
        ircth.ircv[0] = 0;
        ircth.time_start0 = timeAdd2(ircth.time_start0, IRCTH_TIME_INTERVAL);
    }
    
    ircth.time_last = time;
    /*LOGM_TRACE(loggerCtrlBoard, "ircth_process", "dt1=" << ioff(dt1, 1) << ", dt=" << ioff(dt, 1) << ", cnt=" << (int)ircth.cnt
    << ", time_start0=" << ioff((ircth.time_start0 < 0)?(-1):(timeDelta2(ircth_start0, ircth.time_start0)), 1)
    << ", time_last="   << ioff((ircth.time_last < 0)?(-1):(timeDelta2(ircth_start0, ircth.time_last)), 1)
    << ", i" << 0 << "=" << ioff(ircth.ircv[0], 1)
    << ", i" << 1 << "=" << ioff(ircth.ircv[1], 1)
    << ", i" << 2 << "=" << ioff(ircth.ircv[2], 1)
    << ", i" << 3 << "=" << ioff(ircth.ircv[3], 1)
    << ", i" << 4 << "=" << ioff(ircth.ircv[4], 1));*/
    return 1;
}

double ircth_get(ircth_t &ircth, double dt)
// returns number of impulses received in last "dt" milliseconds
{
    double ircv = 0;
    double dt0 = timeDelta2(ircth.time_start0, ircth.time_last);  // dt0: from start of current interval to last time
    //LOGM_TRACE(loggerCtrlBoard, "ircth_get0", "ircv=" << ioff(ircv, 1) << ", dt=" << ioff(dt, 1) << ", dt0=" << ioff(dt0, 1));
    if ((ircth.cnt > 0) && (dt0 >= CTRLBOARD_ZERO_EPS)) {
        if (dt >= dt0 - CTRLBOARD_ZERO_EPS) {
            ircv += ircth.ircv[0];
            dt -= dt0;
        } else {
            ircv += ircth.ircv[0] * dt / dt0;
            dt = 0;
        }
    }
    //LOGM_TRACE(loggerCtrlBoard, "ircth_get1", "ircv=" << ioff(ircv, 1) << ", dt=" << ioff(dt, 1));    
    for(int i = 1; (i < ircth.cnt) && (dt >= CTRLBOARD_ZERO_EPS); i++) {
        if (dt >= IRCTH_TIME_INTERVAL - CTRLBOARD_ZERO_EPS) {
            ircv += ircth.ircv[i];
            dt -= IRCTH_TIME_INTERVAL;
    //LOGM_TRACE(loggerCtrlBoard, "ircth_get2", "ircv=" << ioff(ircv, 1) << ", dt=" << ioff(dt, 1));            
        } else {
            ircv += ircth.ircv[i] * dt / IRCTH_TIME_INTERVAL;
            dt = 0;
    //LOGM_TRACE(loggerCtrlBoard, "ircth_get3", "ircv=" << ioff(ircv, 1) << ", dt=" << ioff(dt, 1));                        
        }
    }
    //LOGM_TRACE(loggerCtrlBoard, "ircth_get.result", "ircv=" << ioff(ircv, 1) << ", dt=" << ioff(dt, 1));
    return ircv; 
}

void ircth_print(ircth_t &ircth)
{
    std::stringstream ss;
    
    double time0 = timeBegin();
    double ircv500  = ircth_get(ircth,  500);
    double ircv1000 = ircth_get(ircth, 1000);
    
    for(int i = 0; i < ircth.cnt; i++) { 
        ss << ", i" << i << "=" << ioff(ircth.ircv[i], 1);
    }
    LOGM_TRACE(loggerCtrlBoard, "ircth_print", "ircv500=" << ioff(ircv500, 2) << ", ircv1000=" << ioff(ircv1000, 2)
        << ", time_start0=" << ioff((ircth.time_start0 < 0)?(-1):(timeDelta2(time0, ircth.time_start0)), 1)
        << ", time_last="   << ioff((ircth.time_last < 0)?(-1):(timeDelta2(time0, ircth.time_last)), 1)
        << ", cnt=" << ircth.cnt << ss.str());
}

/*
void ircth_test()
{
    ircth_t ircth;

    ircth_init(ircth);
    ircth_print(ircth);

    ircth_process(ircth, 2,  timeBegin());
    ircth_print(ircth);
    msleep(50);
    
    for(int i=0; i<40; i++) {
        ircth_process(ircth, 2, timeBegin());
        ircth_print(ircth);
        msleep(100);    
    }
}
*/    
/*
    t = timeAdd2(t, 100);
    ircth_process(ircth, 2, t);
    ircth_print(ircth);

    t = timeAdd2(t, 100);
    ircth_process(ircth, 2, t);
    ircth_print(ircth);

    t = timeAdd2(t, 100);
    ircth_process(ircth, 2, t);
    ircth_print(ircth);

    t = timeAdd2(t, 100);
    ircth_process(ircth, 2, t);
    ircth_print(ircth);

    t = timeAdd2(t, 100);
    ircth_process(ircth, 2, t);
    ircth_print(ircth);

    t = timeAdd2(t, 100);
    ircth_process(ircth, 2, t);
    ircth_print(ircth);
    
    t = timeAdd2(t, 100);
    ircth_process(ircth, 2, t);
    ircth_print(ircth);

//    ircth_get(ircth, 500);
//    ircth_get(ircth, 1000);

    t = timeAdd2(t, 100);
    ircth_process(ircth, 2, t);
    ircth_print(ircth);
    
    ircth_get(ircth, 100);
    ircth_get(ircth, 200);
    ircth_get(ircth, 400);
    ircth_get(ircth, 500);
    ircth_get(ircth, 600);
    ircth_get(ircth, 700);
    ircth_get(ircth, 1500);
    ircth_get(ircth, 1600);
    ircth_get(ircth, 1700);
    ircth_get(ircth, 1800);
    ircth_get(ircth, 2100);
    ircth_get(ircth, 5000);
}
*/

double obstf_to = -1;    // when was the forward obstacle last seen?

const int CTRLB_OBSTF_TIMEOUT = 1000;  // v milisekundach

int ctrlb_obstState(int ctrlb_state, int ctrlb_velocity) 
/* returns 1 if obstacle is detected in that direction in which the robot is moving */
{
    if ((ctrlb_velocity < 0) || (ctrlb_velocity == VEL_ZERO)) {
        return (ctrlb_state == CTRLB_STATE_OBSTA);
    } 

    if (ctrlb_velocity >= VEL_ZERO) {
        int obstf = (ctrlb_state == CTRLB_STATE_OBSTF) || (ctrlb_state == CTRLB_STATE_OBSTA);
        // keep reporting obstacle forward (=1) for addidional CTRLB_OBSTF_TIMEOUT miliseconds
        if (obstf) {
            obstf_to = timeBegin();
        } else { 
            if (obstf_to >= 0) {
                if (timeDelta(obstf_to) < CTRLB_OBSTF_TIMEOUT) {
                    obstf = 1;
                    LOGM_TRACE(loggerCtrlBoard, "ctrlb_obstState", "msg=\"obstacle forward (time)!\", obstf=" << obstf 
                        << ", dt=" << ioff(timeDelta(obstf_to), 2));
                } else { 
                    LOGM_TRACE(loggerCtrlBoard, "ctrlb_obstState", "msg=\"obstacle forward (timeout)!\", obstf=" << obstf 
                        << ", dt=" << ioff(timeDelta(obstf_to), 2));
                    obstf_to = -1;
                }
            }
        }
        return obstf;
    } else {
        return (ctrlb_state == CTRLB_STATE_OBSTB) || (ctrlb_state == CTRLB_STATE_OBSTA);
    } 
}

/* ---------------------------- */

// fixme
extern Threads threads;

ControlBoard::ControlBoard(void)
{
    serialPort = -1;
    serialPort2 = -1;
    rxlen1 = 0;
    rxlen2 = 0;
    
    ircth_init(ircth);
} 

int ControlBoard::init(const char *portName, const char *portName2) 
{
    //ircth_test(); 
     
    serialPort = openSerialPort(portName, CTRLBOARD_PORT1_SPEED);
    
    if (serialPort == -1) {
        LOGM_ERROR(loggerCtrlBoard, "init", "unable to open serial port1 \"" << portName << "\"!");
        return -1;
    }
    // sleep(3);
    LOGM_INFO(loggerCtrlBoard, "init", "serial port1 \"" << portName << "\" (baudrate=" << CTRLBOARD_PORT1_SPEED << ") opened successfully!");

    if (portName2 == NULL) {
        serialPort2 = -1;
        LOGM_WARN(loggerCtrlBoard, "init", "port2 not initialized!");
        return 0;
    }

    serialPort2 = openSerialPort(portName2, CTRLBOARD_PORT2_SPEED);
    
    if (serialPort2 == -1) {
        LOGM_ERROR(loggerCtrlBoard, "init", "unable to open serial port2 \"" << portName2 << "\"!");
        return -2;
    }
    // sleep(3);
    LOGM_INFO(loggerCtrlBoard, "init", "serial port2 \"" << portName2 << "\" (baudrate=" << CTRLBOARD_PORT2_SPEED << ") opened successfully!");

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

void ControlBoard::stop(void)
{
    int result = writef(serialPort, "STOP\n", 6);
    LOGM_INFO(loggerCtrlBoard, "stop", "result=" << result);
}

void ControlBoard::setSpeed(int speed)
{
    char ss[30] = "";

    if(speed < 90) speed = 90;
    if(speed > VEL_MAX) speed = VEL_MAX;

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

void ControlBoard::displayText(const char *str)
{
    char ss[30] = "";
    
    int ll = strlen(str);
    if (ll >= (int)sizeof(ss) - 1) ll = sizeof(ss) - 2;
    
    ss[0] = 'D';
    memcpy((void *)&ss[1], (const void *)str, ll);
    ss[++ll] = '\n';
    
    writef(serialPort, ss, ll + 1);
    ss[ll] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}

/*
LI - LED Index - zapne jednu LEDku
	LI:<INDEX>:<R>:<G>:<B>:<BLINK>
LM - LED Maska - rozsvieti diody podla masky, zatial plati len pre diody 0 - 7, cize hodnota masky je 0-255 resp 1-255
	LM:<MASK>:<R>:<G>:<B>:<BLINK>
*/

void ControlBoard::setLedIndex(int index, int r, int g, int b, int blink)
{
    char ss[30] = "";

    sprintf(ss, "LI:%d:%d:%d:%d:%d\n", index, r, g, b, blink);
    int ll = strlen(ss);
    writef(serialPort, ss, ll);
    ss[ll-1] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}

void ControlBoard::setLedMask(int mask, int r, int g, int b, int blink)
{
    char ss[30] = "";
       
    /* fixme: mask - conversion to hexa string not implemeted */
    //sprintf(ss, "LM:%d:%d:%d:%d:%d\n", mask, r, g, b, blink);
    sprintf(ss, "LM:0xFF:%d:%d:%d:%d\n", /*mask,*/ r, g, b, blink);
    int ll = strlen(ss);
    writef(serialPort, ss, ll);
    ss[ll-1] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}

/*
LP - LED Program
    LP<PROGRAM>
    zatil pozna tieto programy:
        LP0 - vypne vsetky ledky
        LP1 - vsetky ledky svietia nabielo
        LP2 - vsetky ledky svietia na cerveno
        LP3 - vsetky ledky svietia na zeleno
        LP4 - vsetky ledky svietia na modro
*/

void ControlBoard::setLedProgram(int prg)
{
    char ss[6] = "";
       
    ss[0] = 'L';
    ss[1] = 'P';
    ss[2] = (char)('0' + prg);
    ss[3] = '\n';
    
    writef(serialPort, ss, 4);
    ss[3] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}

void ControlBoard::setBallDrop(int cnt)
{
    char ss[6] = "";
       
    ss[0] = 'S';
    ss[1] = '3';
    ss[2] = (char)('0' + cnt);
    ss[3] = '\n';
    
    writef(serialPort, ss, 4);
    ss[3] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}

void ControlBoard::writeString(const char *str)
{
    char ss[30] = "";
    
    int ll = strlen(str);
    if (ll >= (int)sizeof(ss)) ll = sizeof(ss) - 1;
    
    memcpy((void *)ss, (const void *)str, ll);
    ss[ll] = '\n';
    
    writef(serialPort, ss, ll + 1);
    ss[ll] = 0;
    LOGM_INFO(loggerCtrlBoard, "write", "data=\"" << ss << "\"");
}

const unsigned char RX_FRAME_BEGIN = '{';
const unsigned char RX_FRAME_END   = '}';

int ControlBoard::readFrame(int fd, unsigned char *buf, int count, int &rxlen, unsigned char *rxbuff)
{
    //rxbuff[rxlen] = '\0';
    //printf("rxbuff(%d): \"%s\"\n", rxlen, rxbuff);
    int ll = readf(fd, (void *)&rxbuff[rxlen], RXBUFF_LENGTH - rxlen);
    if (ll < 0) {
        if (errno != EAGAIN) {
            LOGM_ERROR(loggerCtrlBoard, "readFrame", "unable to read from serial port (" << ll << ", " << errno << ", " << strerror(errno) << ")!");
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
        // frame end not found 
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

const unsigned char RX_LINE_CR = '\n';
const unsigned char RX_LINE_LF = '\r';

int ControlBoard::readLn(int fd, unsigned char *buf, int count, int &rxlen, unsigned char *rxbuff)
{
    //rxbuff[rxlen] = '\0';
    //printf("rxbuff(%d): \"%s\"\n", rxlen, rxbuff);
    int ll = readf(fd, (void *)&rxbuff[rxlen], RXBUFF_LENGTH - rxlen);
    if (ll < 0) {
        if (errno != EAGAIN) {
            LOGM_ERROR(loggerCtrlBoard, "readLn", "unable to read from serial port (" << ll << ", " << errno << ", " << strerror(errno) << ")!");
            return -1;
        }
        ll = 0;    // ignore EAGAIN
    }
    rxlen += ll;
    //rxbuff[rxlen] = '\0';
    //printf("rxbuff(%d, %d): \"%s\"\n", rxlen, ll, rxbuff);

    // find start of frame
    int i = 0;
    while ((i < rxlen) && ((rxbuff[i] == RX_LINE_CR) || (rxbuff[i] == RX_LINE_LF))) {
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
    while ((i < rxlen) && (rxbuff[i] != RX_LINE_CR) && (rxbuff[i] != RX_LINE_LF)) {
        i++;
    }
    // printf("frame_end: %d\n", i);
    if (i >= rxlen) {
        // frame end not found 
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

    // return all characters except CR/LF
    memcpy((void *)buf, (void *)&rxbuff[0], i);
    // remove frame characters from the buffer
    if (i + 1 < rxlen) {
        memcpy((void *)rxbuff, (void *)&rxbuff[i + 1], rxlen - i - 1);
        rxlen -= i + 1;
    } else {
        rxlen = 0;
    }
    // return line length
    return i;
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

    int ll = readFrame(serialPort2, buf, RXBUFF_LENGTH, rxlen2, rxbuff2);
    if (ll < 0) {
        LOGM_ERROR(loggerCtrlBoard, "getImuData", "readFrame failed!");
        return -1;
    }
    if (ll == 0) {
#ifdef CTRLBOARD_LOG_TRACE0
        LOGM_TRACE(loggerCtrlBoard, "getImuData", "no frame!");
#endif
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

const char RX2_FRAME_HEADER[]   = "R:";
const int  RX2_FRAME_HEADER_LEN =  2;
const char RX2_PARAM_DELIMITER  = ':';
const int  RX2_PARAM_LEN        = 64;    // maximum parameter length

int ControlBoard::parseInt2(unsigned char *buf, int count, int &idx, int &x)
{
    char pp[RX2_PARAM_LEN + 1];

    int ll = 0;
    while ((idx < count) && (buf[idx] != RX2_PARAM_DELIMITER) && (ll < RX2_PARAM_LEN)) {
        pp[ll++] = (char)buf[idx++];
    }
    if ((idx < count) && (buf[idx] == RX2_PARAM_DELIMITER)) {
        idx++;
    }

    pp[ll] = '\0';
    // printf("parseInt2: \"%s\"\n", pp);
    if (sscanf(pp, "%d", &x) < 1) { 
        return -1;
    }

    return 0;
}

int ControlBoard::parseDouble2(unsigned char *buf, int count, int &idx, double &x)
{
    char pp[RX2_PARAM_LEN + 1];

    int ll = 0;
    while ((idx < count) && (buf[idx] != RX2_PARAM_DELIMITER) && (ll < RX2_PARAM_LEN)) {
        pp[ll++] = (char)buf[idx++];
    }
    if ((idx < count) && (buf[idx] == RX2_PARAM_DELIMITER)) {
        idx++;
    }

    pp[ll] = '\0';
    // printf("parseDouble2: \"%s\"\n", pp);
    if (sscanf(pp, "%lf", &x) < 1) { 
        return -1;
    }

    return 0;
}

/*
R:<STAV>:<HEAD>:<IRC>:<S1>:<S2>:<B1>:<T>:<U1>:<U2>:<U3>:<U4>:<U5> 10x za sekundu
R:<STAV>:<HEAD>:<IRC>:<S1>:<S2>:<B1>:<T>  50x za sekundu
*/    
            
int ControlBoard::parseFrameSrvData(unsigned char *buf, int count, 
            int &state, double &heading, int &ircv, int &angle, int &velocity, int &loadd, int &cbtime, 
            int &ulsd1, int &ulsd2, int &ulsd3, int &ulsd4, int &ulsd5)
{
    heading = 0; 
    state = -1;
    ircv = angle = velocity = 0;
    loadd = cbtime = -1;
    ulsd1 = ulsd2 = ulsd3 = ulsd4 = ulsd5 = -1;

    if (count < RX2_FRAME_HEADER_LEN) {
        return -1;  // frame header not found
    }

    if (memcmp(buf, RX2_FRAME_HEADER, RX2_FRAME_HEADER_LEN) != 0) {
        return -2;  // frame header does not match
    }

    int idx = RX2_FRAME_HEADER_LEN;
    if (parseInt2(buf, count, idx, state) < 0) {    
        return -3;  // parameter parsing failed
    }
        
    if (parseDouble2(buf, count, idx, heading) < 0) {    
        return -7;  // parameter parsing failed
    }

    if (parseInt2(buf, count, idx, ircv) < 0) {    
        return -8;  // parameter parsing failed
    }
    
    if (parseInt2(buf, count, idx, angle) < 0) {    
        return -9;  // parameter parsing failed
    }
    
    if (parseInt2(buf, count, idx, velocity) < 0) {    
        return -10;  // parameter parsing failed
    }

    if (parseInt2(buf, count, idx, loadd) < 0) {    
        return -13;  // parameter parsing failed
    }

    if (parseInt2(buf, count, idx, cbtime) < 0) {    
        return -14;  // parameter parsing failed
    }
    
    if (parseInt2(buf, count, idx, ulsd1) < 0) {
        return 0;    // ultrasonic values are optional
        //return -4; // parameter parsing failed
    }
    
    if (parseInt2(buf, count, idx, ulsd2) < 0) {    
        return -5;  // parameter parsing failed
    }
    
    if (parseInt2(buf, count, idx, ulsd3) < 0) {    
        return -6;  // parameter parsing failed
    }

    if (parseInt2(buf, count, idx, ulsd4) < 0) {    
        return -11;  // parameter parsing failed
    }
    
    if (parseInt2(buf, count, idx, ulsd5) < 0) {    
        return -12;  // parameter parsing failed
    }

    return 0;
}

int ControlBoard::getServoData(int &state, double &heading, int &ircv, double &ircv500, int &angle, int &velocity, 
        int &loadd, int &cbtime, int &ulsd1, int &ulsd2, int &ulsd3, int &ulsd4, int &ulsd5)
{
    unsigned char buf[RXBUFF_LENGTH+1];

    double t = timeBegin();
    int ll = readLn(serialPort, buf, RXBUFF_LENGTH, rxlen1, rxbuff1);
    if (ll < 0) {
        LOGM_ERROR(loggerCtrlBoard, "getServoData", "msg=\"readLn failed!\"");
        return -1;
    }
    if (ll == 0) {
#ifdef CTRLBOARD_LOG_TRACE0
        LOGM_TRACE(loggerCtrlBoard, "getServoData", "msg=\"no frame!\"");
#endif
        return 0;
    }

    buf[ll] = '\0';
    LOGM_TRACE(loggerCtrlBoard, "getServoData", "frame=\"" << buf << "\"");

    int res = parseFrameSrvData(buf, ll, state, heading, ircv, angle, velocity, 
                  loadd, cbtime, ulsd1, ulsd2, ulsd3, ulsd4, ulsd5);

    if (res >= 0) {
        ircth_process(ircth, ircv, t);
        //ircth_print(ircth);
        ircv500 = ircth_get(ircth,  500);
    } else {
        ircv500 = 0;
    }

    LOGM_DEBUG(loggerCtrlBoard, "getServoData", "res=" << res << ", state=" << state 
        << ", heading=" << ioff(heading, 2) << ", ircv=" << ircv << ", ircv500=" << ioff(ircv500, 2) << ", angle=" << angle << ", velocity=" << velocity
        << ", loadd=" << loadd << ", cbtime=" << cbtime
        << ", ulsd1=" << ulsd1 << ", ulsd2=" << ulsd2 << ", ulsd3=" << ulsd3 << ", ulsd4=" << ulsd4 << ", ulsd5=" << ulsd5);

    if (res < 0) {
        LOGM_ERROR(loggerCtrlBoard, "getServoData", "msg=\"parseFrame failed!\"");
        return 0;
    }

    return 1;
}

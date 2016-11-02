#include <math.h>
#include <iostream>
#include "myahrs.h"
#include "system.h"
#include "logger.h"

using namespace std;

LOG_DEFINE(loggerAHRS, "AHRSystem");

#ifdef ISTRO_MYAHRS_PLUS

#include "myahrs_plus.hpp"

using namespace WithRobot;

const int MYAHRS_BAUDRATE = 115200;

const char* MYAHRS_DIVIDER = "1";  // 100 Hz

int AHRSystem::init(const char* serial_device) 
{   
    MyAhrsPlus *psensor;

    psensor = new MyAhrsPlus();
    psensor_ = (void *)psensor;
    
    /* start communication with the myAHRS+ */
    if (psensor->start(serial_device, MYAHRS_BAUDRATE) == false) {
        LOGM_ERROR(loggerAHRS, "init", "start failed!");
        return -1;
    }

    /* set ascii output format: select euler angle */
    if (psensor->cmd_ascii_data_format("RPY") == false) {
        LOGM_ERROR(loggerAHRS, "init", "cmd_ascii_data_format failed!");
        return -2;
    }

    if (psensor->cmd_divider(MYAHRS_DIVIDER) == false) {
        LOGM_ERROR(loggerAHRS, "init", "cmd_divider failed!");
        return -3;
    }

    /* set transfer mode: AC = ASCII Message & Continuous mode */
    if (psensor->cmd_mode("AC") == false) {
        LOGM_ERROR(loggerAHRS, "init", "cmd_mode failed!");
        return -4;
    }

    return 0;
}

void AHRSystem::close(void)
{
    MyAhrsPlus *psensor = (MyAhrsPlus *)psensor_;

    /* stop communication */
    psensor->stop();

    delete psensor;
    psensor_ = NULL;
}

int AHRSystem::getData(double &roll, double &pitch, double &yaw)
{
    MyAhrsPlus *psensor = (MyAhrsPlus *)psensor_;

    /* waiting for new data */
    if (psensor->wait_data() == false) {
        LOGM_ERROR(loggerAHRS, "getData", "wait_data failed!");
        return -1;
    }

    /* read counter */
    // uint32_t sample_count = 0;
    // sample_count = sensor.get_sample_count();

    /* copy sensor data */
    SensorData sensor_data;
    psensor->get_data(sensor_data);

    /* get euler angle */
    EulerAngle& e = sensor_data.euler_angle;

    roll  = e.roll;
    pitch = e.pitch;
    yaw   = e.yaw;

    LOGM_TRACE(loggerAHRS, "getData", "roll=" << ioff(e.roll, 2) << ", pitch=" << ioff(e.pitch, 2) << ", yaw=" << ioff(e.yaw, 2));

    return 0;
}

#else 

int AHRSystem::init(const char* serial_device) 
{   
    LOGM_WARN(loggerAHRS, "init", "MOCK AHRSYSTEM implementation, DEBUG ONLY!!");
    return 0;
}

void AHRSystem::close(void)
{
}

int AHRSystem::getData(double &roll, double &pitch, double &yaw)
{
    roll  = 0;
    pitch = 0;
    yaw   = 0;

    msleep(10);

    return 0;
}

#endif

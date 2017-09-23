//#include <iostream>
#include <opencv2/opencv.hpp>
#include "logger.h"

using namespace cv;
using namespace std;

LOG_DEFINE(loggerMTime, "mtime");

double timeBegin(void)
{
    return (double)getTickCount();
}

double timeDelta2(double t1, double t2)
{
    // return time difference in milliseconds
    return (t2 - t1) * 1000.0 / getTickFrequency();
}

double timeDelta(double t)
{
    // return time difference in milliseconds
    return timeDelta2(t, getTickCount());
}

void timeEnd(const string& str, double t)
{
    // print time difference in milliseconds
    LOGM_TRACE(loggerMTime, "time", "m=\"" << str << "\", dt=" << ioff(timeDelta(t), 2));    
}

double timeAdd2(double t, double dt)
// dt = timeDelta2(t, timeAdd2(t, dt))
{
    return t + (dt * getTickFrequency() / 1000.0);
}

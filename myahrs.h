#ifndef __MYAHRS_H__
#define __MYAHRS_H__

#include "system.h"

class AHRSystem {
private:
    void *psensor_;
    
public:
    int init(const char* serial_device);
    void close(void);

    int getData(double &roll, double &pitch, double &yaw);
};

#endif

#ifndef __GPSDEV_H__
#define __GPSDEV_H__

#include "system.h"

class GpsDevice {
private:
    void *pgps_;
    
public:
    int init(void);
    void close(void);

    int getData(double &latitude, double &longitude, double &course);
};

#endif

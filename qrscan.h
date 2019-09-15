#ifndef __QRSCAN_H__
#define __QRSCAN_H__

#include <opencv2/opencv.hpp>
#include "system.h"

#ifdef ISTRO_QRSCANNER
#include <zbar.h>
#endif

class QRScanner {
private:
#ifdef ISTRO_QRSCANNER
    zbar::ImageScanner scanner;
#endif

private:
    int parseDouble(const char *buf, int count, int &idx, double &x);
    int parseFrameGeo(const char *buf, int count, double &latitude, double &longitude);

public:
    QRScanner();

    void init(void);

    int scanGeo(cv::Mat &im, double &latitude, double &longitude);

public:
    void test(void);
};

#endif

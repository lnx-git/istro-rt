#include <iostream>
#include <stdio.h>
#include "qrscan.h"
#include "config.h"
#include "mtime.h"
#include "logger.h"

using namespace cv;
using namespace std;

LOG_DEFINE(loggerQRScanner, "QRScanner");

QRScanner::QRScanner()
{
}

const char QRGEO_FRAME_HEADER[]   = "geo:";
const int  QRGEO_FRAME_HEADER_LEN =  4;
const int  QRGEO_FRAME_LEN        = 64;
const char QRGEO_PARAM_DELIMITER  = ',';
const int  QRGEO_PARAM_LEN        = 32;    // maximum parameter length

int QRScanner::parseDouble(const char *buf, int count, int &idx, double &x)
{
    char pp[QRGEO_PARAM_LEN + 1];

    int ll = 0;
    while ((idx < count) && (buf[idx] != QRGEO_PARAM_DELIMITER) && (ll < QRGEO_PARAM_LEN)) {
        pp[ll++] = (char)buf[idx++];
    }
    if ((idx < count) && (buf[idx] == QRGEO_PARAM_DELIMITER)) {
        idx++;
    }

    pp[ll] = '\0';
    // printf("parseDouble: \"%s\"\n", pp);
    if (sscanf(pp, "%lf", &x) < 1) { 
        return -1;
    }

    return 0;
}

int QRScanner::parseFrameGeo(const char *buf, int count, double &latitude, double &longitude)
// parse geo string: "geo:48.8016394,16.8011145"
{
    latitude = longitude = ANGLE_NONE;

    if (count < QRGEO_FRAME_HEADER_LEN) {
        return -1;  // frame header not found
    }

    if (memcmp(buf, QRGEO_FRAME_HEADER, QRGEO_FRAME_HEADER_LEN) != 0) {
        return -2;  // frame header does not match
    }

    int idx = QRGEO_FRAME_HEADER_LEN;
    double lat = ANGLE_NONE;
    double lon = ANGLE_NONE;

    if (parseDouble(buf, count, idx, lat) < 0) {    
        return -3;  // parameter parsing failed
    }

    if (parseDouble(buf, count, idx, lon) < 0) {    
        return -4;  // parameter parsing failed
    }

    latitude = lat;
    longitude = lon;

    // printf("parseFrameGeo: result OK!\n");
    LOGM_INFO(loggerQRScanner, "parseFrameGeo", "res=0, latitude=" << ioff(latitude, 6) << ", longitude=" << ioff(longitude, 6));
    return 0;
}

#ifdef ISTRO_QRSCANNER

void QRScanner::init(void)
{
    // Configure scanner
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

int QRScanner::scanGeo(Mat &im, double &latitude, double &longitude)
{
    Mat imGray;

    latitude = longitude = ANGLE_NONE;

    // Convert image to grayscale
    double t = timeBegin();
    cvtColor(im, imGray, COLOR_BGR2GRAY);
    timeEnd("QRScanner::scanGeo.cvtColor(BGR2GRAY)", t);

    // Wrap image data in a zbar image
    t = timeBegin();
    zbar::Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
    timeEnd("QRScanner::scanGeo.zbar::Image()", t);

    // Scan the image for barcodes and QRCodes
    t = timeBegin();
    int res = scanner.scan(image);
    timeEnd("QRScanner::scanGeo.zbar::ImageScanner.scan()", t);
    if (res == 0) {
        LOGM_DEBUG(loggerQRScanner, "scanGeo", "zbar::ImageScanner.scan() no symbols were found!");
        return 0;
    }
    if (res < 0) {
        LOGM_ERROR(loggerQRScanner, "scanGeo", "zbar::ImageScanner.scan() failed!");
        return -1;
    }

    // Print results
    res = 0;
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        string type = symbol->get_type_name();
        string data = symbol->get_data();

        int x[4] = { -1, -1, -1, -1 };
        int y[4] = { -1, -1, -1, -1 };

        // Obtain location
        for(int i = 0; i < symbol->get_location_size(); i++) {
            if (i >= 4) break;
            x[i] = symbol->get_location_x(i);
            y[i] = symbol->get_location_y(i);
        }

        // parse "geo" string!
        res = 0;
        if (type.compare("QR-Code") == 0) {
            if (parseFrameGeo(data.c_str(), data.length(), latitude, longitude) >= 0) res = 1;
        }
        if (res > 0) {
            LOGM_INFO(loggerQRScanner, "scanGeo", "type=\"" << type << "\", data=\"" << data
                << "\", res=1, latitude=" << ioff(latitude, 6) << ", longitude=" << ioff(longitude, 6)
                << ", size=" << symbol->get_location_size()
                << ", x1=" << x[0] << ", y1=" << y[0] << ", x2=" << x[1] << ", y2=" << y[1]
                << ", x3=" << x[2] << ", y3=" << y[2] << ", x4=" << x[3] << ", y4=" << y[3]);
            return 1;
        }

        LOGM_DEBUG(loggerQRScanner, "scanGeo", "type=\"" << type << "\", data=\"" << data
            << "\", res=0, latitude=" << ioff(latitude, 6) << ", longitude=" << ioff(longitude, 6)
            << ", size=" << symbol->get_location_size()
            << ", x1=" << x[0] << ", y1=" << y[0] << ", x2=" << x[1] << ", y2=" << y[1]
            << ", x3=" << x[2] << ", y3=" << y[2] << ", x4=" << x[3] << ", y4=" << y[3]);
    }

    // No geodata found!
    return 0;
}

#else

void QRScanner::init(void)
{
    LOGM_WARN(loggerQRScanner, "init", "MOCK QRSCANNER implementation, DEBUG ONLY!!");
}

double qrscan_to = -1;

int QRScanner::scanGeo(Mat &im, double &latitude, double &longitude)
{
    string type = "??QR-Code";
    //string data = "geo:48.8055594,16.8066645";  // Lednice  // "geo:48.8016394,16.8011145";
    string data;

    if (qrscan_to < 0) {
        qrscan_to = timeBegin();
    }

    // after 3 seconds change qr-code values
    if (timeDelta(qrscan_to) < 3000) {
        //data = "geo:48.1564496,17.1593964";    // Park AH - S3
        data = "geo:48.8025706,16.804208";       // Lednice - N4
    } else {
        //data = "geo:48.1565383,17.1567450";    // Park AH
        data = "geo:48.7994356,16.8063114";      // Lednice - M5
    }

    latitude  = ANGLE_NONE;  // 48.8016394;
    longitude = ANGLE_NONE;  // 16.8011145;

    int x[4] = { -1, -1, -1, -1 };
    int y[4] = { -1, -1, -1, -1 };

    int res = 0; 
    if (type.compare("??QR-Code") == 0) {
        if (parseFrameGeo(data.c_str(), data.length(), latitude, longitude) >= 0) res = 1;
    }

    LOGM_DEBUG(loggerQRScanner, "scanGeo", "type=\"" << type << "\", data=\"" << data
        << "\", res=" << res  << ", latitude=" << ioff(latitude, 6) << ", longitude=" << ioff(longitude, 6)
        << ", size=" << 0
        << ", x1=" << x[0] << ", y1=" << y[0] << ", x2=" << x[1] << ", y2=" << y[1]
        << ", x3=" << x[2] << ", y3=" << y[2] << ", x4=" << x[3] << ", y4=" << y[3]);

    return 1;
}

#endif

void QRScanner::test(void)
{
    double latitude, longitude;

    Mat im = imread("sample/zbar-test.jpg");

    scanGeo(im, latitude, longitude);

    im = imread("sample/0001126_cesta_vpravo.jpg");

    scanGeo(im, latitude, longitude);
}

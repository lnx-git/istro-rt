#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "sample.h"
#include "logger.h"

using namespace cv;
using namespace std;

LOG_DEFINE(loggerSample, "sample");

void convert(const Mat& src, Mat& dst)
{
    cvtColor(src, dst, COLOR_BGR2HSV);
    // fixme: pri pouzivani Lab nefunguje evalLT, pretoze je urobena na 45x64x64; pre Lab treba 64x64x64 
    //cvtColor(src, dst, COLOR_BGR2Lab);
}

void convertInv(const Mat& src, Mat& dst)
{
    cvtColor(src, dst, COLOR_HSV2BGR);
    //cvtColor(src, dst, COLOR_Lab2BGR);
}

void convertImage(const Mat& src, Mat& dst, int vv)
{
    Mat tmp;

    cvtColor(src, tmp, COLOR_BGR2HSV);
    //cvtColor(src, tmp, COLOR_BGR2Lab);

    for(int y = 0; y < src.rows; y++) {
        for(int x = 0; x < src.cols; x++) {
            Vec3b v = tmp.at<Vec3b>(y, x);
            v.val[1] = 255;
            if (vv) v.val[2] = 255;
            tmp.at<Vec3b>(y, x) = v;
        }
    }

    cvtColor(tmp, dst, COLOR_HSV2BGR);
    //cvtColor(tmp, dst, COLOR_Lab2BGR);
}

/* ----------------------------------- */

int checkPoint(const Mat& src, Point& p)
{
    if ( p.x < 0 || p.x >= src.cols - SAMPLE_WIDTH || p.y < 0 || p.y >= src.rows - SAMPLE_HEIGHT ) {
        return 0;
    }

    return  1;
}

/* ----------------------------------- */

double timeBegin(void)
{
    return (double)getTickCount();
}

double timeEnd(const string& str, double t)
{
    // print time difference in milliseconds
    double dt = ((double)getTickCount() - t)*1000.0/getTickFrequency();
    LOGM_TRACE(loggerSample, "time", "m=\"" << str << "\", dt=" << ioff(dt, 2));    
}

/* ----------------------------------- */

void dmap_init(int *p)
{
    dmap_set(p, -1);
}

void dmap_set(int *p, int value)
{
    for(int i = 0; i < DEGREE_MAP_COUNT; i++) {
        *(p++) = value;
    }
}

void dmap_fill(int *p, float angle1, float angle2, int value)
// angle1 < angle2
{
    int i1 = trunc(angle1);
    int i2 = trunc(angle2);
    //printf("dmap_fill: angle1=%d, angle2=%d, value=%d\n", i1, i2, value);
    
    if (i1 < 0) i1 = 0;
    if (i1 > DEGREE_MAP_COUNT) i1 = DEGREE_MAP_COUNT;
    if (i2 < 0) i2 = 0;
    if (i2 > DEGREE_MAP_COUNT) i2 = DEGREE_MAP_COUNT;  // i2 could be out of array    
    for(int i = i1; i < i2; i++) {
        if (p[i] < 0) {
            // always write new value
            p[i] = value;
        } else {
            // 1 could be overwritten with 0, but 0 will never be overwritten by 1
            if ((value == 0) && (p[i] != 0)) {
                p[i] = 0;
            }
        }
    }
}

void dmap_finish(int *p)
{
    //dmap_print(p, "dmap_finish.before");

    // fill all -1 at the beginning with the first value >= 0
    int i1 = 0;
    while (i1 < DEGREE_MAP_COUNT) {
        if (p[i1] < 0) {
            i1++;
            continue;
        }
        break;
    }

    if (i1 < DEGREE_MAP_COUNT) {
        for(int i = 0; i < i1; i++) {
            p[i] = p[i1];
        }
    }
    
    // fill all remaining -1 with last found value
    int last = p[0];
    for(int i = 1; i < DEGREE_MAP_COUNT; i++) {
        if (p[i] < 0) {
            p[i] = last;
        } else {
            last = p[i];
        }
    }
    
    //dmap_print(p, "dmap_finish.after");
}

void dmap_apply(int *p, int *p2)
{
    for(int i = 0; i < DEGREE_MAP_COUNT; i++) {
        if (p[i] < 0) {
            // always write new value
            p[i] = p2[i];
        } else {
            // 1 could be overwritten with 0, but 0 will never be overwritten by 1
            if ((p2[i] == 0) && (p[i] != 0)) {
                p[i] = 0;
            }
        }
    }
}

void dmap_find(int *p, int minlen, int mid, int &idx1, int &idx2)
{
    int best_i1 = -1;
    int best_i2 = -1;
    int best_pos = DEGREE_MAP_COUNT;

    int i1, i2, pos, pos2;
    
    i1 = 0; 
    while (i1 < DEGREE_MAP_COUNT) {
        if (p[i1] == 0) {
            i1++;
            continue;
        }
        // we found start of interval of "1" -> i1
        i2 = i1 + 1;
        while ((i2 < DEGREE_MAP_COUNT) && (p[i2] == 1)) {
            i2++;
        }
        i2--;
        // end of interval -> i2
        // check interval length
        if (i2 - i1 + 1 >= minlen) {
            // check interval position
            pos = i1 - mid;
            if (pos < 0) pos = -pos;
            pos2 = i2 - mid;
            if (pos2 < 0) pos2 = -pos2;
            if (pos2 < pos) pos = pos2;
            //printf("findInt: i1=%d, i2=%d, pos=%d\n", i1, i2, pos);
            // pos = interval position
            if (best_pos > pos) {
                best_pos = pos;
                best_i1 = i1;
                best_i2 = i2;
            }
        } 
        // skip to the next interval
        i1 = i2 + 1;
    }    

    idx1 = best_i1;
    idx2 = best_i2;
}

void dmap_print(int *p, const char *str)
{
    char ss[DEGREE_MAP_COUNT + 1];

    for(int i = 0; i < DEGREE_MAP_COUNT; i++) {
        ss[i] = (char)('0' + p[i]);
    }

    ss[DEGREE_MAP_COUNT] = 0;
    LOGM_TRACE(loggerSample, "dmap_print", "m=\"" << str << "\", dmap=[" << ss << "]");    
}

/* ----------------------------------- */

SamplePixels::SamplePixels(void) {
    patchNum = 0;
    imageName = "";
}

void SamplePixels::clear(void) {
    while(patchNum > 0) {
        remove();
    }
    patchNum = 0;
    // image will not be released for caching purposes
}

void SamplePixels::remove(void) {
    if (patchNum <= 0) {
        return;
    }
    patchNum--;
    
    fileNames[patchNum] = "";
    points[patchNum] = Point(0, 0);
    pixels[patchNum].release();
    ctvals[patchNum].release();
}

int SamplePixels::addSample(const string &fileName, Point p) {
    if (patchNum >= MAX_PATCH_NUM - 1) {
        return -1;    // error: max number of patches exceeded
    }
    
    if (imageName.length() > 0) {
        if (imageName != fileName) {
            image.release();
            imageName = fileName;
            image = imread(imageName, 1);
        }
    } else {
        imageName = fileName;
        image = imread(imageName, 1);        
    }
    
    if (!image.data) {
        return -2;    // error: file not found
    }
    
    if (!checkPoint(image, p)) {
        return -3;    // error: incorrect point coordinates
    }
    
    fileNames[patchNum] = fileName;
    points[patchNum] = p;
    
    pixels[patchNum].create(SAMPLE_HEIGHT * SAMPLE_WIDTH, 1, CV_8UC3);
    
    int j = 0;
    for (int y = 0; y < SAMPLE_HEIGHT; y++) {
        for (int x = 0; x < SAMPLE_WIDTH; x++) {
            Vec3b v = image.at<Vec3b>(p.y + y, p.x + x, 0);
            pixels[patchNum].at<Vec3b>(j++) = v;
        }
    }

    //ctvals[patchNum] = pixels[patchNum].clone();
    convert(pixels[patchNum], ctvals[patchNum]);    
    
    patchNum++;
}

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "sample.h"
#include "mtime.h"
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
        LOGM_ERROR(loggerSample, "addSample", "msg=\"Error: max number of patches exceeded!\", filen=\"" << fileName << "\"");
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
        LOGM_ERROR(loggerSample, "addSample", "msg=\"Error: file not found!\", filen=\"" << fileName << "\"");
        return -2;    // error: file not found
    }
    
    if (!checkPoint(image, p)) {
        LOGM_ERROR(loggerSample, "addSample", "msg=\"Error: incorrect point coordinates!\", filen=\"" << fileName << "\"");
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
    return 0;
}

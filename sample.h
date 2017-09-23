#ifndef __SAMPLE_H__
#define __SAMPLE_H__

#include <opencv2/opencv.hpp>

//using namespace cv;
//using namespace std;

const int MAX_PATCH_NUM = 64;

const int SAMPLE_WIDTH = 32;
const int SAMPLE_HEIGHT = 32;
const int SAMPLE_COUNT = SAMPLE_WIDTH * SAMPLE_HEIGHT;

void convert(const cv::Mat& src, cv::Mat& dst);
void convertInv(const cv::Mat& src, cv::Mat& dst);
void convertImage(const cv::Mat& src, cv::Mat& dst, int vv);

int checkPoint(const cv::Mat& src, cv::Point& p);

class SamplePixels {
public:    
    int  patchNum;
    std::string fileNames[MAX_PATCH_NUM];
    cv::Point points[MAX_PATCH_NUM];
    cv::Mat pixels[MAX_PATCH_NUM];
    cv::Mat ctvals[MAX_PATCH_NUM];    // pixels after conversion BGR -> HSV
    
private:
    std::string imageName;
    cv::Mat image;
    
public:    
    SamplePixels(void);
    
    void clear(void);
    void remove(void);
    int addSample(const std::string& fileName, cv::Point p);
};

#endif

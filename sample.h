#ifndef __SAMPLE_H__
#define __SAMPLE_H__

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

const int MAX_PATCH_NUM = 64;

const int SAMPLE_WIDTH = 32;
const int SAMPLE_HEIGHT = 32;
const int SAMPLE_COUNT = SAMPLE_WIDTH * SAMPLE_HEIGHT;

void convert(const Mat& src, Mat& dst);
void convertInv(const Mat& src, Mat& dst);
void convertImage(const Mat& src, Mat& dst, int vv);

int checkPoint(const Mat& src, Point& p);

double timeBegin(void);
double timeEnd(const string& str, double t);

const int DEGREE_MAP_COUNT = 181;  // mapa hodnot (1 = volno, 0 = prekazka) pre kazdy uhol 0..180 stupnov, 0=vpravo, 180=vlavo (counterclockwise)

void dmap_init(int *p);
void dmap_set(int *p, int value);
void dmap_fill(int *p, float angle1, float angle2, int value);
void dmap_finish(int *p);
void dmap_apply(int *p, int *p2);  // p = p <and> p2
void dmap_find(int *p, int minlen, int mid, int &idx1, int &idx2);
void dmap_print(int *p, const char *str);

class SamplePixels {
public:    
    int  patchNum;
    string fileNames[MAX_PATCH_NUM];
    Point points[MAX_PATCH_NUM];
    Mat pixels[MAX_PATCH_NUM];
    Mat ctvals[MAX_PATCH_NUM];    // pixels after conversion BGR -> HSV
    
private:
    string imageName;
    Mat image;
    
public:    
    SamplePixels(void);
    
    void clear(void);
    void remove(void);
    int addSample(const string& fileName, Point p);
};

#endif

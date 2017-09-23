#ifndef __VISION_H__
#define __VISION_H__

#include <opencv2/opencv.hpp>
#include "dmap.h"
#include "sample.h"

using namespace cv;
using namespace std;

const int GRAPH_GAP = 100;

const int CLUSTER_COUNT = 4;
const float CLUSTER_RADIUS_THRESHOLD = 0.8;    // aky pencertil povazujeme za postacujuci pre urcenie radiusu od stredu clustra
const float CLUSTER_RADIUS_MULT = 2.0;         // radius = nasobok * vzdialenost zodpovedajuca percentilu

const int ELINES_COUNT = 37;    // kazdych 5 stupnov (36*5) jedna ciara
const int EPOINTS_OFFSET = 8;   // prvych kolko bodov na ciare bodov preskocime
const int EPOINTS_COUNT = 40 - EPOINTS_OFFSET;   // kolko bodov je na kazdej ciare

const int ELINES_ANGLE_IDX   = 0;  // angle in degrees
const int ELINES_MAXDIST_IDX = 1;  // max distance how far obstacles could be seen (in centimetres)
const int ELINES_LIMIT_IDX   = 2;  // distance behind that obstacles are detected (in centimetres)

const float OBSTACLE_DIST_THRESHOLD = 100;  // vzdialenost za ktorou je koniec cesty povazovany za prekazku (alebo kraj obrazovky)

typedef Point3_<uchar> Point3u;

extern Point center;
extern Point center2;
extern int gui_change;

class VisionCore {

public:
    void calcKMeans(const SamplePixels& sample, Mat& centers, Mat& labels, int *radius);
    int  initEvalLT(Mat& elt, const Mat& centers1, const Mat& centers2, int *radius1, int *radius2);
    int  generateEPoints(Mat& epoints, Mat& epdist, Mat& elines);

    void calcMarkers(const Mat& image, Mat& markers, const Mat& centers1, const Mat& centers2, const int *radius1, const int *radius2, const Mat& elt);
    void calcMarkersIM(const Mat& markers, Mat& markersIM);

//  int  calcTWeight(Mat& image, const Mat &markers, int x1, int y1, int x2, int y2, int x3, int y3);
    void calcEPWeight(const Mat& epoints, const Mat& epdist, const Mat& markersIM, Mat& epweigth, Mat& elweigth);    
    void calcELimitAngle(const Mat& elines, const Mat& elweigth, DegreeMap& dmap);
};

class VisionGui: public VisionCore {

private:
    void drawBGR(Mat& img, int b, int g, int r, Point3u color) ;
    void drawRectangleBGR(Mat& img, int b, int g, int r, int wb, int wg, int wr, const Scalar& color);

public:
    void setCallback(const string& str);

    void drawGrid(Mat& img);
    void drawColors(const SamplePixels& sample, Mat& img);

    void drawClusters(Mat& img, const Mat& centers, const int *radius);
    void drawSample(Mat& image, const SamplePixels &sample, const Scalar& color, const string& sampleFN);
    void printSample(const string& name, const SamplePixels &sample);

    void drawMarkers(Mat& imageMark, const Mat& markers, const Mat& imageGray);
    
    void drawEPoints(Mat& image, const Mat& epoints);
    void drawEPWeight(Mat& image, const Mat& epoints, const Mat& epdist, const Mat& elines, const Mat& epweigth, const Mat& elweigth, const int& angle_min, const int& angle_max);
};

class Vision: public VisionGui {
public:
    // not changed during evaluation
    Mat clusterCenters1, clusterCenters2;
    int clusterRadius1[3*CLUSTER_COUNT];    //  [CLUSTER_COUNT][3]
    int clusterRadius2[3*CLUSTER_COUNT];    //  [CLUSTER_COUNT][3]
    Mat evalLT;
    
    Mat epdist;  
    Mat epoints;
    Mat elines;  // ANGLE = uhol v stupnoch (0..180), MAXDIST = max viditelna dlzka na ciare, LIMIT = hranica za ktorou je detekovana prekazka

public:
    // changed during evaluation -> moved to DataSet (needs to be saved for drawOutput - save_thread)
    //Mat markers;
    //Mat markersIM;

    /* int tweight[5]; */
    //Mat epweigth;
    //Mat elweigth;
    
    //int eangle_min;
    //int eangle_max;
    
public:
    int init(SamplePixels &sampleOnRoad, SamplePixels &sampleOffRoad);

    int eval(Mat &image, Mat& markers, Mat& markersIM, Mat& epweigth, Mat& elweigth, DegreeMap& dmap);
    
    void drawOutput(const Mat &image, Mat &out, const Mat& markers, const Mat& epweigth, const Mat& elweigth, const int& angle_min, const int& angle_max);
};

#endif


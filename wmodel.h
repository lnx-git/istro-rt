#ifndef __WMODEL_H__
#define __WMODEL_H__

#include <opencv2/opencv.hpp>
#include "dmap.h"

using namespace cv;
using namespace std;

const int WMGRID_HEIGHT = 2001;
const int WMGRID_WIDTH  = 2001;

const int WMGRID_CENTER_IDX = 1000;
const int WMGRID_CENTER_IDY = 1000;

const float WMGRID_DMAP_MAX_DIST = 3.0;    // update grid to maximum distance of 3 metres (radius)

const float WMGRID_CELL_DX = 0.1;
const float WMGRID_CELL_DY = 0.1;

const float WMGRID_X_MAX = (WMGRID_WIDTH - WMGRID_CENTER_IDX - 1) * WMGRID_CELL_DX - 2 * WMGRID_DMAP_MAX_DIST;
const float WMGRID_X_MIN = -WMGRID_X_MAX;
const float WMGRID_Y_MAX = (WMGRID_HEIGHT - WMGRID_CENTER_IDY - 1) * WMGRID_CELL_DY - 2 * WMGRID_DMAP_MAX_DIST;
const float WMGRID_Y_MIN = -WMGRID_Y_MAX;

const unsigned char WMGRID_LIDAR_MBIT  = 0x10;   // 1 = information about lidar known
const unsigned char WMGRID_LIDAR_VBIT  = 0x01;   // 1 = lidar detected an obsacle in this grid cell

const unsigned char WMGRID_VISION_MBIT = 0x20;   // 2 = information about vision known
const unsigned char WMGRID_VISION_VBIT = 0x02;   // 2 = vision detected an obsacle in this grid cell

class WMGrid {
public:
    float grid_x0;
    float grid_y0;
    unsigned char g[WMGRID_HEIGHT][WMGRID_WIDTH];
    
private:
    int recenter(float x0, float y0);

public:
    WMGrid();

    void init(void);
    void clear(void);
    int update(const DegreeMap& dmap, float x0, float y0, float alfa, unsigned char mb, unsigned char vb);
    int eval(float x0, float y0, float alfa, DegreeMap& dmap);

    unsigned char get(int gy, int gx);
    void set(int gy, int gx, unsigned char b);
};

class WorldModel {
public:
    WMGrid *pgrid;

    long image_number;
    float last_x0;
    float last_y0;
    float last_alfa;
    
public:
    WorldModel();
    ~WorldModel();

    void init(void);
    void updateGrid(const DegreeMap& dmap, float x0, float y0, float alfa, unsigned char mb, unsigned char vb, long imgnum);
    int  evalGrid(float x0, float y0, float alfa, DegreeMap& dmap);
    void drawGrid(Mat& img);
    void drawGridFull(Mat& img);

public:
    void saveImage(long image_number, const string& str, const Mat& img);
    void test(void);
};

#endif

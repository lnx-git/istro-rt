#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "vision.h"
#include "system.h"
#include "logger.h"

using namespace cv;
using namespace std;

const int EPSIZE = 81; /* velkost bloku v pixloch pre vyhodnocovanie majoritnej farby v okoli bodu - vzdy neparne */
const int EP_THRESHOLDM = 1.0 * 256;  /* 1.2 * 256 = majoritna hodnota musi mat viac nez 20% vacsie zastupenie ako vsetky ostatne */ 

Point center;
Point center2;
int gui_change = 4+8+16+32;

LOG_DEFINE(loggerVCore, "VisionCore");
LOG_DEFINE(loggerVGui, "VisionGui");
LOG_DEFINE(loggerVision, "Vision");

static void onMouse(int event, int x, int y, int flags, void*)
{
    if (event == CV_EVENT_LBUTTONDOWN) {
        center = Point(x, y);
        gui_change = 1;
    }
    if (event == CV_EVENT_RBUTTONDOWN) {
        center2 = Point(x, y);
        gui_change = 2;
    }
}

void VisionCore::calcKMeans(const SamplePixels& sample, Mat& centers, Mat& labels, int *radius) 
{
    Mat points;
    Mat hist1(CLUSTER_COUNT, 256, CV_32SC1);
    Mat hist2(CLUSTER_COUNT, 256, CV_32SC1);
    Mat hist3(CLUSTER_COUNT, 256, CV_32SC1);

    if (!centers.empty()) {
        centers.release();
    }
    if (!labels.empty()) {
        labels.release();
    }

    points.create(sample.patchNum * SAMPLE_WIDTH * SAMPLE_HEIGHT, 1, CV_32FC3);
    
    int j = 0;
    for (int p = 0; p < sample.patchNum; p++) {
        for (int k = 0; k < SAMPLE_WIDTH * SAMPLE_HEIGHT; k++) {
//          Point3u pt = sample.pixels[p].at<Point3u>(k);
            Point3u ct = sample.ctvals[p].at<Point3u>(k);
            points.at<Point3f>(j++) = Point3f(ct.x, ct.y, ct.z);
        }
    }
    
    double t = timeBegin();

    kmeans(points, CLUSTER_COUNT, labels,
        TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
        3, KMEANS_PP_CENTERS, centers);

    timeEnd("VisionCore::calcKMeans.kmeans", t);

    int c1[12];
    
    for (int i = 0; i < CLUSTER_COUNT; i++) {
        Point3u p1 = centers.at<Point3f>(i);
        
        c1[3*i + 0] = cvRound(p1.x);
        c1[3*i + 1] = cvRound(p1.y);
        c1[3*i + 2] = cvRound(p1.z);
        
        for (j = 0; j < 256; j++) {
            hist1.at<int>(i, j) = 0; 
            hist2.at<int>(i, j) = 0;
            hist3.at<int>(i, j) = 0;
        }
    }

    int dd[12];
    
    for (int p = 0; p < sample.patchNum; p++) {
        for (int k = 0; k < SAMPLE_WIDTH * SAMPLE_HEIGHT; k++) {
//          Vec3b v = sample.pixels[p].at<Vec3b>(k);
            Vec3b v = sample.ctvals[p].at<Vec3b>(k);          

            /* fixme: vzdialenost pre hue treba pocitat na kruhu 0 az 180 */
            dd[0] = abs(((int)v.val[0])-c1[0]);
            dd[1] = abs(((int)v.val[1])-c1[1]);
            dd[2] = abs(((int)v.val[2])-c1[2]);
            dd[3] = abs(((int)v.val[0])-c1[3]);
            dd[4] = abs(((int)v.val[1])-c1[4]);
            dd[5] = abs(((int)v.val[2])-c1[5]);
            dd[6] = abs(((int)v.val[0])-c1[6]);
            dd[7] = abs(((int)v.val[1])-c1[7]);
            dd[8] = abs(((int)v.val[2])-c1[8]);
            dd[9] = abs(((int)v.val[0])-c1[9]);
            dd[10] = abs(((int)v.val[1])-c1[10]);
            dd[11] = abs(((int)v.val[2])-c1[11]);
            
            int j = 0;
            int min1 = 999;
            int min1j = -1;
            int min1i = -1;
            for (int i = 0; i < 12; i += 3, j++) {
                // najvacsia vzdialenost r/g/b od daneho clustra
                int max = dd[i];
                if (max < dd[i+1]) max = dd[i+1];
                if (max < dd[i+2]) max = dd[i+2];
                // hladame cluster, ktory ma tuto vzdialenost minimalnu
                if (min1 > max) {
                    min1 = max;
                    min1j = j;
                    min1i = i;
                }
            }
            hist1.at<int>(min1j, dd[min1i])++; 
            hist2.at<int>(min1j, dd[min1i + 1])++; 
            hist3.at<int>(min1j, dd[min1i + 2])++; 
        }
    }    

    for (j = 0; j < 32; j += 4) {
        for (int i = 0; i < CLUSTER_COUNT; i++) {
            int sum1 = 0;
            int sum2 = 0;
            int sum3 = 0;
            for (int k = 0; k < 4; k++) {
                sum1 += hist1.at<int>(i, j + k);
                sum2 += hist2.at<int>(i, j + k);
                sum3 += hist3.at<int>(i, j + k);                
            }
            //printf("hist[%d][%d] = (%d,%d,%d),  ", i, j, sum1, sum2, sum3);
        }
        //printf("\n");
    }
    j = 32; 
    {
        for (int i = 0; i < CLUSTER_COUNT; i++) {
            int sum1 = 0;
            int sum2 = 0;
            int sum3 = 0;
            for (int k = 0; k < 256 - j; k++) {
                sum1 += hist1.at<int>(i, j + k);
                sum2 += hist2.at<int>(i, j + k);
                sum3 += hist3.at<int>(i, j + k);                
            }
            //printf("hist[%d][%d+] = (%d,%d,%d),  ", i, j, sum1, sum2, sum3);
        }
        //printf("\n");
    }

    int ss[CLUSTER_COUNT][3];
    for (int i = 0; i < CLUSTER_COUNT; i++) {
        int sum1 = 0;
        int sum2 = 0;
        int sum3 = 0;
        for (j = 0; j < 256; j ++) {
            sum1 += hist1.at<int>(i, j);
            sum2 += hist2.at<int>(i, j);
            sum3 += hist3.at<int>(i, j);                
        }
        ss[i][0] = sum1;
        ss[i][1] = sum2;
        ss[i][2] = sum3;
    }
    LOGM_DEBUG(loggerVCore, "calcKMeans", "histSum = [" 
        << "[" << ss[0][0] << "," << ss[0][1] << "," << ss[0][2] << "], " 
        << "[" << ss[1][0] << "," << ss[1][1] << "," << ss[1][2] << "], " 
        << "[" << ss[2][0] << "," << ss[2][1] << "," << ss[2][2] << "], " 
        << "[" << ss[3][0] << "," << ss[3][1] << "," << ss[3][2] << "]]");

    for (int i = 0; i < CLUSTER_COUNT; i++) {
        int sum1 = 0;
        int thr1 = CLUSTER_RADIUS_THRESHOLD * ss[i][0];
        for (j = 0; j < 256; j ++) {
            sum1 += hist1.at<int>(i, j);
            if (sum1 >= thr1) break;
        }
//      radius[i] =  20;        
        radius[3*i + 0] =  CLUSTER_RADIUS_MULT * j;
    }
    for (int i = 0; i < CLUSTER_COUNT; i++) {
        int sum2 = 0;
        int thr2 = CLUSTER_RADIUS_THRESHOLD * ss[i][1];
        for (j = 0; j < 256; j ++) {
            sum2 += hist2.at<int>(i, j);
            if (sum2 >= thr2) break;
        }
//      radius[i] =  20;        
        radius[3*i + 1] =  CLUSTER_RADIUS_MULT * j;
    }
    for (int i = 0; i < CLUSTER_COUNT; i++) {
        int sum3 = 0;
        int thr3 = CLUSTER_RADIUS_THRESHOLD * ss[i][2];
        for (j = 0; j < 256; j ++) {
            sum3 += hist3.at<int>(i, j);
            if (sum3 >= thr3) break;
        }
//      radius[i] =  20;        
        radius[3*i + 2] =  CLUSTER_RADIUS_MULT * j;
    }
    LOGM_DEBUG(loggerVCore, "calcKMeans", "histRadius = [" 
        << "[" << radius[3*0 + 0] << "," << radius[3*0 + 1] << "," << radius[3*0 + 2] << "], " 
        << "[" << radius[3*1 + 0] << "," << radius[3*1 + 1] << "," << radius[3*1 + 2] << "], " 
        << "[" << radius[3*2 + 0] << "," << radius[3*2 + 1] << "," << radius[3*2 + 2] << "], " 
        << "[" << radius[3*3 + 0] << "," << radius[3*3 + 1] << "," << radius[3*3 + 2] << "]]");
}

void VisionCore::calcMarkers(const Mat& image, Mat& markers, const Mat& centers1, const Mat& centers2, const int *radius1, const int *radius2, const Mat& elt)
{
    if (markers.empty()) {
        markers.create(image.size(), CV_8UC1);
    }
    
    int c1[12], c2[12];
    int dd[12], x2, x3;
    
    for (int i = 0; i < CLUSTER_COUNT; i++) {
        Point3u p1 = centers1.at<Point3f>(i);
        Point3u p2 = centers2.at<Point3f>(i);
        
        c1[3*i + 0] = cvRound(p1.x);
        c1[3*i + 1] = cvRound(p1.y);
        c1[3*i + 2] = cvRound(p1.z);
        
        c2[3*i + 0] = cvRound(p2.x);
        c2[3*i + 1] = cvRound(p2.y);
        c2[3*i + 2] = cvRound(p2.z);
    }
    
    Mat ctvals;

    double t = timeBegin();
    convert(image, ctvals);
    timeEnd("VisionCore::calcMarkers.cvtColor(BGR2HSV)", t);

    int elt_empty = elt.empty();
    int ctvals_rows = ctvals.rows;
    int ctvals_cols = ctvals.cols;

    t = timeBegin();
    
    uchar *p_ctvals;
    uchar *p_markers;
    for(int y = 0; y < ctvals_rows; y++) {
        /* tutorials/how_to_scan_images.html */
        p_ctvals = ctvals.ptr<uchar>(y);
        p_markers = markers.ptr<uchar>(y);
        for(int x = 0; x < ctvals_cols; x++, p_ctvals += 3, p_markers++) {
            uchar rr = 255;

            if (!elt_empty) {
                rr = elt.at<uchar>((p_ctvals[0] >> 2) * (64 * 64) + (p_ctvals[1] >> 2) * 64 + (p_ctvals[2] >> 2));
            }
            
            if (rr > 4) {
                dd[0] = abs(((int)p_ctvals[0])-c1[0]);
                if (dd[0] > 90) dd[0] = 180 - dd[0];  // Hue sa opakuje po 180
                dd[1] = abs(((int)p_ctvals[1])-c1[1]);
                dd[2] = abs(((int)p_ctvals[2])-c1[2]);
                dd[3] = abs(((int)p_ctvals[0])-c1[3]);
                if (dd[3] > 90) dd[3] = 180 - dd[3];  // Hue sa opakuje po 180
                dd[4] = abs(((int)p_ctvals[1])-c1[4]);
                dd[5] = abs(((int)p_ctvals[2])-c1[5]);
                dd[6] = abs(((int)p_ctvals[0])-c1[6]);
                if (dd[6] > 90) dd[6] = 180 - dd[6];  // Hue sa opakuje po 180
                dd[7] = abs(((int)p_ctvals[1])-c1[7]);
                dd[8] = abs(((int)p_ctvals[2])-c1[8]);
                dd[9] = abs(((int)p_ctvals[0])-c1[9]);
                if (dd[9] > 90) dd[9] = 180 - dd[9];  // Hue sa opakuje po 180
                dd[10] = abs(((int)p_ctvals[1])-c1[10]);
                dd[11] = abs(((int)p_ctvals[2])-c1[11]);
                
                int min1 = 999;
                for (int i = 0, j = 0; i < 12; i += 3, j++) {
                    // hladame cluster, ktory ma kazdu zlozku v ramci radiusu a najvacsiu z tychto vzdialenosti minimalnu
                    if ((dd[i] <= radius1[3*j+0]) && (dd[i+1] <= radius1[3*j+1]) && (dd[i+2] <= radius1[3*j+2])) {
                        // najvacsia vzdialenost r/g/b od daneho clustra
                        int max = dd[i];
                        if (max < dd[i+1]) max = dd[i+1];
                        if (max < dd[i+2]) max = dd[i+2];
                        if (min1 > max) min1 = max;
                    }
                }
                
                dd[0] = abs(((int)p_ctvals[0])-c2[0]);
                if (dd[0] > 90) dd[0] = 180 - dd[0];  // Hue sa opakuje po 180
                dd[1] = abs(((int)p_ctvals[1])-c2[1]);
                dd[2] = abs(((int)p_ctvals[2])-c2[2]);
                dd[3] = abs(((int)p_ctvals[0])-c2[3]);
                if (dd[3] > 90) dd[3] = 180 - dd[3];  // Hue sa opakuje po 180
                dd[4] = abs(((int)p_ctvals[1])-c2[4]);
                dd[5] = abs(((int)p_ctvals[2])-c2[5]);
                dd[6] = abs(((int)p_ctvals[0])-c2[6]);
                if (dd[6] > 90) dd[6] = 180 - dd[6];  // Hue sa opakuje po 180
                dd[7] = abs(((int)p_ctvals[1])-c2[7]);
                dd[8] = abs(((int)p_ctvals[2])-c2[8]);
                dd[9] = abs(((int)p_ctvals[0])-c2[9]);
                if (dd[9] > 90) dd[9] = 180 - dd[9];  // Hue sa opakuje po 180
                dd[10] = abs(((int)p_ctvals[1])-c2[10]);
                dd[11] = abs(((int)p_ctvals[2])-c2[11]);
                
                int min2 = 999;
                for (int i = 0, j = 0; i < 12; i += 3, j++) {
                    // hladame cluster, ktory ma kazdu zlozku v ramci radiusu a najvacsiu z tychto vzdialenosti minimalnu
                    if ((dd[i] <= radius2[3*j+0]) && (dd[i+1] <= radius2[3*j+1]) && (dd[i+2] <= radius2[3*j+2])) {
                        // najvacsia vzdialenost r/g/b od daneho clustra
                        int max = dd[i];
                        if (max < dd[i+1]) max = dd[i+1];
                        if (max < dd[i+2]) max = dd[i+2];
                        if (min2 > max) min2 = max;
                    }
                }
    
                if ((min1 >= 256) && (min2 >= 256)) {
                    rr = 0;
                } else 
                if ((min1 < 256) && (min2 < 256)) {
                    rr = 4;
                } else {
                    rr = (min1 < min2) ? 1 : 2;
                }
            }
            
            *p_markers = rr;    // markers.at<uchar>(y, x) = rr;
        }
    }
    timeEnd("VisionCore::calcMarkers", t);     // calcMarkers(yx%3) = 2ms az 4ms

    // printf("calcMarkers.hit/miss = %d/%d\n", xhit, xmiss);
}

/* calculate Integral Matrix for markers */
/* see "integral image" - https://en.wikipedia.org/wiki/Summed_area_table */
void VisionCore::calcMarkersIM(const Mat& markers, Mat& markersIM)
{
    int rows = markers.rows;
    int cols = markers.cols;

    double t = timeBegin();
    
    // for each marker value (0, 1, 2, 4) calculate one integral image
    if (markersIM.empty()) {
        markersIM.create(rows, cols, CV_32SC4);
    }

    const uchar *p_markers = markers.ptr<uchar>(0);
    int         *p_markersIM = markersIM.ptr<int>(0);
    int         *p_markersIM_prev;
    
    // first row
    int cnt0 = 0;
    int cnt1 = 0;
    int cnt2 = 0;
    int cnt4 = 0;
    for(int x = 0; x < cols; x++) {
        uchar u = *(p_markers++); 
        
        if (u == 0) {
            cnt0++;
        } else
        if (u == 1) {
            cnt1++;
        } else
        if (u == 2) {
            cnt2++;
        } else {
            cnt4++;
        }

        p_markersIM[0] = cnt0;
        p_markersIM[1] = cnt1;
        p_markersIM[2] = cnt2;
        p_markersIM[3] = cnt4;
        p_markersIM += 4;
    }

    // next rows
    for(int y = 1; y < rows; y++) {
        p_markers = markers.ptr<uchar>(y);
        p_markersIM = markersIM.ptr<int>(y);
        p_markersIM_prev = markersIM.ptr<int>(y-1);  // previous row
        
        cnt0 = cnt1 = cnt2 = cnt4 = 0;
        for(int x = 0; x < cols; x++) {
            uchar u = *(p_markers++); 
            
            if (u == 0) {
                cnt0++;
            } else
            if (u == 1) {
                cnt1++;
            } else
            if (u == 2) {
                cnt2++;
            } else {
                cnt4++;
            }
        
            p_markersIM[0] = cnt0 + p_markersIM_prev[0];
            p_markersIM[1] = cnt1 + p_markersIM_prev[1];
            p_markersIM[2] = cnt2 + p_markersIM_prev[2];
            p_markersIM[3] = cnt4 + p_markersIM_prev[3];
            p_markersIM += 4;
            p_markersIM_prev += 4;
        }
    }

    timeEnd("VisionCore::calcMarkersIM", t);
}

/*
int VisionCore::calcTWeight(Mat& image, const Mat &markers, int x1, int y1, int x2, int y2, int x3, int y3) 
{
    if (!image.empty()) {
        line(image, Point(x1, y1), Point(x2, y2), Scalar( 255, 255, 255), 1, 8, 0);
        line(image, Point(x1, y1), Point(x3, y3), Scalar( 255, 255, 255), 1, 8, 0);
    }
    
//  double t = timeBegin();
    
    int cnt = 0;
    int total = 0;
    
    float k1 = ((float)(x2 - x1)) / (y2 - y1);
    float k2 = ((float)(x3 - x1)) / (y2 - y1);
    for(int y = y1; y <= y2; y++) {
        int xx1 = x1 + (y - y1) * k1;
        int xx2 = x1 + (y - y1) * k2;
        //line(image, Point(xx1, y), Point(xx2, y), Scalar( 255, 255, 255), 1, 8, 0);
        for(int x = xx1; x <= xx2; x++) {
            total++;
            if (markers.at<uchar>(y, x) == 1) cnt++;
        }    
    }
    
//  timeEnd("VisionCore::calcTweight", t);
    
    if (!image.empty()) {
        int y = y2 - ((float)(y2 - y1)) * cnt / total;
        int xx = (x2 + x3) / 2;
        int x = xx - ((float)(xx - x1)) * cnt / total;
        line(image, Point(x, y), Point(x, y2), Scalar( 255, 255, 255), 5, 8, 0);
    }
    
    return 1000 * (((float)cnt) / total);
}
*/

/* calculate event points and event lines weigths */
/* epweight obsahuje pre kazdy bod jednu z hodnot 1/2/0/4/255, podla toho, co prevazuje v hodnotach markers (255 = ziadna hodnota neprekrocila threshold) */
/* elweight pre kazdu ciaru obsahuje dlzku, ktora zodpoveda poslednemu pointu s hodnotou 1 (onroad) */
void VisionCore::calcEPWeight(const Mat& epoints, const Mat& epdist, const Mat& markersIM, Mat& epweigth, Mat& elweigth)
{
    int rows = markersIM.rows;
    int cols = markersIM.cols;
    
    double t = timeBegin();
    
    epweigth.create(epoints.size(), CV_8UC1);
    elweigth.create(ELINES_COUNT, 1, CV_32FC1);
    
    for(int i = 0; i < ELINES_COUNT; i++) {
        int first2 = -1;    // prvy index pointu s hodnotou inou ako 1
        for(int j = 0; j < EPOINTS_COUNT; j++) {
            Point p = epoints.at<Point>(i, j);
            
            if ((p.x < 0) && (p.y < 0)) {
                // point is outside image
                epweigth.at<uchar>(i, j) = 255;
                if (first2 < 0) {
                    first2 = j;
                }
                continue;
            }
            
            int x1 = p.x - (EPSIZE - 1)/2;
            int y1 = p.y - (EPSIZE - 1)/2;
            int x2 = p.x + (EPSIZE - 1)/2;
            int y2 = p.y + (EPSIZE - 1)/2;
            
            if (x1 < 0) {
                x2 += -x1;
                x1 = 0;
            }
            if (y1 < 0) {
                y2 += -y1;
                y1 = 0;
            }
            if (x2 >= cols) {
                x1 -= (x2 - cols + 1);
                x2 = cols - 1;
            }
            if (y2 >= rows) {
                y1 -= (y2 - rows + 1);
                y2 = rows - 1;
            }
            
            Vec4i vA = markersIM.at<Vec4i>(y1, x1);  // upper left
            Vec4i vB = markersIM.at<Vec4i>(y1, x2);  // upper right
            Vec4i vC = markersIM.at<Vec4i>(y2, x1);  // lower left
            Vec4i vD = markersIM.at<Vec4i>(y2, x2);  // lower right
            
            int cnt0 = vD[0] + vA[0] - vB[0] - vC[0];
            int cnt1 = vD[1] + vA[1] - vB[1] - vC[1];
            int cnt2 = vD[2] + vA[2] - vB[2] - vC[2];
            int cnt4 = vD[3] + vA[3] - vB[3] - vC[3];
            
            int cnt0t = cnt0 * EP_THRESHOLDM / 256;
            int cnt1t = cnt1 * EP_THRESHOLDM / 256;
            int cnt2t = cnt2 * EP_THRESHOLDM / 256;
            int cnt4t = cnt4 * EP_THRESHOLDM / 256;
            
            uchar u;
            if ((cnt1 >= cnt2t) && (cnt1 >= cnt0t) && (cnt1 >= cnt4t)) {
                u = 1;
            } else
            if ((cnt2 >= cnt1t) && (cnt2 >= cnt0t) && (cnt2 >= cnt4t)) {
                u = 2;
            } else
            if ((cnt4 >= cnt1t) && (cnt4 >= cnt2t) && (cnt4 >= cnt0t)) {
                u = 4;
            } else   
            if ((cnt0 >= cnt1t) && (cnt0 >= cnt2t) && (cnt0 >= cnt4t)) {
                u = 0;
            } else {
                u = 255;
            }
            epweigth.at<uchar>(i, j) = u;
            
            if ((first2 < 0) && (u != 1)) {
                //printf("calcEPWeight[%d, %d]: x1=%d, y1=%d, x2=%d, y2=%d; cnt0=%d, cnt1=%d, cnt2=%d, cnt4=%d; u=%d, first2=%d\n", i, j, x1, y1, x2, y2, cnt0, cnt1, cnt2, cnt4, (int)u, first2);
                first2 = j;
            }
        }
        if (first2 == 0) {
            // uz prvy point ma inu hodnotu ako 1
            elweigth.at<float>(i) = 0.0;
        } else 
        if (first2 < 0) {
            // vsetky pointy maju hodnotu 1
            elweigth.at<float>(i) = epdist.at<float>(EPOINTS_COUNT - 1);
        } else {
            // point s indexom "first2" je prvy point s hodnotou inou ako 1, takze vzdialenost je totozna so vzdialenostou k predchadzajucemu bodu
            elweigth.at<float>(i) = epdist.at<float>(first2 - 1);
        }
        //printf("calcEPWeight[%d]: first2=%d, elweigth=%d\n", i, first2, (int)round(elweigth.at<float>(i)));
    }
    
    timeEnd("VisionCore::calcEPWeight", t);
    
    /*int i = 0;
    for(; i < ELINES_COUNT - 4; i += 4) {
        printf("calcEPWeight: elweigth[%2d]=%3d, elweigth[%2d]=%3d, elweigth[%2d]=%3d, elweigth[%2d]=%3d\n", i, (int)round(elweigth.at<float>(i)),
            i+1, (int)round(elweigth.at<float>(i+1)), i+2, (int)round(elweigth.at<float>(i+2)), i+3, (int)round(elweigth.at<float>(i+3)));
    }
    for(; i < ELINES_COUNT; i++) {
        printf("calcEPWeight: elweigth[%2d]=%3d\n", i, (int)round(elweigth.at<float>(i)));
    }*/    
}

void VisionCore::calcELimitAngle(const Mat& elines, const Mat& elweigth, int *dmap)
{    
    dmap_init(dmap);
    float dd = 1;
    for(int i = 0; i < ELINES_COUNT; i++) {
        float angle1 = elines.at<Vec3f>(i)[ELINES_ANGLE_IDX];
        int val = (elweigth.at<float>(i) + 0.001) >= elines.at<Vec3f>(i)[ELINES_LIMIT_IDX];
        // for the last one we use old value of dd
        if (i + 1 < ELINES_COUNT) {
            float angle2 = elines.at<Vec3f>(i+1)[ELINES_ANGLE_IDX];
            dd = (angle2 - angle1) / 2;    
        }
        dmap_fill(dmap, angle1 - dd, angle1 + dd, val);
    }
    dmap_finish(dmap);
    
/*    
    const int DMAP_MIN_INTERVAL_LENGTH = 20;
    int angle_min = -1;
    int angle_max = -1;
    int mid = DEGREE_MAP_COUNT / 2;
    dmap_find(dmap, DMAP_MIN_INTERVAL_LENGTH, mid, angle_min, angle_max);
    printf("VisionCore::calcELimitAngle(): min=%d, max=%d\n", angle_min, angle_max);
*/    
}

int VisionCore::initEvalLT(Mat& elt, const Mat& centers1, const Mat& centers2, int *radius1, int *radius2) 
{
    Mat map_hsv(45 * 64 * 64, 1, CV_8UC3);
    Mat map_rgb(map_hsv.size(), CV_8UC3);
    
    double t = timeBegin();
    
    int j = 0;
    for(int h = 0; h < 45; h++) {
        for(int s = 0; s < 64; s++) {
            for(int v = 0; v < 64; v++) {
                map_hsv.at<Vec3b>(j++) = Vec3b(h * 4, s * 4, v * 4);
            }
        }
    }
    
    convertInv(map_hsv, map_rgb);
    
    Mat elt_empty;
    elt.create(map_hsv.size(), CV_8UC1);
    calcMarkers(map_rgb, elt, centers1, centers2, radius1, radius2, elt_empty);
    
    int cnt0 = 0;
    int cnt1 = 0;
    int cnt2 = 0;
    int cnt4 = 0;
    int cnt5 = 0;
    for(int h = 0; h < 44; h++) {
        for(int s = 0; s < 63; s++) {
            for(int v = 0; v < 63; v++) {

                uchar u  = elt.at<uchar>(h * (64 * 64) + s * 64 + v);
                uchar u2 = elt.at<uchar>(h * (64 * 64) + s * 64 + (v + 1));
                uchar u3 = elt.at<uchar>(h * (64 * 64) + (s + 1) * 64 + v);
                uchar u4 = elt.at<uchar>(h * (64 * 64) + (s + 1) * 64 + (v + 1));
                uchar u5 = elt.at<uchar>((h + 1) * (64 * 64) + s * 64 + v);
                uchar u6 = elt.at<uchar>((h + 1) * (64 * 64) + s * 64 + (v + 1));
                uchar u7 = elt.at<uchar>((h + 1) * (64 * 64) + (s + 1) * 64 + v);
                uchar u8 = elt.at<uchar>((h + 1) * (64 * 64) + (s + 1) * 64 + (v + 1));
        
                if ((u != u2) || (u != u3) || (u != u4) || (u != u5) || (u != u6) || (u != u7) || (u != u8)) {
                    u = elt.at<uchar>(h * (64 * 64) + s * 64 + v) = 5;
                }
                
                if (u == 0) {
                    cnt0++;
                } else
                if (u == 1) {
                    cnt1++;
                } else
                if (u == 2) {
                    cnt2++;
                } else
                if (u == 4) {
                    cnt4++;
                } else
                if (u == 5) {
                    cnt5++;
                }
            }
        }
    }

    LOGM_DEBUG(loggerVCore, "initEvalLT", "cnt = [" << cnt0 << ", " << cnt1 << ", " << cnt2 << ", " << cnt4 << ", " << cnt5 << "]");
    
    timeEnd("VisionCore::initEvalLT", t);
    
    return 0;
}

const double PERSPECTIVE_Y0 = 433;
const double PERSPECTIVE_K1 = -23603.31;
const double PERSPECTIVE_D1 = 54.55;
const double PERSPECTIVE_K2 = 626.87;
const double PERSPECTIVE_D2 = 67.16;

const double PERSPECTIVE_Y1 = 390;  // kde sa nachadza 5 metrov - kolko pixlov od spodnej strany obrazovky

int VisionCore::generateEPoints(Mat& epoints, Mat& epdist, Mat& elines) 
{
    epdist.create(EPOINTS_COUNT, 1, CV_32FC1);
    epoints.create(ELINES_COUNT, EPOINTS_COUNT, CV_32SC2);  // x, y
    elines.create(ELINES_COUNT, 1, CV_32FC3);
    
    // epdist - v akej vzdialenosti od robota (cm) lezi ktory bod
    for(int j = 0; j < EPOINTS_COUNT; j++) {
        float y = (j + EPOINTS_OFFSET) * (PERSPECTIVE_Y1 / (EPOINTS_COUNT + EPOINTS_OFFSET));  // kolko pixlov od podnej strany obrazovky chceme dalsiu elipsu            
        float r = (PERSPECTIVE_K1 + PERSPECTIVE_Y0 * PERSPECTIVE_D1 - y * PERSPECTIVE_D1) / (y - PERSPECTIVE_Y0);
        /* float r = j * 25 / 2;   // radius: 0..5 metrov, kazdych 12,5cm   */
        
        epdist.at<float>(j) = r;
        //printf("epdist[%d]=%d\n", j, (int)round(r));
    }

    for(int i = 0; i < ELINES_COUNT; i++) {
        float fi = i * M_PI / (ELINES_COUNT - 1);    // uhol: 0 az PI (180)
        float maxd = -1;
        float maxd_prev = -1;
        bool onscreen = true;
        bool onscreen_prev = true;        
        for(int j = 0; j < EPOINTS_COUNT; j++) {
            float d = epdist.at<float>(j) * sin(fi);
            float x = epdist.at<float>(j) * cos(fi);
            
            int xp = 320 + round(x * PERSPECTIVE_K2 / (PERSPECTIVE_D2 + d)); 
            int yp = 479 - round(PERSPECTIVE_K1 / (PERSPECTIVE_D1 + d) + PERSPECTIVE_Y0); 
            
            //if ((i==0) && (j<15)) printf("generateEPoints: %d, %d\n", xp, yp);            

            onscreen_prev = onscreen;
            onscreen = (xp >= 0) && (xp < 640) && (yp >= 0) && (yp < 480);
            if (onscreen_prev) {
                epoints.at<Point>(i, j) = Point(xp, yp);
                maxd_prev = maxd;
                maxd = epdist.at<float>(j);
            }  else {
                epoints.at<Point>(i, j) = Point(-1, -1);
            }
        }
        elines.at<Vec3f>(i)[ELINES_ANGLE_IDX] = i * 180 / (ELINES_COUNT - 1);
        elines.at<Vec3f>(i)[ELINES_MAXDIST_IDX] = maxd;
        // maxd_prev is the distance to the point that is before the last one (=> last on the screen)
        if (maxd_prev > OBSTACLE_DIST_THRESHOLD) {
            elines.at<Vec3f>(i)[ELINES_LIMIT_IDX] = OBSTACLE_DIST_THRESHOLD;
        } else {
            elines.at<Vec3f>(i)[ELINES_LIMIT_IDX] = maxd_prev;
        }
//      if (i <= ELINES_COUNT / 2) {
//          printf("elines[%d]: angle=%d, maxd=%d, limit=%d\n", i, (int)round(elines.at<Vec3f>(i)[ELINES_ANGLE_IDX]),
//              (int)round(elines.at<Vec3f>(i)[ELINES_MAXDIST_IDX]), (int)round(elines.at<Vec3f>(i)[ELINES_LIMIT_IDX]));
//      }
    }
    //char key = (char)waitKey(0);    
    
    return 0;
}

/* ---------------------- */

void VisionGui::setCallback(const string& str)
{
    setMouseCallback( "imageSample", onMouse, 0 );
}

static inline int G1X(int x) { return GRAPH_GAP / 2 + x; }
static inline int G1Y(int y) { return GRAPH_GAP / 2 + 256 - y; }
static inline int G2X(int x) { return 256 + 3 * GRAPH_GAP / 2 + x; }
static inline int G2Y(int y) { return GRAPH_GAP / 2 + 256 - y; }
static inline int G3X(int x) { return 512 + 5 * GRAPH_GAP / 2 + x; }
// najvyssia hodnota Saturacie bude hore, aby najjasnejsie farby boli hore rovnako ako pri grafe Value
static inline int G3Y(int y) { return GRAPH_GAP / 2 + 1 + y; }    // { return GRAPH_GAP / 2 + 256 - y; }

void VisionGui::drawGrid(Mat& img) 
{
    Mat axis(19, 256, CV_8UC3);
    Mat ctax(19, 256, CV_8UC3);
    
    // pre HSV nemaju vyznam hodnoty (H, S, 0) -> nahradime (H, S, 255);  pre BGR je to OK
    for (int i = 0; i < 256; i++) {
        axis.at<Point3u>(0, i) = Point3_<uchar>(i, 0, 0);
        axis.at<Point3u>(1, i) = Point3_<uchar>(0, i, 0);
        axis.at<Point3u>(2, i) = Point3_<uchar>(0, 0, i);
        
        axis.at<Point3u>(3, i) = Point3_<uchar>(i, i, 0);
        axis.at<Point3u>(4, i) = Point3_<uchar>(i, 0, i);
        axis.at<Point3u>(5, i) = Point3_<uchar>(0, i, i);
        
        axis.at<Point3u>(6, i) = Point3_<uchar>(255-i, i, 255);    // Point3_<uchar>(255-i, i, 0);
        axis.at<Point3u>(7, i) = Point3_<uchar>(i, 0, 255-i);
        axis.at<Point3u>(8, i) = Point3_<uchar>(0, i, 255-i);
        
        axis.at<Point3u>(9, i) = Point3_<uchar>(255, i, 255);    // Point3_<uchar>(255, i, 0);
        axis.at<Point3u>(10, i) = Point3_<uchar>(i, 0, 255);
        axis.at<Point3u>(11, i) = Point3_<uchar>(0, i, 255);
        
        axis.at<Point3u>(12, i) = Point3_<uchar>(i, 255, 255);    // Point3_<uchar>(i, 255, 0);
        axis.at<Point3u>(13, i) = Point3_<uchar>(255, 0, i);
        axis.at<Point3u>(14, i) = Point3_<uchar>(0, 255, i);

        axis.at<Point3u>(15, i) = Point3_<uchar>(0, i, 96);
        axis.at<Point3u>(16, i) = Point3_<uchar>(i, 255, 96);
        axis.at<Point3u>(17, i) = Point3_<uchar>(i, 0, 255);    // Point3_<uchar>(i, 128, 255);
        axis.at<Point3u>(18, i) = Point3_<uchar>(i, i, 255);
    }
    
    convertInv(axis, ctax);
    
    // image2.at(y, x) = BGR    // pre BGR pouzijeme axis, pre HSV pouzijeme ctax
    for (int i = 0; i < 256; i++) {
        img.at<Point3u>(G1Y(-1), G1X(i)) = ctax.at<Point3u>(15, i);    // axis.at<Point3u>(1, i);
        img.at<Point3u>(G1Y(i), G1X(-1)) = ctax.at<Point3u>(2, i);
        img.at<Point3u>(G1Y(i), G1X(i)) = ctax.at<Point3u>(5, i);
        img.at<Point3u>(G1Y(255 - i), G1X(i)) = ctax.at<Point3u>(8, i);
        img.at<Point3u>(G1Y(256), G1X(i)) = ctax.at<Point3u>(11, i);
        img.at<Point3u>(G1Y(i), G1X(256)) = ctax.at<Point3u>(14, i);
        
        img.at<Point3u>(G2Y(-1), G2X(i)) = ctax.at<Point3u>(16, i);    // axis.at<Point3u>(0, i);
        img.at<Point3u>(G2Y(i), G2X(-1)) = ctax.at<Point3u>(2, i);
        img.at<Point3u>(G2Y(i), G2X(i)) = ctax.at<Point3u>(4, i);
        img.at<Point3u>(G2Y(255 - i), G2X(i)) = ctax.at<Point3u>(7, i);
        img.at<Point3u>(G2Y(256), G2X(i)) = ctax.at<Point3u>(10, i);
        img.at<Point3u>(G2Y(i), G2X(256)) = ctax.at<Point3u>(13, i);
        
        img.at<Point3u>(G3Y(i), G3X(-1)) = ctax.at<Point3u>(11, i);    // axis.at<Point3u>(1, i);
        img.at<Point3u>(G3Y(-1), G3X(i)) = ctax.at<Point3u>(17, i);    // axis.at<Point3u>(0, i);
        img.at<Point3u>(G3Y(i), G3X(i)) = ctax.at<Point3u>(18, i);    // ctax.at<Point3u>(3, i);
        img.at<Point3u>(G3Y(i), G3X(255 - i)) = ctax.at<Point3u>(6, i);
        img.at<Point3u>(G3Y(i), G3X(256)) = ctax.at<Point3u>(9, i);
        img.at<Point3u>(G3Y(256), G3X(i)) = ctax.at<Point3u>(12, i);
    }
}

void VisionGui::drawBGR(Mat& img, int b, int g, int r, Point3u color) 
{
    img.at<Point3u>(G1Y(r), G1X(g), 0) = color;    // y=R, x=G
    img.at<Point3u>(G2Y(r), G2X(b), 0) = color;    // y=R, x=B
    img.at<Point3u>(G3Y(g), G3X(b), 0) = color;    // y=B, x=G
}

void VisionGui::drawRectangleBGR(Mat& img, int b, int g, int r, int wb, int wg, int wr, const Scalar& color)
{
    rectangle(img, Point(G1X(g - wg), G1Y(r - wr)), Point(G1X(g + wg), G1Y(r + wr)), color, 1, 8, 0);
    rectangle(img, Point(G2X(b - wb), G2Y(r - wr)), Point(G2X(b + wb), G2Y(r + wr)), color, 1, 8, 0);
    rectangle(img, Point(G3X(b - wb), G3Y(g - wg)), Point(G3X(b + wb), G3Y(g + wg)), color, 1, 8, 0);
}

void VisionGui::drawColors(const SamplePixels& sample, Mat& img) {
    for (int p = 0; p < sample.patchNum; p++) {
        int j = 0;
        for (int y = 0; y < SAMPLE_HEIGHT; y++) {
            for (int x = 0; x < SAMPLE_WIDTH; x++) {
                Point3u pt = sample.pixels[p].at<Point3u>(j);
                Point3u ct = sample.ctvals[p].at<Point3u>(j++);

                img.at<Point3u>(y, p*SAMPLE_WIDTH + x, 0) = pt;
                drawBGR(img, ct.x, ct.y, ct.z, pt);
            }
        }
    }
}


void VisionGui::drawClusters(Mat& img, const Mat& centers, const int *radius)
{
    if (centers.empty()) {
        return;
    }

    for (int i = 0; i < CLUSTER_COUNT; i++) {
        Point3u pt = centers.at<Point3f>(i);
        
        int b = cvRound(pt.x);
        int g = cvRound(pt.y);
        int r = cvRound(pt.z);
        //drawBGR(img, b, g, r, Point3u(255, 255, 255));
        drawBGR(img, b-1, g, r, Point3u(255, 255, 255));
        drawBGR(img, b+1, g, r, Point3u(255, 255, 255));
        drawBGR(img, b, g-1, r, Point3u(255, 255, 255));
        drawBGR(img, b, g+1, r, Point3u(255, 255, 255));
        drawBGR(img, b, g, r-1, Point3u(255, 255, 255));
        drawBGR(img, b, g, r+1, Point3u(255, 255, 255));
        
        drawRectangleBGR(img, b, g, r, radius[3*i+0], radius[3*i+1], radius[3*i+2], Scalar( 255, 255, 255));
        
        Mat pt2(1, 1, CV_8UC3);
        Mat ct2(1, 1, CV_8UC3);
        
        pt2.at<Point3u>(0, 0, 0) = Point3u(b, g, r);
        convertInv(pt2, ct2);
        Point3u ct = ct2.at<Point3u>(0, 0, 0);
        
        for (int y = 0; y < SAMPLE_HEIGHT/2; y++) {
            for (int x = 0; x < SAMPLE_WIDTH/2; x++) {
                img.at<Point3u>(SAMPLE_HEIGHT * 2 + i*SAMPLE_HEIGHT/2 + y, x, 0) = ct;
            }
        }
    }    
}


void VisionGui::drawMarkers(Mat& imageMark, const Mat& markers, const Mat& imageGray)
{
    imageMark.create(imageGray.size(), CV_8UC3);
    for(int y = 0; y < imageGray.rows; y++) {
        for(int x = 0; x < imageGray.cols; x++) {
            int index = markers.at<uchar>(y, x);
            if (index == 0) {
                imageMark.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
            } else 
            if (index == 1) {
                imageMark.at<Vec3b>(y, x) = Vec3b(0, 0, 255);  // red
            } else 
            if (index == 4) {
                imageMark.at<Vec3b>(y, x) = Vec3b(0, 255, 0);  // green
            } else {
                imageMark.at<Vec3b>(y, x) = Vec3b(255, 0, 0);  // blue
            }
        }
    }
    imageMark = imageMark*0.5 + imageGray*0.5;
}

void VisionGui::drawSample(Mat& image, const SamplePixels &sample, const Scalar& color, const string& sampleFN)
{
    for (int p = 0; p < sample.patchNum; p++) {
        if (sampleFN == sample.fileNames[p]) {
            if ((sample.points[p].x >= 0) && (sample.points[p].y >= 0)) {
                rectangle(image, Point(sample.points[p].x - 2, sample.points[p].y - 2), Point(sample.points[p].x + SAMPLE_WIDTH + 2, sample.points[p].y + SAMPLE_HEIGHT + 2), color, 1, 8, 0);
                rectangle(image, Point(sample.points[p].x - 1, sample.points[p].y - 1), Point(sample.points[p].x + SAMPLE_WIDTH + 1, sample.points[p].y + SAMPLE_HEIGHT + 1), Scalar( 255, 255, 255), 1, 8, 0);
            }
        }
    }
}

void VisionGui::printSample(const string& name, const SamplePixels &sample)
{
    /* fixme: znak "\" treba nahradit "\\" */
    cout << "int load" << name << "(SamplePixels &sample)" << endl << "{" << endl;
    for (int p = 0; p < sample.patchNum; p++) {
         cout << "    if (sample.addSample(\"" << sample.fileNames[p] << "\", Point("<< sample.points[p].x <<", " << sample.points[p].y << ")) < 0) return -1;" << endl;
    }
    cout << "    return 0;" << endl << "}" << endl;
}

void VisionGui::drawEPoints(Mat& image, const Mat& epoints) 
{
    for(int i = 0; i < ELINES_COUNT; i++) {
        for(int j = 0; j < EPOINTS_COUNT; j++) {
            Point p = epoints.at<Point>(i, j);

            if ((p.x >= 0) && (p.x < 640) && (p.y >= 0) && (p.y < 480)) {                
                image.at<Vec3b>(p.y, p.x) = Vec3b(255, 255, 0);
                if (p.x > 0) {
                    image.at<Vec3b>(p.y, p.x-1) = Vec3b(255, 255, 0);
                }
                if (p.x < 639) {            
                    image.at<Vec3b>(p.y, p.x+1) = Vec3b(255, 255, 0);
                }
                if (p.y > 0) {            
                    image.at<Vec3b>(p.y-1, p.x) = Vec3b(255, 255, 0);
                }
                if (p.y < 479) {            
                    image.at<Vec3b>(p.y+1, p.x) = Vec3b(255, 255, 0);
                }
            }
        }    
    }
}


void VisionGui::drawEPWeight(Mat& image, const Mat& epoints, const Mat& epdist, const Mat& elines, const Mat& epweigth, const Mat& elweigth, const int& angle_min, const int& angle_max)
{   
    for(int i = 0; i < ELINES_COUNT; i++) {       
        float w = elweigth.at<float>(i);
        float wl = elines.at<Vec3f>(i)[ELINES_LIMIT_IDX];
        int k = EPOINTS_COUNT;    // index prveho bodu, ktory ma vzdialenost vacsiu ako hodnota pre danu ciaru
        int kl = EPOINTS_COUNT;    // index posledneho bodu, ktory ma vzdialenost mensiu ako limit (na detekciu prekazky) pre danu ciaru
        for(int j = 0; j < EPOINTS_COUNT; j++) {
            float d = epdist.at<float>(j);
            if (d - 0.001 <= wl) {
                kl = j;
            }            
            if (w < d) {
                k = j;
                break;
            }
            //line(image, Point(image.cols / 2, image.rows - 1), epoints.at<Point>(i, j), Scalar( 255, 255, 255), 5, 8, 0);
        }
        // nakreslime ciaru do predchadzajuceho bodu
        if ((--k) >= 0) {
            if (k > kl) {
                int angle = elines.at<Vec3f>(i)[ELINES_ANGLE_IDX];
                if ((angle >= angle_min) && (angle <= angle_max)) {
                    line(image, epoints.at<Point>(i, kl), epoints.at<Point>(i, k), Scalar( 0, 200, 0), 2, 8, 0);
                    line(image, Point(image.cols / 2, image.rows - 1), epoints.at<Point>(i, kl), Scalar( 0, 255, 0), 2, 8, 0);                    
                } else {
                    line(image, epoints.at<Point>(i, kl), epoints.at<Point>(i, k), Scalar( 200, 200, 200), 2, 8, 0);
                    line(image, Point(image.cols / 2, image.rows - 1), epoints.at<Point>(i, kl), Scalar( 255, 255, 255), 2, 8, 0);
                }
            } else {
                line(image, Point(image.cols / 2, image.rows - 1), epoints.at<Point>(i, kl), Scalar( 255, 0, 255), 2, 8, 0);
            }
        } else {
            int dx = epoints.at<Point>(i, 0).x - image.cols / 2;
            int dy = epoints.at<Point>(i, 0).y - (image.rows - 1);
            line(image, Point(image.cols / 2, image.rows - 1), Point(image.cols / 2 + dx / 2, image.rows - 1 + dy / 2), Scalar( 255, 0, 255), 2, 8, 0);
        }
    }
}

/*--------------------------*/

int Vision::init(SamplePixels &sampleOnRoad, SamplePixels &sampleOffRoad) 
{    
    double t = timeBegin();

//  if (!imageSample.data) {
//      printf("Vision::testTrain(): error: no imageSample data!\n");
//      return -1;
//  }

    Mat clusterLabels;

//  if (!checkPoint(imageSample, sampleOnRoad.points[0])) {
//      printf("Vision::testTrain(): error: wrong onRoad coordinates!\n");
//      return -2;
//  }
                            
    calcKMeans(sampleOnRoad, clusterCenters1, clusterLabels, clusterRadius1);

//  if (!checkPoint(imageSample, sampleOffRoad.points[0])) {
//      printf("Vision::testTrain(): error: wrong offRoad coordinates!\n");
//      return -2;
//  }
    
    calcKMeans(sampleOffRoad, clusterCenters2, clusterLabels, clusterRadius2);
    
    initEvalLT(evalLT, clusterCenters1, clusterCenters2, clusterRadius1, clusterRadius2);

    generateEPoints(epoints, epdist, elines);
    
#ifdef ISTRO_GUI
    namedWindow("vision", 0); 

    Mat img;
    img.create(640,480,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    imshow("vision", img);
#endif

    timeEnd("Vision::init", t);

    return 0;    
}

int Vision::eval(Mat &image, Mat& markers, Mat& markersIM, Mat& epweigth, Mat& elweigth, int *dmap)
//int Vision::eval(Mat &image, int& angle_min, int& angle_max) 
{    
    double t = timeBegin();    

    if ( !image.data ) {
        LOGM_ERROR(loggerVision, "eval", "no image data!");
        return -1;
    }
    
    calcMarkers(image, markers, clusterCenters1, clusterCenters2, clusterRadius1, clusterRadius2, evalLT);
    calcMarkersIM(markers, markersIM);

/*    
    Mat imageTmp;
    double t = timeBegin();    
    tweight[0] = vision.calcTWeight(imageTmp, markers,   0, 160,   0, 479, 320, 479);
    tweight[1] = vision.calcTWeight(imageTmp, markers, 160, 160,  80, 479, 400, 479);
    tweight[2] = vision.calcTWeight(imageTmp, markers, 320, 160, 160, 479, 480, 479);
    tweight[3] = vision.calcTWeight(imageTmp, markers, 480, 160, 240, 479, 560, 479);
    tweight[4] = vision.calcTWeight(imageTmp, markers, 639, 160, 320, 479, 639, 479);
    timeEnd("Vision::calcTweight[5]", t);
    printf("calcTWeight() = (%d, %d, %d, %d, %d)\n", tweight[0], tweight[1], tweight[2], tweight[3], tweight[4]);
*/

    calcEPWeight(epoints, epdist, markersIM, epweigth, elweigth);
    calcELimitAngle(elines, elweigth, dmap);
    
//  angle_min = eangle_min; 
//  angle_max = eangle_max; 
            
    timeEnd("Vision::eval", t);
        
    return 0;
}

void Vision::drawOutput(const Mat &image, Mat &out, const Mat& markers, const Mat& epweigth, const Mat& elweigth, const int& angle_min, const int& angle_max)
//void Vision::drawOutput(const Mat &image, Mat &out)
{
    Mat imageTmp, imageGray;

    double t = timeBegin();

    cvtColor(image, imageTmp, COLOR_BGR2GRAY);
    cvtColor(imageTmp, imageGray, COLOR_GRAY2BGR);

    drawMarkers(out, markers, imageGray);
    
    drawEPWeight(out, epoints, epdist, elines, epweigth, elweigth, angle_min, angle_max);
    drawEPoints(out, epoints);
    
#ifdef ISTRO_GUI    
    imshow("vision", out);
#endif
    
    timeEnd("Vision::drawOutput", t);
}

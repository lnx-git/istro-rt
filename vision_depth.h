#ifndef __VISION_DEPTH_H__
#define __VISION_DEPTH_H__

#include "system.h"
#include "camera.h"

#ifdef ISTRO_VISION_DEPTH

/* kedze robime diferencie 8 pixelov, tak uz 6 pixelovy pas povazujeme za prekazku, ktoru treba zdetekovat;
   vacsinu pre hodnotu "0" ziskame iba pri malych blokoch napr 11x11 */
const int VISIOND_EP_SIZE = 11; /* velkost bloku v pixloch pre vyhodnocovanie majoritnej farby v okoli bodu - vzdy neparne */
const int VISIOND_EP_THRESHOLDM = 1.2 * 256;  /* 1.2 * 256 = majoritna hodnota musi mat viac nez 20% vacsie zastupenie ako vsetky ostatne */ 

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

const int DRLIMIT_DELTA_MIN_LIMIT = 6000;    // do 6 metrov vyhodnocujeme prekazky typu schod
const int DRLIMIT_DELTA_MAX_LIMIT = 3000;    // do 3 metrov vyhodnocujeme prekazky typu jama

class VisionDepth {
private:
    int drl_dist_ref[CAMERA_DEPTH_FRAME_HEIGHT];
    int drl_dist_min[CAMERA_DEPTH_FRAME_HEIGHT];    // minimalna vzdialenost, kde uz vsetko povazujeme za prekazku
    int drl_delta2_min[DRLIMIT_DELTA_MIN_LIMIT];    // minimalna derivacia pre prekazky typu schod
    int drl_delta2_max[DRLIMIT_DELTA_MAX_LIMIT];    // maximalna derivacia pre prekazky typu jama

private:
    void drlimit_init(void);

public:
    static void getTestData(Mat &depth);
    void test(void);

public:
    int  init(void);
    void close(void);

    void process(const Mat &depth, Mat &depth_pred0, Mat &depth_pred);

    void drawOutput(const Mat &image, const Mat &depth_pred, Mat &out);
};

#endif

#endif

#ifndef __NAVMAP_H__
#define __NAVMAP_H__

#include <string>
#include <opencv2/opencv.hpp>

const int NAVMAP_FLAG_CROSSING    =  1;
const int NAVMAP_FLAG_LEAF        =  2;
const int NAVMAP_FLAG_ROUTE       =  4;    // segment is part of planned route
const int NAVMAP_FLAG_ROUTE_OPDIR =  8;      
const int NAVMAP_FLAG_TMP_PLAN    = 16;    // temporal node/segment for Planning purposes
const int NAVMAP_FLAG_INVISIBLE   = 32;
const int NAVMAP_FLAG_DELETE      = 64;

const long long NAVMAP_NODE_P11    = -11;
const long long NAVMAP_NODE_P12    = -12;
const long long NAVMAP_SEGMENT_P12 = -13;
const long long NAVMAP_SEGMENT_P1A = -14;
const long long NAVMAP_SEGMENT_P1B = -15;

const long long NAVMAP_NODE_P21    = -21;
const long long NAVMAP_NODE_P22    = -22;
const long long NAVMAP_SEGMENT_P22 = -23;
const long long NAVMAP_SEGMENT_P2A = -24;
const long long NAVMAP_SEGMENT_P2B = -25;

const long long NAVMAP_SEGMENT_P3  = -30;

extern int navmap_node_cnt;

typedef struct {
    long long  node_id;
    double     longitude;
    double     latitude;
    double     x;
    double     y;
    int        flags;
} navmap_node_t;

extern navmap_node_t navMapNode[];

extern int navmap_segment_cnt;

typedef struct {
    long long  way_id;
    int        node1_idx;
    int        node2_idx;
    int        flags;
} navmap_segment_t;

extern navmap_segment_t navMapSegment[];

typedef struct {
    char   name[20];
    char   desc[200];
    char   style[20];
    double longitude;
    double latitude;
} aux_point_t;

int navmap_init(void);

int navmap_node_add(long long node_id, double lat, double lon, int flags);
int navmap_segment_add(long long way_id, int node1_idx, int node2_idx, int flags);

int navmap_compress(void);
int navmap_delete(void);

int navmap_load(const std::string& fname);    /* load from .OSM file - requires ISTRO_LIBXML2 */

int navmap_export_kml(const std::string& fname, aux_point_t *paux = NULL, aux_point_t *paux2 = NULL);
int navmap_export_data(const std::string& fname);

int navmap_draw(cv::Mat& img, aux_point_t *paux = NULL, aux_point_t *paux2 = NULL);

double navmap_dist2PS_XY(double x, double y, int flags = 0, int *pidx = NULL, double *pxx = NULL, double *pyy = NULL);
double navmap_dist2PS_LL(double lat, double lon, int flags = 0, int *pidx = NULL, double *pllat = NULL, double *pllon = NULL);

int navmap_routeFwd(double lat, double lon, double dist, double& lat2, double& lon2);

double navmap_planRouteLL(double lat1, double lon1, double lat2, double lon2);
void   navmap_plan_delete(void);

void navmap_test(void);

#endif

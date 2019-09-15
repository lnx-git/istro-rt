#include <math.h>
#include <stdio.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include "mtime.h"
#include "system.h"
#include "config.h"    // ANGLE_NONE
#include "navig.h"
#include "navmap.h"
#include "navmap_data.h"
#include "logger.h"

using namespace std;
using namespace cv;

LOG_DEFINE(loggerNavMap, "navmap");

int navmap_node_cnt = 0;
int navmap_segment_cnt = 0;

const int NAVMAP_NODE_NUM = 5000;
const int NAVMAP_SEGMENT_NUM  = 4000;

navmap_node_t navMapNode[NAVMAP_NODE_NUM];
navmap_segment_t navMapSegment[NAVMAP_SEGMENT_NUM];

int navmap_node_add(long long node_id, double lat, double lon, int flags)
{
    if (navmap_node_cnt >= NAVMAP_NODE_NUM) {
        LOGM_ERROR(loggerNavMap, "navmap_node_add", "msg=\"error: navMapNode[] array is full!\", navmap_node_cnt=" << navmap_node_cnt);
        return -1;
    }

    int idx = navmap_node_cnt;
    navMapNode[idx].node_id = node_id;  
    navMapNode[idx].longitude = lon;
    navMapNode[idx].latitude = lat;
    navMapNode[idx].flags = flags;
    
    navigation_getXY(lat, lon, navMapNode[idx].x, navMapNode[idx].y);

    navmap_node_cnt++;

    return idx;
}

int navmap_segment_add(long long way_id, int node1_idx, int node2_idx, int flags)
{
    if (navmap_segment_cnt >= NAVMAP_SEGMENT_NUM) {
        LOGM_ERROR(loggerNavMap, "navmap_segment_add", "msg=\"error: navMapSegment[] array is full!\", navmap_segment_cnt=" << navmap_segment_cnt);
        return -1;
    }

    int idx = navmap_segment_cnt;
    navMapSegment[idx].way_id = way_id;  
    navMapSegment[idx].node1_idx = node1_idx;
    navMapSegment[idx].node2_idx = node2_idx;
    navMapSegment[idx].flags = flags;
    
    navmap_segment_cnt++;

    return idx;
}

int navmap_compress(void)
// remove unused nodes + calculate flags
{
    int node_used[NAVMAP_NODE_NUM];
    int new_idx[NAVMAP_NODE_NUM];
    
    int old_node_cnt = navmap_node_cnt;

    for (int i = 0; i < navmap_node_cnt; i++) {
        node_used[i] = 0;
        new_idx[i] = -1;
    }

    for (int j = 0; j < navmap_segment_cnt; j++) {
        node_used[navMapSegment[j].node1_idx]++;
        node_used[navMapSegment[j].node2_idx]++;
    }

    int nidx = 0;
    // reallocate nodes - overwrite all unused nodes
    for (int i = 0; i < navmap_node_cnt; i++) {
        if (node_used[i] > 0) {
            new_idx[i] = nidx;
            if (nidx != i) {
                node_used[nidx] = node_used[i];
                navMapNode[nidx] = navMapNode[i];
            }
            nidx++;
        }
    }

    navmap_node_cnt = nidx;

    // update node flags 
    for (int i = 0; i < navmap_node_cnt; i++) {
        int flags = 0;
        if (node_used[i] > 2) {
            flags |= NAVMAP_FLAG_CROSSING;
        } else
        if (node_used[i] < 2) {
            flags |= NAVMAP_FLAG_LEAF;
        }
        navMapNode[i].flags = (navMapNode[i].flags & ~(NAVMAP_FLAG_CROSSING | NAVMAP_FLAG_LEAF)) | flags;
    }
    
    int res = 0;
    if (navmap_node_cnt != old_node_cnt) {

        // recalculate node indexes for all segments
        for (int j = 0; j < navmap_segment_cnt; j++) {
            navMapSegment[j].node1_idx = new_idx[navMapSegment[j].node1_idx];
            navMapSegment[j].node2_idx = new_idx[navMapSegment[j].node2_idx];
        }

        res = 1;
    }

    LOGM_INFO(loggerNavMap, "navmap_compress", "res=" << res << ", navmap_node_cnt=" << navmap_node_cnt << ", navmap_segment_cnt=" << navmap_segment_cnt
        << ", old_node_cnt=" << old_node_cnt);
        
    return res;
}

int navmap_delete(void)
// delete nodes marked with flag NAVMAP_FLAG_DELETE
{
    int old_node_cnt = navmap_node_cnt;
    int old_segment_cnt = navmap_segment_cnt;
    
    int sidx = 0;
    // reallocate segments - overwrite all unused segments
    for (int j = 0; j < navmap_segment_cnt; j++) {
        if ((navMapSegment[j].flags & NAVMAP_FLAG_DELETE) == 0) {
            if (sidx != j) {
                navMapSegment[sidx] = navMapSegment[j];
            }
            sidx++;
        }
    }
    
    int res = 0;
    if (navmap_segment_cnt != sidx) {
        navmap_segment_cnt = sidx;

        // call compress to remove unused nodes
        navmap_compress();
        
        res = 1;
    }
    
    LOGM_INFO(loggerNavMap, "navmap_delete", "res=" << res << ", navmap_node_cnt=" << navmap_node_cnt << ", navmap_segment_cnt=" << navmap_segment_cnt
        << ", old_node_cnt=" << old_node_cnt << ", old_segment_cnt=" << old_segment_cnt);

    return res;
}

int navmap_export_kml(const string& fname, aux_point_t *paux /* = NULL */, aux_point_t *paux2 /* = NULL */)
{
    long long way_id;
    int node_idx, flags;
    int segment_exported[NAVMAP_SEGMENT_NUM];
    int navp_cnt = 0, node_cnt = 0, seg_cnt = 0, seg2_cnt = 0, aux_cnt = 0, aux2_cnt = 0;

    FILE *f = fopen(fname.c_str(), "w");

    if (f == NULL) {
        LOGM_ERROR(loggerNavMap, "navmap_export_kml", "msg=\"error: unable to open file!\", fname=\"" << fname << "\"");
        return -1;
    }

    fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://earth.google.com/kml/2.0\">\"\n");
    fprintf(f, "<Folder><name>%s</name>\n", fname.c_str());
    // fprintf(f, "<Style id=\"style1\"><IconStyle><scale>0.8</scale><color>ff00ff00</color><Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon></IconStyle></Style>\n");
    fprintf(f, "<Style id=\"style1\"><IconStyle><scale>0.5</scale><Icon><href>http://earth.google.com/images/kml-icons/track-directional/track-none.png</href></Icon></IconStyle></Style>\n");
    fprintf(f, "<Style id=\"style2\"><IconStyle><scale>0.8</scale><color>ff00ff00</color><Icon><href>http://maps.google.com/mapfiles/kml/shapes/star.png</href></Icon></IconStyle></Style>\n");
    fprintf(f, "<Style id=\"style3\"><IconStyle><scale>0.6</scale><color>ff808080</color><Icon><href>http://maps.google.com/mapfiles/kml/shapes/forbidden.png</href></Icon></IconStyle></Style>\n");
    fprintf(f, "<Style id=\"style4\"><IconStyle><scale>0.6</scale><color>ff00ff00</color><Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon></IconStyle></Style>\n");
    fprintf(f, "<Style id=\"lstyle1\"><LineStyle><color>FFFF0000</color><width>2</width></LineStyle></Style>\n");
    fprintf(f, "<Style id=\"lstyle2\"><LineStyle><color>FF00FF00</color><width>2</width></LineStyle></Style>\n");
    fprintf(f, "<Style id=\"lstyle3\"><LineStyle><color>FF0000FF</color><width>2</width></LineStyle></Style>\n");

    fprintf(f, "<Folder><name>navigationPoint</name>\n");
    for (int i = 0; i < NAVIGATION_POINT_COUNT; i++) {
        if ((navigationPoint[i].longitude == ANGLE_NONE) || (navigationPoint[i].latitude == ANGLE_NONE)) continue;
        fprintf(f, "<Placemark><description>point_idx=%d</description>", i);
        fprintf(f, "<name>%s</name><styleUrl>style4</styleUrl><Point><altitudeMode>clampToGround</altitudeMode><coordinates>%10.7f,%10.7f</coordinates></Point></Placemark>\n", 
            (char *)navigationPoint[i].name, navigationPoint[i].longitude, navigationPoint[i].latitude);
        navp_cnt++;
    }
    fprintf(f, "</Folder>\n");

    fprintf(f, "<Folder><name>navMapNode</name>\n");
    for (int i = 0; i < navmap_node_cnt; i++) {
        if ((navMapNode[i].flags & (NAVMAP_FLAG_CROSSING | NAVMAP_FLAG_LEAF)) == 0) continue;
        fprintf(f, "<Placemark><description>node_id=%lld</description>", navMapNode[i].node_id);
        fprintf(f, "<name></name><styleUrl>style1</styleUrl><Point><altitudeMode>clampToGround</altitudeMode><coordinates>%10.7f,%10.7f</coordinates></Point></Placemark>\n", 
            navMapNode[i].longitude, navMapNode[i].latitude);
        node_cnt++;
    }
    fprintf(f, "</Folder>\n");

    fprintf(f, "<Folder><name>navMapSegment</name>\n");
    for (int j = 0; j < navmap_segment_cnt; j++) {
        segment_exported[j] = 0;
    }
    for (int j = 0; j < navmap_segment_cnt; j++) {
        if (segment_exported[j]) continue;
        if ((navMapSegment[j].flags & NAVMAP_FLAG_INVISIBLE) > 0) continue;
        way_id = navMapSegment[j].way_id;
        
        seg_cnt++;

        double alt = 0.0;
        fprintf(f, "<Placemark><name>way_id=%lld</name>", way_id);
        if ((navMapSegment[j].flags & NAVMAP_FLAG_ROUTE) > 0) {
            fprintf(f, "<styleUrl>lstyle2</styleUrl>");
            alt = 0.1; // affect the ordering of vector layers by giving an artifical altitude; needs to be small, so would not be visible
            seg2_cnt++;
        } else {
            fprintf(f, "<styleUrl>lstyle1</styleUrl>");
        }
        fprintf(f, "<LineString id=\"way%lld\"><extrude>0</extrude><tessellate>0</tessellate><altitudeMode>clampToGround</altitudeMode>\n", way_id);
        fprintf(f, "<coordinates>");
        
        double lon = navMapNode[navMapSegment[j].node1_idx].longitude;
        double lat = navMapNode[navMapSegment[j].node1_idx].latitude;
        fprintf(f, "%10.7f,%10.7f,%0.1f", lon, lat, alt);
        
        lon = navMapNode[navMapSegment[j].node2_idx].longitude;
        lat = navMapNode[navMapSegment[j].node2_idx].latitude;
        fprintf(f, ",%10.7f,%10.7f,%0.1f", lon, lat, alt);
        
        segment_exported[j] = 1;
        node_idx = navMapSegment[j].node2_idx;
        flags = navMapSegment[j].flags;
        for(int k = j+1; k < navmap_segment_cnt; k++) {
            if (way_id != navMapSegment[k].way_id) break;
            if (node_idx != navMapSegment[k].node1_idx) break;
            if (flags != navMapSegment[k].flags) break;
            
            lon = navMapNode[navMapSegment[k].node2_idx].longitude;
            lat = navMapNode[navMapSegment[k].node2_idx].latitude;
            fprintf(f, ",%10.7f,%10.7f,%0.1f", lon, lat, alt);
            
            segment_exported[k] = 1;
            node_idx = navMapSegment[k].node2_idx;
        }
        fprintf(f, "</coordinates></LineString>");
        fprintf(f, "</Placemark>\n");
    }
    fprintf(f, "</Folder>\n");

    if (paux != NULL) {
        fprintf(f, "<Folder><name>auxPoint</name>\n");
        
        while (paux->style[0] != 0) {
            fprintf(f, "<Placemark><description>%s</description><name>%s</name><styleUrl>%s</styleUrl>", paux->desc, paux->name, paux->style);
            fprintf(f, "<Point><altitudeMode>clampToGround</altitudeMode><coordinates>%10.7f,%10.7f</coordinates></Point></Placemark>\n", 
                paux->longitude, paux->latitude);
            paux++;
            aux_cnt++;
        }
        fprintf(f, "</Folder>\n");
    }
    
    if (paux2 != NULL) {
        fprintf(f, "<Folder><name>auxLine</name>\n");
        fprintf(f, "<Placemark><name>aux_line</name>");
        fprintf(f, "<styleUrl>lstyle3</styleUrl>");
        fprintf(f, "<LineString id=\"aux_line\"><extrude>0</extrude><tessellate>0</tessellate><altitudeMode>clampToGround</altitudeMode>\n");
        fprintf(f, "<coordinates>");
        
        while (paux2->style[0] != 0) {
            if (aux2_cnt > 0) fprintf(f, ",");
            fprintf(f, "%0.7f,%0.7f,%0.1f", paux2->longitude, paux2->latitude, 0.0);
            paux2++;
            aux2_cnt++;
        }
        fprintf(f, "</coordinates></LineString>");
        fprintf(f, "</Placemark>\n");
        fprintf(f, "</Folder>\n");
    }
    
    fprintf(f, "</Folder></kml>\n");

    fclose (f);

    LOGM_DEBUG(loggerNavMap, "navmap_export_kml", "fname=\"" << fname << "\""
        << ", navp_cnt=" << navp_cnt << ", node_cnt=" << node_cnt 
        << ", seg_cnt=" << seg_cnt << ", seg2_cnt=" << seg2_cnt << ", aux_cnt=" << aux_cnt << ", aux2_cnt=" << aux2_cnt);
    
    return 0;
}

const int NAVMAP_IMG_WIDTH  = 640;
const int NAVMAP_IMG_HEIGHT = 480;
const int NAVMAP_IMG_WDIST  = 150.0;  // 150 metres per one image

int navmap_draw(Mat& img, aux_point_t *paux /* = NULL */, aux_point_t *paux2 /* = NULL */)
{
    int navp_cnt = 0, node_cnt = 0, seg_cnt = 0, seg2_cnt = 0, aux_cnt = 0, aux2_cnt = 0;
    
    double t = timeBegin();

    img.create(NAVMAP_IMG_HEIGHT, NAVMAP_IMG_WIDTH, CV_8UC3);
    img.setTo(Scalar(0, 60, 0));
    
    double ix0 = NAVMAP_IMG_WIDTH / 2.0;    // midpoint
    double iy0 = NAVMAP_IMG_HEIGHT / 2.0;
    double ik = NAVMAP_IMG_WIDTH / NAVMAP_IMG_WDIST;
    
    double lat0 = ANGLE_NONE;
    double lon0 = ANGLE_NONE;
    
    if (paux != NULL) {
        if (paux->style[0] != 0) {
            lat0 = paux->latitude;        
            lon0 = paux->longitude;
        }
    }  
    
    if ((lat0 >= ANGLE_OK) || (lon0 >= ANGLE_OK)) {
        for (int i = 0; i < NAVIGATION_POINT_COUNT; i++) {
            if ((navigationPoint[i].longitude == ANGLE_NONE) || (navigationPoint[i].latitude == ANGLE_NONE)) continue;
            lat0 = navigationPoint[i].latitude;
            lon0 = navigationPoint[i].longitude;
            break;
        }
    }
    
    if ((lat0 >= ANGLE_OK) || (lon0 >= ANGLE_OK)) {
        LOGM_ERROR(loggerNavMap, "navmap_draw", "msg=\"error: midpoint not found!\"");
        return -1;
    }
    
    double x0, y0;
    navigation_getXY(lat0, lon0, x0, y0);

    for (int j = 0; j < navmap_segment_cnt; j++) {
        if ((navMapSegment[j].flags & NAVMAP_FLAG_INVISIBLE) > 0) continue;
        
        seg_cnt++;
        
        double x1 = navMapNode[navMapSegment[j].node1_idx].x;
        double y1 = navMapNode[navMapSegment[j].node1_idx].y;
        double x2 = navMapNode[navMapSegment[j].node2_idx].x;
        double y2 = navMapNode[navMapSegment[j].node2_idx].y;

        double ix1 = ix0 + (x1 - x0) * ik;
        double iy1 = iy0 - (y1 - y0) * ik;
        double ix2 = ix0 + (x2 - x0) * ik;
        double iy2 = iy0 - (y2 - y0) * ik;

        Scalar color = Scalar(255, 60, 60);
        if ((navMapSegment[j].flags & NAVMAP_FLAG_ROUTE) > 0) {
            color = Scalar(60, 255, 60);
            seg2_cnt++;
        }
        
        line(img, Point(ix1, iy1), Point(ix2, iy2), color, 2, 8, 0);
    }
    
    if (paux2 != NULL) {       
        if (paux2->style[0] != 0) {
            double x1, y1;
            double lon1 = paux2->longitude;
            double lat1 = paux2->latitude;
            navigation_getXY(lat1, lon1, x1, y1);
            
            double ix1 = ix0 + (x1 - x0) * ik;
            double iy1 = iy0 - (y1 - y0) * ik;

            paux2++;
        
            while (paux2->style[0] != 0) {
                double x2, y2;
                double lon2 = paux2->longitude;
                double lat2 = paux2->latitude;
                navigation_getXY(lat2, lon2, x2, y2);

                double ix2 = ix0 + (x2 - x0) * ik;
                double iy2 = iy0 - (y2 - y0) * ik;
                Scalar color = Scalar(60, 60, 255);
                line(img, Point(ix1, iy1), Point(ix2, iy2), color, 1, 8, 0);

                ix1 = ix2;
                iy1 = iy2;
                paux2++;
                aux2_cnt++;
            }
        }
    }
    
    for (int i = 0; i < navmap_node_cnt; i++) {
        if ((navMapNode[i].flags & (NAVMAP_FLAG_CROSSING | NAVMAP_FLAG_LEAF)) == 0) continue;
                    
        double x, y;
        double lon = navMapNode[i].longitude;
        double lat = navMapNode[i].latitude;
        navigation_getXY(lat, lon, x, y);
            
        double ix = ix0 + (x - x0) * ik;
        double iy = iy0 - (y - y0) * ik;
        
        Scalar color = Scalar(150, 150, 150);
        rectangle(img, Point(ix - 2, iy - 2), Point(ix + 2, iy + 2), color, CV_FILLED);
        
        // text: navMapNode[i].node_id

        node_cnt++;
    }
    
    for (int i = 0; i < NAVIGATION_POINT_COUNT; i++) {
        if ((navigationPoint[i].longitude == ANGLE_NONE) || (navigationPoint[i].latitude == ANGLE_NONE)) continue;
        
        double x, y;
        double lon = navigationPoint[i].longitude;
        double lat = navigationPoint[i].latitude;
        navigation_getXY(lat, lon, x, y);
            
        double ix = ix0 + (x - x0) * ik;
        double iy = iy0 - (y - y0) * ik;
        
        Scalar color = Scalar(0, 255, 255);
        rectangle(img, Point(ix - 2, iy - 2), Point(ix + 2, iy + 2), color, CV_FILLED);
        
        // text: navigationPoint[i].name

        navp_cnt++;
    }
    
    if (paux != NULL) {
        while (paux->style[0] != 0) {
            double x, y;
            double lon = paux->longitude;
            double lat = paux->latitude;
            navigation_getXY(lat, lon, x, y);
            
            double ix = ix0 + (x - x0) * ik;
            double iy = iy0 - (y - y0) * ik;
        
            Scalar color = Scalar(60, 60, 255);
            rectangle(img, Point(ix - 2, iy - 2), Point(ix + 2, iy + 2), color, CV_FILLED);
            
            paux++;
            aux_cnt++;
        }
    }

    LOGM_DEBUG(loggerNavMap, "navmap_draw", "navp_cnt=" << navp_cnt << ", node_cnt=" << node_cnt << ", seg_cnt=" << seg_cnt << ", seg2_cnt=" << seg2_cnt << ", aux_cnt=" << aux_cnt << ", aux2_cnt=" << aux2_cnt);

    timeEnd("navmap::navmap_draw", t);
    
    return 0;
}

/*
void navmap_import_data(void)
{
    int i = 0;
    int j = 0;
    navmap_node_t *n;
    navmap_segment_t *s;

    n = &navMapNode[i++];  n->node_id = 1l;  n->longitude = 0;  n->latitude = 0;  n->x = 0;  n->y = 0;  n->flags = 0;

    s = &navMapSegment[j++];  s->way_id = 1l;  s->node1_idx = 2;  s->node2_idx = 3;  s->flags = 0;

    navmap_node_cnt = i;
    navmap_segment_cnt = j;
}
*/

int navmap_export_data(const string& fname)
{
    FILE *f = fopen(fname.c_str(), "w");

    if (f == NULL) return -1;

    fprintf(f, "void navmap_import_data(void)\n");
    fprintf(f, "{\n");
    fprintf(f, "    int i = 0;\n");
    fprintf(f, "    int j = 0;\n");
    fprintf(f, "    navmap_node_t *n;\n");
    fprintf(f, "    navmap_segment_t *s;\n\n");

    for (int i = 0; i < navmap_node_cnt; i++) {
        fprintf(f, "    n = &navMapNode[i++];  n->node_id = %lldll;  n->longitude = %10.7f;  n->latitude = %10.7f;  n->x = %0.3f;  n->y = %0.3f;  n->flags = %d;\n", 
            navMapNode[i].node_id, navMapNode[i].longitude, navMapNode[i].latitude, navMapNode[i].x, navMapNode[i].y, navMapNode[i].flags);
    }
    fprintf(f, "\n");

    for (int j = 0; j < navmap_segment_cnt; j++) {
        fprintf(f, "    s = &navMapSegment[j++];  s->way_id = %lldll;  s->node1_idx = %d;  s->node2_idx = %d;  s->flags = %d;\n", 
            navMapSegment[j].way_id, navMapSegment[j].node1_idx, navMapSegment[j].node2_idx, navMapSegment[j].flags);
    }
    fprintf(f, "\n");
    
    fprintf(f, "    navmap_node_cnt = i;\n");
    fprintf(f, "    navmap_segment_cnt = j;\n");
    fprintf(f, "}\n");

    fclose (f);
    
    LOGM_DEBUG(loggerNavMap, "navmap_export_data", "fname=\"" << fname << "\"");
    
    return 0;
}

#ifdef ISTRO_LIBXML2

#include <libxml/parser.h>
#include <libxml/tree.h>

int navmap_load_osmway(xmlNode * root_node, long long way_id)
/*
  <way id='-55737'>
    <nd ref='1043055070' />
    <nd ref='1043055071' />
    <tag k='highway' v='footway' />
  </way>
  <way id='343313714'>
    <nd ref='3501768873' />
    <nd ref='3501768874' />
    <tag k='highway' v='track' />
    <tag k='tracktype' v='grade3' />
  </way>
*/
{
    int last_node_idx = -1;
    int old_segment_cnt = navmap_segment_cnt;
    int accept_way = 0;   // 1 = footway, 2 = track
    int track_grade3 = 0;

    for(int i = 0; i < navmap_segment_cnt; i++) {
        if (navMapSegment[i].way_id == way_id) {
            LOGM_ERROR(loggerNavMap, "navmap_load_osmway", "msg=\"error: duplicate way_id identifier found!\", way_id=" << way_id)
            return -1;
        }
    }

    xmlNode *node = NULL;
    for (node = root_node->children; node; node = node->next) {
        if (node->type != XML_ELEMENT_NODE) continue;

        if (xmlStrcmp(node->name, (const xmlChar *) "nd") == 0) {
            xmlChar *refStr = xmlGetProp(node, (const xmlChar *) "ref");
            if (refStr == NULL) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osmway", "msg=\"error: Way::Nd.Ref property is missing!\", way_id=" << way_id);
                continue;
            }
            long long ref = strtoll ((const char *)refStr, NULL, 0);

            int node_idx = -1;
            for(int i = 0; i < navmap_node_cnt; i++) {
                if (navMapNode[i].node_id == ref) {
                    node_idx = i;
                    continue;
                }
            }

            if (node_idx < 0) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osmway", "msg=\"error: no navMapNode found for Way::Nd.Ref property!\", ref=" << ref);
                continue;
            }

            if (last_node_idx >= 0) {
                if (navmap_segment_cnt >= NAVMAP_SEGMENT_NUM) {
                    LOGM_ERROR(loggerNavMap, "navmap_load_osmway", "msg=\"error: navMapSegment[] array is full!\", navmap_segment_cnt=" << navmap_segment_cnt);
                    continue;
                }

                navMapSegment[navmap_segment_cnt].way_id = way_id;
                navMapSegment[navmap_segment_cnt].node1_idx = last_node_idx;
                navMapSegment[navmap_segment_cnt].node2_idx = node_idx;
                navMapSegment[navmap_segment_cnt].flags = 0;
                //navMapSegment[navmap_segment_cnt].flags = 0;

                navmap_segment_cnt++;
                LOGM_DEBUG(loggerNavMap, "navmap_load_osmway", "way_id=" << way_id << ", type=\"nd\", ref=" << ref 
                    << ", cnt=" << navmap_segment_cnt << ", node1_idx=" << last_node_idx << ", node2_idx=" << node_idx);
            } else {
                LOGM_DEBUG(loggerNavMap, "navmap_load_osmway", "way_id=" << way_id << ", type=\"nd\", ref=" << ref);
            }
            last_node_idx = node_idx;

        } else
        if (xmlStrcmp(node->name, (const xmlChar *) "tag") == 0) {
            xmlChar *kStr = xmlGetProp(node, (const xmlChar *) "k");
            xmlChar *vStr = xmlGetProp(node, (const xmlChar *) "v");
            if ((kStr == NULL) || (vStr == NULL)) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osmway", "msg=\"error: Way::Nd.k/v property is missing!\", way_id=" << way_id);
                continue;
            }

            // accept only ways with this tag: <tag k='highway' v='footway' />
            if ((xmlStrcmp(kStr, (const xmlChar *) "highway") == 0) && 
                (xmlStrcmp(vStr, (const xmlChar *) "footway") == 0)) {
                accept_way = 1;
            }
            // accept also 'highway=track' (Lednice), but only if 'tracktype!=grade3'
            if ((xmlStrcmp(kStr, (const xmlChar *) "highway") == 0) && 
                (xmlStrcmp(vStr, (const xmlChar *) "track") == 0)) {
                accept_way = 2;
            }
            // accept also 'highway=service' (ISTRO_MAP_BA_FEISTU)
            if ((xmlStrcmp(kStr, (const xmlChar *) "highway") == 0) && 
                (xmlStrcmp(vStr, (const xmlChar *) "service") == 0)) {
                accept_way = 3;
            }
            if ((xmlStrcmp(kStr, (const xmlChar *) "tracktype") == 0) && 
                (xmlStrcmp(vStr, (const xmlChar *) "grade3") == 0)) {
                track_grade3 = 1;
            }
#ifdef ISTRO_MAP_MLAZNE_HAMRNIKY
            // pre RoboOrienteering aj kriky dame do mapy <tag k='natural' v='scrub'/>
            if ((xmlStrcmp(kStr, (const xmlChar *) "natural") == 0) && 
                (xmlStrcmp(vStr, (const xmlChar *) "scrub") == 0)) {
                accept_way = 1;
            }
            // pre RoboOrienteering aj hranice parku dame do mapy <tag k='landuse' v='greenfield'/>
            if ((xmlStrcmp(kStr, (const xmlChar *) "landuse") == 0) && 
                (xmlStrcmp(vStr, (const xmlChar *) "greenfield") == 0)) {
                accept_way = 1;
            }
#endif 

            LOGM_DEBUG(loggerNavMap, "navmap_load_osmway", "way_id=" << way_id << ", type=\"tag\""
                << ", k=\"" << kStr << "\", v=\"" << vStr << "\", accept_way=" << accept_way);
        }
    }

    // ignore ways with 'tracktype=grade3'
    if ((accept_way == 2) && (track_grade3 == 1)) {
        accept_way = 0;
    }

    if (!accept_way && (navmap_segment_cnt != old_segment_cnt)) {
        LOGM_DEBUG(loggerNavMap, "navmap_load_osmway", "msg=\"not a footway - ignoring all way segments!\", way_id=" << way_id
            << ", segment_cnt=" << navmap_segment_cnt - old_segment_cnt << ", old_segment_cnt=" << old_segment_cnt);
        navmap_segment_cnt = old_segment_cnt;
    }

    return 0;
}

int navmap_load_osm(xmlNode * root_node)
/*
  <osm version='0.6' generator='JOSM'>
    <node id='-56028' action='modify' lat='48.15664036493' lon='17.15802927383' />
    <way id='-55737' action='modify'>...</way>
  </osm>
*/
{
    xmlNode *node = NULL;

    if (xmlStrcmp(root_node->name, (const xmlChar *) "osm") != 0) { 
        LOGM_ERROR(loggerNavMap, "navmap_load_osm", "msg=\"error: Root Node is not OSM!\"");
        return -1;
    }

    for (node = root_node->children; node; node = node->next) {
        if (node->type != XML_ELEMENT_NODE) continue;

        if (xmlStrcmp(node->name, (const xmlChar *) "node") == 0) {
            xmlChar *idStr = xmlGetProp(node, (const xmlChar *) "id");
            if (idStr == NULL) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osm", "msg=\"error: Node.Id property is missing!\"");
                continue;
            }
            xmlChar *lonStr = xmlGetProp(node, (const xmlChar *) "lon");
            xmlChar *latStr = xmlGetProp(node, (const xmlChar *) "lat");
            if ((lonStr == NULL) || (latStr == NULL)) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osm", "msg=\"error: Node.Lon/Lat property is missing!\", id=\"" << idStr << "\"");
                continue;
            }

            long long id = strtoll ((const char *)idStr, NULL, 0);
            double lon = strtod ((const char *)lonStr, NULL);
            double lat = strtod ((const char *)latStr, NULL);

            if (id == 0) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osm", "msg=\"error: Node.Id property is zero!\", id=" << id);
                continue;
            }
            for(int i = 0; i < navmap_node_cnt; i++) {
                if (navMapNode[i].node_id == id) {
                    LOGM_ERROR(loggerNavMap, "navmap_load_osm", "msg=\"error: duplicate node_id identifier found!\", node_id=" << id)
                    continue;
                }
            }
            if (navmap_node_cnt >= NAVMAP_NODE_NUM) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osm", "msg=\"error: navMapNode[] array is full!\", navmap_node_cnt=" << navmap_node_cnt);
                continue;
            }

            navMapNode[navmap_node_cnt].node_id = id;
            navMapNode[navmap_node_cnt].longitude = lon;
            navMapNode[navmap_node_cnt].latitude = lat;
            navMapNode[navmap_node_cnt].x = 0;
            navMapNode[navmap_node_cnt].y = 0;
            navMapNode[navmap_node_cnt].flags = 0;

            navmap_node_cnt++;

            LOGM_DEBUG(loggerNavMap, "navmap_load_osm", "type=\"node\", cnt=" << navmap_node_cnt << ", id=" << id 
                << ", lat=" << ioff(lat, 8) << ", lon=" << ioff(lon, 8));
        } else
        if (xmlStrcmp(node->name, (const xmlChar *) "way") == 0) {
            xmlChar *idStr = xmlGetProp(node, (const xmlChar *) "id");
            if (idStr == NULL) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osm", "msg=\"error: Way.Id property is missing!\"");
                continue;
            }

            long long id = strtoll ((const char *)idStr, NULL, 0);

            if (id == 0) {
                LOGM_ERROR(loggerNavMap, "navmap_load_osm", "msg=\"error: Way.Id property is zero!\", id=\"" << idStr << "\"");
                continue;
            }

            LOGM_DEBUG(loggerNavMap, "navmap_load_osm", "type=\"way\", id=" << id);

            navmap_load_osmway(node, id);
        }

    }

    return 0;
}

int navmap_load(const string& fname)
{
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL;

    LOGM_INFO(loggerNavMap, "navmap_load", "msg=\"loading started...\"");

    navmap_node_cnt = 0;
    navmap_segment_cnt = 0;

    /* parse the file and get the DOM */
    doc = xmlReadFile(fname.c_str(), NULL, 0);

    if (doc == NULL) {
        LOGM_ERROR(loggerNavMap, "navmap_load", "msg=\"error: could not parse file!\", fname=\""<< fname << "\"");
        return -1;
    }

    /* get the root element node */
    root_element = xmlDocGetRootElement(doc);

    navmap_load_osm(root_element);

    /* free the document */
    xmlFreeDoc(doc);

    /* free the global variables that may have been allocated by the parser */
    xmlCleanupParser();

    navmap_compress();
     
    LOGM_INFO(loggerNavMap, "navmap_load", "msg=\"loading finished...\"" << ", navmap_node_cnt=" << navmap_node_cnt << ", navmap_segment_cnt=" << navmap_segment_cnt);

    return 0;
}

#else

int navmap_load(const string& fname)
{
    LOGM_ERROR(loggerNavMap, "navmap_load", "msg=\"error: compiled without LIBXML2!\"");

    return -1;
}

#endif

void navmap_calcXY(void)
{
    for (int i = 0; i < navmap_node_cnt; i++) {
        navigation_getXY(navMapNode[i].latitude, navMapNode[i].longitude, navMapNode[i].x, navMapNode[i].y);
    }
}

int navmap_init(void)
{
#ifdef ISTRO_LIBXML2
    /* initialize the library and check potential ABI mismatches */
    LIBXML_TEST_VERSION
#endif

    /* load using navmap_data.cpp */
    navmap_import_data();
    navmap_calcXY();

    if ((navmap_node_cnt <= 0) || (navmap_segment_cnt <= 0)) {
        LOGM_ERROR(loggerNavMap, "navmap_init", "msg=\"error: no node/segment loaded!\"" 
            << ", navmap_node_cnt=" << navmap_node_cnt << ", navmap_segment_cnt=" << navmap_segment_cnt);
        return -1;
    }

    LOGM_INFO(loggerNavMap, "navmap_init", "navmap_node_cnt=" << navmap_node_cnt << ", navmap_segment_cnt=" << navmap_segment_cnt);

    navmap_export_kml("out/navmap-export.kml");

    return 0;
}

double navmap_dist2PS_XY(double x, double y, int flags /* = 0 */, int *pidx /* = NULL */, double *pxx /* = NULL */, double *pyy /* = NULL */)
// squared distance between point (x, y) and navigation map segments
{
    int    min_idx;
    double min_dd2, min_xx, min_yy;
    
    double t = timeBegin();
       
    min_idx = -1;
    min_dd2 = 99999999;
    min_xx = min_yy = 0;
    for(int j = 0; j < navmap_segment_cnt; j++) {
        // if flags are specified, then use only segments with these flags
        if ((flags > 0) && ((navMapSegment[j].flags & flags) == 0)) continue;
        
        double x1 = navMapNode[navMapSegment[j].node1_idx].x;
        double y1 = navMapNode[navMapSegment[j].node1_idx].y;
        double x2 = navMapNode[navMapSegment[j].node2_idx].x;
        double y2 = navMapNode[navMapSegment[j].node2_idx].y;
        
        double xx, yy, dd2;
        dd2 = navigation_dist2PL(x, y, x1, y1, x2, y2, &xx, &yy);

        if (dd2 < min_dd2) {
            min_dd2 = dd2;
            min_idx = j;
            min_xx  = xx;
            min_yy  = yy;
        }
    }
    
    if (min_idx < 0) {
        LOGM_ERROR(loggerNavMap, "navmap_dist2PS_XY", "msg=\"error: no segment found!\"" << ", navmap_segment_cnt=" << navmap_segment_cnt); 
    }
    
    if (pidx != NULL)  *pidx = min_idx;
    if (pxx != NULL)  *pxx = min_xx;
    if (pyy != NULL)  *pyy = min_yy;
    
    timeEnd("navmap::navmap_dist2PS", t);
    return min_dd2;
}

double navmap_dist2PS_LL(double lat, double lon, int flags /* = 0 */, int *pidx /* = NULL */, double *pllat /* = NULL */, double *pllon /* = NULL */)
// squared distance between coordinates (lat, lon) and navigation map segments
{
    int idx = -1;
    double x, y, dd2, xx, yy, llon, llat;

    navigation_getXY(lat, lon, x, y);
    dd2 = navmap_dist2PS_XY(x, y, flags, &idx, &xx, &yy);

    if (pidx != NULL)  *pidx = idx;
    
    if ((pllat != NULL) || (pllon != NULL)) {
        navigation_getLL(xx, yy, llat, llon);
        if (pllat != NULL)  *pllat = llat;
        if (pllon != NULL)  *pllon = llon;
    }

    return dd2;
}

double navmap_dist2NN(int node1_idx, int node2_idx)
{
    double dx = navMapNode[node1_idx].x - navMapNode[node2_idx].x;
    double dy = navMapNode[node1_idx].y - navMapNode[node2_idx].y;
    return dx * dx + dy * dy;
}

double navmap_dist2NL(int node_idx, double lat, double lon)
{
    double x, y;
    navigation_getXY(lat, lon, x, y);
    
    double dx = navMapNode[node_idx].x - x;
    double dy = navMapNode[node_idx].y - y;
    return dx * dx + dy * dy;
}

const double NAVMAP_ZERO_EPS = 0.000001;

int navmap_fwdNN_XY(int node1_idx, int node2_idx, double dist, double& x, double& y)
// find a point in direction node1 -> node2, that is dist meters away from node1
{
    x = navMapNode[node1_idx].x;
    y = navMapNode[node1_idx].y;

    double dx = navMapNode[node2_idx].x - x;
    double dy = navMapNode[node2_idx].y - y;
    double dd = sqrt(dx * dx + dy * dy);
    
    // check if segment has zero length - avoid division by zero
    if (dd < NAVMAP_ZERO_EPS) {
        return 1;
    }
    
    x += dx * dist / dd;
    y += dy * dist / dd;
    
    return 0;
}

int navmap_fwdNN_LL(int node1_idx, int node2_idx, double dist, double& lat, double& lon)
// find a point in direction node1 -> node2, that is dist meters away from node1
{
    int res;
    double x, y;
    
    res = navmap_fwdNN_XY(node1_idx, node2_idx, dist, x, y);
    navigation_getLL(x, y, lat, lon);
    
    return res;
}

int navmap_routeFwd(double lat, double lon, double dist, double& lat2, double& lon2)
// find a point on route that is <dist> meters closer to the route end
{
    int sidx = -1;
    double llat,  llon;

    // calculate closest segment on route and corresponding projection point
    navmap_dist2PS_LL(lat, lon, NAVMAP_FLAG_ROUTE, &sidx, &llat, &llon);
    
    if (sidx < 0) {
        lat2 = lat;
        lon2 = lon;
        LOGM_ERROR(loggerNavMap, "navmap_routeFwd", "msg=\"error: no segment found!\"" << ", navmap_segment_cnt=" << navmap_segment_cnt); 
        return -1;
    }

    // which node is first in direction of our route?
    int nidx1 = navMapSegment[sidx].node1_idx;
    int nidx2 = navMapSegment[sidx].node2_idx;
    if ((navMapSegment[sidx].flags & NAVMAP_FLAG_ROUTE_OPDIR) > 0) {
        int i = nidx2;
        nidx2 = nidx1;
        nidx1 = i;
    }
    
    // distance from projection point to first node
    double dd = sqrt(navmap_dist2NL(nidx1, llat,  llon));
    
    dist += dd;
    
    while (sidx >= 0) {
        // calculate segment length
        double seg_dd = sqrt(navmap_dist2NN(nidx1, nidx2));
        
        // is the point on this segment?
        if (dist <= seg_dd) {
            navmap_fwdNN_LL(nidx1, nidx2, dist, lat2, lon2);
            return 0;
        }
    
        dist -= seg_dd;
        lat2 = navMapNode[nidx2].latitude;
        lon2 = navMapNode[nidx2].longitude;
        
        // find next segment on route
        sidx = -1;
        nidx1 = nidx2;
        nidx2 = -1;
        for(int j = 0; j < navmap_segment_cnt; j++) {
            if ((navMapSegment[j].flags & NAVMAP_FLAG_ROUTE) == 0) continue;
            
            if ((navMapSegment[j].flags & NAVMAP_FLAG_ROUTE_OPDIR) == 0) {
                if (navMapSegment[j].node1_idx == nidx1) {
                    sidx = j;
                    nidx2 = navMapSegment[j].node2_idx;
                    break;
                }
            } else {
                if (navMapSegment[j].node2_idx == nidx1) {
                    sidx = j;
                    nidx2 = navMapSegment[j].node1_idx;
                    break;
                }
            }
        }
    }
    
    return 0;
}

double navmap_planRouteNN(int node1_idx, int node2_idx)
{
    int node_used[NAVMAP_NODE_NUM];
    double node_dd[NAVMAP_NODE_NUM];     // distance from node1_idx
    int node_seg_idx[NAVMAP_NODE_NUM];   // shortest route was using this segment 

    for (int i = 0; i < navmap_node_cnt; i++) {
        node_used[i] = 0;
        node_dd[i]   = -1;
        node_seg_idx[i] = -1;
    }

    // clear all NAVMAP_FLAG_ROUTE flags
    for (int j = 0; j < navmap_segment_cnt; j++) {
        if ((navMapSegment[j].flags & NAVMAP_FLAG_ROUTE) > 0) {
            navMapSegment[j].flags &= ~(NAVMAP_FLAG_ROUTE | NAVMAP_FLAG_ROUTE_OPDIR);
        }
    }
    
    node_dd[node1_idx] = 0;

    while (true) {
        
        // find unused node with lowest distance 
        int min_idx = -1;
        double min_dd = -1;
        for (int i = 0; i < navmap_node_cnt; i++) {
            if (node_used[i]) continue;
            if (node_dd[i] < 0) continue;
            
            if ((min_idx < 0) || (node_dd[i] < min_dd)) {
                min_idx = i;
                min_dd  = node_dd[i];
            }
        }
        //LOGM_DEBUG(loggerNavMap, "navmap_planNN", "min_idx=" << min_idx << ", min_dd=" << min_dd);
        
        // no unused node found - error: graph is not connected?
        if (min_idx < 0) {
            LOGM_ERROR(loggerNavMap, "navmap_planNN", "msg=\"error: no route found!\", node1_idx=" << node1_idx << ", node2_idx=" << node2_idx);
            return -1;
        }
        
        // mark node as used
        int n1_idx = min_idx;
        node_used[n1_idx] = 1;
        
        if (n1_idx == node2_idx) {
            // success: shortest route to our destination node was found!
            break;
        }
        
        // find all segments connected to our node
        // fixme: to be optimized
        for (int j = 0; j < navmap_segment_cnt; j++) {
            if ((navMapSegment[j].node1_idx != n1_idx) && (navMapSegment[j].node2_idx != n1_idx)) continue;

            int n2_idx = navMapSegment[j].node1_idx;
            if (n2_idx == n1_idx) {
                n2_idx = navMapSegment[j].node2_idx;
            }

            //LOGM_DEBUG(loggerNavMap, "navmap_planNN", "j=" << j << ", n1_idx=" << n1_idx << ", n2_idx=" << n2_idx);
            
            double dx = navMapNode[n1_idx].x - navMapNode[n2_idx].x;
            double dy = navMapNode[n1_idx].y - navMapNode[n2_idx].y;
            double dd = sqrt(dx * dx + dy * dy);
            
            if ((node_dd[n2_idx] < 0) || (node_dd[n2_idx] > node_dd[n1_idx] + dd)) {
                node_dd[n2_idx] = node_dd[n1_idx] + dd;
                node_seg_idx[n2_idx] = j;  // the shortest route to node2 was by using segment with index j
                
                //LOGM_DEBUG(loggerNavMap, "navmap_planNN", "j=" << j << ", n1_idx=" << n1_idx << ", n2_idx=" << n2_idx 
                //    << ", node_dd[n2_idx] = " << node_dd[n2_idx]);
            }
        }
    }
    
    // prepare output = the path we found
    int n1_idx = node2_idx;
    while (n1_idx != node1_idx) {
        int j = node_seg_idx[n1_idx];
        
        // fixme: do not use flags
        navMapSegment[j].flags |= NAVMAP_FLAG_ROUTE;
        
        int n2_idx = navMapSegment[j].node1_idx;
        if (n2_idx == n1_idx) {
            n2_idx = navMapSegment[j].node2_idx;
            navMapSegment[j].flags |= NAVMAP_FLAG_ROUTE_OPDIR;
        }
        
        n1_idx = n2_idx;
    }
    
    double dd = node_dd[node2_idx];

    LOGM_DEBUG(loggerNavMap, "navmap_planRouteNN", "dd=" << ioff(dd, 1) << ", node1_idx=" << node1_idx << ", node2_idx=" << node2_idx);

    return dd;
}

void navmap_plan_delete(void)
{
    //=== delete old temporary segments and nodes & clear invisible flags
    for (int j = 0; j < navmap_segment_cnt; j++) {
        if ((navMapSegment[j].flags & NAVMAP_FLAG_TMP_PLAN) > 0) {
            navMapSegment[j].flags |= NAVMAP_FLAG_DELETE;
        }
        if ((navMapSegment[j].flags & NAVMAP_FLAG_INVISIBLE) > 0) {
            navMapSegment[j].flags &= ~NAVMAP_FLAG_INVISIBLE;
        }
        if ((navMapSegment[j].flags & NAVMAP_FLAG_ROUTE) > 0) {
            navMapSegment[j].flags &= ~(NAVMAP_FLAG_ROUTE | NAVMAP_FLAG_ROUTE_OPDIR);
        }
    }
    navmap_delete();
}

double navmap_planRouteLL(double lat1, double lon1, double lat2, double lon2)
{
    //=== delete old temporary segments and nodes
    navmap_plan_delete();
    
    //=== calculate projection points ===
    int segment1_idx, segment2_idx;
    double llon1, llat1, llon2, llat2;

    double t = timeBegin();
    
    double dd2A = navmap_dist2PS_LL(lat1, lon1, 0, &segment1_idx, &llat1, &llon1);
    double dd2B = navmap_dist2PS_LL(lat2, lon2, 0, &segment2_idx, &llat2, &llon2);
    
    //=== segments and nodes for START ===
    int node1A_idx, node1B_idx, node12_idx;    /* node11_idx */ 
    int segment1A_idx, segment1B_idx;          /* segment12_idx */

    node1A_idx = navMapSegment[segment1_idx].node1_idx;
    node1B_idx = navMapSegment[segment1_idx].node2_idx;
    
    // add temporal nodes
    /* node11_idx = navmap_node_add(NAVMAP_NODE_P11, lat1, lon1, NAVMAP_FLAG_TMP_PLAN | NAVMAP_FLAG_LEAF); */ 
    node12_idx = navmap_node_add(NAVMAP_NODE_P12, llat1, llon1, NAVMAP_FLAG_TMP_PLAN | NAVMAP_FLAG_CROSSING);
    
    // add temporal segments
    /* segment12_idx = */  /* navmap_segment_add(NAVMAP_SEGMENT_P12, node11_idx, node12_idx, NAVMAP_FLAG_TMP_PLAN); */
    segment1A_idx = navmap_segment_add(NAVMAP_SEGMENT_P1A, node12_idx, node1A_idx, NAVMAP_FLAG_TMP_PLAN);
    segment1B_idx = navmap_segment_add(NAVMAP_SEGMENT_P1B, node12_idx, node1B_idx, NAVMAP_FLAG_TMP_PLAN);
    navMapSegment[segment1_idx].flags |= NAVMAP_FLAG_INVISIBLE;
    
    //=== segments and nodes for STOP ===
    int node2A_idx, node2B_idx, node21_idx, node22_idx;
    int segment2A_idx, segment2B_idx;    /* segment22_idx */

    node2A_idx = navMapSegment[segment2_idx].node1_idx;
    node2B_idx = navMapSegment[segment2_idx].node2_idx;
    
    // add temporal nodes
    node21_idx = navmap_node_add(NAVMAP_NODE_P21, lat2, lon2, NAVMAP_FLAG_TMP_PLAN | NAVMAP_FLAG_LEAF);
    node22_idx = navmap_node_add(NAVMAP_NODE_P22, llat2, llon2, NAVMAP_FLAG_TMP_PLAN | NAVMAP_FLAG_CROSSING);
    
    // add temporal segments
    /* segment22_idx = */  navmap_segment_add(NAVMAP_SEGMENT_P22, node21_idx, node22_idx, NAVMAP_FLAG_TMP_PLAN);
    segment2A_idx = navmap_segment_add(NAVMAP_SEGMENT_P2A, node22_idx, node2A_idx, NAVMAP_FLAG_TMP_PLAN);
    segment2B_idx = navmap_segment_add(NAVMAP_SEGMENT_P2B, node22_idx, node2B_idx, NAVMAP_FLAG_TMP_PLAN);
    navMapSegment[segment2_idx].flags |= NAVMAP_FLAG_INVISIBLE;
    
    //=== connect A to B directly 
    /* int segment3_idx; */

    if (segment1_idx == segment2_idx) {
        /* segment3_idx = */  navmap_segment_add(NAVMAP_SEGMENT_P3, node12_idx, node22_idx, NAVMAP_FLAG_TMP_PLAN);
        /* aby neboli prekryvy pri vykreslovani KML, tak vykreslime iba tie co su blizsie k bodu usecky */
        if (navmap_dist2NN(node12_idx, node1A_idx) > navmap_dist2NN(node22_idx, node2A_idx)) {
            navMapSegment[segment1A_idx].flags |= NAVMAP_FLAG_INVISIBLE;  
        } else {
            navMapSegment[segment2A_idx].flags |= NAVMAP_FLAG_INVISIBLE;
        }
        if (navmap_dist2NN(node12_idx, node1B_idx) > navmap_dist2NN(node22_idx, node2B_idx)) {
            navMapSegment[segment1B_idx].flags |= NAVMAP_FLAG_INVISIBLE;
        } else {
            navMapSegment[segment2B_idx].flags |= NAVMAP_FLAG_INVISIBLE;
        }
    }

    //=== plan route between two existing nodes
    double dd = navmap_planRouteNN(node12_idx, node21_idx);  // (node11_idx, node21_idx)

    timeEnd("navmap::navmap_planRouteLL", t);

    LOGM_DEBUG(loggerNavMap, "navmap_planRouteLL", "dd=" << ioff(dd, 1) << ", lat1=" << ioff(lat1, 8) << ", lon1=" << ioff(lon1, 8)
        << ", lat2=" << ioff(lat2, 8) << ", lon2=" << ioff(lon2, 8) << ", dd2A=" << ioff(dd2A, 1) << ", dd2B=" << ioff(dd2B, 1));

    return dd;
}

void navmap_test(void)
{
#ifdef ISTRO_MAP_LEDNICE_ZZAHRADA
    navmap_load("conf/lednice-zzahrada.osm");
    navmap_export_kml("out/lednice-zzahrada.kml");
    navmap_export_data("out/lednice-zzahrada.cpp");
#endif

#ifdef ISTRO_MAP_BA_PARKAH
    navmap_load("conf/navmap-ba-parkah.osm");
    navmap_export_kml("out/navmap-ba-parkah.kml");
    navmap_export_data("out/navmap-ba-parkah.cpp");
#endif

#ifdef ISTRO_MAP_BA_HRADZA
    navmap_load("conf/navmap-ba-hradza.osm");
    navmap_export_kml("out/navmap-ba-hradza.kml");
    navmap_export_data("out/navmap-ba-hradza.cpp");
#endif

#if defined(ISTRO_MAP_BA_FEISTU) || defined(ISTRO_MAP_BA_FEISTU2)
    navmap_load("conf/navmap-ba-feistu.osm");
    navmap_export_kml("out/navmap-ba-feistu.kml");
    navmap_export_data("out/navmap-ba-feistu.cpp");
#endif

#if defined(ISTRO_MAP_PISEK_PALSADY) || defined(ISTRO_MAP_PISEK_PALSADY2)
    navmap_load("conf/navmap-pisek-palsady.osm");
    navmap_export_kml("out/navmap-pisek-palsady.kml");
    navmap_export_data("out/navmap-pisek-palsady.cpp");
#endif

#ifdef ISTRO_MAP_MLAZNE_HAMRNIKY
//  navmap_load("conf/navmap-mlazne_hamrniky.osm");
//  navmap_export_kml("out/navmap-mlazne_hamrniky.kml");
//  navmap_export_data("out/navmap-mlazne_hamrniky.cpp");
    navmap_load("conf/navmap-mlazne_marathon.osm");
    navmap_export_kml("out/navmap-mlazne_marathon.kml");
    navmap_export_data("out/navmap-mlazne_marathon.cpp");
#endif 

#ifdef ISTRO_MAP_BA_SADJK
    navmap_load("conf/navmap-ba-sadjk.osm");
    navmap_export_kml("out/navmap-ba-sadjk.kml");
    navmap_export_data("out/navmap-ba-sadjk.cpp");
#endif
}

/*
void navmap_test(void) 
// test funkcie navmap_dist2PS_LL() so zakreslenim projekcnych bodov do kml
{
    // load from OSM file and export data
    //navmap_load("conf/navmap-ba-parkah.osm");
    //navmap_export_data("out/navmap-export.cpp");

    // auxiliary points
    int i = 0, idx = 0;
    double dd2, lon, lat;
    aux_point_t auxpt[3+3+1];
    
    strcpy(auxpt[i].name, "X1");
    sprintf(auxpt[i].desc, "xy");
    strcpy(auxpt[i].style, "style4");
    auxpt[i].longitude = (17.1592887 + 2*17.1593964) / 3;    // N3+S3
    auxpt[i].latitude  = (48.1570191 + 2*48.1564496) / 3;
    
    dd2 = navmap_dist2PS_LL(auxpt[i].latitude, auxpt[i].longitude, 0, &idx, &lat, &lon);
    navMapSegment[idx].flags |= NAVMAP_FLAG_STYLE2;

    //double dd3 = navigation_dist2PL(x,  y, 
    //    navMapNode[navMapSegment[idx].node1_idx].x, navMapNode[navMapSegment[idx].node1_idx].y, 
    //    navMapNode[navMapSegment[idx].node2_idx].x, navMapNode[navMapSegment[idx].node2_idx].y);
    
    strcpy(auxpt[++i].name, "X1p");
    sprintf(auxpt[i].desc, "idx=%d, dd2=%0.5f", idx, dd2);
    strcpy(auxpt[i].style, "style4");
    auxpt[i].longitude = lon;
    auxpt[i].latitude  = lat;
    
    //strcpy(auxpt[++i].name, "X1pA");
    //sprintf(auxpt[i].desc, "x=%0.5f, y=%0.5f", navMapNode[navMapSegment[idx].node1_idx].x, navMapNode[navMapSegment[idx].node1_idx].y);
    //strcpy(auxpt[i].style, "style4");
    //auxpt[i].longitude = navMapNode[navMapSegment[idx].node1_idx].longitude;
    //auxpt[i].latitude  = navMapNode[navMapSegment[idx].node1_idx].latitude;
    
    //strcpy(auxpt[++i].name, "X1pB");
    //sprintf(auxpt[i].desc, "x=%0.5f, y=%0.5f", navMapNode[navMapSegment[idx].node2_idx].x, navMapNode[navMapSegment[idx].node2_idx].y);
    //strcpy(auxpt[i].style, "style4");
    //auxpt[i].longitude = navMapNode[navMapSegment[idx].node2_idx].longitude;
    //auxpt[i].latitude  = navMapNode[navMapSegment[idx].node2_idx].latitude;
      
    strcpy(auxpt[++i].name, "X2");
    sprintf(auxpt[i].desc, "xy");
    strcpy(auxpt[i].style, "style4");
    auxpt[i].longitude = (17.1592887 + 2*17.1554731) / 3;    // S1+N3
    auxpt[i].latitude  = (48.1570191 + 2*48.1562979) / 3;

    dd2 = navmap_dist2PS_LL(auxpt[i].latitude, auxpt[i].longitude, 0, &idx, &lat, &lon);
    navMapSegment[idx].flags |= NAVMAP_FLAG_STYLE2;

    strcpy(auxpt[++i].name, "X2p");
    sprintf(auxpt[i].desc, "idx=%d, dd2=%0.5f", idx, dd2);
    strcpy(auxpt[i].style, "style4");
    auxpt[i].longitude = lon;
    auxpt[i].latitude  = lat;

    strcpy(auxpt[++i].name, "X3");
    sprintf(auxpt[i].desc, "xy");
    strcpy(auxpt[i].style, "style4");
    auxpt[i].longitude = (17.1592887 + 17.1616555 + 17.1593964) / 3;    // N3+N2+S3
    auxpt[i].latitude  = (48.1570191 + 48.1572591 + 48.1564496) / 3;

    dd2 = navmap_dist2PS_LL(auxpt[i].latitude, auxpt[i].longitude, 0, &idx, &lat, &lon);
    navMapSegment[idx].flags |= NAVMAP_FLAG_STYLE2;

    strcpy(auxpt[++i].name, "X3p");
    sprintf(auxpt[i].desc, "idx=%d, dd2=%0.5f", idx, dd2);
    strcpy(auxpt[i].style, "style4");
    auxpt[i].longitude = lon;
    auxpt[i].latitude  = lat;

    auxpt[++i].style[0] = 0;

    navmap_export_kml("out/navmap-export2.kml", auxpt);
}
*/

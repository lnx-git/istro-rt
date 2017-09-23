#include <math.h>
#include <sstream>
#include "dmap.h"
#include "logger.h"

LOG_DEFINE(loggerDegreeMap, "DegreeMap");

DegreeMap::DegreeMap()
{
    init();
}

void DegreeMap::init(void)
{
    set(-1, -1, -1);
}

void DegreeMap::set(int value, int dd, int md)
{
    for(int i = 0; i < DEGREE_MAP_COUNT; i++) {
        dmap[i] = value;
        dist[i] = dd;
        maxd[i] = md;
    }
}

void DegreeMap::fill(float angle1, float angle2, int value, int dd, int md)
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
        // always write new value
        if (dmap[i] < 0) {
            dmap[i] = value;
            dist[i] = dd;
            maxd[i] = md;
        } else
        // 1 could be overwritten with 0, but 0 will never be overwritten by 1
        if ((value == 0) && (dmap[i] > 0)) {
            dmap[i] = 0;
            dist[i] = dd;
            maxd[i] = md;
        } else
        if ((value == dmap[i]) && (dd >= 0) && (dist[i] > dd)) {  // store the minimum distance (in case of quality issues)
            dist[i] = dd;
            maxd[i] = md;
        }
    }
}

void DegreeMap::finish(void)
{
    //dmap_print(p, "dmap_finish.before");

    // fill all -1 at the beginning with the first value >= 0
    int i1 = 0;
    while (i1 < DEGREE_MAP_COUNT) {
        if (dmap[i1] < 0) {
            i1++;
            continue;
        }
        break;
    }

    if (i1 < DEGREE_MAP_COUNT) {
        for(int i = 0; i < i1; i++) {
            dmap[i] = dmap[i1];
            dist[i] = dist[i1];
            maxd[i] = maxd[i1];
        }
    }
    
    // fill all remaining -1 with last found value
    int last = dmap[0];
    int lastd = dist[0];
    int lastm = maxd[0];
    for(int i = 1; i < DEGREE_MAP_COUNT; i++) {
        if (dmap[i] < 0) {
            dmap[i] = last;
            dist[i] = lastd;
            maxd[i] = lastm;
        } else {
            last = dmap[i];
            lastd = dist[i];
            lastm = maxd[i];
        }
    }
    
    //dmap_print(p, "dmap_finish.after");
}

void DegreeMap::apply(const DegreeMap &d2)
{
    for(int i = 0; i < DEGREE_MAP_COUNT; i++) {
        // always write new value
        if (dmap[i] < 0) {
            dmap[i] = d2.dmap[i];
            dist[i] = d2.dist[i];
            maxd[i] = d2.maxd[i];
        } else
        // 1 could be overwritten with 0, but 0 will never be overwritten by 1
        if ((d2.dmap[i] == 0) && (dmap[i] > 0)) {
            dmap[i] = 0;
            dist[i] = d2.dist[i];
            maxd[i] = d2.maxd[i];
        } else
        if ((d2.dmap[i] == dmap[i]) && (d2.dist[i] >= 0) && (dist[i] > d2.dist[i])) {    // store the minimum distance
            dist[i] = d2.dist[i];
            maxd[i] = d2.maxd[i];
        }
    }
}

void DegreeMap::shrink(float mult)
{
    for(int i = 0; i < DEGREE_MAP_COUNT; i++) {
        if (dist[i] > 0) {
            dist[i] = round(mult * dist[i]);
        }
        if (maxd[i] > 0) {
            maxd[i] = round(mult * maxd[i]);
        }
    }
}

void DegreeMap::find(int minlen, int mid, int &idx1, int &idx2)
{
    int best_i1 = -1;
    int best_i2 = -1;
    int best_pos = DEGREE_MAP_COUNT;

    int i1, i2, pos, pos2;
    
    i1 = 0; 
    while (i1 < DEGREE_MAP_COUNT) {
        if (dmap[i1] == 0) {
            i1++;
            continue;
        }
        // we found start of interval of "1" -> i1
        i2 = i1 + 1;
        while ((i2 < DEGREE_MAP_COUNT) && (dmap[i2] == 1)) {
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

void DegreeMap::print(const std::string &str)
{
    char ss[DEGREE_MAP_COUNT + 1];

    for(int i = 0; i < DEGREE_MAP_COUNT; i++) {
        ss[i] = (char)('0' + dmap[i]);
    }
    ss[DEGREE_MAP_COUNT] = 0;

    LOGM_TRACE(loggerDegreeMap, "print", "m=\"" << str << "\", dmap=[" << ss << "]");    

    std::stringstream ss2;
    
    for(int j = 0; j < DEGREE_MAP_COUNT; j++) { 
        ss2 << iozf(dist[j], 3) << ",";
    }

    LOGM_TRACE(loggerDegreeMap, "print", "m=\"" << str << "\", dist=[" << ss2.str() << "]");    

    std::stringstream ss3;

    for(int j = 0; j < DEGREE_MAP_COUNT; j++) { 
        ss3 << iozf(maxd[j], 3) << ",";
    }

    LOGM_TRACE(loggerDegreeMap, "print", "m=\"" << str << "\", maxd=[" << ss3.str() << "]");    
}

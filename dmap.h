#ifndef __DMAP_H__
#define __DMAP_H__

#include <string>

const int DEGREE_MAP_COUNT = 181;

class DegreeMap {
public:
    int dmap[DEGREE_MAP_COUNT];  // map of values (1 = free, 0 = obstacle, -1 = NA) for every angle 0..180 degrees, 0=right, 90=in front, 180=left (counterclockwise)
    int dist[DEGREE_MAP_COUNT];  // distance to obstacle (in centimeters), <0 =  no obstacle
    int maxd[DEGREE_MAP_COUNT];  // max distance where the obstacles could be detected (in centimeters)

public:
    DegreeMap();

public:
    void init(void);
    void set(int value, int dd, int md);
    void fill(float angle1, float angle2, int value, int dd, int md);
    void finish(void);
    void apply(const DegreeMap& d2);  // p = p <and> p2
    void shrink(float mult);
    void find(int minlen, int mid, int &idx1, int &idx2);
    void print(const std::string& str);
};

#endif

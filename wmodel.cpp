#include <stdio.h>
#include <math.h>
#include <iostream>
#include "wmodel.h"
#include "mtime.h"
#include "dmap.h"
#include "logger.h"
#include "config.h"    // ANGLE_NONE
#include "navig.h"

using namespace std;

LOG_DEFINE(loggerWorldModel, "WorldModel");

const float WMGRID_DISTANCE_DMAP  = 100;    // obstacle avoidance distance (in centimeters)

const float WMGRID_ZERO_EPS = 0.000001;
const int   WMGRID_INT_MULT = 100000;
const int   WMGRID_INT_HALF =  50000;

WMGrid::WMGrid()
{
    init();
}

void WMGrid::init(void)
{
    grid_x0 = 0;
    grid_y0 = 0;
    clear();
}

void WMGrid::clear(void)
{
    memset(g, 0, sizeof(g[0][0]) * WMGRID_HEIGHT * WMGRID_WIDTH);
}

unsigned char WMGrid::get(int gy, int gx)
{
    if ((gx < 0) || (gy < 0) || (gx >= WMGRID_WIDTH) || (gy >= WMGRID_HEIGHT)) {
        LOGM_ERROR(loggerWorldModel, "WMGrid_get", "msg=\"error: index out of bounds!\", gx=" << gx << ", gy=" << gy);    
        return 0;
    }
    return g[gy][gx];
}

void WMGrid::set(int gy, int gx, unsigned char b)
{
    if ((gx < 0) || (gy < 0) || (gx >= WMGRID_WIDTH) || (gy >= WMGRID_HEIGHT)) {
        LOGM_ERROR(loggerWorldModel, "WMGrid_set", "msg=\"error: index out of bounds!\", gx=" << gx << ", gy=" << gy);    
        return;
    }
    g[gy][gx] = b;
}

void calcg2x(int gx, int gy, int &gx2b, int &gy2b, int &gx2c, int &gy2c, int ddy, int dd, int dgx, int dgy) 
{
    if (dgy > 0) {
        ddy += dd;
        gx2b = gx + dgx;
        if (ddy >= WMGRID_INT_HALF) {
            ddy -= WMGRID_INT_MULT;
            gy2b = gy + 1;
        } else {
            gy2b = gy;
        }
        ddy += dd;
        gx2c = gx2b + dgx;
        if (ddy >= WMGRID_INT_HALF) {
            ddy -= WMGRID_INT_MULT;
            gy2c = gy2b + 1;
        } else {
            gy2c = gy2b;
        }
    } else {
        ddy -= dd;
        gx2b = gx + dgx;
        if (ddy <= -WMGRID_INT_HALF) {
            ddy += WMGRID_INT_MULT;
            gy2b = gy - 1;
        } else {
            gy2b = gy;
        }
        ddy -= dd;
        gx2c = gx2b + dgx;
        if (ddy <= -WMGRID_INT_HALF) {
            ddy += WMGRID_INT_MULT;
            gy2c = gy2b - 1;
        } else {
            gy2c = gy2b;
        }            
    }
    /* LOGM_TRACE(loggerWorldModel, "WMGrid_update", "msg=\"set4\"" << ", dgy=" << dgy
        << ", gx2b=" << gx2b << ", gy2b=" << gy2b << ", gx2c=" << gx2c << ", gy2c=" << gy2c); */
}

void calcg2y(int gx, int gy, int &gx2b, int &gy2b, int &gx2c, int &gy2c, int ddx, int dd, int dgx, int dgy) 
{
    if (dgx > 0) {
        ddx += dd;
        gy2b = gy + dgy;
        if (ddx >= WMGRID_INT_HALF) {
            ddx -= WMGRID_INT_MULT;
            gx2b = gx + 1;
        } else {
            gx2b = gx;
        }
        ddx += dd;
        gy2c = gy2b + dgy;
        if (ddx >= WMGRID_INT_HALF) {
            ddx -= WMGRID_INT_MULT;
            gx2c = gx2b + 1;
        } else {
            gx2c = gx2b;
        }
    } else {
        ddx -= dd;
        gy2b = gy + dgy;
        if (ddx <= -WMGRID_INT_HALF) {
            ddx += WMGRID_INT_MULT;
            gx2b = gx - 1;
        } else {
            gx2b = gx;
        }
        ddx -= dd;
        gy2c = gy2b + dgy;
        if (ddx <= -WMGRID_INT_HALF) {
            ddx += WMGRID_INT_MULT;
            gx2c = gx2b - 1;
        } else {
            gx2c = gx2b;
        }            
    }
    /* LOGM_TRACE(loggerWorldModel, "WMGrid_update", "msg=\"set5\"" << ", dgx=" << dgx 
        << ", gx2b=" << gx2b << ", gy2b=" << gy2b << ", gx2c=" << gx2c << ", gy2c=" << gy2c); */
}

int WMGrid::recenter(float x0, float y0)
{
    grid_x0 = x0;
    grid_y0 = y0;
    // fixme
    clear();
    LOGM_TRACE(loggerWorldModel, "WMGrid_recenter", ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2));    
    return 0;
}

int WMGrid::update(const DegreeMap& dmap, float x0, float y0, float alfa, unsigned char mb, unsigned char vb)
{
    double t = timeBegin();
    int vb_cnt = 0;
    int vb_gx[6 * DEGREE_MAP_COUNT], vb_gy[6 * DEGREE_MAP_COUNT];    // vb_gx[3 * DEGREE_MAP_COUNT], vb_gy[3 * DEGREE_MAP_COUNT];
    
    if (((x0 - grid_x0) < WMGRID_X_MIN) || ((x0 - grid_x0) > WMGRID_X_MAX) || 
        ((y0 - grid_y0) < WMGRID_Y_MIN) || ((y0 - grid_y0) > WMGRID_Y_MAX)) {
        recenter(x0, y0);
    }

    float a = (alfa + 90) * M_PI / 180.0;
    const float da = M_PI / (DEGREE_MAP_COUNT - 1);    // po 1 stupni
    unsigned char vbn = ~vb;

    int gx0 = WMGRID_CENTER_IDX + round((x0 - grid_x0) / WMGRID_CELL_DX);
    int gy0 = WMGRID_CENTER_IDY + round((y0 - grid_y0) / WMGRID_CELL_DY);

//  int last_gx2 = -1;  int last_gy2 = -1;  int lastv = -1;  int lasti = -1;
    int ddy0 = (int)((((y0 - grid_y0) / WMGRID_CELL_DY) - round((y0 - grid_y0) / WMGRID_CELL_DY)) * WMGRID_INT_MULT);
    int ddx0 = (int)((((x0 - grid_x0) / WMGRID_CELL_DX) - round((x0 - grid_x0) / WMGRID_CELL_DX)) * WMGRID_INT_MULT);
            
    for(int i = 0; i < DEGREE_MAP_COUNT; i++, a -= da) {
        if (dmap.dmap[i] < 0) continue;
        
        int v;
        float d;
        if (dmap.dist[i] >= 0) {
            v = 0;  // obstacle detected at distance "dist"
            d = dmap.dist[i] / 100.0;  // centimetres -> metres
        } else {
            v = 1;  // no obstacle detected at max distance "maxd"
            d = dmap.maxd[i] / 100.0;  // centimetres -> metres
        }
        if ((v < 0) || (d < 0)) continue;  // "d" not initialized
        if ((d - WMGRID_ZERO_EPS) > WMGRID_DMAP_MAX_DIST) {
            v = 1;  // no obstacle
            d = WMGRID_DMAP_MAX_DIST;
        }
        float x2 = x0 + d * sin(a);
        float y2 = y0 + d * cos(a);
                    
        int gx = gx0;
        int gy = gy0;
        int gx2 = WMGRID_CENTER_IDX + round((x2 - grid_x0) / WMGRID_CELL_DX);
        int gy2 = WMGRID_CENTER_IDY + round((y2 - grid_y0) / WMGRID_CELL_DY);
        int gx2b, gy2b, gx2c, gy2c;

/*      // speedup: dont draw the "same" line twice - is causing empty cells => not used(cant skip any lines/degrees)
        if ((last_gx2 == gx2) && (last_gy2 == gy2) && (lastv == v) && (i <= lasti + 1)) {
//          LOGM_TRACE(loggerWorldModel, "WMGrid_update", "msg=\"skip line\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
//              << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2
//              << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2) << ", d=" << ioff(d , 2));
            continue;
        }
        last_gx2 = gx2;  last_gy2 = gy2;  lastv = v; lasti = i;
*/        
        
        // is there an obstacle? add to obstacle list
        if (v == 0) {
            vb_gx[vb_cnt] = gx2;
            vb_gy[vb_cnt] = gy2;
            vb_cnt++;
        }

        float dx = x2 - x0;
        float dy = y2 - y0;
        float dx_abs = (dx >= 0)?(dx):(-dx);
        float dy_abs = (dy >= 0)?(dy):(-dy);

        if (dx_abs >= dy_abs) {
            // X is main axis for drawing line
            int dgx = (dx >= 0)?(+1):(-1);

            int dd;
            if (dx_abs > WMGRID_ZERO_EPS) {    // avoid division by zero
                dd = (int)((dy_abs / dx_abs) * WMGRID_INT_MULT);          // assert: WMGRID_CELL_DX == WMGRID_CELL_DY
            } else {
                dd = 0;  
            }
            int ddy = ddy0;
            if (dy >= 0) {
                for(; gx != gx2; gx += dgx) {
                    /* LOGM_TRACE(loggerWorldModel, "WMGrid_update", "msg=\"set1a\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                        << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                        << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2) << ", d=" << ioff(d , 2) 
                        << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2) 
                        << ", ddy=" << ddy << ", dd=" << dd << ", dgx=" << dgx); */
                    set(gy, gx, (get(gy, gx) | mb) & vbn);  // g[gy][gx] = (g[gy][gx] | mb) & vbn;
                    ddy += dd;
                    if (ddy >= WMGRID_INT_HALF) {
                        ddy -= WMGRID_INT_MULT;
                        gy++;
                    }
                }
                // calc 2 points after the endpoint
                if (v == 0) {
                    calcg2x(gx, gy, gx2b, gy2b, gx2c, gy2c, ddy, dd, dgx, +1);
                }
            } else {
                for(; gx != gx2; gx += dgx) {
                    /* LOGM_TRACE(loggerWorldModel, "WMGrid_update", "msg=\"set1b\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                        << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                        << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2) << ", d=" << ioff(d , 2) 
                        << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2) 
                        << ", ddy=" << ddy << ", dd=" << dd << ", dgx=" << dgx); */
                    set(gy, gx, (get(gy, gx) | mb) & vbn);  // g[gy][gx] = (g[gy][gx] | mb) & vbn;
                    ddy -= dd;
                    if (ddy <= -WMGRID_INT_HALF) {
                        ddy += WMGRID_INT_MULT;
                        gy--;
                    }
                }
                // calc 2 points after the endpoint
                if (v == 0) {
                    calcg2x(gx, gy, gx2b, gy2b, gx2c, gy2c, ddy, dd, dgx, -1);
                }
            }
        } else {
            // Y is main axis for drawing line
            int dgy = (dy >= 0)?(+1):(-1);

            int dd;
            if (dy_abs > WMGRID_ZERO_EPS) {    // avoid division by zero
                dd = (int)((dx_abs / dy_abs) * WMGRID_INT_MULT);          // assert: WMGRID_CELL_DX == WMGRID_CELL_DY
            } else {
                dd = 0;  
            }
            int ddx = ddx0;
            if (dx >= 0) {
                for(; gy != gy2; gy += dgy) {
                    /* LOGM_TRACE(loggerWorldModel, "WMGrid_update", "msg=\"set2a\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                        << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                        << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2) << ", d=" << ioff(d , 2) 
                        << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2) 
                        << ", ddx=" << ddx << ", dd=" << dd << ", dgy=" << dgy); */
                    set(gy, gx, (get(gy, gx) | mb) & vbn);  // g[gy][gx] = (g[gy][gx] | mb) & vbn;
                    ddx += dd;
                    if (ddx >= WMGRID_INT_HALF) {
                        ddx -= WMGRID_INT_MULT;
                        gx++;
                    }
                }
                // calc 2 points after the endpoint
                if (v == 0) {
                    calcg2y(gx, gy, gx2b, gy2b, gx2c, gy2c, ddx, dd, +1, dgy);
                }
            } else {
                for(; gy != gy2; gy += dgy) {
                    /* LOGM_TRACE(loggerWorldModel, "WMGrid::update", "msg=\"set2b\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                        << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                        << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2) << ", d=" << ioff(d , 2) 
                        << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2) 
                        << ", ddx=" << ddx << ", dd=" << dd << ", dgy=" << dgy); */
                    set(gy, gx, (get(gy, gx) | mb) & vbn);  // g[gy][gx] = (g[gy][gx] | mb) & vbn;
                    ddx -= dd;
                    if (ddx <= -WMGRID_INT_HALF) {
                        ddx += WMGRID_INT_MULT;
                        gx--;
                    }
                }
                // calc 2 points after the endpoint
                if (v == 0) {
                    calcg2y(gx, gy, gx2b, gy2b, gx2c, gy2c, ddx, dd, -1, dgy);
                }
            }
        }
        // no obstacle - last point should also be cleared
        if (v > 0) {
            set(gy, gx, (get(gy, gx) | mb) & vbn);  // g[gy][gx] = (g[gy][gx] | mb) & vbn;
        }
        if ((gx != gx2) || (gy != gy2)) {
            if (v > 0) {
                set(gy2, gx2, (get(gy2, gx2) | mb) & vbn);  // g[gy2][gx2] = (g[gy2][gx2] | mb) & vbn;    // quick&dirty fix
            }
            /* LOGM_TRACE(loggerWorldModel, "WMGrid_update", "msg=\"end-point error\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2) << ", d=" << ioff(d , 2) 
                << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2)); */
        }
        if (v == 0) {
            vb_gx[vb_cnt] = gx2b;
            vb_gy[vb_cnt] = gy2b;
            vb_cnt++;
            vb_gx[vb_cnt] = gx2c;
            vb_gy[vb_cnt] = gy2c;
            vb_cnt++;
        }
    }
    
    // apply all information about obstacles at the end (to avoid overwrites from white space)
    for(int i = 0; i < vb_cnt; i++) {
        /* LOGM_DEBUG(loggerWorldModel, "WMGrid_update", "msg=\"set3\"" << ", i=" << i << ", gx=" << vb_gx[i] << ", gy=" << vb_gy[i] << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)); */
        set(vb_gy[i], vb_gx[i], (get(vb_gy[i], vb_gx[i]) | mb) | vb);  // g[vb_gy[i]][vb_gx[i]] = (g[vb_gy[i]][vb_gx[i]] | mb) | vb;
    }
    
    timeEnd("WMGrid::update", t);

    return 1;
}

int WMGrid::eval(float x0, float y0, float alfa, DegreeMap& dmap)
{
    double t = timeBegin();

    dmap.init();
    if (((x0 - grid_x0) < WMGRID_X_MIN) || ((x0 - grid_x0) > WMGRID_X_MAX) || 
        ((y0 - grid_y0) < WMGRID_Y_MIN) || ((y0 - grid_y0) > WMGRID_Y_MAX)) {
        return -1;        
    }

    float a = (alfa + 90) * M_PI / 180.0;
    const float da = M_PI / (DEGREE_MAP_COUNT - 1);    // po 1 stupni

    int gx0 = WMGRID_CENTER_IDX + round((x0 - grid_x0) / WMGRID_CELL_DX);
    int gy0 = WMGRID_CENTER_IDY + round((y0 - grid_y0) / WMGRID_CELL_DY);

    int ddy0 = (int)((((y0 - grid_y0) / WMGRID_CELL_DY) - round((y0 - grid_y0) / WMGRID_CELL_DY)) * WMGRID_INT_MULT);
    int ddx0 = (int)((((x0 - grid_x0) / WMGRID_CELL_DX) - round((x0 - grid_x0) / WMGRID_CELL_DX)) * WMGRID_INT_MULT);

    for(int i = 0; i < DEGREE_MAP_COUNT; i++, a -= da) {
        int v = 1;  // no obstacle
        int dist = 0;

        float x2 = x0 + WMGRID_DMAP_MAX_DIST * sin(a);
        float y2 = y0 + WMGRID_DMAP_MAX_DIST * cos(a);
                    
        int gx = gx0;
        int gy = gy0;
        int gx2 = WMGRID_CENTER_IDX + round((x2 - grid_x0) / WMGRID_CELL_DX);
        int gy2 = WMGRID_CENTER_IDY + round((y2 - grid_y0) / WMGRID_CELL_DY);

        float dx = x2 - x0;
        float dy = y2 - y0;
        float dx_abs = (dx >= 0)?(dx):(-dx);
        float dy_abs = (dy >= 0)?(dy):(-dy);

        if (dx_abs >= dy_abs) {
            // X is main axis for drawing line
            int dgx = (dx >= 0)?(+1):(-1);

            int dd;
            int ddi;
            if (dx_abs > WMGRID_ZERO_EPS) {    // avoid division by zero
                dd = (int)((dy_abs / dx_abs) * WMGRID_INT_MULT);          // assert: WMGRID_CELL_DX == WMGRID_CELL_DY
                ddi = (int)(WMGRID_DMAP_MAX_DIST * WMGRID_INT_MULT / (dx_abs / WMGRID_CELL_DX));  // ()/(gx2 - gx)
            } else {
                dd = 0;  
                ddi = 0;
            }
            int ddy = ddy0;
            if (dy >= 0) {
                for(; gx != gx2; gx += dgx, dist += ddi) {
                    /* LOGM_TRACE(loggerWorldModel, "WMGrid_eval", "msg=\"eval1a\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                        << ", dist=" << dist << ", ddi=" << ddi
                        << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                        << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2)
                        << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2) 
                        << ", ddy=" << ddy << ", dd=" << dd << ", dgx=" << dgx); */
                    if ((get(gy, gx) & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) > 0) {
                        v = 0;  // obstacle detected at distance "dist"
                        break;
                    }
                    ddy += dd;
                    if (ddy >= WMGRID_INT_HALF) {
                        ddy -= WMGRID_INT_MULT;
                        gy++;
                    }
                }
            } else {
                for(; gx != gx2; gx += dgx, dist += ddi) {
                    /* LOGM_TRACE(loggerWorldModel, "WMGrid_eval", "msg=\"eval1b\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                        << ", dist=" << dist << ", ddi=" << ddi
                        << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                        << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2)
                        << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2) 
                        << ", ddy=" << ddy << ", dd=" << dd << ", dgx=" << dgx); */
                    if ((get(gy, gx) & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) > 0) {
                        v = 0;  // obstacle detected at distance "dist"
                        break;
                    }
                    ddy -= dd;
                    if (ddy <= -WMGRID_INT_HALF) {
                        ddy += WMGRID_INT_MULT;
                        gy--;
                    }
                }
            }
        } else {
            // Y is main axis for drawing line
            int dgy = (dy >= 0)?(+1):(-1);

            int dd;
            int ddi;
            if (dy_abs > WMGRID_ZERO_EPS) {    // avoid division by zero
                dd = (int)((dx_abs / dy_abs) * WMGRID_INT_MULT);          // assert: WMGRID_CELL_DX == WMGRID_CELL_DY
                ddi = (int)(WMGRID_DMAP_MAX_DIST * WMGRID_INT_MULT / (dy_abs / WMGRID_CELL_DY));  // ()/(gy2 - gy)
            } else {
                dd = 0;  
                ddi = 0;
            }
            int ddx = ddx0;
            if (dx >= 0) {
                for(; gy != gy2; gy += dgy, dist += ddi) {
                    /* LOGM_TRACE(loggerWorldModel, "WMGrid_eval", "msg=\"eval2a\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                        << ", dist=" << dist << ", ddi=" << ddi
                        << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                        << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2)
                        << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2) 
                        << ", ddx=" << ddx << ", dd=" << dd << ", dgy=" << dgy); */
                    if ((get(gy, gx) & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) > 0) {
                        v = 0;  // obstacle detected at distance "dist"
                        break;
                    }
                    ddx += dd;
                    if (ddx >= WMGRID_INT_HALF) {
                        ddx -= WMGRID_INT_MULT;
                        gx++;
                    }
                }
            } else {
                for(; gy != gy2; gy += dgy, dist += ddi) {
                    /* LOGM_TRACE(loggerWorldModel, "WMGrid_eval", "msg=\"eval2b\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                        << ", dist=" << dist << ", ddi=" << ddi
                        << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                        << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2)
                        << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2) 
                        << ", ddx=" << ddx << ", dd=" << dd << ", dgy=" << dgy); */
                    if ((get(gy, gx) & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) > 0) {
                        v = 0;  // obstacle detected at distance "dist"
                        break;
                    }
                    ddx -= dd;
                    if (ddx <= -WMGRID_INT_HALF) {
                        ddx += WMGRID_INT_MULT;
                        gx--;
                    }
                }
            }
        }
        // check endpoint
        if (v > 0) {
            if ((get(gy2, gx2) & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) > 0) {
                v = 0;  // obstacle detected at distance "dist"
                dist = (int)(WMGRID_DMAP_MAX_DIST * WMGRID_INT_MULT); 
            }
        }
        
        // no obstacle
        if (v > 0) {
            /* LOGM_TRACE(loggerWorldModel, "WMGrid_eval", "msg=\"no obstacle\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                << ", dist=" << dist << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2)
                << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2)); */
            dmap.fill(i, i + 1, v, -1, (int)(WMGRID_DMAP_MAX_DIST * 100.0));
        } else
        // obstacle detected, is distance below "obstacle avoidance threshold"?
        if (dist > (int)((WMGRID_DISTANCE_DMAP / 100.0) * WMGRID_INT_MULT)) {
            /* LOGM_TRACE(loggerWorldModel, "WMGrid_eval", "msg=\"obstacle_far\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                << ", dist=" << dist << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2)
                << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2)); */
            dmap.fill(i, i + 1, 1, (int)(dist * 100.0 / WMGRID_INT_MULT), (int)(WMGRID_DMAP_MAX_DIST * 100.0));
        } else {
            /* LOGM_TRACE(loggerWorldModel, "WMGrid_eval", "msg=\"obstacle_near\"" << ", i=" << i << ", v=" << v << ", a=" << ioff(a, 4) 
                << ", dist=" << dist << ", gx=" << gx << ", gy=" << gy << ", gx2=" << gx2 << ", gy2=" << gy2 << ", grid_x0=" << ioff(grid_x0, 2) << ", grid_y0=" << ioff(grid_y0, 2)
                << ", x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", x2=" << ioff(x2, 2) << ", y2=" << ioff(y2, 2)
                << ", dx=" << ioff(dx, 2) << ", dy=" << ioff(dy, 2) << ", dx_abs=" << ioff(dx_abs, 2) << ", dy_abs=" << ioff(dy_abs, 2)); */
            dmap.fill(i, i + 1, 0, (int)(dist * 100.0 / WMGRID_INT_MULT), (int)(WMGRID_DMAP_MAX_DIST * 100.0));
        }
    }
    dmap.finish();
        
    timeEnd("WMGrid::eval", t);

    return 0;
}


WorldModel::WorldModel() 
{   
    pgrid = new WMGrid();
 
    init();
}

WorldModel::~WorldModel() 
{   
    delete pgrid;
    pgrid = NULL;
}

void WorldModel::init(void)
{
    image_number = 0;
    last_x0 = 0;
    last_y0 = 0;
    last_alfa = 0;
    last_ref = -1;
    last_angle = (int)ANGLE_NONE;
    last_angle_min = (int)ANGLE_NONE;
    last_angle_max = (int)ANGLE_NONE;

    pgrid->init();
}

void WorldModel::updateGrid(const DegreeMap& dmap, float x0, float y0, float alfa, 
         int process_ref, int process_angle, int process_angle_min, int process_angle_max,
         unsigned char mb, unsigned char vb, long imgnum)
{
    int res = pgrid->update(dmap, x0, y0, alfa, mb, vb);
    if (res > 0) {
        image_number = imgnum;
        last_x0 = x0;
        last_y0 = y0;
        last_alfa = alfa;
        last_ref = process_ref;
        last_angle = process_angle;
        last_angle_min = process_angle_min;
        last_angle_max = process_angle_max;
    }
    LOGM_DEBUG(loggerWorldModel, "updateGrid", "x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", alfa=" << ioff(alfa, 2) 
        << ", process_ref=" << process_ref << ", process_angle=" << process_angle << ", process_angle_min=" << process_angle << ", process_angle_max=" << process_angle_max
        << ", imgnum=" << imgnum << ", res=" << res);    
}

int  WorldModel::evalGrid(float x0, float y0, float alfa, DegreeMap& dmap)
{
    int res = pgrid->eval(x0, y0, alfa, dmap);
    LOGM_DEBUG(loggerWorldModel, "evalGrid", "x0=" << ioff(x0, 2) << ", y0=" << ioff(y0, 2) << ", alfa=" << ioff(alfa, 2) 
        << ", res=" << res);    
    return res;
}

const int WMGRID_DRAW_GWIDTH2 = 38;    // number of grid cells drawn horizontaly - one half
const int WMGRID_DRAW_GHEIGHT2 = 38;   // number of grid cells drawn verticaly - one half

const int WMGRID_DRAW_GWIDTH  = 2 * WMGRID_DRAW_GWIDTH2  + 1;
const int WMGRID_DRAW_GHEIGHT = 2 * WMGRID_DRAW_GHEIGHT2 + 1;

void WorldModel::drawGrid(Mat& img)
{
    double t = timeBegin();

    img.create(480,640,CV_8UC3);
    img.setTo(Scalar(20,20,20));
//    line(img, Point(0,320), Point(640,320), Scalar(128,128,128), 1, 8, 0);
//    line(img, Point(320,0), Point(320,480), Scalar(128,128,128), 1, 8, 0);

    // center = robot 
    int gx0 = WMGRID_CENTER_IDX + round((last_x0 - pgrid->grid_x0) / WMGRID_CELL_DX);
    int gy0 = WMGRID_CENTER_IDY + round((last_y0 - pgrid->grid_y0) / WMGRID_CELL_DY);
    
    int gx1 = gx0 - WMGRID_DRAW_GWIDTH2;  // top left corner
    if (gx1 < 0) gx1 = 0;
    if (gx1 > WMGRID_WIDTH - WMGRID_DRAW_GWIDTH) gx1 = WMGRID_WIDTH - WMGRID_DRAW_GWIDTH;

    int gy1 = gy0 - WMGRID_DRAW_GHEIGHT2;
    if (gy1 < 0) gy1 = 0;
    if (gy1 > WMGRID_HEIGHT - WMGRID_DRAW_GHEIGHT) gy1 = WMGRID_HEIGHT - WMGRID_DRAW_GHEIGHT;

    int gx2 = gx1 + 2 * WMGRID_DRAW_GWIDTH2;
    int gy2 = gy1 + 2 * WMGRID_DRAW_GHEIGHT2;

    const int dx = 6;    // width of one grid cell in pixels
    const int dy = 6;    // height of one grid cell in pixels

    float offx = (last_x0 - pgrid->grid_x0) - round((last_x0 - pgrid->grid_x0) / WMGRID_CELL_DX) * WMGRID_CELL_DX;  // -WMGRID_CELL_DX/2 < offx <= WMGRID_CELL_DX/2
    offx = offx * dx / WMGRID_CELL_DX;  // -3 < offx < +3
    float offy = (last_y0 - pgrid->grid_y0) - round((last_y0 - pgrid->grid_y0) / WMGRID_CELL_DY) * WMGRID_CELL_DY;  // -WMGRID_CELL_DY/2 < offy <= WMGRID_CELL_DY/2
    offy = offy * dy / WMGRID_CELL_DY;  // -3 < offy < +3
    offy = -offy;

    int x1 = round(-dx / 2 - offx + (640 - WMGRID_DRAW_GWIDTH * dx) / 2);
    int y1 = round(-dy / 2 - offy + (480 - WMGRID_DRAW_GHEIGHT * dy) / 2);

    rectangle(img, Point(x1, y1), Point(x1 + (gx2 - gx1 + 1) * dx - 1, y1 + (gy2 - gy1 + 1) * dy - 1), Scalar(80, 80, 80), CV_FILLED);
    
    int y = y1;
    for(int gy = gy2; gy >= gy1; gy--) {
        int x = x1;
        for(int gx = gx1; gx <= gx2; gx++) {
            Scalar color = Scalar(80, 80, 80);
            unsigned char b = pgrid->get(gy, gx);

            if ((b & (WMGRID_LIDAR_MBIT | WMGRID_VISION_MBIT)) == 0) {
                // grey = no information
                // color = Scalar(80, 80, 80);
            } else
            if ((b & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) == 0) {
                // green = no obstacle (dark = one source, light = lidar and vision)
                if ((b & (WMGRID_LIDAR_MBIT | WMGRID_VISION_MBIT)) == (WMGRID_LIDAR_MBIT | WMGRID_VISION_MBIT)) {
                    color = Scalar(0, 120, 0);
                } else {
                    color = Scalar(0,  80, 0);
                }
            } else {                                                                
                // red = vision, blue = lidar
                if ((b & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) == (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) {
                    color = Scalar(255, 60, 255);
                } else 
                if ((b & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) == (WMGRID_LIDAR_VBIT)) {
                    //color = Scalar(255, 60, 60);
                    color = Scalar(60, 60, 255);
                } else {
                    //color = Scalar(60, 60, 255);
                    color = Scalar(255, 60, 60);
                }
            }

            if (color[0] != 80) {
                rectangle(img, Point(x, y), Point(x + dx - 1, y + dy - 1), color, CV_FILLED);
            }
            x += dx;
        }
        y += dy;
    }
    
    int xx = round(x1 + ((gx0 - gx1) + 0.5) * dx + offx);
    int yy = round(y1 + ((gy0 - gy1) + 0.5) * dy + offy);
    
    /* draw navigation points */
    if (last_ref > 0) {
        int navp_cnt = navigation_point_cnt();
        for(int i = 0; i < navp_cnt; i++) {
            double dxx = (navigationPointXY[i].x - last_x0) * dx / WMGRID_CELL_DX;
            double dyy = (navigationPointXY[i].y - last_y0) * dy / WMGRID_CELL_DY;
            if ((dxx >= -WMGRID_DRAW_GWIDTH2 * dx) && (dxx <= WMGRID_DRAW_GWIDTH2 * dx) && 
                (dyy >= -WMGRID_DRAW_GHEIGHT2 * dy) && (dyy <= WMGRID_DRAW_GHEIGHT2 * dy)) {
                int navp_xx = xx + ((int)dxx);
                int navp_yy = yy - ((int)dyy);
                line(img, Point(navp_xx - 3, navp_yy), Point(navp_xx + 3, navp_yy), Scalar(0, 255, 255), 1, 8, 0);
                line(img, Point(navp_xx, navp_yy - 3), Point(navp_xx, navp_yy + 3), Scalar(0, 255, 255), 1, 8, 0);
                putText(img, (char *)navigationPoint[i].name, Point(navp_xx + 3, navp_yy + 3), CV_FONT_HERSHEY_DUPLEX, 0.8, Scalar(0, 255, 255)); 
            }
        }
    }

    /* draw robot position */
    line(img, Point(xx - 3, yy), Point(xx + 3, yy), Scalar(255, 255, 255), 1, 8, 0);
    line(img, Point(xx, yy - 3), Point(xx, yy + 3), Scalar(255, 255, 255), 1, 8, 0);

    int dxx = 120 * sin(last_alfa * M_PI / 180.0);
    int dyy = 120 * cos(last_alfa * M_PI / 180.0);
    line(img, Point(xx, yy), Point(xx + dxx, yy - dyy), Scalar(255, 255, 255), 1, 8, 0);

    if ((last_angle_min < ANGLE_OK) && (last_angle_max < ANGLE_OK)) {
        int angle1 = (last_alfa + 90 - last_angle_min);
        int angle2 = (last_alfa + 90 - last_angle_max);
        dxx = 80 * sin(angle1 * M_PI / 180.0);
        dyy = 80 * cos(angle1 * M_PI / 180.0);
        line(img, Point(xx, yy), Point(xx + dxx, yy - dyy), Scalar(255, 255, 0), 1, 8, 0);
        dxx = 80 * sin(angle2 * M_PI / 180.0);
        dyy = 80 * cos(angle2 * M_PI / 180.0);
        line(img, Point(xx, yy), Point(xx + dxx, yy - dyy), Scalar(255, 255, 0), 1, 8, 0);
        ellipse(img, Point(xx, yy), Size(80, 80), 0, angle1 - 90, angle2 - 90, Scalar(255, 255, 0));
    }
    if (last_angle < ANGLE_OK) {
        dxx = 100 * sin((last_alfa + 90 - last_angle) * M_PI / 180.0);
        dyy = 100 * cos((last_alfa + 90 - last_angle) * M_PI / 180.0);
        line(img, Point(xx, yy), Point(xx + dxx, yy - dyy), Scalar(0, 255, 255), 1, 8, 0);
    }

    LOGM_DEBUG(loggerWorldModel, "drawGrid", "last_x0=" << ioff(last_x0, 2) << ", last_y0=" << ioff(last_y0, 2) << ", last_alfa=" << ioff(last_alfa, 2)
        << ", gx0=" << gx0 << ", gy0=" << gy0
        << ", offx=" << ioff(offx, 2) << ", offy=" << ioff(offy, 2)
        << ", x1=" << x1 << ", y1=" << y1 
        << ", xx=" << xx << ", yy=" << yy
        << ", xxd=" << xx-x1 << ", yyd=" << yy-y1);

    timeEnd("WorldModel::drawGrid", t);
}

void WorldModel::drawGridFull(Mat& img)
{
    double t = timeBegin();

    const int dx = 2;    // width of one grid cell in pixels
    const int dy = 2;    // height of one grid cell in pixels

    img.create(WMGRID_HEIGHT * dy, WMGRID_WIDTH * dx, CV_8UC3);
    img.setTo(Scalar(20, 20, 20));
    
    int gx1 = 0;  // top left corner
    int gy1 = 0;
    int gx2 = WMGRID_WIDTH - 1;
    int gy2 = WMGRID_HEIGHT - 1;

    int x1 = 0;
    int y1 = 0;

    rectangle(img, Point(x1, y1), Point(x1 + (gx2 - gx1 + 1) * dx - 1, y1 + (gy2 - gy1 + 1) * dy - 1), Scalar(80, 80, 80), CV_FILLED);

    int cnt = 0;    
    int y = y1;
    for(int gy = gy2; gy >= gy1; gy--) {
        int x = x1;
        for(int gx = gx1; gx <= gx2; gx++) {
            Scalar color = Scalar(80, 80, 80);
            unsigned char b = pgrid->get(gy, gx);

            if ((b & (WMGRID_LIDAR_MBIT | WMGRID_VISION_MBIT)) == 0) {
                // grey = no information
                // color = Scalar(80, 80, 80);
            } else
            if ((b & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) == 0) {
                // green = no obstacle (dark = one source, light = lidar and vision)
                if ((b & (WMGRID_LIDAR_MBIT | WMGRID_VISION_MBIT)) == (WMGRID_LIDAR_MBIT | WMGRID_VISION_MBIT)) {
                    color = Scalar(0, 120, 0);
                } else {
                    color = Scalar(0,  80, 0);
                }
            } else {                                                                
                // red = vision, blue = lidar
                if ((b & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) == (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) {
                    color = Scalar(255, 60, 255);
                } else 
                if ((b & (WMGRID_LIDAR_VBIT | WMGRID_VISION_VBIT)) == (WMGRID_LIDAR_VBIT)) {
                    //color = Scalar(255, 60, 60);
                    color = Scalar(60, 60, 255);
                } else {
                    //color = Scalar(60, 60, 255);
                    color = Scalar(255, 60, 60);
                }
            }

            if (color[0] != 80) {
                rectangle(img, Point(x, y), Point(x + dx - 1, y + dy - 1), color, CV_FILLED);
                cnt++;
            }
            x += dx;
        }
        y += dy;
    }
    
    LOGM_DEBUG(loggerWorldModel, "drawGridFull", "x1=" << x1 << ", y1=" << y1 << ", cnt=" << cnt);

    timeEnd("WorldModel::drawGridFull", t);
}

void WorldModel::saveImage(long image_number, const string& str, const Mat& img)
{
    const string outputDir = "out\\";
    const long save_rand = 1111111;

    char filename[256];
//  long tick_number = getTickCount();

    if (!img.empty()) {
        double t = timeBegin();
        sprintf(filename, "%srt2020_%07u_%07u_%s", outputDir.c_str(), (unsigned int)save_rand, (unsigned int)(image_number<0?999999:image_number), str.c_str());
        imwrite(filename, img);
        sprintf(filename, "WorldModel::saveImage('%s')", str.c_str());
        timeEnd(filename, t);
    }
}

void WorldModel::test(void)
{
/*
    long image_number = 0;
    
    Mat img;
    DegreeMap dmap;
    DegreeMap dmap2;
    
    float x0   = 0.0;
    float y0   = 0.0;
    float alfa = 0.0;

    dmap.init();
    dmap.fill(  0, 180, 0, 300, 300);
    dmap.finish();

    updateGrid(dmap, x0, y0, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.03, y0 + 0.03, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.06, y0 + 0.06, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.09, y0 + 0.09, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.12, y0 + 0.12, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.15, y0 + 0.15, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.18, y0 + 0.18, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.21, y0 + 0.21, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.24, y0 + 0.24, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.27, y0 + 0.27, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    updateGrid(dmap, x0 + 0.30, y0 + 0.30, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);

    evalGrid(x0 + 1, y0 + 1, alfa, dmap2);
    dmap2.print("dmap2");
    updateGrid(dmap2, x0, y0, alfa + 180, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);

    updateGrid(dmap, x0 + 500, y0 + 400, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);

    evalGrid(x0 + 500 + 1, y0 + 400 + 1, alfa, dmap2);
    dmap2.print("dmap2");
    updateGrid(dmap2, x0 + 500, y0 + 400, alfa + 180, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);

    dmap.init();
    dmap.fill(  0,  45, 1,  -1, 300);
    dmap.fill( 45,  90, 0,  50, 300);
    dmap.fill( 90, 135, 0, 100, 300);
    dmap.fill(135, 180, 0, 150, 300);
    dmap.finish();
    
    updateGrid(dmap, x0, y0, alfa, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    
    updateGrid(dmap, x0, y0 - 0.5, alfa + 90, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
    
    updateGrid(dmap, x0 - 0.5, y0 - 0.5, alfa - 45, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);

    updateGrid(dmap, x0 + 0.5, y0, alfa + 90, WMGRID_LIDAR_MBIT, WMGRID_LIDAR_VBIT, image_number);
    drawGrid(img);    
    saveImage(image_number++, "grid.png", img);
*/
}

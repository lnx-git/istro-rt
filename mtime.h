#ifndef __MTIME_H__
#define __MTIME_H__

#include <string>

double timeBegin(void);
void   timeEnd(const std::string& str, double t);

double timeAdd2(double t, double dt);     // result = t + dt   (dt is in milliseconds)
double timeDelta(double t);
double timeDelta2(double t1, double t2);  // result = t2 - t1  (milliseconds)

#endif

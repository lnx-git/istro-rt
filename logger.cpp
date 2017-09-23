#ifndef ISTRO_LOGGER_LOG4CX

// correctly include <pthread.h> with HAVE_STRUCT_TIMESPEC
#include "threads.h"

pthread_mutex_t  logger_mtx;

void lmi(void) 
{
    pthread_mutex_init(&logger_mtx, NULL);
}

void lmd(void) 
{
    pthread_mutex_destroy(&logger_mtx);
}

#include <sys/time.h>
#include <iostream>
#include <iomanip>
#include <ctime>

/*
void lml(void) 
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    pthread_mutex_lock(&logger_mtx);
    
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    
    strftime(buffer,sizeof(buffer),"%Y-%m-%d %I:%M:%S,000 ",timeinfo);

    std::cout << buffer;
}
*/

void lml(void) 
{
    timeval curTime;

    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;

    char buffer [80];
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

    pthread_mutex_lock(&logger_mtx);

    std::cout << buffer << "," << std::setfill('0') << std::setw(3) << ((int)milli) << std::setw(0) << " ";
}


/*
// C++ 2011
#include <chrono>
#include <ctime>
#include <iostream>

void lml(void) 
{
    using namespace std;
    using namespace std::chrono;

    system_clock::time_point now = system_clock::now();
    system_clock::duration tp = now.time_since_epoch();

    tp -= duration_cast<seconds>(tp);

    time_t tt = system_clock::to_time_t(now);

    pthread_mutex_lock(&logger_mtx);

    std::printf("%04u-%02u-%02u %02u:%02u:%02u.%03u ", t.tm_year + 1900,
                t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec,
                static_cast<unsigned>(tp / milliseconds(1)));
}
*/

void lmu(void) 
{
    pthread_mutex_unlock(&logger_mtx);
}

#endif

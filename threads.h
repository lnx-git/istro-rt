#ifndef __THREADS_H__
#define __THREADS_H__

#ifdef WIN32
#ifndef HAVE_STRUCT_TIMESPEC
#define HAVE_STRUCT_TIMESPEC 1
#endif
#endif

// fixme: this header file must be included before any other .h files with "using namespace cv;"
// fixme: otherwise an "error: reference to 'ACCESS_MASK' is ambiguous" will be reported

#include <pthread.h>

//const int THDATA_TYPE_NONE = 0;

const int THDATA_STATE_NONE              =   0;
const int THDATA_STATE_NEW               =   1;
const int THDATA_STATE_CAMERA_CAPTURING  =   2;
const int THDATA_STATE_CAMERA_CAPTURED   =   3;
const int THDATA_STATE_LIDAR_CAPTURING   =   4;
const int THDATA_STATE_LIDAR_CAPTURED    =   5;
const int THDATA_STATE_CTRLBOARD         =   6;  // mutex for ctrlboard serial port
const int THDATA_STATE_CTRLBOARD_LOCK    =   7;
const int THDATA_STATE_SHARED            =   8;  // shared dataset for GPS / control board / compass
const int THDATA_STATE_SHARED_LOCK       =   9;
const int THDATA_STATE_VISION_PROCESSING =  10;
const int THDATA_STATE_VISION_PROCESSED  =  11;
const int THDATA_STATE_PROCESSING        =  12;
const int THDATA_STATE_PROCESSED         =  13;
const int THDATA_STATE_SAVING            =  14;
const int THDATA_STATE_SAVED             =  15;

const int THREADS_NUM = 8;
const int THDATA_NUM = 13;

// CAMERA_CAPTURED, CAMERA_CAPTURING, LIDAR_CAPTURED, LIDAR_CAPTURING, PROCESSED, PROCESSING, SAVING

// thread_testcancel: returns 1 if the thread should be terminated
int thread_testcancel(void);

// thread_exit: every thread should call before returning from the start() routine
void thread_exit(void);

typedef struct { 
    //int  type;
    int  state;
    void *data;
} thdata_t;

class Threads {
private:    
    pthread_t threads[THREADS_NUM];
    
    int  thdata_len;    // number of used thdata records (0 .. THDATA_NUM-1)
    thdata_t  thdata[THDATA_NUM];
    pthread_mutex_t  thdata_mtx;

public:
    int init(void *data1, void *data2, void *data3, void *data4, void *data5, void *data6, void *data7, void *data8, void *data9, void *data10, void *data11, void *data12, void *data13);
    void close(void);

    int start(void *(*t1)(void *), void *(*t2)(void *), void *(*t3)(void *), void *(*t4)(void *), void *(*t5)(void *), void *(*t6)(void *), void *(*t7)(void *), void *(*t8)(void *));
    int stop(void);

private:
    void dataLock(void);
    void dataUnlock(void);
    
    void getState(char *str);
    void printState(int state_new, char *str);

public:
    void *getData(/*int type,*/ int state, int state_new);
    void *getData2(/*int type,*/ int state, int state_new, void **ppdata2, int state2);

    void *checkData(/*int type,*/ int state, int state_new);

    int  setData(void *data, int state_new, int max_cnt);    
    //int  cleanData(/*int type,*/ int state, int state_new);
};

#endif

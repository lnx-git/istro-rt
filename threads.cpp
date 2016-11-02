#include <stdio.h>
#include "threads.h"
#include "system.h"
#include "logger.h"

// signal to all threads request for termination
int thread_cancel = 0;

LOG_DEFINE(loggerThreads, "Threads");

int thread_testcancel(void) 
{
    return thread_cancel;
}

int thread_exit(void) 
{
    pthread_exit(NULL);
}

int Threads::init(void *data1, void *data2, void *data3, void *data4, void *data5, void *data6, void *data7, void *data8, void *data9, void *data10, void *data11)
{
    thdata_len = 0;
    
    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data1;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data2;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data3;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data4;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data5;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data6;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data7;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data8;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data9;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data10;

    thdata[thdata_len].state = THDATA_STATE_NEW;
    thdata[thdata_len++].data = data11;
    
    printState();
    
    if (pthread_mutex_init(&thdata_mtx, NULL) != 0) {
        LOGM_ERROR(loggerThreads, "init", "error creating thdata mutex!");
        return -1;
    }
    
    return 0;
}

void Threads::close(void) 
{
    if (pthread_mutex_destroy(&thdata_mtx) != 0) {
        LOGM_ERROR(loggerThreads, "close", "error destroying thdata mutex!");
    }
}

void Threads::dataLock(void)
{
    pthread_mutex_lock(&thdata_mtx);
}

void Threads::dataUnlock(void)
{
    pthread_mutex_unlock(&thdata_mtx);
}

int Threads::start(void *(*t1)(void *), void *(*t2)(void *), void *(*t3)(void *), void *(*t4)(void *), void *(*t5)(void *), void *(*t6)(void *), void *(*t7)(void *)) 
{
    int res;
    pthread_attr_t attr;

    LOGM_INFO(loggerThreads, "start", "starting threads...");

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
   
    res = pthread_create(&threads[0], &attr, t1, (void *)NULL);
    if (res < 0) {
        LOGM_ERROR(loggerThreads, "start", "error creating thread[0]!");
        pthread_attr_destroy(&attr);
        return -1;
    }

    res = pthread_create(&threads[1], &attr, t2, (void *)NULL);
    if (res < 0) {
        LOGM_ERROR(loggerThreads, "start", "error creating thread[1]!");
        pthread_attr_destroy(&attr);
        return -2;
    }

    res = pthread_create(&threads[2], &attr, t3, (void *)NULL);
    if (res < 0) {
        LOGM_ERROR(loggerThreads, "start", "error creating thread[2]!");
        pthread_attr_destroy(&attr);
        return -3;
    }

    res = pthread_create(&threads[3], &attr, t4, (void *)NULL);
    if (res < 0) {
        LOGM_ERROR(loggerThreads, "start", "error creating thread[3]!");
        pthread_attr_destroy(&attr);
        return -3;
    }

    res = pthread_create(&threads[4], &attr, t5, (void *)NULL);
    if (res < 0) {
        LOGM_ERROR(loggerThreads, "start", "error creating thread[4]!");
        pthread_attr_destroy(&attr);
        return -4;
    }

    res = pthread_create(&threads[5], &attr, t6, (void *)NULL);
    if (res < 0) {
        LOGM_ERROR(loggerThreads, "start", "error creating thread[5]!");
        pthread_attr_destroy(&attr);
        return -5;
    }

    res = pthread_create(&threads[6], &attr, t7, (void *)NULL);
    if (res < 0) {
        LOGM_ERROR(loggerThreads, "start", "error creating thread[6]!");
        pthread_attr_destroy(&attr);
        return -6;
    }
    
    pthread_attr_destroy(&attr);

    return 0;
}

int Threads::stop(void) 
{
    void *status;    
    
    thread_cancel = 1;
    
    LOGM_INFO(loggerThreads, "stop", "joining threads...");
    for(int i = 0; i < THREADS_NUM; i++) {
       pthread_join(threads[i], &status);
    }

    LOGM_INFO(loggerThreads, "stop", "finished");
    return 0;
}

void *Threads::getData(/*int type,*/ int state, int state_new)
{
    int idx = -1;
    while (idx < 0) {

        dataLock();

        // find data
        for(int i = 0; i < thdata_len; i++) {
            if (thdata[i].state == state) {
                idx = i;
                break;
            }
        }
        // set new state
        if (idx >= 0) {
            thdata[idx].state = state_new;
            printState();
        }
        
        dataUnlock();

        if (idx < 0) {
            if (thread_testcancel()) {
                // if the thread should be terminated: exit immediatelly and return NULL
                return NULL;
            } else {
                // no data found in requested state - busy wait 
                msleep(2);
            }
        }
    }
    
    return thdata[idx].data;
}

void *Threads::getData2(/*int type,*/ int state, int state_new, void **ppdata2, int state2)
{
    int idx1 = -1;
    int idx2 = -1;
    *ppdata2 = NULL;
    while ((idx1 < 0) || (idx2 < 0)) {

        dataLock();

        // find data
        for(int i = 0; i < thdata_len; i++) {
            if (thdata[i].state == state) {
                idx1 = i;
            }
            if (thdata[i].state == state2) {
                idx2 = i;
            }
        }
        // set new state
        if ((idx1 >= 0) && ((idx2 >= 0))) {
            thdata[idx1].state = state_new;
            thdata[idx2].state = state_new;
            printState();
        }
        
        dataUnlock();

        if ((idx1 < 0) || (idx2 < 0))  {
            if (thread_testcancel()) {
                // if the thread should be terminated: exit immediatelly and return NULL
                return NULL;
            } else {
                // no data found in requested state - busy wait 
                msleep(2);
            }
        }
    }
    
    *ppdata2 = thdata[idx2].data;
    return thdata[idx1].data;
}

int  Threads::printState()
{
    char str[THDATA_NUM + 1];

    for(int i = 0; i < thdata_len; i++) {
        if (thdata[i].state <= 9) {
            str[i] = (char)('0' + thdata[i].state);
        } else {
            str[i] = (char)('A' + thdata[i].state - 10);
        }
    }

    str[thdata_len] = 0;
    LOGM_TRACE(loggerThreads, "printState", "state=[" << str << "]");    
}

// nastavi stav datasetu na "state_new"
// max_cnt: kolko najviac datasetov moze byt spolu v stave "state_new" (vratane tohto), ostatne su nastavene do stavu NEW
int  Threads::setData(void *data, int state_new, int max_cnt)
{
    int idx = -1;

    dataLock();

    int cnt = 0;
    for(int i = 0; i < thdata_len; i++) {
        if (thdata[i].data == data) {
            idx = i;
            break;
        } 
    }

    if (idx >= 0) {
        for(int i = 0; i < thdata_len; i++) {
            if (thdata[i].state == state_new) {
                cnt++;
                if ((max_cnt > 0) && (cnt >= max_cnt)) {
                    LOGM_TRACE(loggerThreads, "setData", "thdata[" << i <<"].state=" << state_new << " cleaned!");
                    thdata[i].state = THDATA_STATE_NEW;
                }
            }
        }
        thdata[idx].state = state_new;
        printState();
    }
    
    dataUnlock();

    if (idx < 0) {
        LOGM_ERROR(loggerThreads, "setData", "error finding thdata!");
        return -1;
    }

    return 0;
}

/*
int Threads::cleanData(int state, int state_new)
{
    int cnt = 0;

    dataLock();

    for(int i = 0; i < thdata_len; i++) {
        if (thdata[i].state == state) {
            LOGM_TRACE(loggerThreads, "cleanData", "thdata[" << i <<"].state=" << state << " cleaned!");
            thdata[i].state = state_new;
            cnt++;
        }
    }

    dataUnlock();

    return cnt;
}
*/

#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <opencv2/opencv.hpp>
#include "dmap.h"
#include "system.h"
    
using namespace cv;
using namespace std;

const int LIDAR_DATA_NUM = 360 * 2;

typedef struct { 
    int   sync;
    float angle;
    float distance;
    int   quality;
} lidar_data_t;

#ifdef ISTRO_LIDAR_TIM571

#include <stdint.h>

#define TIM571_ADDR "169.254.0.5"
#define TIM571_PORT 2111

#define TIM571_DATA_COUNT 811
#define TIM571_TOTAL_ANGLE (3 * M_PI / 2)
#define TIM571_TOTAL_ANGLE_DEG 270
#define TIM571_SIZE_OF_ONE_STEP (TIM571_TOTAL_ANGLE / (double)(TIM571_DATA_COUNT - 1))
#define TIM571_SIZE_OF_ONE_DEG ((TIM571_DATA_COUNT - 1) / TIM571_TOTAL_ANGLE_DEG)
#define TIM571_MAX_DISTANCE 15000
#define TIM571_FREQUENCY 15

#define MAX_SENTENCE_LENGTH 10000

// status information received from the sensor (typical values in comments)
typedef struct tim571_status_struct {
    uint16_t firmware_version;     // 1
    uint16_t sopas_device_id;      // configurable in SOPAS
    uint32_t serial_number;
    uint8_t error;                 // 0 = device is ready
    uint32_t scanning_frequency;   // 1500  in 1/100 Hz
    float multiplier;              // 1
    int32_t starting_angle;        // -450000 (in 1/10000 of degree)
    uint32_t angular_step;         // 3333    (in 1/10000 of degree)
    uint16_t data_count;           // number of distance/rssi data
    uint8_t rssi_available;        // 0 = not receiving rssi data
    char name[17];                 // name configurable in SOPAS or "\0"
} tim571_status_data;

#endif

class Lidar {  
#ifdef ISTRO_LIDAR_TIM571    
private:
    unsigned char program_runs;

private:
    int online;
    int sockfd;

    int connect_tim571();

    uint8_t sentence[MAX_SENTENCE_LENGTH];
    char *sp;

    int skip_to_sentence_start();
    char *get_next_word();
    char *get_next_str();

    int send_start_measurement();
    int read_sentence(int log);
    int process_sentence();

private:
    uint16_t tim571_firmware;
    uint16_t tim571_sopas_device_id;
    uint32_t tim571_serial_num;
    uint32_t tim571_timestamp;
    uint8_t tim571_error;
    uint32_t tim571_telegram_counter;
    uint32_t tim571_scanning_frequency;
    float tim571_multiplier;
    int32_t tim571_starting_angle;
    uint16_t tim571_angular_step;
    uint16_t tim571_data_count;
    uint8_t tim571_rssi_channels;
    uint16_t *tim571_data;
    uint8_t *tim571_rssi_data;
    char tim571_name[18];

    void copy_status_data(tim571_status_data *status_data);

private:
    int init_tim571();

    void log_tim571_data(tim571_status_data *sd, uint16_t *dist, uint8_t *rssi);  // write all data to log file
#endif

public:
    int init(const char *portName);
    void close(void);
    
    bool checkHealth(void);

    int getData(lidar_data_t *data, int& data_cnt);
    int process(const lidar_data_t *data, const int& data_cnt, DegreeMap& dmap, int &stop);
    int drawOutput(const lidar_data_t *data, const int& data_cnt, Mat& img, /*const DegreeMap& dmap,*/ int stop, int angle_min, int angle_max, int process_angle, int process_angle_min, int process_angle_max, long image_number);
};

#endif

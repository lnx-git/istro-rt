#ifndef __VISIONN_H__
#define __VISIONN_H__

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define VISIONN_ADDR "127.0.0.1"
#define VISIONN_PORT 7001

#define VISIONN_READBUF_LEN  2000000

class VisioNN {
private:
    int sockfd;
    unsigned char* sock_readbuf;

    int connect_nnserver();

public:
    int init(void);
    void close(void);

    int send_hello(void);
    int send_imgpr(const Mat& img);
    int readn(int sockfd, uint8_t* buf, int len);
    int recv_skip(char ch);
    int recv_hello(void);
    int recv_imgpr(Mat& img);
};

#endif

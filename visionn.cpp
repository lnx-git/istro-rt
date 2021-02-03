#include <stdio.h>
#include <stdlib.h>
#include "visionn.h"
#include "mtime.h"
#include "logger.h"

LOG_DEFINE(loggerVisioNN, "VisioNN");

#ifdef ISTRO_VISIONN

/* sockets */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

int VisioNN::connect_nnserver()
{
    //printf("connecting...\n");
    LOGM_INFO(loggerVisioNN, "connect_nnserver", "msg=\"connecting...\"");  // fixme _ADDR, _PORT
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
        //printf("error: cannot open socket!\n");
    	LOGM_ERROR(loggerVisioNN, "connect_nnserver", "msg=\"error: cannot open socket!\"");
        return -1;
    }

    struct sockaddr_in remoteaddr;
    remoteaddr.sin_family = AF_INET;
    remoteaddr.sin_addr.s_addr = inet_addr(VISIONN_ADDR);
    remoteaddr.sin_port = htons(VISIONN_PORT);

    if (connect(sockfd, (struct sockaddr*)&remoteaddr, sizeof(remoteaddr)) < 0)
    {
        //printf("error: connecting socket!\n");
    	LOGM_ERROR(loggerVisioNN, "connect_nnserver", "msg=\"error: connecting socket!\"");
        return -2;
    }

    //printf("connected...\n");
    LOGM_INFO(loggerVisioNN, "connect_nnserver", "msg=\"connected\"");
    return 0;
}

int VisioNN::init(void) 
{
    sockfd = -1;
    sock_readbuf = (unsigned char *)malloc(VISIONN_READBUF_LEN);

    if (connect_nnserver() < 0)
    {
        //printf("error: connect_nnserver() failed!\n");
        LOGM_ERROR(loggerVisioNN, "init()", "msg=\"error: connect_nnserver() failed!\"");
        return -1;   
    }

    return 0; 
}

void VisioNN::close(void) 
{
    free(sock_readbuf);
    if (sockfd >= 0) {
        ::close(sockfd);    // unistd::close()
    }
}

int VisioNN::send_hello(void)
{
    static const char *cmd = "{\"VISIONN_HELLO\":\"client\"}\n";
    //printf("%s", cmd);
    //LOGM_TRACE(loggerVisioNN, "send_hello", "data=\"" << cmd << "\"");
    if (write(sockfd, cmd, strlen(cmd)) < (int)strlen(cmd))
    {
        //printf("error: writing to socket!\n");
        LOGM_ERROR(loggerVisioNN, "send_hello", "msg=\"error: writing to socket!\"");
        return -1;
    }
    return 0;
}

int VisioNN::send_imgpr(const Mat& img)
{
    double t = timeBegin();
    vector<uchar> buf;
    //vector<int> params;
    //params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //params.push_back(0);
    //imencode(".png", img, buf, params);
    imencode(".bmp", img, buf);
    timeEnd("VisioNN::send_imgpr.imencode", t);

    char cmd[64];
    int buflen = buf.size();
    sprintf(cmd, "{\"VISIONN_IMPRQ\":[%07d,\"", buflen);
    //printf("%s", cmd);
    //LOGM_TRACE(loggerVisioNN, "send_imgpr", "data=\"" << cmd << "\"");
    if (write(sockfd, cmd, strlen(cmd)) < (int)strlen(cmd))
    {
        //printf("error: writing to socket!\n");
        LOGM_ERROR(loggerVisioNN, "send_imgpr", "msg=\"error: writing to socket!\"");
        return -1;
    }

    if (write(sockfd, &buf[0], buflen) < buflen)
    {
        //printf("error: writing to socket!\n");
        LOGM_ERROR(loggerVisioNN, "send_imgpr", "msg=\"error: writing #2 to socket!\"");
        return -2;
    }

    static const char *cmd2 = "\"]}\n";
    //printf("%s", cmd2);
    // fixme: loguje aj konce riadkov!
    //LOGM_TRACE(loggerVisioNN, "send_imgpr", "data2=\"" << cmd2 << "\"");
    if (write(sockfd, cmd2, strlen(cmd2)) < (int)strlen(cmd2))
    {
        //printf("error: writing to socket!\n");
        LOGM_ERROR(loggerVisioNN, "send_imgpr", "msg=\"error: writing #3 to socket!\"");
        return -3;
    }

    return 0;
}


#define VISIONN_BUFFER_LEN  256
#define VISIONN_PHEAD_LEN    17

int VisioNN::recv_hello(void)
{
    uint8_t data[VISIONN_BUFFER_LEN];

    if (recv_skip('{') < 0) {
        LOGM_ERROR(loggerVisioNN, "recv_hello", "msg=\"error: -1!\"");
        return -1;
    }

    data[0] = '{';
    int n = readn(sockfd, &data[1], VISIONN_PHEAD_LEN-1);
    if (n < VISIONN_PHEAD_LEN-1) {
        //printf("recv_hello: error: n=%d\n", n);
        LOGM_ERROR(loggerVisioNN, "recv_hello", "msg=\"error: -2!\"");
        return -2;
    }
    data[++n] = 0;
    //printf("recv_hello: %s\n", data);
    //LOGM_TRACE(loggerVisioNN, "recv_hello", "data=\"" << data << "\"");

    // data: {"VISIONN_HELLO":
    if ((data[2] != 'V') || (data[10] != 'H') || (data[14] != 'O')) {
        //printf("recv_hello: error: wrong answer!");
        LOGM_ERROR(loggerVisioNN, "recv_hello", "msg=\"error: -3!\"");
        return -3;
    }

    recv_skip('}');
    return 0;
}

typedef vector<unsigned char> readbuf_t;

string cvtype2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

int VisioNN::recv_imgpr(Mat &pred)
{
    uint8_t data[VISIONN_BUFFER_LEN];

    if (recv_skip('{') < 0) {
        LOGM_ERROR(loggerVisioNN, "recv_imgpr", "msg=\"error: -1!\"");
        return -1;
    }

    data[0] = '{';
    int n = readn(sockfd, &data[1], VISIONN_PHEAD_LEN-1);
    if (n < VISIONN_PHEAD_LEN-1) {
        //printf("recv_imgpr: error: n=%d\n", n);
        LOGM_ERROR(loggerVisioNN, "recv_imgpr", "msg=\"error: -2!\"");
        return -2;
    }
    data[++n] = 0;
    //printf("recv_imgpr: %s\n", data);
    //LOGM_TRACE(loggerVisioNN, "recv_imgpr", "data=\"" << data << "\"");

    // data: {"VISIONN_IMPRS":
    if ((data[2] != 'V') || (data[10] != 'I') || (data[14] != 'S')) {
        //printf("recv_imgpr: error: wrong answer!");
        LOGM_ERROR(loggerVisioNN, "recv_imgpr", "msg=\"error: -3!\"");
        return -3;
    }
    n = readn(sockfd, data, 10);
    if (n < 10) {
        //printf("recv_imgpr: error: n=%d\n", n);
        LOGM_ERROR(loggerVisioNN, "recv_imgpr", "msg=\"error: -4!\"");
        return -4;
    }
    // data: [0004667,"
    data[n] = 0;
    //printf("recv_imgpr2: %s\n", data);
    //LOGM_TRACE(loggerVisioNN, "recv_imgpr", "data2=\"" << data << "\"");

    int buflen = -1;
    data[8]=0;
    sscanf((char *)&data[1], "%d", &buflen);
    //printf("recv_imgpr: buflen=%d\n", buflen);
    //LOGM_TRACE(loggerVisioNN, "recv_imgpr", "buflen=" << buflen);
    if ((buflen < 0) || (buflen > VISIONN_READBUF_LEN)) {
        //printf("recv_imgpr: error: buflen=%d\n", buflen);
        LOGM_ERROR(loggerVisioNN, "recv_imgpr", "msg=\"error: -5!\"");
        return -5;
    }

    if (readn(sockfd, sock_readbuf, buflen) < 0) {
        //printf("recv_imgpr: error6\n");
        LOGM_ERROR(loggerVisioNN, "recv_imgpr", "msg=\"error: -6!\"");
        return -6;
    }

    readbuf_t readbuf(sock_readbuf, sock_readbuf + buflen);
    pred = imdecode(Mat(readbuf), 1);
    //printf("pred.size: %d, %d, %d, %s\n", pred.rows, pred.cols, pred.dims, cvtype2str(pred.type()).c_str());
    //LOGM_TRACE(loggerVisioNN, "recv_imgpr", "pred.rows=" << pred.rows << ", pred.cols=" << pred.cols << ", pred.dims=" << pred.dims << ", pred.type=" << cvtype2str(pred.type()));

    recv_skip('}');
    return 0;
}

int VisioNN::readn(int sockfd, uint8_t* buf, int len)
{
    int total = 0;
    while (total < len) {
        int n = read(sockfd, buf + total, len - total);
        if (n < 1) {
            //printf("readn: error: n=%d, len=%d, total=%d\n", n, len, total);
            LOGM_ERROR(loggerVisioNN, "readn", "msg=\"error: -1!\"");
            return -1;
        }
        total += n;
    }
    return total;
}

int VisioNN::recv_skip(char ch)
{
    uint8_t data[VISIONN_BUFFER_LEN];

    int nn = 0;
    while (1) {
        int n = read(sockfd, &data[nn], 1);
        if (n < 1) {
            //printf("recv_skip: error: n=%d\n", n);
            LOGM_ERROR(loggerVisioNN, "recv_skip", "msg=\"error: -1!\"");
            return -1;
        }
        if (data[nn++] == ch) break;
    }
    data[nn] = 0;
    //if (ch != '{') {
        //printf("recv_skip: %s\n", data); 
        //LOGM_TRACE(loggerVisioNN, "recv_skip", "data=\"" << data << "\"");
    //}

    return 0;
}

/*
int main(int argc, char** argv)
{
    VisioNN visionn;

    visionn.init();

    visionn.send_hello();
    visionn.recv_hello();

    Mat img = imread("testing_image.png", 1);
    visionn.send_imgpr(img);

    Mat pred;
    visionn.recv_imgpr(pred);

    visionn.send_hello();
    visionn.recv_hello();

    visionn.close();

    return 0;
}
*/

#endif

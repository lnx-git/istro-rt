#ifndef __CONTROL_BOARD_H__
#define __CONTROL_BOARD_H__

// Steering angle limits
#define SA_STRAIGHT	92
#define SA_MIN	SA_STRAIGHT-50
#define SA_MAX	SA_STRAIGHT+50
#define SA_FIX  (SA_STRAIGHT - 90)

// Velocity limits
#define VEL_ZERO	90
#define	VEL_MAX	VEL_ZERO+20
#define	VEL_MIN	VEL_ZERO-20
#define VEL_OPT	VEL_ZERO+8

const int RXBUFF_LENGTH = 1024;

class ControlBoard {
private:
    int serialPort;
    int serialPort2;

    int writef(int fd, const void *buf, int count);
    int readf(int fd, void *buf, int count);

private:
    int rxlen;
    unsigned char rxbuff[RXBUFF_LENGTH+1];

    int readFrame(int serialPort, unsigned char *buf, int count);
    int parseInt(unsigned char *buf, int count, int &idx, int &x);
    int parseDouble(unsigned char *buf, int count, int &idx, double &x);
    int parseFrameBnoEVC(unsigned char *buf, int count, double &euler_x, double &euler_y, double &euler_z, int &calib_gyro, int &calib_accel, int &calib_mag);

public:
    ControlBoard();

    int init(const char *portName, const char *portName2);
    void close(void);
    
    void start(void);
    void setSteeringAngle(int angle);
    void setSpeed(int speed);

    int getImuData(double &euler_x, double &euler_y, double &euler_z, int &calib_gyro, int &calib_accel, int &calib_mag);
};

#endif

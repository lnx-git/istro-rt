#ifndef __CONTROL_BOARD_H__
#define __CONTROL_BOARD_H__

/*
Zadefinnoval som to takoto:
#define S1_MIN 215
#define S1_CENTER 335
#define S1_MAX 455

#define S2_MIN 222
#define S2_CENTER 335
#define S2_MAX 445

S1 je servo riadenia, S1_MIN je plny pravy, S1_MAX je plny lavy, stred je odhadom S1_CENTER
S2 je motor, S2_MAX je plny dopredu, S2_min je plny dozadu, S_CENTER je stred (STOP).
*/

// Steering angle limits
#define SA_STRAIGHT    334    //340    // upravene 13.4.2019 na 340, lebo pri 334 zatacal moc vlavo
#define SA_MIN         215    //SA_STRAIGHT-50
#define SA_MAX         455    //SA_STRAIGHT+50
#define SA_FIX         (SA_STRAIGHT - 90)
#define SA_MULT        (120.0 / 90)

// Velocity limits
#define VEL_ZERO    335
#define VEL_MAX     VEL_ZERO + 445
#define VEL_MIN     VEL_ZERO - 222
#define VEL_OPT     VEL_ZERO + 8     // lipol: (8 + 7)
#define VEL_BACK    VEL_ZERO - 11    // lipol: (-10 - 5)
//#define VEL_HIGH    VEL_ZERO + 16

#define CTRLB_STATE_START  0   // nic nerobi
#define CTRLB_STATE_STOP   1   // zastavi robota, ale ak je predchadzajuci stav bol stop tak nic nerobi
#define CTRLB_STATE_FWD    2   // robot ide dopredu, pozera ci nie je prekazka pred robotom, prepina do stavu OBSTF 
#define CTRLB_STATE_BCK    3   // robot ide dozadu, pozera ci nie je prekazka za robotom robotom, prepina do stavu OBSTB
#define CTRLB_STATE_RECO   4   // recovery - nepouziva sa
#define CTRLB_STATE_OBSTF  5   // prekazka vpredu; nedovoli ist robotovi dopredu, ak predchadzajuci stav bol fwd tak zastavi robota
#define CTRLB_STATE_OBSTB  6   // prekazka vzadu; nedovoli ist robotovi dozadu, ak predchadzajuci stab bol bck tak zastavi robota
#define CTRLB_STATE_OBSTA  7   // prekazka vpredu aj vzadu; v podstate ako stop a nedovoli ist robotovi ani dopredu ani dozadu
#define CTRLB_STATE_EBTN   8   // emergency button; bolo stlacene emergency tlacitko, zastavi robota ak ebtn bol predchadzajuci stav iny ako ebtn inac nic nerobi

#define CTRLB_STATE_OBSTACLE(s) ((s == CTRLB_STATE_OBSTF) || (s == CTRLB_STATE_OBSTB) || (s == CTRLB_STATE_OBSTA) || (s == CTRLB_STATE_EBTN))

#define CTRLB_LED_PROGRAM_OFF    0
#define CTRLB_LED_PROGRAM_WHITE  1
#define CTRLB_LED_PROGRAM_RED    2
#define CTRLB_LED_PROGRAM_GREEN  3
#define CTRLB_LED_PROGRAM_BLUE   4

const int RXBUFF_LENGTH = 1024;

const int IRCTH_LENGTH        = 10+1;  // 10 elements = 200*10 = 2 seconds; +1 placeholder for next 200ms 
const int IRCTH_TIME_INTERVAL =  200;  // (milliseconds)
const int IRCTH_MAX_DTIME     = 5000;  // maximum time between two ircv samples (milliseconds)

// structure for Incremental Rotary Encoders time history data
typedef struct { 
    int    cnt;          // current number of elements
    double ircv[IRCTH_LENGTH];  // number of impulses received in particular 200ms interval
    double time_start0;  // start time for the first array element (tickcount)
    double time_last;    // time when the last data were received (tickcount) 
} ircth_t;

void   ircth_init(ircth_t &ircth);
int    ircth_process(ircth_t &ircth, int ircv, double time);
double ircth_get(ircth_t &ircth, double dt);
void   ircth_print(ircth_t &ircth);

int ctrlb_obstState(int ctrlb_state, int ctrlb_velocity);

class ControlBoard {
private:
    int serialPort;
    int serialPort2;

    int writef(int fd, const void *buf, int count);
    int readf(int fd, void *buf, int count);

private:
    int rxlen1;
    int rxlen2;
    unsigned char rxbuff1[RXBUFF_LENGTH+1];
    unsigned char rxbuff2[RXBUFF_LENGTH+1];

    ircth_t ircth;
    
    int readFrame(int fd, unsigned char *buf, int count, int &rxlen, unsigned char *rxbuff);
    int readLn(int fd, unsigned char *buf, int count, int &rxlen, unsigned char *rxbuff);

    int parseInt(unsigned char *buf, int count, int &idx, int &x);
    int parseDouble(unsigned char *buf, int count, int &idx, double &x);
    int parseFrameBnoEVC(unsigned char *buf, int count, double &euler_x, double &euler_y, double &euler_z, int &calib_gyro, int &calib_accel, int &calib_mag);

    int parseInt2(unsigned char *buf, int count, int &idx, int &x);
    int parseDouble2(unsigned char *buf, int count, int &idx, double &x);
    int parseFrameSrvData(unsigned char *buf, int count, 
            int &state, double &heading, int &ircv, int &angle, int &velocity, int &loadd, int &cbtime, 
            int &ulsd1, int &ulsd2, int &ulsd3, int &ulsd4, int &ulsd5);

public:
    ControlBoard();

    int init(const char *portName, const char *portName2);
    void close(void);
    
    void start(void);
    void stop(void);
    void setSteeringAngle(int angle);
    void setSpeed(int speed);
    void displayText(const char *str);
    void setLedIndex(int index, int r, int g, int b, int blink);
    void setLedMask(int mask, int r, int g, int b, int blink);
    void setLedProgram(int prg);
    void writeString(const char *str);
void setXX4(void); 
void setXX5(void);
    void setBallDrop(int cnt);

    int getImuData(double &euler_x, double &euler_y, double &euler_z, int &calib_gyro, int &calib_accel, int &calib_mag);
    int getServoData(int &state, double &heading, int &ircv, double &ircv500, int &angle, int &velocity, int &loadd, int &cbtime, 
                     int &ulsd1, int &ulsd2, int &ulsd3, int &ulsd4, int &ulsd5);
};

#endif

#ifndef __CONFIG_H__
#define __CONFIG_H__

static const double ANGLE_NONE = 999999;
static const double ANGLE_OK   = 999998;   // < ANGLE_OK

class Config {   
public:
    int useControlBoard;
    int useControlBoard2;
    int useGPSDevice;
    int useAHRSystem;
    int useLidar;
    int useNosave;            // do not save images to filesystem
    int useCamera;
    int useNoWait;            // do not wait after program start
    int useStartTime;
    int useNavigation;
    
    char* ControlBoardPortName;
    char* ControlBoard2PortName;
    char* LidarPortName;
    char* AHRSystemPortName;

    char* ControlBoardInitStr;
    
    int startHour;
    int startMinute;
    int waitDelay;            // in seconds
    
    // compass calibration - what gps course corresponds to which IMU (ahrs/ctrlb2) yaw
    int calibGpsAzimuth;
    int calibImuYaw;

    // navigation
    int navigationImuYaw;    // navigation towards fixed angle given by IMU (ahrs/ctrlb2) yaw
    char* navigationPath;

    int velocityFwd;
    int velocityBack;

public:    
    Config();

    int  parseArguments(int argc, char** argv);
    void printArguments(void);
};

#endif

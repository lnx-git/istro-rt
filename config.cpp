#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "navig.h"
#include "ctrlboard.h"
#include "logger.h"

LOG_DEFINE(loggerConfig, "Config");

Config::Config(void) 
{   
    useControlBoard = 0;
    useControlBoard2 = 0;
    useGPSDevice = 1;
    useAHRSystem = 0;
    useLidar = 0;
    useNosave = 0;
    useCamera = 1;
    useNoWait = 0;
    useStartTime = 0;
    useNavigation = 0;
    useQRScan = 1;

    ControlBoardPortName = NULL;
    ControlBoard2PortName = NULL;
    LidarPortName = NULL;
    AHRSystemPortName = NULL;

    ControlBoardInitStr = NULL;

    startHour = -1;
    startMinute = -1;
    waitDelay = 120;  //in seconds

    calibGpsAzimuth = (int)ANGLE_NONE;
    calibImuYaw = (int)ANGLE_NONE;

    navigationImuYaw = (int)ANGLE_NONE;    // navigation towards fixed angle given by ahrs yaw

    velocityFwd     = VEL_OPT - VEL_ZERO;
    velocityFwd2    = -1;
    velocityFwd3    = -1;
    velocityBack    = VEL_BACK - VEL_ZERO;
}

int Config::parseArguments(int argc, char** argv)
{
    // parse the command line arguments
    if( argc < 2) {
        printf("USAGE: <program name> -cb <control_board_device> -cb2 <control_board_2nd_device> -nogps -lidar <lidar_device> -ahrs <arhs_device> -nosave -nowait -h <start_hour> -m <start_min> -cg <gps_azimuth> -ca <ahrs_yaw> -navy <ahrs_yaw> -path <PxPyPz> -i <init_cmd>\n");
        LOGM_ERROR(loggerConfig, "parseArguments", "missing arguments!");
        return -1;
    }
    
    for(int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "-nosave") == 0) 
            useNosave=1;
        
        if (strcmp(argv[i], "-cb") == 0) { 
            if ((i + 1) < argc) {
                useControlBoard = 1; 
                ControlBoardPortName = argv[++i];
            }        
        }
      
        if (strcmp(argv[i], "-cb2") == 0) { 
            if ((i + 1) < argc) {
                useControlBoard2 = 1; 
                ControlBoard2PortName = argv[++i];
            }        
        }

        if (strcmp(argv[i], "-i") == 0) { 
            if ((i + 1) < argc) {
                ControlBoardInitStr = argv[++i];
            }        
        }
      
        if (strcmp(argv[i], "-ahrs") == 0) { 
            if ((i + 1) < argc) {
                useAHRSystem = 1; 
                AHRSystemPortName = argv[++i];
            }        
        }
      
        if (strcmp(argv[i], "-nogps") == 0) { 
            useGPSDevice = 0; 
        }
      
        if (strcmp(argv[i], "-lidar") == 0) { 
            if ((i + 1) < argc) {
                useLidar = 1; 
                LidarPortName = argv[++i];
            }        
        }
        
        if (strcmp(argv[i], "-nowait") == 0) 
            useNoWait=1;
      
        if (strcmp(argv[i], "-h") == 0) { 
            if ((i + 1) < argc) {
                startHour = atoi(argv[++i]);
                useStartTime = 1;
            }        
        }
      
        if (strcmp(argv[i], "-m") == 0) { 
            if ((i + 1) < argc) {
                startMinute = atoi(argv[++i]);
                useStartTime = 1;
            }        
        }
        
        if (strcmp(argv[i], "-cg") == 0) { 
            if ((i + 1) < argc) {
                if (useGPSDevice && (useAHRSystem || useControlBoard2)) { 
                    calibGpsAzimuth = atoi(argv[++i]);
                } else {
                    LOGM_WARN(loggerConfig, "parseArguments", "parameter \"-cg\" requires gps and ahrs/cb2!");
                }
            }        
        }
        
        if(strcmp(argv[i], "-ca") == 0) { 
            if ((i + 1) < argc) {
                if (useGPSDevice && (useAHRSystem || useControlBoard2)) { 
                    calibImuYaw = atoi(argv[++i]);
                } else {
                    LOGM_WARN(loggerConfig, "parseArguments", "parameter \"-ca\" requires gps and ahrs/cb2!");
                }
            }        
        }
      
        if(strcmp(argv[i], "-navy") == 0) { 
            if ((i + 1) < argc) {
                if (useAHRSystem || useControlBoard2) { 
                    navigationImuYaw = atoi(argv[++i]);
                } else {
                    LOGM_WARN(loggerConfig, "parseArguments", "parameter \"-navy\" requires ahrs/cb2!");
                }
            }        
        }
        if (strcmp(argv[i], "-path") == 0) { 
            if ((i + 1) < argc) {
                if (useGPSDevice) { 
                    navigationPath = argv[++i];
                    useNavigation = 1;
                } else {
                    LOGM_WARN(loggerConfig, "parseArguments", "parameter \"-path\" requires gps!");
                }
            }        
        }

        if (strcmp(argv[i], "-vf") == 0) { 
            if ((i + 1) < argc) {
                velocityFwd = atoi(argv[++i]);
            }
        }
        if (strcmp(argv[i], "-vf2") == 0) { 
            if ((i + 1) < argc) {
                velocityFwd2 = atoi(argv[++i]);
            }
        }
        if (strcmp(argv[i], "-vf3") == 0) { 
            if ((i + 1) < argc) {
                velocityFwd3 = atoi(argv[++i]);
            }
        }
        if (strcmp(argv[i], "-vb") == 0) { 
            if ((i + 1) < argc) {
                velocityBack = atoi(argv[++i]);
            }
        }

        if (strcmp(argv[i], "-p1la") == 0) { 
            if ((i + 1) < argc) {
                navigationPoint[0].latitude = atof(argv[++i]);
            }
        }
        if (strcmp(argv[i], "-p1lo") == 0) { 
            if ((i + 1) < argc) {
                navigationPoint[0].longitude = atof(argv[++i]);
            }
        }
        if (strcmp(argv[i], "-p2la") == 0) { 
            if ((i + 1) < argc) {
                navigationPoint[1].latitude = atof(argv[++i]);
            }
        }
        if (strcmp(argv[i], "-p2lo") == 0) { 
            if ((i + 1) < argc) {
                navigationPoint[1].longitude = atof(argv[++i]);
            }
        }
        if (strcmp(argv[i], "-p3la") == 0) { 
            if ((i + 1) < argc) {
                navigationPoint[2].latitude = atof(argv[++i]);
            }
        }
        if (strcmp(argv[i], "-p3lo") == 0) { 
            if ((i + 1) < argc) {
                navigationPoint[2].longitude = atof(argv[++i]);
            }
        }        
    }
    
    if ((startHour < 0) || (startMinute < 0)) {
        useStartTime = 0;
    }

    if (useStartTime) {
        useNoWait = 1;
    }

    if (useNavigation) {
        navigationImuYaw = (int)ANGLE_NONE;
    }

    /* update speeds so that: velocityFwd <= velocityFwd2 <= velocityFwd3 */
    if (velocityFwd2 < velocityFwd) {
        velocityFwd2 = velocityFwd;
    }
    if (velocityFwd3 < velocityFwd2) {
        velocityFwd3 = velocityFwd2;
    }
    
    return 0;
}

void Config::printArguments(void) 
{
    if(useControlBoard) {
        LOGM_INFO(loggerConfig, "printArguments", "ControlBoard=\"" <<  ControlBoardPortName << "\"");
        if(ControlBoardInitStr) {
            LOGM_INFO(loggerConfig, "printArguments", "CBInitStr=\"" << ControlBoardInitStr << "\"");
        }
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "ControlBoard=FALSE");
    }
    if(useControlBoard2) {
        LOGM_INFO(loggerConfig, "printArguments", "ControlBoard2=\"" <<  ControlBoard2PortName << "\"");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "ControlBoard2=FALSE");
    }
    if(useLidar) {
        LOGM_INFO(loggerConfig, "printArguments", "Lidar=\"" <<  LidarPortName << "\"");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "Lidar=FALSE");
    }
    if(useAHRSystem) {
        LOGM_INFO(loggerConfig, "printArguments", "AHRSystem=\"" <<  AHRSystemPortName << "\"");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "AHRSystem=FALSE");
    }
    if(useGPSDevice) {
        LOGM_INFO(loggerConfig, "printArguments", "GPS=TRUE");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "GPS=FALSE");
    }
    if(useCamera) {
        LOGM_INFO(loggerConfig, "printArguments", "Camera=TRUE");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "Camera=FALSE");
    }
    if(useNosave) {
        LOGM_INFO(loggerConfig, "printArguments", "NoSave=TRUE");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "NoSave=FALSE");
    }
    if(useNoWait) {
        LOGM_INFO(loggerConfig, "printArguments", "NoWait=TRUE");
    }
    if(useStartTime) {
        LOGM_INFO(loggerConfig, "printArguments", "StartTime=\"" << startHour << ":" << startMinute << "\"");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "StartTime=FALSE");
    }
    if(navigationImuYaw < (int)ANGLE_OK) {
        LOGM_INFO(loggerConfig, "printArguments", "NavigationImuYaw=" << navigationImuYaw);
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "NavigationImuYaw=FALSE");
    }
    if(useNavigation) {
        LOGM_INFO(loggerConfig, "printArguments", "NavigationPath=\"" << navigationPath << "\"");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "NavigationPath=FALSE");
    }
    if(useQRScan) {
        LOGM_INFO(loggerConfig, "printArguments", "QRScan=TRUE");
    } else {
        LOGM_INFO(loggerConfig, "printArguments", "QRScan=FALSE");
    }
    LOGM_INFO(loggerConfig, "printArguments", "CalibGpsAzimuth=" << calibGpsAzimuth);
    LOGM_INFO(loggerConfig, "printArguments", "CalibImuYaw=" << calibImuYaw);

    LOGM_INFO(loggerConfig, "printArguments", "VelocityFwd=" << velocityFwd);
    LOGM_INFO(loggerConfig, "printArguments", "VelocityFwd2=" << velocityFwd2);
    LOGM_INFO(loggerConfig, "printArguments", "VelocityFwd3=" << velocityFwd3);
    LOGM_INFO(loggerConfig, "printArguments", "VelocityBack=" << velocityBack);

    LOGM_INFO(loggerConfig, "printArguments", "NavP[0].name=\"" << (char *)navigationPoint[0].name << "\""
        << ", NavP[0].longitude=" << ioff(navigationPoint[0].longitude, 6) << ", NavP[0].latitude=" << ioff(navigationPoint[0].latitude, 6));
    LOGM_INFO(loggerConfig, "printArguments", "NavP[1].name=\"" << (char *)navigationPoint[1].name << "\""
        << ", NavP[1].longitude=" << ioff(navigationPoint[1].longitude, 6) << ", NavP[1].latitude=" << ioff(navigationPoint[1].latitude, 6));
    LOGM_INFO(loggerConfig, "printArguments", "NavP[2].name=\"" << (char *)navigationPoint[2].name << "\""
        << ", NavP[2].longitude=" << ioff(navigationPoint[2].longitude, 6) << ", NavP[2].latitude=" << ioff(navigationPoint[2].latitude, 6));
}

#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#ifndef WIN32

#define ISTRO_LOGGER_LOG4CXX
#define ISTRO_SERIAL_TERMIOS
//#define ISTRO_CAMERA_OPENCV
#define ISTRO_CAMERA_REALSENSE
#define ISTRO_CAMERA_RS_DEPTH          /* spracuvaj aj depth obrazky */
//#define ISTRO_RPLIDAR_V104
#define ISTRO_RPLIDAR_V106
//#define ISTRO_LIDAR_TIM571
#define ISTRO_GPSD_CLIENT
#define ISTRO_GEOGRAPHIC_LIB
#define ISTRO_QRSCANNER
#define ISTRO_LIBXML2
//#define ISTRO_MYAHRS_PLUS
//#define ISTRO_GUI
#define ISTRO_LIDAR_FILTERSUN          /* undefine to ignore LIDAR sun reflections (undefine in cloudy weather) */
//#define ISTRO_NAVMAP_NO_ROUTING      /* nebude pouzivat cesty v mape na routovanie - pre RoboOrienteering */
//#define ISTRO_MAP_BA_PARKAH
//#define ISTRO_MAP_BA_HRADZA
//#define ISTRO_MAP_BA_FEISTU
//#define ISTRO_MAP_BA_FEISTU2         /* bez odbocky na parkovisko pri Y2 */
//#define ISTRO_MAP_LEDNICE_ZZAHRADA
//#define ISTRO_MAP_PISEK_PALSADY
//#define ISTRO_MAP_PISEK_PALSADY2     /* bez odbociek pri starte Robotem Rovne */
#define ISTRO_MAP_BA_SADJK
//#define ISTRO_MAP_BA_NABREZIE
//#define ISTRO_MAP_MLAZNE_HAMRNIKY    /* RoboOrienteering */
//#define ISTRO_MAP_DEGGENDORF
//#define ISTRO_MAP_KLONDAJK
#define ISTRO_VISIONN
//#define ISTRO_VISION_YELLOWLANE      /* niekedy nemusi dobre detekovat slnkom oziarenu cestu */
//#define ISTRO_VISION_FISHEYE         /* deformacia kvoli fisheye efektu sirokouhlej kamery */
//#define ISTRO_VISION_BLACKMODE       /* za "travu" (OffRoad) bude povazovat vsetko tmave/cierne */
//#define ISTRO_VISION_ORANGECONE      /* namiesto vyhybaniu trave robota pritahuju kuzele - pre RoboOrienteering */
#define ISTRO_VISION_HDVIEW            /* zdeformovanie obrazku pre kameru 16:9 do obrazku 4:3 */
#define ISTRO_VISION_DEPTH
//#define ISTRO_NAVIG_BALLDROP         /* namiesto AREA_UN/LOADING, vykladanie lopticiek - pre RoboOrienteering */
//#define ISTRO_WRONGWAY_DISABLE
#define ISTRO_CALIB2                   /* priebezne loguj vysledky "fake" kalibracie */

#else

//#define ISTRO_LOGGER_LOG4CXX
//#define ISTRO_SERIAL_TERMIOS
//#define ISTRO_CAMERA_OPENCV
//#define ISTRO_CAMERA_REALSENSE
//#define ISTRO_RPLIDAR_V104
//#define ISTRO_RPLIDAR_V106
//#define ISTRO_LIDAR_TIM571
//#define ISTRO_GPSD_CLIENT
//#define ISTRO_GEOGRAPHIC_LIB
//#define ISTRO_QRSCANNER
//#define ISTRO_LIBXML2
//#define ISTRO_MYAHRS_PLUS
//#define ISTRO_GUI
#define ISTRO_LIDAR_FILTERSUN          /* undefine to ignore LIDAR sun reflections (undefine in cloudy weather)*/
//#define ISTRO_NAVMAP_NO_ROUTING      /* nebude pouzivat cesty v mape na routovanie - pre RoboOrienteering */
//#define ISTRO_MAP_BA_PARKAH
//#define ISTRO_MAP_BA_HRADZA
//#define ISTRO_MAP_BA_FEISTU
//#define ISTRO_MAP_BA_FEISTU2         /* bez odbocky na parkovisko pri Y2 */
//#define ISTRO_MAP_LEDNICE_ZZAHRADA
//#define ISTRO_MAP_PISEK_PALSADY
//#define ISTRO_MAP_PISEK_PALSADY2     /* bez odbociek pri starte Robotem Rovne */
#define ISTRO_MAP_BA_SADJK
//#define ISTRO_MAP_BA_NABREZIE
//#define ISTRO_MAP_MLAZNE_HAMRNIKY    /* RoboOrienteering */
//#define ISTRO_MAP_DEGGENDORF
//#define ISTRO_MAP_KLONDAJK
//#define ISTRO_VISIONN
//#define ISTRO_VISION_YELLOWLANE      /* niekedy nemusi dobre detekovat slnkom oziarenu cestu */
//#define ISTRO_VISION_FISHEYE         /* deformacia kvoli fisheye efektu sirokouhlej kamery */
//#define ISTRO_VISION_BLACKMODE       /* za "travu" (OffRoad) bude povazovat vsetko tmave/cierne */
//#define ISTRO_VISION_ORANGECONE      /* namiesto vyhybaniu trave robota pritahuju kuzele - pre RoboOrienteering */
#define ISTRO_VISION_HDVIEW            /* zdeformovanie obrazku pre kameru 16:9 do obrazku 4:3 */
#define ISTRO_VISION_DEPTH
//#define ISTRO_NAVIG_BALLDROP         /* namiesto AREA_UN/LOADING, vykladanie lopticiek - pre RoboOrienteering */
//#define ISTRO_WRONGWAY_DISABLE
#define ISTRO_CALIB2                   /* priebezne loguj vysledky "fake" kalibracie */

#endif

//#define CTRLBOARD_LOG_TRACE0         /* logovanie casov na urovni ctrlboard */
//#define THREADS_LOG_TRACE0           /* logovanie stavu buffrov Threads::printState() */
//#define SAVETHREAD_LOG_TRACE0        /* logovanie ignorovanych frejmov a prazdnych cakani v save_thread */
//#define THDATA_LOG_TRACE0            /* logovanie casov readData/writeData() */


#if defined(ISTRO_RPLIDAR_V104) || defined(ISTRO_RPLIDAR_V106)
#define ISTRO_RPLIDAR
#endif

int msleep(long ms);

int openSerialPort(const char* portName, int speed);
void closeSerialPort(int serialPort);

int writeSerialPort(int serialPort, const void *buf, int count);
int readSerialPort(int serialPort, void *buf, int count);

#endif

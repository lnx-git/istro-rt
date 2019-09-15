#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>

using namespace std;

const char LINE_END  = '\n';
const char LINE_END2 = '\r';
const int LINE_SIZE = 1024;
const int BUFFER_SIZE = 1024 * 1024;

#ifndef WIN32

const char *LOG_FNAME      = "/home/istrobotics/projects/istro_rt2019/out/istro_rt2019.log";
const char *OUT_TXT_FNAME  = "/var/www/html/ramdisk/out/istro_rt2019_out.txt";
const char *OUT_HTML_FNAME = "/var/www/html/ramdisk/out/istro_rt2019_out.html";
const char *OUT_JSON_FNAME = "/var/www/html/ramdisk/out/istro_rt2019_out.json";
//const char *OUT_TXT_FNAME  = "/tmp/ramdisk/out/istro_rt2019_out.txt";
//const char *OUT_HTML_FNAME = "/tmp/ramdisk/out/istro_rt2019_out.html";
//const char *OUT_JSON_FNAME = "/tmp/ramdisk/out/istro_rt2019_out.json";


/* ubuntu */
//const char *LOG_FNAME      = "/home/tf/projects/istro_rt2019/out/istro_rt2019.log";
//const char *OUT_TXT_FNAME  = "/home/tf/projects/istro_rt2019/out/istro_rt2019_out.txt";
//const char *OUT_HTML_FNAME = "/home/tf/projects/istro_rt2019/out/istro_rt2019_out.html";
//const char *OUT_JSON_FNAME = "/home/tf/projects/istro_rt2019/out/istro_rt2019_out.json";

#else

const char *LOG_FNAME      = "C:\\Private\\robot\\istrobtx\\sources\\istro_rt2019\\out\\istro_rt2019.log";
const char *OUT_TXT_FNAME  = "C:\\Private\\robot\\istrobtx\\sources\\istro_rt2019\\out\\istro_rt2019_out.txt";
const char *OUT_HTML_FNAME = "C:\\Private\\robot\\istrobtx\\sources\\istro_rt2019\\out\\istro_rt2019_out.html";
const char *OUT_JSON_FNAME = "C:\\Private\\robot\\istrobtx\\sources\\istro_rt2019\\out\\istro_rt2019_out.json";

#endif

const char *OUT_IMG_DNAME = "out/";

char buffer[BUFFER_SIZE + 1];

int msleep(long ms)
{
    struct timespec t;

    if (ms < 1000) {   
        t.tv_sec = 0;
        t.tv_nsec = ms * 1000000;
    } else {   
        t.tv_sec = (int)(ms / 1000);
        t.tv_nsec = (ms - ((long)t.tv_sec * 1000)) * 1000000;
    }   

    return nanosleep(&t, NULL);
}

typedef struct { 
    const char  *str1;
    const char  *str2;
    int   dd;      // distance in characters 
    int   flags;
} find_pattern_t;

const int FIND_FLAGS_CAMERA_IMG = 1;
const int FIND_FLAGS_VISION_IMG = 2;
const int FIND_FLAGS_LIDAR_IMG  = 3;
const int FIND_FLAGS_WMGRID_IMG = 4;
const int FIND_FLAGS_NAVMAP_IMG = 5;

const int FIND_PATTERN_COUNT = 16;

find_pattern_t findPattern[FIND_PATTERN_COUNT] = 
{
    { "saveImage(): camera_image=", NULL, 0, FIND_FLAGS_CAMERA_IMG },
    { "saveImage(): vision_image=", NULL, 0, FIND_FLAGS_VISION_IMG },
    { "saveImage(): lidar_image=",  NULL, 0, FIND_FLAGS_LIDAR_IMG },
    { "saveImage(): wmgrid_image=", NULL, 0, FIND_FLAGS_WMGRID_IMG },
    { "saveImage(): navmap_image=", NULL, 0, FIND_FLAGS_NAVMAP_IMG },
    { "process_thread(): process_angle", NULL, 0, 0 },         // istro::process_thread(): process_angle("NAV_ANGLE"): 
    { "gps_writeData(): fix=", NULL, 0, 0 },                   // "gps_writeData....fix="
    { "process_readData(): process_change", NULL, 0, 0 },      // istro::process_readData(): process_change=52, gps_speed=0.021, 
    { "INFO", "calib_process", -1, 0 },                        // INFO  [process] istro::calib_process(): msg="calibration finished!", 
    { "calib_reset", NULL, 0, 0 },                             // INFO  [process] istro::calib_reset(): msg="calibration interrupted!", reason="gps_course difference",
    { "INFO", "wrongway_check", -1, 0 },                       // INFO  [process] istro::wrongway_check(): msg="wrongway-check start!", 
    { "wrongway_process", NULL, 0, 0 },                        // istro::wrongway_process(): msg="wrongway in progress (STOP2)...!",
    { "navigation point passed", NULL, 0, 0 },                 // istro::gps_thread(): msg="navigation point passed!", pos=2, name="M4", ...
    { "navig::navigation", NULL, 0, 0 },                       // navig::navigation_next_point(): msg="point found!", name="M1", ...  / navig::navigation_point_get(): msg="coordinates acquired!", name="M1", ...
    { "loadarea_process", NULL, 0, 0 },                        // istro::loadarea_process(): msg="loading area - waiting!", ... / "unloading area - waiting!" / "qrscan coordinates - waiting!"
    { "ControlBoard::write(): data=\"D", NULL, 0, 0 }          // ControlBoard::write(): data="DAf: S0 V343 A335 G999"
};

//  { "process_thread", "process_angle", 4, 0 },

char findResult[FIND_PATTERN_COUNT][LINE_SIZE + 1];

// fixme: possible buffer overflow
char * strcpy_edq(char * dst, const char * src)
// strcpy with inserting escape characters for double quotes:  msg="wrongway" -> msg=\"wrongway\"
{
    char * res = dst;
    while (*src != 0) {
       if (*src == '"') {
           *(dst++) = '\\';
       }
       *(dst++) = *(src++);
    }
    *dst = 0;
    return res;
}

int process_line(char const *p, size_t n)
{
    char line[LINE_SIZE + 1];
    if (n > LINE_SIZE) {
        n = LINE_SIZE;
    }

    memcpy(line, p, n);
    line[n] = 0;
//printf("process_line: \"%s\"\n", line);

    for(int i = 0; i < FIND_PATTERN_COUNT; i++) { 
//printf("pattern[%d]\n", i);
        char *ss;
        char *pl = line;
        int found = 0;
        do {
            ss = strstr(pl, findPattern[i].str1);
            if (ss == NULL) {
//printf("str1 not found\n");
                pl = NULL;
                continue;  // not found
            }
            if (findPattern[i].str2 == NULL) {
//printf("str1 FOUND\n");
                found = 1;
                continue;  // found "str1"
            }
            if (findPattern[i].str2 != NULL) {
                if (findPattern[i].dd >= 0) {
//printf("memcmp\n");
                    int ll1 = strlen(findPattern[i].str1) + findPattern[i].dd;
                    int ll2 = strlen(findPattern[i].str2);
                    if ((&(ss[ll1 + ll2]) - line) <= (int)n) {
                        int rr = memcmp(&(ss[ll1]), findPattern[i].str2, ll2);
                        if (rr == 0) {
//printf("str1...str2 FOUND\n");
                            found = 1;
                            continue;  // found "str1...str2"
                        }
                    }
                } else {
                    char *ss2;
//printf("strstr\n");
                    ss2 = strstr(ss, findPattern[i].str2);
                    if (ss2 != NULL) {
//printf("str1.*str2 FOUND\n");
                        found = 1;
                        continue;  // found "str1.*str2"
                    }
                }
            }            
//printf("next\n");
            pl = ss;
            pl++;
        } while ((pl != NULL) && (found == 0));
        if (found) {
//printf("result: FOUND\n");
            if (findPattern[i].flags == 0) {
                strcpy(&(findResult[i][0]), line);
            } else {
//printf("result: FOUND image\n");
                int ll = strlen(findPattern[i].str1);
                if (ss[ll] == '\"') {
//printf("result: found image - #1\n");
                    char *ss2;
                    ss2 = strstr(&(ss[ll+1]), "\"");
                    if (ss2 != NULL) {
//printf("result: found image - #2\n");
                        int ll2 = ss2 - ss - ll - 1;
                        memcpy(&(findResult[i][0]), &(ss[ll+1]), ll2);
                        findResult[i][ll2] = 0;
//printf("result: found image: \"%s\"\n", &(findResult[i][0]));
                    }
                }
//printf("result: found image - end\n");
            }
        } else {
//printf("result: not found\n");
        }
    }

    return 0;
}

size_t buffer_cnt = 0;    // number of unprocessed bytes in buffer
long int off = 0;

int init()
{
//printf("init\n");
    buffer_cnt = 0;
    off = 0;
    for(int i = 0; i < FIND_PATTERN_COUNT; i++) { 
        findResult[i][0] = 0;
    }
    return 0;
}

int writeResult()
{
    FILE *f;

    /* write TXT */
    f = fopen(OUT_TXT_FNAME, "w");
    if (f == NULL) {
        printf("Error opening output file: \"%s\"!\n", OUT_TXT_FNAME);
        return -1;
    }

    for(int i = 0; i < FIND_PATTERN_COUNT; i++) { 
        fprintf(f, "%s\n", findResult[i]);
    }

    fclose(f);

    /* write HTML */
    f = fopen(OUT_HTML_FNAME, "w");
    if (f == NULL) {
        printf("Error opening output file: \"%s\"!\n", OUT_HTML_FNAME);
        return -2;
    }

    fprintf(f, "<!DOCTYPE html>\n<html>\n<body>\n");
    fprintf(f, "<p style=\"font-family:Courier; font-size: 20px\">\n");
    for(int i = 0; i < FIND_PATTERN_COUNT; i++) { 
        fprintf(f, "%s<br>\n", findResult[i]);
    }
    fprintf(f, "</p>\n</body>\n</html>\n");

    fclose(f);

    f = fopen(OUT_JSON_FNAME, "w");
    if (f == NULL) {
        printf("Error opening output file: \"%s\"!\n", OUT_JSON_FNAME);
        return -3;
    }

    fprintf(f, "{\n");
    for(int i = 0; i < FIND_PATTERN_COUNT; i++) { 
        if (findResult[i][0] != 0) {
            if (findPattern[i].flags == FIND_FLAGS_CAMERA_IMG) {
                fprintf(f, "  \"camera_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult[i]);
            } else 
            if (findPattern[i].flags == FIND_FLAGS_VISION_IMG) {
                fprintf(f, "  \"vision_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult[i]);            
            } else 
            if (findPattern[i].flags == FIND_FLAGS_LIDAR_IMG) {
                fprintf(f, "  \"lidar_image\":  \"%s%s\",\n", OUT_IMG_DNAME, findResult[i]);            
            } else 
            if (findPattern[i].flags == FIND_FLAGS_WMGRID_IMG) {
                fprintf(f, "  \"wmgrid_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult[i]);            
            } else 
            if (findPattern[i].flags == FIND_FLAGS_NAVMAP_IMG) {
                fprintf(f, "  \"navmap_image\": \"%s%s\",\n", OUT_IMG_DNAME, findResult[i]);            
            }
        }
    }
    fprintf(f, "  \"items\": [\n");
    int found = 0;
    char line[LINE_SIZE + 1];
    for(int i = 0; i < FIND_PATTERN_COUNT; i++) { 
        if (findResult[i][0] != 0) {
            if (findPattern[i].flags == 0) {
                if (found) {
                    fprintf(f, ",\n");
                }
                found = 1;
                fprintf(f, "    \"%s\"", strcpy_edq(line, findResult[i]));
            }
        }
    }
    fprintf(f, "\n  ]\n}\n");

    fclose(f);

    return 0;
}

int main()
{
    FILE *fd = NULL;

    init();
    while (true) {
        /* open & seek */
        if (fd == NULL) {
            fd = fopen(LOG_FNAME, "rb");
            if (fd == NULL) {
                init();
printf("open: \"error: could not open file!\"\n");
            } else
            if (off > 0) {
//printf("lseek: off=%d\n", (int)off);
                if (fseek(fd, off, SEEK_SET) != 0) {
printf("lseek: \"fseek error\"\n");
                    fclose(fd);
                    fd = NULL;
                    init();
                }
            }
        }

        /* read &  process */
        if (fd != NULL) {
            int buffer_full = 0;
            do {
                /* read next bytes until end of buffer or until end of file */
                size_t read_cnt;
                read_cnt = fread(&(buffer[buffer_cnt]), 1, BUFFER_SIZE - buffer_cnt, fd);
                if ((read_cnt < 0) || (read_cnt > BUFFER_SIZE - buffer_cnt)) {
printf("read: \"error2: could not read from file!\"\n");
                    read_cnt = 0;
                    fclose(fd);
                    fd = NULL;
                    init();
                    break;
                }
    
                off += read_cnt;
                buffer_cnt += read_cnt;
                buffer_full = (buffer_cnt == BUFFER_SIZE);
//printf("read_cnt=%d, buffer_cnt=%d, buffer_full=%d\n", (int)read_cnt, (int)buffer_cnt, buffer_full);
            
                char const *pb = buffer;
            
                size_t n = buffer_cnt;
                while (n > 0) {
                    size_t nl;
                    char const *pl;
            
                    pl = (char const *)memchr(pb, LINE_END, n);
                    if (pl == NULL)
                        break;
        
                    nl = pl - pb;  // chars to be processed
//printf("nl=%d, n=%d\n", nl, n);
    
                    if ((nl > 0) && (*(pl-1) == LINE_END2)) {
//printf("line_end2\n");
                        process_line(pb, nl - 1);
                    } else {
                        process_line(pb, nl);
                    }
            
                    pb = pl;
                    pb++;
                    n -= nl + 1;
                }
            
//printf("n=%d\n", n);
                // line end not found in buffer? ignore all characters
                if (n == BUFFER_SIZE) {
                    buffer_cnt = 0;
                } else
                // some unprocessed data left?
                if (n > 0) {
                    buffer_cnt = n;
                    memcpy(buffer, pb, n);        
                } else {
                    buffer_cnt = 0;
                } 
            } while (buffer_full);
        }

        /* write, close & sleep */
        if (fd != NULL) {  
            writeResult();
            fclose(fd);
            fd = NULL;
        }
        msleep(100);
    }

//printf("end...");
}

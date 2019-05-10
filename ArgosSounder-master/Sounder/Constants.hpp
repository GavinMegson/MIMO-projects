#ifndef CONSTANTS 
#define CONSTANTS 

#define ENABLE_CPU_ATTACH
//#define DEBUG1
//#define DEBUG2

#define EVENT_PACKAGE_RECEIVED 0
#define EVENT_CROPPED 1
#define EVENT_ZF 2
#define EVENT_DEMUL 3


#define TASK_CROP 0
#define TASK_ZF 1
#define TASK_DEMUL 2

static const int MAX_FRAME_INC = 2e3;

#endif

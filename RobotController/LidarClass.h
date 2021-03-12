#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

	void readRegmark(char *regmark);  //这里写函数声明

#ifdef __cplusplus
}
#endif





#define VERSION 10
#define DATETIME_MAX 16
#define STR_MAX 256

#define  VERTICAL_MANUAL_MODE 0
#define  VERTICAL_AUTO_MODE 1
#define  HORIZONTAL_MODE 2

using namespace std;

// Define only for URG debugging
//#define RAW_OUTPUT

#if defined(RAW_OUTPUT)
static FILE* Raw_fd_ = NULL;
#endif


struct sensorFrameXY{
	float x;
	float y;
};


enum {
  Timeout = 1000,               // [msec]
  EachTimeout = 2,              // [msec]
  LineLength = 64 + 3 + 1 + 1 + 1 + 16,
};

static HANDLE HCom = INVALID_HANDLE_VALUE;
static int ReadableSize = 0;
static char* ErrorMessage = "no error.";


typedef struct {
  enum {
    MODL = 0,		//!< Sensor Model
    DMIN,			//!< Min detection range [mm]
    DMAX,			//!< Man detection range [mm]
    ARES,			//!< Angular resolution (division of 360degree)
    AMIN,			//!< Min Measurement step
    AMAX,			//!< Max Measurement step
    AFRT,			//!< Front Step 
    SCAN,			//!< Standard scan speed
  };
  string model;		//!< Obtained Sensor Model,  MODL
  long distance_min;		//!< Obtained DMIN 
  long distance_max;		//!< Obtained DMAX 
  int area_total;		//!< Obtained ARES 
  int area_min;			//!< Obtained AMIN 
  int area_max;			//!< Obtained AMAX 
  int area_front;		//!< Obtained AFRT 
  int scan_rpm;			//!< Obtained SCAN 

  int first;			//!< Scan Start position
  int last;			//!< Scan end position
  int max_size;			//!< Max. size of data
  long last_timestamp; //!< Time stamp of the last obtained data
} urg_state_t;

 void delay(int msec);
 int com_changeBaudrate(long baudrate);
 int com_connect(const char* device, long baudrate) ;
 void com_disconnect(void);
 int com_send(const char* data, int size);
 int com_recv(char* data, int max_size, int timeout);
 int urg_sendTag(const char* tag);
 int urg_readLine(char *buffer);
 int urg_sendMessage(const char* command, int timeout, int* recv_n);
 int urg_changeBaudrate(long baudrate);
 int urg_getParameters(urg_state_t* state);
 int urg_connect(urg_state_t* state,const char* port, const long baudrate);
 void urg_disconnect(void);
 int urg_captureByGD(const urg_state_t* state);
 int urg_captureByMD(const urg_state_t* state, int capture_times, unsigned int CaptureAngle);
 long urg_decode(const char data[], int data_byte);
 int urg_addRecvData(const char buffer[], long data[], int* filled);
 int urg_addRecvDataTimeStamp(const char buffer[], long data[], int* filled);
 int checkSum(char buffer[], int size, char actual_sum);
 int urg_receiveData(urg_state_t* state, long data[], size_t max_size);
void outputData(long data[], int n, size_t total_index);

class LidarClass
{
public:

    sensorFrameXY sensorFrame[1081];

	LidarClass(void);
	~LidarClass(void);
	int * m_scan_data;
	unsigned int m_scanline_num;
	char m_lidar_com[6];

	char m_lidar_portnum[3];

	int m_lidar_baudrate;
	int m_lidar_err;
	char m_FileName[256];
	urg_state_t m_Lidar_state;
	FILE *m_OutputFile;
	HANDLE m_hComm;
	int m_Lidar_InstMode;

	unsigned int m_CaptureAngle;
	char filename[30];
	time_t now_time;
	struct tm *scan_time;
public:
	bool Initial(void);
	bool LidarClass::Initial(char* Lidar_ComNum);
	bool Scan(char *filename);
	bool closeConnect(void);
	//bool connect();
};

#pragma once
#include <iostream>		// Needed for printf etc
#include <objbase.h>	// Needed for COM functionality
#include "xsens_cmt.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <conio.h>		// included for _getch and _kbhit
#include <time.h>
#include <math.h>
//#include"PTUDriverMFCDlg.h"

using namespace std;

// this macro tests for an error and exits the program with a message if there was one
//#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printf("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

typedef struct OData{
	CmtCalData caldata;
	CmtEuler euler_data;
	CmtVector velocity_data;
	CmtMatrix matrix_data;
	double x;
	double y;
	double z;
}ObtainData;

// used to signal that the user initiated the exit, so we do not wait for an extra keypress-
class IMUClass
{
private:
	HANDLE thread;
public:
	IMUClass();
	~IMUClass();
	void HardwareScan(int portNum, int baudRate);
	void MtSettings();
	void StartCmt(int portNum, int baudRate);
	void stopCmt();
	void RecordCmt(char filename[]);
};


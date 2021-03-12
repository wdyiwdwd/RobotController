#pragma once
#include <stdio.h>		// Needed for printf etc
#include <objbase.h>	// Needed for COM functionality
#include "xsens_cmt.h"
#include <conio.h>		// included for _getch and _kbhit
//#include"PTUDriverMFCDlg.h"

// this macro tests for an error and exits the program with a message if there was one
//#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printf("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }


typedef struct OData{
	CmtCalData caldata;
	CmtEuler euler_data;
	CmtVector velocity_data;
	double latitude;
	double longitude;
	double altitude;
	}ObtainData;
// used to signal that the user initiated the exit, so we do not wait for an extra keypress-


class XsensCmtControl
{
public:
	HANDLE thread;
public:
	XsensCmtControl();
	~XsensCmtControl();
	void HardwareScan(int portNum, int baudRate);
	void MtSettings();
	void StartCmt(int portNum, int baudRate);
	void stopCmt();
	void RecordCmt(char filename[]);	
};

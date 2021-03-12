#include "stdafx.h"
#include "CameraClass.h"


CameraClass::CameraClass()
{
	this->startupTime = 1;
	this->captureTimes = 1;
	this->cameraNum = 0;
}

CameraClass::CameraClass(double startupTime, int captureTimes, int cameraNum)
{
	this->startupTime = startupTime;
	this->captureTimes = captureTimes;
	this->cameraNum = cameraNum;
}

CameraClass::~CameraClass()
{
}

bool CameraClass::takePicture(char filename[]) 
{
	this->capture = cvCreateCameraCapture(this->cameraNum);
	clock_t start = clock();
	//.....³ÌĞòÖ÷Ìå

	for (int i = 0; i < this->captureTimes; i++){
		while (1){
			this->image = cvQueryFrame(this->capture);
			if ((double)(clock() - start) / CLOCKS_PER_SEC > this->startupTime) break;
		}
		cvSaveImage(filename, this->image);
	}
	//delete this->capture;
	//delete this->image;
	cvReleaseCapture(&capture);
	return true;
}

void CameraClass::setStartupTime(double startupTime){
	this->startupTime = startupTime;
}

void CameraClass::setCaptureTimes(int captureTimes){
	this->captureTimes = captureTimes;
}

void CameraClass::setCameraNum(int cameraNum){
	this->cameraNum = cameraNum;
}
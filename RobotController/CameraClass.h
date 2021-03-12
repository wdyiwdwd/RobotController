#pragma once
#include <memory>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

using namespace std;

class CameraClass
{
private:
	CvCapture* capture;
	IplImage* image;
	double  startupTime;
	int captureTimes;
	int cameraNum;
public:
	CameraClass();
	CameraClass(double startupTime, int captureTimes, int cameraNum);
	~CameraClass();
	bool takePicture(char filename[]);
	void setStartupTime(double startupTime);
	void setCaptureTimes(int captureTimes);
	void setCameraNum(int cameraNum);
};


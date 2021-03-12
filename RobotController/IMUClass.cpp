#include "IMUClass.h"
#define KEY "0445F-0F456-486A9-4851A"
//平均次数
#define MeanTimes 10
//增加最近多次测量结果求平均的功能，以消弱测量误差带来的影响。
long instance = -1;
//临时保存单次数据
ObtainData obtainData;
//保存最终数据的结构
ObtainData finalObtainData;
//保存最初的的旋转矩阵
cv::Mat firstMatrix;
//保存最初的的旋转欧拉角
cv::Mat firstEuler;
int userQuit;
CmtOutputMode mode;
CmtOutputSettings settings;
unsigned short mtCount = 0;
int screenSensorOffset;
int temperatureOffset;
CmtDeviceId deviceIds[256];
//xsens::Cmt3 cmt3;
//Packet* packet;
CmtVector positionLLA;
//目前累计次数
int measureTimes = 1;
//上一次的计数时间
clock_t lasttime = 0;

IMUClass::IMUClass()
{
	userQuit = 0;
	mtCount = 0;
	screenSensorOffset = 0;
	temperatureOffset = 0;
}

IMUClass::~IMUClass()
{
}

void IMUClass::HardwareScan(int portNum, int baudRate)
{

	//感觉像是硬件扫描一般的东西，但是不知道什么作用，
	//因为这里并没有给端口号之类的
	XsensResultValue res;
	CmtPortInfo portInfo[256];
	uint32_t portCount = 1;
	char serialNumber[] = KEY;
	//if (strcmp(serialNumber,"b8r6RCoGjQJVsytwUMo8WCRiJiVCCdoL11cCj4HqnaKPHtTn") == 0)
	//printf("Warning: Using the demo key as a serial code will limit CMT functionality to 1000 calls. Enter your own serial code for unlimited CMT functionality.\n");

	instance = cmtCreateInstance(serialNumber);
	if (instance != -1)
		printf("CMT instance created\n\n");
	else {
		printf("Creation of CMT instance failed, probably because of an invalid serial number\n");
		exit(1);
	}

	res = cmtScanPort(portNum, &portInfo[0], baudRate, 3000, 5);

	//printf("Scanning for connected Xsens devices...");
	//res = cmtScanPorts(portInfo, &portCount, 115200, 3000, 5);
	//EXIT_ON_ERROR(res,"cmtScanPorts");
	//printf("done\n");


	//这个线程感觉在这应该就终止了吧。
	//未找到IMU设备，所以return了
	if (portCount == 0) {
		//printf("No MotionTrackers found\n\n");
		printf("未找到IMU设备！\n");
		//MessageBox(dialog->m_hWnd,_T("未找到IMU设备！"),0,0);
		//MessageBox(dialog->m_hWnd, "hahfaf", 0, 0);

		return;
		//exit(0);
	}

	//for(int i = 0; i < (int)portCount; i++) {	
	////	printf("Using COM port %d at %d baud\n\n",
	//		//(long) portInfo[i].m_portNr, portInfo[i].m_baudrate);	
	//}
	//
	//printf("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for (int p = 0; p < (int)portCount; p++){
		res = cmtOpenPort(instance, portInfo[p].m_portNr, portInfo[p].m_baudrate);
		if (res != XRV_OK)
			printf("打开IMU设备失败！\n");
		//exit(1);
	}
	//printf("done\n\n");

	//get the Mt sensor count.
	//printf("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	res = cmtGetMtCount(instance, &mtCount);
	if (res != XRV_OK)
		printf("获取IMU设备数量失败！\n");
	//exit(1);
	//printf("MotionTracker count: %i\n\n",mtCount);

	// retrieve the device IDs 
	//printf("Retrieving MotionTrackers device ID(s)\n");
	for (unsigned int j = 0; j < mtCount; j++){
		res = cmtGetMtDeviceId(instance, &deviceIds[j], j);
		if (res != XRV_OK)
			printf("获取DeviceID失败！\n");
		//exit(1);
		//printf("Device ID at index %i: %08x\n",j,(long) deviceIds[j]);
	}

	// make sure that we get the freshe3st data
	//printf("\nSetting queue mode so that we always get the latest data\n\n");
	res = cmtSetQueueMode(instance, CMT_QM_LAST);
	if (res != XRV_OK)
		printf("设置工作方式失败！\n");
	//exit(1);

}
void IMUClass::MtSettings(void)
{
	XsensResultValue res = XRV_OK;
	mode = CMT_OUTPUTMODE_POSITION | CMT_OUTPUTMODE_VELOCITY | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
	settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
	res = cmtGotoConfig(instance);
	//if (res != XRV_OK)
	//MessageBox(dialog->m_hWnd,_T("进入配置模式失败！"),0,0);
	//EXIT_ON_ERROR(res, "cmtGotoConfig");

	unsigned short sampleFreq;
	res = cmtGetSampleFrequency(instance, &sampleFreq, deviceIds[0]);

	// set the device output mode for the device(s)
	//printf("Configuring your mode selection");
	for (int i = 0; i<mtCount; i++) {
		if (cmtIdIsMtig(deviceIds[i])) {
			res = cmtSetDeviceMode(instance, mode, settings, sampleFreq, deviceIds[i]);
		}
		else {
			res = cmtSetDeviceMode(instance, mode & 0xFF0F, settings, sampleFreq, deviceIds[i]);
		}
		//if (res != XRV_OK)
		//MessageBox(dialog->m_hWnd,_T("设置设备模式失败！"),0,0);
		//EXIT_ON_ERROR(res, "setDeviceMode");
	}

	// start receiving data
	res = cmtGotoMeasurement(instance);
	//if (res != XRV_OK)
	//MessageBox(dialog->m_hWnd,_T("回到测量模式失败！"),0,0);
	//EXIT_ON_ERROR(res, "cmtGotoMeasurement");
}
DWORD WINAPI CMTProcess(LPVOID lpParam)
{

	XsensResultValue res = XRV_OK;
	//packet = new Packet((unsigned short)mtCount,cmt3.isXm());
	double tdata;
	unsigned short sdata;

	res = cmtGetNextDataBundle(instance);
	res = cmtDataGetSampleCounter(instance, &sdata, deviceIds[0], NULL);
	finalObtainData.x = 0;
	finalObtainData.y = 0;
	finalObtainData.x = 0;
	res = cmtDataGetOriMatrix(instance, &obtainData.matrix_data, deviceIds[0], NULL);
	firstMatrix  = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	for (int i = 0; i < 3; i++) {
		double* pxvec = firstMatrix.ptr<double>(i);
		for (int j = 0; j < 3; j++) {
			finalObtainData.matrix_data.m_data[i][j] = obtainData.matrix_data.m_data[i][j];
			pxvec[j] = obtainData.matrix_data.m_data[i][j];
		}
	}
	res = cmtDataGetOriEuler(instance, &obtainData.euler_data, deviceIds[0], NULL);
	firstEuler = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	double* pxvec = firstEuler.ptr<double>(0);
	pxvec[0] = obtainData.euler_data.m_roll;
	pxvec[1] = obtainData.euler_data.m_pitch;
	pxvec[2] = obtainData.euler_data.m_yaw;
	res = cmtDataGetVelocity(instance, &obtainData.velocity_data, deviceIds[0]);
	finalObtainData.velocity_data.m_data[0] = obtainData.velocity_data.m_data[0];
	finalObtainData.velocity_data.m_data[1] = obtainData.velocity_data.m_data[1];
	finalObtainData.velocity_data.m_data[2] = obtainData.velocity_data.m_data[2];
	lasttime = clock();
	while (res == XRV_OK) {
		res = cmtGetNextDataBundle(instance);
		res = cmtDataGetSampleCounter(instance, &sdata, deviceIds[0], NULL);
		//  获取方向矩阵
		res = cmtDataGetOriMatrix(instance, &obtainData.matrix_data, deviceIds[0], NULL);
		cv::Mat tempMatrix = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
		for (int i = 0; i < 3; i++) {
			double* pxvec = tempMatrix.ptr<double>(i);
			for (int j = 0; j < 3; j++) {
				pxvec[j] = obtainData.matrix_data.m_data[i][j];
			}
		}
		// 获取欧拉角
		res = cmtDataGetOriEuler(instance, &obtainData.euler_data, deviceIds[0], NULL);
		cv::Mat tempEuler = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		double* pxvec = tempEuler.ptr<double>(0);
		pxvec[0] = obtainData.euler_data.m_roll;
		pxvec[1] = obtainData.euler_data.m_pitch;
		pxvec[2] = obtainData.euler_data.m_yaw;
		// 获取速度
		res = cmtDataGetVelocity(instance, &obtainData.velocity_data, deviceIds[0]);
		finalObtainData.velocity_data.m_data[0] = obtainData.velocity_data.m_data[0];
		finalObtainData.velocity_data.m_data[1] = obtainData.velocity_data.m_data[1];
		finalObtainData.velocity_data.m_data[2] = obtainData.velocity_data.m_data[2];
		// 确保时间间隔不为0
		Sleep(1);
		clock_t thistime = clock();
		double timeInterval = double(thistime - lasttime) / CLOCKS_PER_SEC;
		lasttime = thistime;
		printf("time cost %f\n", timeInterval);
		// 积分运算
		cv::Mat xyz = (cv::Mat_<double>(3, 1) << finalObtainData.x, finalObtainData.y, finalObtainData.z);
		//printf("x: %f\n", finalObtainData.x);
		//printf("y: %f\n", finalObtainData.y);
		//printf("z: %f\n", finalObtainData.z);
		cv::Mat vel = (cv::Mat_<double>(3, 1) << obtainData.velocity_data.m_data[0], obtainData.velocity_data.m_data[1], obtainData.velocity_data.m_data[2]);
		//printf("velx: %f\n", obtainData.velocity_data.m_data[0]);
		//printf("vely: %f\n", obtainData.velocity_data.m_data[1]);
		//printf("velz: %f\n", obtainData.velocity_data.m_data[2]);
		printf("roll: %f\n", obtainData.euler_data.m_roll);
		printf("pitch: %f\n", obtainData.euler_data.m_pitch);
		printf("yaw: %f\n", obtainData.euler_data.m_yaw);
		cv::Mat tempResult = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		cv::Mat orientationMatrix = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
		// orientationMatrix = (firstMatrix.t() * tempMatrix);
		orientationMatrix = tempMatrix;
		cv::Mat I = cv::Mat::ones(3, 1, tempResult.type());
		// cv::multiply(orientationMatrix, vel, tempResult, timeInterval);
		// cv::multiply(orientationMatrix, vel, tempResult, timeInterval);
		tempResult = orientationMatrix * vel;
		tempResult = tempResult.mul(I, timeInterval);
		// tempResult = timeInterval * orientationMatrix * vel;
		xyz += tempResult;
		finalObtainData.x = xyz.at<double>(0, 0);
		finalObtainData.y = xyz.at<double>(1, 0);
		finalObtainData.z = xyz.at<double>(2, 0);
		// 写回最终的结果
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				finalObtainData.matrix_data.m_data[i][j] = orientationMatrix.at<double>(i, j);
			}
		}

		//cv::Mat differenceEuler = tempEuler - firstEuler;
		//cv::Mat Euler2Matrix = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
		//cv::Rodrigues(differenceEuler, Euler2Matrix);
		finalObtainData.euler_data.m_roll = tempEuler.at<double>(0, 0);
		finalObtainData.euler_data.m_pitch = tempEuler.at<double>(1, 0);
		finalObtainData.euler_data.m_yaw = tempEuler.at<double>(2, 0);
	}
	return 0;
}
void IMUClass::StartCmt(int portNum, int baudRate)
{
	HardwareScan(portNum, baudRate);
	MtSettings();
	Sleep(20);
	this->thread = CreateThread(
		NULL,              // default security attributes
		0,                 // use default stack size  
		CMTProcess,        // thread function 
		NULL,             // argument to thread function 
		0,                 // use default creation flags 
		NULL);           // returns the thread identifier 




}

void IMUClass::stopCmt()
{
	cmtClose(instance);
}

void IMUClass::RecordCmt(char filename[])
{
	FILE *fp = fopen(filename, "w");
	if ((fp == NULL))
	{
		exit(0);
	}
	else
	{
		fprintf(fp, "%15f", finalObtainData.euler_data.m_roll);
		fprintf(fp, "%15f", finalObtainData.euler_data.m_pitch);
		fprintf(fp, "%15f", finalObtainData.euler_data.m_yaw);
		fprintf(fp, "\r\n");
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				fprintf(fp, "%15f", finalObtainData.matrix_data.m_data[i][j]);
			}
			fprintf(fp, "\r\n");
			//if (i == 0) fprintf(fp, "%15f\r\n", finalObtainData.x);
			//if (i == 1) fprintf(fp, "%15f\r\n", finalObtainData.y);
			//if (i == 2) fprintf(fp, "%15f\r\n", finalObtainData.z);
		}
		//fprintf(fp, "%15f", finalObtainData.euler_data.m_roll);
		//fprintf(fp, "%15f", finalObtainData.euler_data.m_pitch);
		//fprintf(fp, "%15f", finalObtainData.euler_data.m_yaw);
		//fprintf(fp, "0\t0\t0\t1\r\n");
		//fprintf(fp, "\r\n");
		/*cv::Mat tempEuler = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		double* pxvec = tempEuler.ptr<double>(0);
		pxvec[0] = finalObtainData.euler_data.m_roll;
		pxvec[1] = finalObtainData.euler_data.m_pitch;
		pxvec[2] = finalObtainData.euler_data.m_yaw;
		cv::Mat orientationMatrix = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
		cv::Rodrigues(tempEuler, orientationMatrix);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				fprintf(fp, "%15f", orientationMatrix.at<double>(i, j));
			}
			fprintf(fp, "\r\n");*/
			//if (i == 0) fprintf(fp, "%15f\r\n", finalObtainData.x);
			//if (i == 1) fprintf(fp, "%15f\r\n", finalObtainData.y);
			//if (i == 2) fprintf(fp, "%15f\r\n", finalObtainData.z);
		//}
		//Sleep(20);
	}
	fclose(fp);

}
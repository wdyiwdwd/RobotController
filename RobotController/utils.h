#ifndef UTILS
#define UTILS

#include <string>
#include <thread>
#include "RobotBase.h"
#include "LidarClass.h"
#include "CameraClass.h"
#include "IMUClass.h"
#include "ptu.h"

class utils
{
private:

	LidarClass lidar;
	CameraClass camera;
	IMUClass xsens;
	RobotBase robotBase;

	// 打开云台用的端口信息
	portstream_fd COMstream;

	//RoboBase parameter
	std::string BaseComPort; //机器人底座连接的端口
	int MaxVel; //机器人基座最大速度
	int BumpDistance; //机器人基座判断障碍物的距离
	int IsSimulator; //机器人基座是否在模拟器中运行
	int StepLength; //机器人前进步长
	int StepBackLength; //机器人后退步长
	int StepRotation; //机器人单次旋转角度


	//Camera parameter
	double StartUpTime; //相机启动需要一定的时间 
	int CameraTimes;  //相机拍照的次数
	int CameraNum;

	//Lidar  parameter
	std::string LidarComPort; //激光连接端口

	//PTU parameter
	std::string PTUComPort;   // 云台连接端口
	int StartPosition;  //起始位置
	int EndPosition;    //终止位置
	int PTUBaudRate;       //波特率
	int Resolution;     //精度
	int CaptureTimes;    //扫描次数
	
	//IMU parameter
	int IMUComPortNum;  //IMU设备端口号，数字
	int IMUBauRate;     //IMU设备波特率

	//Filenames
	std::string LidarFilename; // 激光数据的保存文件名
	std::string CameraFilename; //摄影机数据的保存文件名
	std::string XsensFilename; //陀螺仪数据保存的文件名
	std::string OdometryFilename; //里程计数据保存的文件名

	//thread of camera and lidar
	std::shared_ptr<thread> CameraThread;
	std::shared_ptr<thread> LidarThread;

public:
	utils();
	~utils();

	void paraInitial();
	void equipInitial();
	void rehome();
	bool pause();  //if pausing then start, if starting the pause return new state
	RobotBase* base();
};


#endif
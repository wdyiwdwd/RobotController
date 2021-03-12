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

	// ����̨�õĶ˿���Ϣ
	portstream_fd COMstream;

	//RoboBase parameter
	std::string BaseComPort; //�����˵������ӵĶ˿�
	int MaxVel; //�����˻�������ٶ�
	int BumpDistance; //�����˻����ж��ϰ���ľ���
	int IsSimulator; //�����˻����Ƿ���ģ����������
	int StepLength; //������ǰ������
	int StepBackLength; //�����˺��˲���
	int StepRotation; //�����˵�����ת�Ƕ�


	//Camera parameter
	double StartUpTime; //���������Ҫһ����ʱ�� 
	int CameraTimes;  //������յĴ���
	int CameraNum;

	//Lidar  parameter
	std::string LidarComPort; //�������Ӷ˿�

	//PTU parameter
	std::string PTUComPort;   // ��̨���Ӷ˿�
	int StartPosition;  //��ʼλ��
	int EndPosition;    //��ֹλ��
	int PTUBaudRate;       //������
	int Resolution;     //����
	int CaptureTimes;    //ɨ�����
	
	//IMU parameter
	int IMUComPortNum;  //IMU�豸�˿ںţ�����
	int IMUBauRate;     //IMU�豸������

	//Filenames
	std::string LidarFilename; // �������ݵı����ļ���
	std::string CameraFilename; //��Ӱ�����ݵı����ļ���
	std::string XsensFilename; //���������ݱ�����ļ���
	std::string OdometryFilename; //��̼����ݱ�����ļ���

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
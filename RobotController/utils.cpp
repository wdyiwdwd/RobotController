#include "utils.h"

#include "config_operate.h"


#ifndef GLOBAL_PAUSE
#define GLOBAL_PAUSE
bool GLOBAL_LIDAR_PAUSE = 0;
bool GLOBAL_LIDAR_FINISHED = 1;
bool GLOBAL_CAMERA_FINISHED = 1;
#endif

void setPosition(short temp)
{
	char status;
	if ((status = set_desired(TILT, POSITION, (PTU_PARM_PTR *)&temp, ABSOLUTE)) == TRUE)
		cout << "set position error" << endl;
	else 
		cout << "\nTILT_SET_ABS_POSITION executed" << endl;
	if ((status = await_completion()) == PTU_OK)
		cout << "AWAIT_COMPLETION executed " << status << endl;
	else
		cout << "await error" << endl;
}

void reHomePTU()
{
	signed short int val = 1000;
	set_desired(PAN, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
	set_desired(TILT, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
	val = 0;
	set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
	set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
	await_completion();

}


void utils::paraInitial()
{
	config_operate cos("config.ini");

	//RoboBase parameter
	this->BaseComPort = cos.getValue("BaseComPort");; //�����˵������ӵĶ˿�
	this->MaxVel = cos.getNumber("MaxVel");; //�����˻�������ٶ�
	this->BumpDistance = cos.getNumber("BumpDistance"); //�����˻����ж��ϰ���ľ���
	this->IsSimulator = cos.getNumber("IsSimulator"); //�����˻����Ƿ���ģ��������
	this->StepLength = cos.getNumber("StepLength"); //������ǰ������
	this->StepBackLength = cos.getNumber("StepBackLength"); //�����˺��˲���
	this->StepRotation = cos.getNumber("StepRotation"); //�����˵�����ת�Ƕ�


	//Camera parameter
	this->StartUpTime = cos.getNumber("StartUpTime"); //���������Ҫһ����ʱ�� 
	this->CameraTimes = cos.getNumber("CameraTimes");  //������յĴ���  Ŀǰ����1Ҳûʲô�� ���Ժ������ӹ���
	this->CameraNum = cos.getNumber("CameraNum");  //�������� ��0��ʼ

	//Lidar  parameter
	this->LidarComPort = cos.getValue("LidarComPort"); //�������Ӷ˿�

	//PTU parameter
	this->PTUComPort = cos.getValue("PTUComPort"); // ��̨���Ӷ˿�
	this->StartPosition = cos.getNumber("StartPosition");  //��ʼλ��
	this->EndPosition = cos.getNumber("EndPosition");   //��ֹλ��
	this->PTUBaudRate = cos.getNumber("PTUBaudRate");      //������
	this->Resolution = cos.getNumber("Resolution");    //����
	this->CaptureTimes = cos.getNumber("CaptureTimes");    //ɨ�����

	//IMU parameter
	this->IMUComPortNum = cos.getNumber("IMUComPortNum");  //IMU�豸�˿ںţ�����
	this->IMUBauRate = cos.getNumber("IMUBauRate");   //IMU�豸������


	
	//���������������������
	int argc = 3;
	char** argv = new char *[argc];
	argv[0] = "";
	if (this->IsSimulator) 
		argv[1] = "";
	else
		argv[1] = "-robotPort\0";
	argv[2] = new char[5];
	strcpy(argv[2], this->BaseComPort.c_str());
	this->robotBase.initParser(argc, argv);
	this->robotBase.setMaxVel(this->MaxVel);
	this->robotBase.setStepSize(this->StepLength, this->StepBackLength, this->StepRotation);

	//��������������ӣ���ʼ������
	this->robotBase.connect();
	this->robotBase.attachSonar();
	this->robotBase.run();
	this->robotBase.setBumpDistance(this->BumpDistance);

	//�����������
	this->camera.setStartupTime(this->StartUpTime);
	this->camera.setCaptureTimes(this->CameraTimes);
	this->camera.setCameraNum(this->CameraNum);


	//��ʼ�����豸
	//�������ǲ��ռ�����
	this->xsens.StartCmt(this->IMUComPortNum, this->IMUBauRate); //�˺����ڲ��Ѿ�ʵ�ֶ��߳�

	//��lidar�Ķ˿�
	cout << this->LidarComPort.c_str() << endl;

	this->lidar.Initial((char*)this->LidarComPort.c_str());


	//��̨�������趨
	set_baud_rate(this->PTUBaudRate);

	//����̨�˿�
	COMstream = open_host_port((char*)this->PTUComPort.c_str());

	//�ж���̨�˿���û�д�
	if (COMstream == PORT_NOT_OPENED){
		//MessageBox(0, ("Failagain"), 0, 0);
		cout << "Serial Port setup error." << endl;
	}
	else{
		cout << "Sucess" << endl;
		//MessageBox(0, ("Success"), 0, 0);
	}

	//һ�Ѷ���δ֪
	//Just in case there is an ISM on the line, talk directly to PTU.
	SerialStringOut(COMstream, (unsigned char *)"   ");
	select_unit(0);
	cout << "Firmware version: " << firmware_version() << endl << "Firmware version command executed successfully" << endl;

	//��̨��������
	int pan_resolution_control = 1;
	double seconds_arc = 0.0002778;
	double pan_resolution = get_desired(PAN, RESOLUTION);
	double tilt_resolution = get_desired(TILT, RESOLUTION);
	int tilt_step = (int)(30 / (tilt_resolution*this->Resolution*seconds_arc));
	int pan_step = (int)(30 / (pan_resolution*pan_resolution_control*seconds_arc));

}

void LidarProcess(LidarClass* lidar, portstream_fd COMstream, int CaptureTimes, int StartPosition, int EndPosition, int Resolution, string LidarFilename) {
	GLOBAL_LIDAR_FINISHED = 0;
	//�������
	for (int i = 1; i <= CaptureTimes; i++)
	{
		//��̨��λ
		reHomePTU();

		//������̨����
		short temp = StartPosition;

		while (temp >= EndPosition)
		{
			while (GLOBAL_LIDAR_PAUSE){};

			setPosition(temp);

			temp -= 1 * Resolution;

			setPosition(temp);

			// do_delay(100);

			lidar->Scan((char*)LidarFilename.c_str());

		}
		GLOBAL_LIDAR_FINISHED = 1;
	}
	//ֹͣ�״�����
	// lidar->closeConnect();
	reHomePTU();
	await_completion();
	// close_host_port(COMstream);
}

void CameraProcess(CameraClass* camera, string CameraFilename) {
	GLOBAL_CAMERA_FINISHED = 0;
	camera->takePicture((char*)CameraFilename.c_str());
	GLOBAL_CAMERA_FINISHED = 1;
}

void utils::equipInitial()
{
	//�����ļ���
	//time_t nowTime = time(0);
	//tm* time = {0};
	//localtime_s(time, &nowTime);
	//char timeStr[30];
	//strftime(timeStr, DATETIME_MAX, "%Y%m%d_%H%M%S", time);
	//std::string str = timeStr;
	//this->LidarFilename = str + "_lidar.txt";
	//this-> = time + "_lidar.txt";
	//�������̻߳򼤹��̻߳��ڹ�������ȴ����ǽ���
	if (!GLOBAL_CAMERA_FINISHED) {
		cout << "waiting for camera thread join" << endl;
		return;
	}
	if (!GLOBAL_LIDAR_FINISHED) {
		cout << "waiting for lidar thread join" << endl;
		return;
	}

	time_t curtime = time(NULL);
	tm* p = localtime(&curtime);
	char filename[100] = { 0 };
	sprintf(filename, "%d%02d%02d%02d%02d%02d", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
	std::string str = filename;
	this->LidarFilename = str + "_lidar.txt";
	this->CameraFilename = str + "_camera.jpg";
	this->XsensFilename = str + "_imu.txt";
	this->OdometryFilename = str + "_odo.txt";

	//��¼��̼Ƶ�����
	this->base()->recordOdoDelInfo((char*)this->OdometryFilename.c_str());

	////�������ǲ��ռ�����
	//this->xsens.StartCmt(this->IMUComPortNum, this->IMUBauRate); //�˺����ڲ��Ѿ�ʵ�ֶ��߳�

	////��lidar�Ķ˿�
	//cout << this->LidarComPort.c_str() << endl;

	//this->lidar.Initial((char*)this->LidarComPort.c_str());


	////��̨�������趨
	//set_baud_rate(this->PTUBaudRate);

	////����̨�˿�
	//portstream_fd COMstream;
	//COMstream = open_host_port((char*)this->PTUComPort.c_str());

	////�ж���̨�˿���û�д�
	//if (COMstream == PORT_NOT_OPENED){
	//	//MessageBox(0, ("Failagain"), 0, 0);
	//	cout << "Serial Port setup error." << endl;
	//}
	//else{
	//	cout << "Sucess" << endl;
	//	//MessageBox(0, ("Success"), 0, 0);
	//}

	////һ�Ѷ���δ֪
	////Just in case there is an ISM on the line, talk directly to PTU.
	//SerialStringOut(COMstream, (unsigned char *)"   ");
	//select_unit(0);
	//cout << "Firmware version: " << firmware_version() << endl << "Firmware version command executed successfully" << endl;

	////��̨��������
	//int pan_resolution_control = 1;
	//double seconds_arc = 0.0002778;
	//double pan_resolution = get_desired(PAN, RESOLUTION);
	//double tilt_resolution = get_desired(TILT, RESOLUTION);
	//int tilt_step = (int)(30 / (tilt_resolution*this->Resolution*seconds_arc));
	//int pan_step = (int)(30 / (pan_resolution*pan_resolution_control*seconds_arc));

	//�����һ����Ƭ
	//this->camera.takePicture((char*)this->CameraFilename.c_str());
	this->CameraThread = std::make_shared<thread>(CameraProcess, &this->camera, this->CameraFilename);
	this->CameraThread->detach();

	//���������ǻ�õ����ݲ��ر�
	this->xsens.RecordCmt((char*)this->XsensFilename.c_str());

	//��̨�ͼ��⿪ʼ����
	this->LidarThread = std::make_shared<thread>(
		LidarProcess, 
		&this->lidar, 
		COMstream, 
		this->CaptureTimes,
		this->StartPosition, 
		this->EndPosition,
		this->Resolution,
		this->LidarFilename);
	this->LidarThread->detach();
}



RobotBase* utils::base(){
	return &(this->robotBase);
}

void utils::rehome() {
	cout << "rehome" << endl;
	reHomePTU();
}

bool utils::pause() {
	GLOBAL_LIDAR_PAUSE = GLOBAL_LIDAR_PAUSE ? 0 : 1; //�ı�GLOBAL_LIDAR_PAUSE��ֵ
	cout << "pause:" << GLOBAL_LIDAR_PAUSE << endl;
	return GLOBAL_LIDAR_PAUSE;
}

utils::utils()
{
	this->paraInitial();
}

utils::~utils()
{
	//ֹͣ�״�����
	lidar.closeConnect();
	reHomePTU();
	await_completion();
	close_host_port(COMstream);
	// ֹͣimu
	//this->xsens.stopCmt();
}


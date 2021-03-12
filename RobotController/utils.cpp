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
	this->BaseComPort = cos.getValue("BaseComPort");; //机器人底座连接的端口
	this->MaxVel = cos.getNumber("MaxVel");; //机器人基座最大速度
	this->BumpDistance = cos.getNumber("BumpDistance"); //机器人基座判断障碍物的距离
	this->IsSimulator = cos.getNumber("IsSimulator"); //机器人基座是否在模拟器运行
	this->StepLength = cos.getNumber("StepLength"); //机器人前进步长
	this->StepBackLength = cos.getNumber("StepBackLength"); //机器人后退步长
	this->StepRotation = cos.getNumber("StepRotation"); //机器人单次旋转角度


	//Camera parameter
	this->StartUpTime = cos.getNumber("StartUpTime"); //相机启动需要一定的时间 
	this->CameraTimes = cos.getNumber("CameraTimes");  //相机拍照的次数  目前多余1也没什么用 可以后续增加功能
	this->CameraNum = cos.getNumber("CameraNum");  //相机的序号 从0开始

	//Lidar  parameter
	this->LidarComPort = cos.getValue("LidarComPort"); //激光连接端口

	//PTU parameter
	this->PTUComPort = cos.getValue("PTUComPort"); // 云台连接端口
	this->StartPosition = cos.getNumber("StartPosition");  //起始位置
	this->EndPosition = cos.getNumber("EndPosition");   //终止位置
	this->PTUBaudRate = cos.getNumber("PTUBaudRate");      //波特率
	this->Resolution = cos.getNumber("Resolution");    //精度
	this->CaptureTimes = cos.getNumber("CaptureTimes");    //扫描次数

	//IMU parameter
	this->IMUComPortNum = cos.getNumber("IMUComPortNum");  //IMU设备端口号，数字
	this->IMUBauRate = cos.getNumber("IMUBauRate");   //IMU设备波特率


	
	//读入程序参数传输给机器人
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

	//程序与机器人连接，初始化声纳
	this->robotBase.connect();
	this->robotBase.attachSonar();
	this->robotBase.run();
	this->robotBase.setBumpDistance(this->BumpDistance);

	//设置相机参数
	this->camera.setStartupTime(this->StartUpTime);
	this->camera.setCaptureTimes(this->CameraTimes);
	this->camera.setCameraNum(this->CameraNum);


	//初始化各设备
	//打开陀螺仪并收集数据
	this->xsens.StartCmt(this->IMUComPortNum, this->IMUBauRate); //此函数内部已经实现多线程

	//打开lidar的端口
	cout << this->LidarComPort.c_str() << endl;

	this->lidar.Initial((char*)this->LidarComPort.c_str());


	//云台波特率设定
	set_baud_rate(this->PTUBaudRate);

	//打开云台端口
	COMstream = open_host_port((char*)this->PTUComPort.c_str());

	//判断云台端口有没有打开
	if (COMstream == PORT_NOT_OPENED){
		//MessageBox(0, ("Failagain"), 0, 0);
		cout << "Serial Port setup error." << endl;
	}
	else{
		cout << "Sucess" << endl;
		//MessageBox(0, ("Success"), 0, 0);
	}

	//一堆东西未知
	//Just in case there is an ISM on the line, talk directly to PTU.
	SerialStringOut(COMstream, (unsigned char *)"   ");
	select_unit(0);
	cout << "Firmware version: " << firmware_version() << endl << "Firmware version command executed successfully" << endl;

	//云台参数设置
	int pan_resolution_control = 1;
	double seconds_arc = 0.0002778;
	double pan_resolution = get_desired(PAN, RESOLUTION);
	double tilt_resolution = get_desired(TILT, RESOLUTION);
	int tilt_step = (int)(30 / (tilt_resolution*this->Resolution*seconds_arc));
	int pan_step = (int)(30 / (pan_resolution*pan_resolution_control*seconds_arc));

}

void LidarProcess(LidarClass* lidar, portstream_fd COMstream, int CaptureTimes, int StartPosition, int EndPosition, int Resolution, string LidarFilename) {
	GLOBAL_LIDAR_FINISHED = 0;
	//拍摄次数
	for (int i = 1; i <= CaptureTimes; i++)
	{
		//云台复位
		reHomePTU();

		//控制云台运行
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
	//停止雷达连接
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
	//构建文件名
	//time_t nowTime = time(0);
	//tm* time = {0};
	//localtime_s(time, &nowTime);
	//char timeStr[30];
	//strftime(timeStr, DATETIME_MAX, "%Y%m%d_%H%M%S", time);
	//std::string str = timeStr;
	//this->LidarFilename = str + "_lidar.txt";
	//this-> = time + "_lidar.txt";
	//如果相机线程或激光线程还在工作，则等待它们结束
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

	//记录里程计的数据
	this->base()->recordOdoDelInfo((char*)this->OdometryFilename.c_str());

	////打开陀螺仪并收集数据
	//this->xsens.StartCmt(this->IMUComPortNum, this->IMUBauRate); //此函数内部已经实现多线程

	////打开lidar的端口
	//cout << this->LidarComPort.c_str() << endl;

	//this->lidar.Initial((char*)this->LidarComPort.c_str());


	////云台波特率设定
	//set_baud_rate(this->PTUBaudRate);

	////打开云台端口
	//portstream_fd COMstream;
	//COMstream = open_host_port((char*)this->PTUComPort.c_str());

	////判断云台端口有没有打开
	//if (COMstream == PORT_NOT_OPENED){
	//	//MessageBox(0, ("Failagain"), 0, 0);
	//	cout << "Serial Port setup error." << endl;
	//}
	//else{
	//	cout << "Sucess" << endl;
	//	//MessageBox(0, ("Success"), 0, 0);
	//}

	////一堆东西未知
	////Just in case there is an ISM on the line, talk directly to PTU.
	//SerialStringOut(COMstream, (unsigned char *)"   ");
	//select_unit(0);
	//cout << "Firmware version: " << firmware_version() << endl << "Firmware version command executed successfully" << endl;

	////云台参数设置
	//int pan_resolution_control = 1;
	//double seconds_arc = 0.0002778;
	//double pan_resolution = get_desired(PAN, RESOLUTION);
	//double tilt_resolution = get_desired(TILT, RESOLUTION);
	//int tilt_step = (int)(30 / (tilt_resolution*this->Resolution*seconds_arc));
	//int pan_step = (int)(30 / (pan_resolution*pan_resolution_control*seconds_arc));

	//相机拍一张照片
	//this->camera.takePicture((char*)this->CameraFilename.c_str());
	this->CameraThread = std::make_shared<thread>(CameraProcess, &this->camera, this->CameraFilename);
	this->CameraThread->detach();

	//储存陀螺仪获得的数据并关闭
	this->xsens.RecordCmt((char*)this->XsensFilename.c_str());

	//云台和激光开始工作
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
	GLOBAL_LIDAR_PAUSE = GLOBAL_LIDAR_PAUSE ? 0 : 1; //改变GLOBAL_LIDAR_PAUSE的值
	cout << "pause:" << GLOBAL_LIDAR_PAUSE << endl;
	return GLOBAL_LIDAR_PAUSE;
}

utils::utils()
{
	this->paraInitial();
}

utils::~utils()
{
	//停止雷达连接
	lidar.closeConnect();
	reHomePTU();
	await_completion();
	close_host_port(COMstream);
	// 停止imu
	//this->xsens.stopCmt();
}


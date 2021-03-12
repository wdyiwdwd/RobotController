#include "XsensCmtControl.h"
#define KEY "0445F-0F456-486A9-4851A"
//ƽ������
#define MeanTimes 10
//���������β��������ƽ���Ĺ��ܣ�������������������Ӱ�졣
	long instance = -1;
	//��ʱ���浥������
	ObtainData obtainData;
	//�����������ݵĽṹ
	ObtainData finalObtainData;
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
	//Ŀǰ�ۼƴ���
	int measureTimes=1;

XsensCmtControl::XsensCmtControl()
{
	userQuit=0;
	mtCount=0;
	screenSensorOffset=0;
	temperatureOffset=0;
}

XsensCmtControl::~XsensCmtControl()
{
}

void XsensCmtControl::HardwareScan(int portNum, int baudRate)
{

	//�о�����Ӳ��ɨ��һ��Ķ��������ǲ�֪��ʲô���ã�
	//��Ϊ���ﲢû�и��˿ں�֮���
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


	//����̸߳о�����Ӧ�þ���ֹ�˰ɡ�
	//δ�ҵ�IMU�豸������return��
	if (portCount == 0) {
		//printf("No MotionTrackers found\n\n");
		printf("δ�ҵ�IMU�豸��\n");
		//MessageBox(dialog->m_hWnd,_T("δ�ҵ�IMU�豸��"),0,0);
		//MessageBox(dialog->m_hWnd, "hahfaf", 0, 0);
		
		return ;
		//exit(0);
	}

	//for(int i = 0; i < (int)portCount; i++) {	
	////	printf("Using COM port %d at %d baud\n\n",
	//		//(long) portInfo[i].m_portNr, portInfo[i].m_baudrate);	
	//}
	//
	//printf("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for(int p = 0; p < (int)portCount; p++){
		res = cmtOpenPort(instance, portInfo[p].m_portNr, portInfo[p].m_baudrate);
		if (res != XRV_OK)
			printf("��IMU�豸ʧ�ܣ�\n");
		//exit(1);
	}
	//printf("done\n\n");

	//get the Mt sensor count.
	//printf("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	res = cmtGetMtCount(instance,&mtCount);
	if (res != XRV_OK)
		printf("��ȡIMU�豸����ʧ�ܣ�\n");
	//exit(1);
	//printf("MotionTracker count: %i\n\n",mtCount);

	// retrieve the device IDs 
	//printf("Retrieving MotionTrackers device ID(s)\n");
	for(unsigned int j = 0; j < mtCount; j++ ){
		res = cmtGetMtDeviceId(instance, &deviceIds[j], j);
		if (res != XRV_OK)
			printf("��ȡDeviceIDʧ�ܣ�\n");
		//exit(1);
		//printf("Device ID at index %i: %08x\n",j,(long) deviceIds[j]);
	}	

	// make sure that we get the freshest data
	//printf("\nSetting queue mode so that we always get the latest data\n\n");
	res = cmtSetQueueMode(instance,CMT_QM_LAST);
	if (res != XRV_OK)
		printf("���ù�����ʽʧ�ܣ�\n");
	//exit(1);

}
void XsensCmtControl::MtSettings(void) 
{
	XsensResultValue res = XRV_OK;
	mode = CMT_OUTPUTMODE_POSITION|CMT_OUTPUTMODE_VELOCITY| CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
	settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
	res = cmtGotoConfig(instance);
	//if (res != XRV_OK)
	//MessageBox(dialog->m_hWnd,_T("��������ģʽʧ�ܣ�"),0,0);
	//EXIT_ON_ERROR(res, "cmtGotoConfig");

	unsigned short sampleFreq;
	res = cmtGetSampleFrequency(instance, &sampleFreq, deviceIds[0]);

	// set the device output mode for the device(s)
	//printf("Configuring your mode selection");
	for (int i=0; i<mtCount; i++) {
		if (cmtIdIsMtig(deviceIds[i])) {
			res = cmtSetDeviceMode(instance, mode,settings, sampleFreq, deviceIds[i]);
		} else {
			res = cmtSetDeviceMode(instance, mode & 0xFF0F, settings, sampleFreq, deviceIds[i]);
		}
		//if (res != XRV_OK)
			//MessageBox(dialog->m_hWnd,_T("�����豸ģʽʧ�ܣ�"),0,0);
		//EXIT_ON_ERROR(res, "setDeviceMode");
	}

	// start receiving data
	res = cmtGotoMeasurement(instance);
	//if (res != XRV_OK)
		//MessageBox(dialog->m_hWnd,_T("�ص�����ģʽʧ�ܣ�"),0,0);
	//EXIT_ON_ERROR(res, "cmtGotoMeasurement");
}
DWORD WINAPI CMTProcess(LPVOID lpParam) 
{

	XsensResultValue res = XRV_OK;
	//packet = new Packet((unsigned short)mtCount,cmt3.isXm());
	double tdata;
	unsigned short sdata;
	CmtCalData caldata;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;
	
	while(res == XRV_OK)
	{
		res = cmtGetNextDataBundle(instance);
		res = cmtDataGetSampleCounter(instance, &sdata, deviceIds[0] ,NULL);
		if(measureTimes==MeanTimes)
		{
			finalObtainData.altitude=finalObtainData.altitude/MeanTimes;
			finalObtainData.latitude=finalObtainData.latitude/MeanTimes;
			finalObtainData.longitude=finalObtainData.longitude/MeanTimes;

			finalObtainData.velocity_data.m_data[0]=finalObtainData.velocity_data.m_data[0]/MeanTimes;
			finalObtainData.velocity_data.m_data[1]=finalObtainData.velocity_data.m_data[1]/MeanTimes;
			finalObtainData.velocity_data.m_data[2]=finalObtainData.velocity_data.m_data[2]/MeanTimes;

			finalObtainData.caldata.m_acc.m_data[0]=finalObtainData.caldata.m_acc.m_data[0]/MeanTimes;
			finalObtainData.caldata.m_acc.m_data[1]=finalObtainData.caldata.m_acc.m_data[1]/MeanTimes;
			finalObtainData.caldata.m_acc.m_data[2]=finalObtainData.caldata.m_acc.m_data[2]/MeanTimes;

			finalObtainData.caldata.m_gyr.m_data[0]=finalObtainData.caldata.m_gyr.m_data[0]/MeanTimes;
			finalObtainData.caldata.m_gyr.m_data[1]=finalObtainData.caldata.m_gyr.m_data[1]/MeanTimes;
			finalObtainData.caldata.m_gyr.m_data[2]=finalObtainData.caldata.m_gyr.m_data[2]/MeanTimes;

			finalObtainData.euler_data.m_roll=finalObtainData.euler_data.m_roll/MeanTimes;
			finalObtainData.euler_data.m_pitch=finalObtainData.euler_data.m_pitch/MeanTimes;
			finalObtainData.euler_data.m_yaw=finalObtainData.euler_data.m_yaw/MeanTimes;
			measureTimes=1;
			//finalObtainData.altitude=0;
			//finalObtainData.latitude=0;
			//finalObtainData.longitude=0;

			//finalObtainData.velocity_data.m_data[0]=0;
			//finalObtainData.velocity_data.m_data[1]=0;
			//finalObtainData.velocity_data.m_data[2]=0;

			//finalObtainData.caldata.m_acc.m_data[0]=0;
			//finalObtainData.caldata.m_acc.m_data[1]=0;
			//finalObtainData.caldata.m_acc.m_data[2]=0;

			//finalObtainData.caldata.m_gyr.m_data[0]=0;
			//finalObtainData.caldata.m_gyr.m_data[1]=0;
			//finalObtainData.caldata.m_gyr.m_data[2]=0;

			//finalObtainData.euler_data.m_roll=0;
			//finalObtainData.euler_data.m_pitch=0;
			//finalObtainData.euler_data.m_yaw=0;
			break;
		}
		else{measureTimes++;}
		//get sample count, goto position & display.
		//sdata = packet->getSampleCounter();
			for (unsigned int i = 0; i < mtCount; i++) {	
				// Output Temperature
				if ((mode & CMT_OUTPUTMODE_TEMP) != 0) {					
					//gotoxy(0,4 + i * screenSensorOffset);	
					res = cmtDataGetTemp(instance,  &tdata, deviceIds[i],NULL);
					//printf("%6.2f", tdata);
				}

				//gotoxy(0,5 + temperatureOffset + i * screenSensorOffset);	// Output Calibrated data
				if ((mode & CMT_OUTPUTMODE_CALIB) != 0) {
					res = cmtDataGetCalData(instance, &caldata, deviceIds[i]);
					//caldata = packet->getCalData(i);
					//printf("%6.2f\t%6.2f\t%6.2f" , caldata.m_acc.m_data[0], caldata.m_acc.m_data[1], caldata.m_acc.m_data[2]);
					obtainData.caldata=caldata;
					finalObtainData.caldata.m_acc.m_data[0]+=obtainData.caldata.m_acc.m_data[0];
					finalObtainData.caldata.m_acc.m_data[1]+=obtainData.caldata.m_acc.m_data[1];
					finalObtainData.caldata.m_acc.m_data[2]+=obtainData.caldata.m_acc.m_data[2];

					finalObtainData.caldata.m_gyr.m_data[0]+=obtainData.caldata.m_gyr.m_data[0];
					finalObtainData.caldata.m_gyr.m_data[1]+=obtainData.caldata.m_gyr.m_data[1];
					finalObtainData.caldata.m_gyr.m_data[2]+=obtainData.caldata.m_gyr.m_data[2];
					//dialog->m_accx.SetWindowTextW(str);
					//dialog->SendMessage(WM_MYUPDATEDATA,(WPARAM)&caldata); 
					//gotoxy(0,7 + temperatureOffset + i * screenSensorOffset);
					
					//printf("%6.2f\t%6.2f\t%6.2f", caldata.m_gyr.m_data[0], caldata.m_gyr.m_data[1], caldata.m_gyr.m_data[2] );

					//gotoxy(0,9 + temperatureOffset + i * screenSensorOffset);
					//printf("%6.2f\t%6.2f\t%6.2f",caldata.m_mag.m_data[0], caldata.m_mag.m_data[1], caldata.m_mag.m_data[2]);
					//gotoxy(0,13 + temperatureOffset + i * screenSensorOffset);
				}
				if ((mode & CMT_OUTPUTMODE_VELOCITY	) != 0) {					
					//if (packet->containsVelocity()){
					//printf("%6.2f\t%6.2f\t%6.2f" , caldata.m_acc.m_data[0], caldata.m_acc.m_data[1], caldata.m_acc.m_data[2]);
					//obtainData.velocity_data=packet->getVelocity(i);
					res = cmtDataGetVelocity(instance, &obtainData.velocity_data, deviceIds[i]);
					finalObtainData.velocity_data.m_data[0]+=obtainData.velocity_data.m_data[0];
					finalObtainData.velocity_data.m_data[1]+=obtainData.velocity_data.m_data[1];
					finalObtainData.velocity_data.m_data[2]+=obtainData.velocity_data.m_data[2];
					//}
					//dialog->m_accx.SetWindowTextW(str);
					//dialog->SendMessage(WM_MYUPDATEDATA,(WPARAM)&caldata); 
					//gotoxy(0,7 + temperatureOffset + i * screenSensorOffset);
					
					//printf("%6.2f\t%6.2f\t%6.2f", caldata.m_gyr.m_data[0], caldata.m_gyr.m_data[1], caldata.m_gyr.m_data[2] );

					//gotoxy(0,9 + temperatureOffset + i * screenSensorOffset);
					//printf("%6.2f\t%6.2f\t%6.2f",caldata.m_mag.m_data[0], caldata.m_mag.m_data[1], caldata.m_mag.m_data[2]);
					//gotoxy(0,13 + temperatureOffset + i * screenSensorOffset);
				}
				if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {					
					switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
					case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
						// Output: quaternion
						res = cmtDataGetOriQuat(instance, &qat_data, deviceIds[i]);
						//printf("%6.3f\t%6.3f\t%6.3f\t%6.3f\n", qat_data.m_data[0], qat_data.m_data[1],qat_data.m_data[2],qat_data.m_data[3]);
						break;

					case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
						// Output: Euler
						res = cmtDataGetOriEuler(instance, &euler_data, deviceIds[i]);
						obtainData.euler_data=euler_data;
						finalObtainData.euler_data.m_roll+=obtainData.euler_data.m_roll;
						finalObtainData.euler_data.m_pitch+=obtainData.euler_data.m_pitch;
						finalObtainData.euler_data.m_yaw+=obtainData.euler_data.m_yaw;

						//printf("%6.1f\t%6.1f\t%6.1f\n", euler_data.m_roll,euler_data.m_pitch, euler_data.m_yaw);
						break;

					case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
						// Output: Cosine Matrix
						res = cmtDataGetOriMatrix(instance, &matrix_data, deviceIds[i],NULL);
						//printf("%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[0][0],matrix_data.m_data[0][1], matrix_data.m_data[0][2]);
						//printf("%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[1][0],matrix_data.m_data[1][1], matrix_data.m_data[1][2]);
						//printf("%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[2][0],matrix_data.m_data[2][1], matrix_data.m_data[2][2]);
						break;
					default:
						;
					}

					if ((mode & CMT_OUTPUTMODE_POSITION) != 0) {
					 if (cmtDataContainsPositionLLA(instance, deviceIds[i]) != 0) {
						res = cmtDataGetPositionLLA(instance, &positionLLA, deviceIds[i]);
						if (res != XRV_OK) {
							printf("error %ud", res);
						}
			
							for (int i = 0; i < 3; i++) {
								double deg = positionLLA.m_data[i];
								printf("%f ", deg);
								double min = (deg - (int)deg)*60;
								double sec = (min - (int)min)*60;
								// printf("%3d\xa7%2d\'%2.2lf\"\t", (int)deg, (int)min, sec);
							}
							printf("\n");
							obtainData.latitude=positionLLA.m_data[0];
							obtainData.longitude=positionLLA.m_data[1];
							obtainData.altitude=positionLLA.m_data[2];
								
							finalObtainData.altitude+=obtainData.altitude;
							finalObtainData.latitude+=obtainData.latitude;
							finalObtainData.longitude+=obtainData.longitude;

							//printf(" %3.2lf\n", positionLLA.m_data[2]);
						} else {
							printf("No position data available\n");
						}
					}
				}
			}	
	//dialog->SendMessage(WM_MYUPDATEDATA,(WPARAM)&obtainData); 
	}
	return 0;
}
void XsensCmtControl::StartCmt(int portNum, int baudRate)
{
	HardwareScan(portNum, baudRate);
	MtSettings();
	Sleep(20);
	this->thread=CreateThread( 
            NULL,              // default security attributes
            0,                 // use default stack size  
            CMTProcess,        // thread function 
            NULL,             // argument to thread function 
            0,                 // use default creation flags 
            NULL);           // returns the thread identifier 

	


}

void XsensCmtControl::stopCmt()
{
	cmtClose(instance);	
}
void XsensCmtControl::RecordCmt(char filename[])
{
	FILE *fp=fopen(filename,"w");
	if((fp==NULL))
	{
		exit(0);
	}
	else
	{
		fprintf(fp,"%15f",finalObtainData.latitude);
		fprintf(fp,"%15f",finalObtainData.longitude);
		fprintf(fp,"%15f",finalObtainData.altitude);
		fprintf(fp,"%15f",finalObtainData.euler_data.m_roll);
		fprintf(fp,"%15f",finalObtainData.euler_data.m_pitch);
		fprintf(fp,"%15f",finalObtainData.euler_data.m_yaw);
		fprintf(fp,"\r\n");
	//Sleep(20);
	}
	finalObtainData.altitude=0;
	finalObtainData.latitude=0;
	finalObtainData.longitude=0;

	finalObtainData.velocity_data.m_data[0]=0;
	finalObtainData.velocity_data.m_data[1]=0;
	finalObtainData.velocity_data.m_data[2]=0;

	finalObtainData.caldata.m_acc.m_data[0]=0;
	finalObtainData.caldata.m_acc.m_data[1]=0;
	finalObtainData.caldata.m_acc.m_data[2]=0;

	finalObtainData.caldata.m_gyr.m_data[0]=0;
	finalObtainData.caldata.m_gyr.m_data[1]=0;
	finalObtainData.caldata.m_gyr.m_data[2]=0;

	finalObtainData.euler_data.m_roll=0;
	finalObtainData.euler_data.m_pitch=0;
	finalObtainData.euler_data.m_yaw=0;
	fclose(fp);

}
#include "LidarClass.h"
//#include "Robot.h"
#define PI 3.1415926

void delay(int msec)
{
  Sleep(msec);
}


int com_changeBaudrate(long baudrate)
{
  DCB dcb;

  GetCommState(HCom, &dcb);
  dcb.BaudRate = baudrate;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.fParity = FALSE;
  dcb.StopBits = ONESTOPBIT;
  SetCommState(HCom, &dcb);

  return 0;
}

// serial connection
 int com_connect(const char* device, long baudrate) {

#if defined(RAW_OUTPUT)
  Raw_fd_ = fopen("raw_output.txt", "w");
#endif

  char adjust_device[16];

  _snprintf(adjust_device, 16, "\\\\.\\%s", device);
  WCHAR lp_adjust_device[16];
  memset(lp_adjust_device, 0, sizeof(lp_adjust_device));
  MultiByteToWideChar(CP_ACP, 0, adjust_device, strlen(adjust_device) + 1, lp_adjust_device,
	  sizeof(lp_adjust_device) / sizeof(lp_adjust_device[0]));

  HCom = CreateFile(lp_adjust_device, GENERIC_READ | GENERIC_WRITE, 0,
                     NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

  if (HCom == INVALID_HANDLE_VALUE) {
    return -1;
  }

  // Baud Rate setting
  return com_changeBaudrate(baudrate);
}


 void com_disconnect(void)
{
  if (HCom != INVALID_HANDLE_VALUE) {
    CloseHandle(HCom);
    HCom = INVALID_HANDLE_VALUE;
  }
}


 int com_send(const char* data, int size)
{
  DWORD n;
  WriteFile(HCom, data, size, &n, NULL);
  return n;
}


 int com_recv(char* data, int max_size, int timeout)
{
  if (max_size <= 0) {
    return 0;
  }

  // Read and Return data of requested size, if readable
  if (ReadableSize < max_size) {
    DWORD dwErrors;
    COMSTAT ComStat;
    ClearCommError(HCom, &dwErrors, &ComStat);
    ReadableSize = ComStat.cbInQue;
  }

  if (max_size > ReadableSize) {
    COMMTIMEOUTS pcto;
    int each_timeout = 2;

    if (timeout == 0) {
      max_size = ReadableSize;

    } else {
      if (timeout < 0) {
	timeout = 0;
	each_timeout = 0;
      }

      GetCommTimeouts(HCom, &pcto);
      pcto.ReadIntervalTimeout = timeout;
      pcto.ReadTotalTimeoutMultiplier = each_timeout;
      pcto.ReadTotalTimeoutConstant = timeout;
      SetCommTimeouts(HCom, &pcto);
    }
  }

  DWORD n;
  ReadFile(HCom, data, (DWORD)max_size, &n, NULL);
#if defined(RAW_OUTPUT)
  if (Raw_fd_) {
    for (int i = 0; i < n; ++i) {
      fprintf(Raw_fd_, "%c", data[i]);
    }
    fflush(Raw_fd_);
  }
#endif
  if (n > 0) {
    ReadableSize -= n;
  }

  return n;
}


// Send data(Commands) to URG 
 int urg_sendTag(const char* tag)
{
  char send_message[LineLength];
  _snprintf(send_message, LineLength, "%s\n", tag);
  int send_size = (int)strlen(send_message);
  com_send(send_message, send_size);

  return send_size;
}


// Read data (Reply) from URG until the termination 
 int urg_readLine(char *buffer)
{
  int i;
  for (i = 0; i < LineLength -1; ++i) {
    char recv_ch;
    int n = com_recv(&recv_ch, 1, Timeout);
    if (n <= 0) {
      if (i == 0) {
        return -1;              // Timeout
      }
      break;
    }
    if ((recv_ch == '\r') || (recv_ch == '\n')) {
      break;
    }
    buffer[i] = recv_ch;
  }
  buffer[i] = '\0';

  return i;
}


// Send data (Commands) to URG and wait for reply
 int urg_sendMessage(const char* command, int timeout, int* recv_n)
{
  int send_size = urg_sendTag(command);
  int recv_size = send_size + 2 + 1 + 2;
  char buffer[LineLength];

  int n = com_recv(buffer, recv_size, timeout);
  *recv_n = n;

  if (n < recv_size) {
    // received size error
    return -1;
  }

  if (strncmp(buffer, command, send_size -1)) {
    // incorrect command error
    return -1;
  }

  // !!! If possible do calculate check-sum to verify data

  // Convert the received data to Hex and return data.
  char reply_str[3] = "00";
  reply_str[0] = buffer[send_size];
  reply_str[1] = buffer[send_size + 1];
  return strtol(reply_str, NULL, 16);
}


// Change bau rate
 int urg_changeBaudrate(long baudrate)
{
  char buffer[] = "SSxxxxxx\r";
  _snprintf(buffer, 10, "SS%06d\r", baudrate);
  int dummy = 0;
  int ret = urg_sendMessage(buffer, Timeout, &dummy);

  if ((ret == 0) || (ret == 3) || (ret == 4)) {
    return 0;
  } else {
    return -1;
  }
}


// Read URG parameters
 int urg_getParameters(urg_state_t* state)
{
  // Parameter read and prcessing (use)
  urg_sendTag("PP");
  char buffer[LineLength];
  int line_index = 0;
  enum {
    TagReply = 0,
    DataReply,
    Other,
  };
  int line_length;
  for (; (line_length = urg_readLine(buffer)) > 0; ++line_index) {

    if (line_index == Other + urg_state_t::MODL) {
      buffer[line_length - 2] = '\0';
      state->model = &buffer[5];

    } else if (line_index == Other + urg_state_t::DMIN) {
      state->distance_min = atoi(&buffer[5]);

    } else if (line_index == Other + urg_state_t::DMAX) {
      state->distance_max = atoi(&buffer[5]);

    } else if (line_index == Other + urg_state_t::ARES) {
      state->area_total = atoi(&buffer[5]);

    } else if (line_index == Other + urg_state_t::AMIN) {
      state->area_min = atoi(&buffer[5]);
      state->first = state->area_min;

    } else if (line_index == Other + urg_state_t::AMAX) {
      state->area_max = atoi(&buffer[5]);
      state->last = state->area_max;

    } else if (line_index == Other + urg_state_t::AFRT) {
      state->area_front = atoi(&buffer[5]);

    } else if (line_index == Other + urg_state_t::SCAN) {
      state->scan_rpm = atoi(&buffer[5]);
    }
  }

  if (line_index <= Other + urg_state_t::SCAN) {
    return -1;
  }
  // Caluclate size of the data if necessary
  state->max_size = state->area_max +1;

  return 0;
}


// Process to connect with URG 
 int urg_connect(urg_state_t* state,
                       const char* port, const long baudrate)
{
  static char message_buffer[LineLength];

  if (com_connect(port, baudrate) < 0) {
    _snprintf(message_buffer, LineLength,
              "Cannot connect COM device: %s", port);
    ErrorMessage = message_buffer;
    return -1;
  }

  const long try_baudrate[] = { 19200, 115200, 38400 };
  size_t n = sizeof(try_baudrate) / sizeof(try_baudrate[0]);
  for (size_t i = 0; i < n; ++i) {

    // Change baud rate to detect the compatible baud rate with sensor
    if (com_changeBaudrate(try_baudrate[i])) {
      ErrorMessage = "change baudrate fail.";
      return -1;
    }

    // Change to SCIP2.0 mode
    int recv_n = 0;
    urg_sendMessage("SCIP2.0", Timeout, &recv_n);
    if (recv_n <= 0) {
      // If there is no reply it is considered as baud rate incompatibility, try with different baud rate
      continue;
    }

    // Change the baud rate if not match the setting
    if (try_baudrate[i] != baudrate) {
      urg_changeBaudrate(baudrate);

      // !!! The change will be implemented after 1 scan.
      // !!! Thus, Host or PC should wait to send the command
      delay(100);

      com_changeBaudrate(baudrate);
    }

    // Read parameter
    if (urg_getParameters(state) < 0) {
      ErrorMessage =
        "PP command fail.\n"
	"This COM device may be not URG, or URG firmware is too old.\n"
        "SCIP 1.1 protocol is not supported. Please update URG firmware.";
      return -1;
    }
    state->last_timestamp = 0;

    // URG is detected
    return 0;
  }

  // URG is not detected
  ErrorMessage = "no urg ports.";
  return -1;
}


// Disconnect URG 
 void urg_disconnect(void)
{
  com_disconnect();
}


// Data read using GD-Command
 int urg_captureByGD(const urg_state_t* state)
{
  char send_message[LineLength];
  _snprintf(send_message, LineLength,
            "GD%04d%04d%02d", state->first, state->last, 1);

  return urg_sendTag(send_message);
}


// Data read using MD-Command
// Changed by TN
 int urg_captureByMD(const urg_state_t* state, int capture_times, unsigned int CaptureAngle)
{
  // The capture time is changed to 00(infinity), if requested capture times over 100.
  // In this case, QT or RS command is needed to stop getting data
  if (capture_times >= 100) {
    capture_times = 0;
  }

  char send_message[LineLength];
  _snprintf(send_message, LineLength, "MD%04d%04d%02d%01d%02d",
            (state->area_front - 2*CaptureAngle), (state->area_front + 2*CaptureAngle), 1, 0, capture_times);

  return urg_sendTag(send_message);
}


// Decode 6 bit data from URG 
 long urg_decode(const char data[], int data_byte)
{
  long value = 0;
  for (int i = 0; i < data_byte; ++i) {
    value <<= 6;
    value &= ~0x3f;
    value |= data[i] - 0x30;
  }
  return value;
}


// Receive URG distance data 
 int urg_addRecvData(const char buffer[], long data[], int* filled)
{
  static int remain_byte = 0;
  static char remain_data[3];
  const int data_byte = 3;

  const char* pre_p = buffer;
  const char* p = pre_p;

  if (*filled <= 0) {
    remain_byte = 0;
  }

  if (remain_byte > 0) {
    memmove(&remain_data[remain_byte], buffer, data_byte - remain_byte);
    data[*filled] = urg_decode(remain_data, data_byte);
    ++(*filled);
    pre_p = &buffer[data_byte - remain_byte];
    p = pre_p;
    remain_byte = 0;
  }

  do {
    ++p;
    if ((p - pre_p) >= static_cast<int>(data_byte)) {
      data[*filled] = urg_decode(pre_p, data_byte);
      ++(*filled);
      pre_p = p;
    }
  } while (*p != '\0');
  remain_byte = (int)(p - pre_p);
  memmove(remain_data, pre_p, remain_byte);

  return 0;
}


// Receive URG distance data for TimeStamp
 int urg_addRecvDataTimeStamp(const char buffer[], long data[], int* filled) {

  static int remain_byte = 0;
  static char remain_data[4];
  const int data_byte = 4;

  const char* pre_p = buffer;
  const char* p = pre_p;

  if (remain_byte > 0) {
    memmove(&remain_data[remain_byte], buffer, data_byte - remain_byte);
    data[*filled] = urg_decode(remain_data, data_byte);
    ++(*filled);
    pre_p = &buffer[data_byte - remain_byte];
    p = pre_p;
    remain_byte = 0;
  }

  do {
    ++p;
    if ((p - pre_p) >= static_cast<int>(data_byte)) {
      data[*filled] = urg_decode(pre_p, data_byte);
      ++(*filled);
      pre_p = p;
    }
  } while (*p != '\0');
  remain_byte = p - pre_p;
  memmove(remain_data, pre_p, remain_byte);

  return 0;
}


 int checkSum(char buffer[], int size, char actual_sum)
{
  char expected_sum = 0x00;
  int i;

  for (i = 0; i < size; ++i) {
    expected_sum += buffer[i];
  }
  expected_sum = (expected_sum & 0x3f) + 0x30;

  return (expected_sum == actual_sum) ? 0 : -1;
}


// Receive URG data
 int urg_receiveData(urg_state_t* state, long data[], size_t max_size)
{
  int filled = 0;

  // Fill the positions upto first or min by 19 (non-measurement range)
  for (int i = state->first -1; i >= 0; --i) {
    data[filled++] = 19;
  }

  char message_type = 'M';
  char buffer[LineLength];
  int line_length;
  for (int line_count = 0; (line_length = urg_readLine(buffer)) >= 0; ++line_count) {

    // Verify the checksum
    if ((line_count > 3) && (line_length >= 3)) {
      if (checkSum(buffer, line_length - 1, buffer[line_length - 1]) < 0) {
	fprintf(stderr, "line_count: %d: %s\n", line_count, buffer);
        return -1;
      }
    }

    if ((line_count >= 6) && (line_length == 0)) {

      // End of data receive
      for (size_t i = filled; i < max_size; ++i) {
	// Fill the position upto data end by 19 (non-measurement range)
        data[filled++] = 19;
      }
      return filled;

    } else if (line_count == 0) {
      // Judge the message (Command) by first letter of receive data
      if ((buffer[0] != 'M') && (buffer[0] != 'G')) {
        return -1;
      }
      message_type = buffer[0];

    } else if (! strncmp(buffer, "99b", 3)) {
      // Detect "99b" and assume [time-stamp] and [data] to follow
      line_count = 4;

    } else if ((line_count == 1) && (message_type == 'G')) {
      line_count = 4;

    } else if (line_count == 4) {
      // "99b" Fixed
      if (strncmp(buffer, "99b", 3)) {
        return -1;
      }

    } else if (line_count == 5) {
      // TimeStamp
      state->last_timestamp = urg_decode(buffer, 4);

    } else if (line_count >= 6) {
      // Received Data
      if (line_length > (64 + 1)) {
        line_length = (64 + 1);
      }
      buffer[line_length -1] = '\0';
      int ret = urg_addRecvData(buffer, data, &filled);
      if (ret < 0) {
        return ret;
      }
    }
  }
  return -1;
}


void outputData(long data[], int n, size_t total_index)
{
  char output_file[] = "data_xxxxxxxxxx.csv";
  _snprintf(output_file, sizeof(output_file), "data_%03d.csv", total_index);
  FILE* fd = fopen(output_file, "w");
  if (! fd) {
    perror("fopen");
    return;
  }

  for (int i = 0; i < n; ++i) {
    fprintf(fd, "%ld, ", data[i]);
  }
  fprintf(fd, "\n");

  fclose(fd);
}
LidarClass::LidarClass(void):
m_lidar_baudrate(115200),
m_Lidar_InstMode(0),
m_scanline_num(0)
{
	time(&this->now_time);
	this->scan_time=localtime(&this->now_time);
	strftime(this->filename,DATETIME_MAX,"%Y%m%d_%H%M%S",this->scan_time);
    strncat(this->filename, ".txt", 4);
	sprintf(m_lidar_com,"COM");
	
	//m_scan_data=cvCreateMat(1000,1084,CV_32SC1);

}

LidarClass::~LidarClass(void)
{
}
bool LidarClass::Scan(char filename[])
{
	
	_tzset();
	FILE *outputfile = fopen(filename,"a+");
//	int *temp_data = this->m_scan_data->data.i;
	if(outputfile == NULL){
    printf("Cannot open %s \n",filename);
    //getchar();
    exit(1);
  }
	int scan_count=0;

	long* data = new long [1081];


	float* angledata=new float[1081];
	for (int i=0;i<1081;i++)
	{
		angledata[i]=-3*PI/4+i*270*PI/(1081*180);
	}

		urg_captureByMD(&this->m_Lidar_state, 1, this->m_CaptureAngle);
		int n = urg_receiveData(&this->m_Lidar_state, data, 1081);


	for(int i=0;i<1081;i++) // to get the relative polar coordinate of each scan point

	{
	 this->sensorFrame[i].x=data[i];
	 this->sensorFrame[i].y=angledata[i];
	}

	
for(int i=0;i<1081;i++)
{
fprintf(outputfile,"%ld ",data[i]);
}
fprintf(outputfile,"\r\n");
delete [] data;
	/*for(int i=0;i<scan_count;i++){
		for(int j=0;j<this->m_scan_data->width;j++){
				fprintf(outputfile, "%ld ", (temp_data+i*this->m_scan_data->step/sizeof(int))[j]);
		}
		fprintf(outputfile, "\n");
	}*/


	fclose(outputfile);
	
	return true;
_exit:
  // URG DISCONNECT
  urg_disconnect();
   
  return false;
}

bool LidarClass::Initial(void)
{
	return true;
}
bool LidarClass::closeConnect()
{
urg_disconnect();
return true;
}
bool LidarClass::Initial(char* Lidar_ComNum)
{
	//这里应该是组成一个新的字符串，用以打开相应的端口
	//sprintf(this->m_lidar_com,"COM");
	//strncat(this->m_lidar_com,Lidar_ComNum, 2);
	sprintf(this->m_lidar_com, Lidar_ComNum);

	// 在这里是连接激光传感器
	this->m_lidar_err = urg_connect(&this->m_Lidar_state, this->m_lidar_com, this->m_lidar_baudrate);
	if(this->m_lidar_err<0)
	{
		printf("Can not connect to LIDAR\n");
		return false;
	}
	else{
		printf("LIDAR:Using #%s\n", this->m_lidar_com);
	}
	

	this->m_Lidar_InstMode = VERTICAL_AUTO_MODE;
	
	this->m_CaptureAngle = 270;
	this->m_CaptureAngle = (0 <= this->m_CaptureAngle && this->m_CaptureAngle <= 270) ? this->m_CaptureAngle : 270;


	return true;
}
/*
广东工业大学 赵家俊
2022.01.03	Version 0.0
*/
#include <ros/ros.h>
#include <fcntl.h>      //open函数的头文件
#include <termios.h>    //串口驱动函数
#include <unistd.h>
#include <errno.h>    
#include <stdio.h>      //标准输入输出头文件
#include <string.h>
#include <std_msgs/Float32.h>
#include "nav40_demo.h"
#include "ch_imu_driver.hpp"

using namespace std;

#define NAV_DL     122    //数据长度
#define NAV_DH1    0xEB   //帧头
#define NAV_DH2    0x90   //帧头
#define MAXSIZE    1024   //缓冲区长度

char FrameHead[]={(char)NAV_DH1, (char)NAV_DH2};

APM_Datatype APM;            //创建帧结构体

#ifdef ORIGIN
typedef struct
{
	unsigned char Recbuf[MAXSIZE];  //缓冲数组
	int tail;              //尾指针
	int head;              //头指针
}Suqueue;

Suqueue queue_cycle;          //创建缓冲数组            
unsigned int checksum = 0;  //校验和
unsigned int checkRes_L, checkRes_H; //4个字节
unsigned char temp_buf[122]={0};
//设置波特率，初始化串口
int set_uart_baudrate(const int _fd, unsigned int baud)
{
	int speed;
	switch (baud) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	tcgetattr(_fd, &uart_config); //获取终端参数

	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSTOPB;
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	uart_config.c_cflag &= ~CRTSCTS;

	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	uart_config.c_iflag = 0;

	uart_config.c_oflag = 0;

	uart_config.c_cc[VTIME] = 0;
	uart_config.c_cc[VMIN] = 1;

	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return 0;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return 0;
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		return 0;
	}

	return 1;
}
#else
ImuDriverNode::ImuDriverNode() : n_("~")
{
	n_.param("dog_device", imu_device_, std::string("/dev/ttyUSB0"));
	n_.param("baud", baud_, 115200);
	n_.param("framerate", framerate_, 300);
	n_.param("isVerif", isVerif_, false);

	pub_imu = n_.advertise<sensor_msgs::Imu>("/imu/data", 5);

	connect();
}

ImuDriverNode::~ImuDriverNode()
{
	disconnect();
}

void ImuDriverNode::connect()
{
	try
  	{
		my_serial.setPort(imu_device_);
		my_serial.setBaudrate(baud_);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		my_serial.setTimeout(to);
		my_serial.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR("Unable to open port %s", imu_device_.c_str());
		ROS_ERROR("%s", e.what());
	}

	// check conection parameter
	ROS_INFO("Starting \033[1;32;40m'%s'\033[0m at \033[1;32;40m%u\033[0m, Verif: %d", imu_device_.c_str(), (unsigned int)baud_, isVerif_);
}

void ImuDriverNode::disconnect()
{
	my_serial.close();
	ROS_ERROR("%s error %d, %s", "close", errno, strerror(errno));
	exit(EXIT_FAILURE);
}

void ImuDriverNode::checkPort()
{
	// mtx.lock();
	std::string readData;
	// serialPort.Read(readData);
	my_serial.readline(readData, (size_t)128, std::string(FrameHead));

	// std::cout<<"readData.size() : "<<std::dec<<readData.size()<<std::endl;
	if(readData.size()!=(unsigned int)readData[0] || readData.size()!=NAV_DL) // 验证帧长度
		return;

	unsigned int checkSum = 0;
	unsigned int checkRes_L = 0;
	unsigned int checkRes_H = 0;
	checkSum += (unsigned char)NAV_DH1;
	checkSum += (unsigned char)NAV_DH2;
	for(size_t i=0; i<readData.size()-4; i++)
		checkSum += (unsigned char)readData[i];
	checkRes_L = checkSum & 0x00ff;
	checkRes_H = ((checkSum & 0xff00)>>8);
	// cout << "0x" <<  std::hex << checkRes_L << ", "<< "0x"<<checkRes_H <<std::endl;
	if(checkRes_L != readData[readData.size()-4] || checkRes_H != readData[readData.size()-3])
	{	
		// std::cout<<"____________________________"<<std::endl;
		return;
	}
	
	// for(unsigned char n : readData)
    //     cout << "0x" <<  std::hex << static_cast<unsigned short>(n) << " " ;//输出十六进制FF

	unsigned int offset = 0;
	memcpy(&APM.zhen_len, readData.c_str()+offset , 1); offset+=1;  //注入 缓冲区
	memcpy(&APM.zhen_flag, readData.c_str()+offset , 1); offset+=1;
	memcpy(&APM.counter, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.state, readData.c_str()+offset , 1); offset+=1;

	memcpy(&APM.pitch, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.roll, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.yaw, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.yaw_gps, readData.c_str()+offset , 4); offset+=4;

	memcpy(&APM.pitch_rate, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.roll_rate, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.yaw_rate, readData.c_str()+offset , 4); offset+=4;

	memcpy(&APM.lon, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.lat, readData.c_str()+offset , 4); offset+=4;

	memcpy(&APM.alt_baro, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.alt_gps, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.alt, readData.c_str()+offset , 4); offset+=4;

	memcpy(&APM.velocity_x, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.velocity_y, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.velocity_z, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.velocity_air, readData.c_str()+offset , 4); offset+=4;

	memcpy(&APM.accel_x, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.accel_y, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.accel_z, readData.c_str()+offset , 4); offset+=4;

	memcpy(&APM.satellite_num, readData.c_str()+offset , 1); offset+=1;

	memcpy(&APM.hdop, readData.c_str()+offset , 2); offset+=2;
	memcpy(&APM.vdop, readData.c_str()+offset , 2); offset+=2;

	memcpy(&APM.gps_status, readData.c_str()+offset , 1); offset+=1;
	memcpy(&APM.gps_hh, readData.c_str()+offset , 1); offset+=1;
	memcpy(&APM.gps_mm, readData.c_str()+offset , 1); offset+=1;
	memcpy(&APM.gps_ss, readData.c_str()+offset , 1); offset+=1;

	memcpy(&APM.temperature, readData.c_str()+offset , 1); offset+=1;

	memcpy(&APM.HDT, readData.c_str()+offset , 2); offset+=2;
	memcpy(&APM.HDG_Dev, readData.c_str()+offset , 2); offset+=2;

	memcpy(&APM.redundancy, readData.c_str()+offset , 1); offset+=1;
	memcpy(&APM.GPS0_DT, readData.c_str()+offset , 1); offset+=1;
	memcpy(&APM.GPS1_DT, readData.c_str()+offset , 1); offset+=1;

	memcpy(&APM.GPS_vx, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.GPS_vy, readData.c_str()+offset , 4); offset+=4;
	memcpy(&APM.GPS_vz, readData.c_str()+offset , 4); offset+=4;

	memcpy(&APM.gps_ms, readData.c_str()+offset , 2); offset+=2;
	memcpy(&APM.gps_day, readData.c_str()+offset , 1); offset+=1;
	memcpy(&APM.gps_week, readData.c_str()+offset , 2); offset+=2;

	memcpy(&APM.ahrs_state, readData.c_str()+offset , 1); offset+=1;

	// memcpy(&APM, readData.c_str(), sizeof(readData.c_str()));

	sensor_msgs::Imu msg;
	msg.header.frame_id = "/imu";
	msg.header.stamp = ros::Time::now();
	msg.angular_velocity.x = APM.roll_rate;
	msg.angular_velocity.y = APM.pitch_rate;
	msg.angular_velocity.z = -APM.yaw_rate;
	// msg.linear_acceleration.x = APM.accel_x;
	// msg.linear_acceleration.y = APM.accel_y;
	// msg.linear_acceleration.z = APM.accel_z;
	Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(APM.roll, Eigen::Vector3f::UnitX()));
	Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(APM.pitch, Eigen::Vector3f::UnitY()));
	Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(APM.yaw, Eigen::Vector3f::UnitZ())); 
	Eigen::Quaternionf rotation;
	rotation=yawAngle * pitchAngle * rollAngle;
	// 东北地坐标系转成XYZ坐标系
	Eigen::Vector3f accel(APM.accel_x, -APM.accel_y, -APM.accel_z);
	accel = rotation * accel;
	msg.linear_acceleration.x = accel.x();
	msg.linear_acceleration.y = accel.y();
	msg.linear_acceleration.z = accel.z();
	msg.orientation.x = rotation.x();
	msg.orientation.y = rotation.y();
	msg.orientation.z = rotation.z();
	msg.orientation.w = rotation.w();

	pub_imu.publish(msg);
}

void ImuDriverNode::spin()
{
    ros::Rate loop_rate(framerate_);
    while(ros::ok())
    {
		checkPort();
		// ros::spinOnce();
        loop_rate.sleep();
    }
}

#endif

int main(int argc, char **argv)
{
    //初始化
    ros::init(argc, argv, "nav40_node");
    // ros::NodeHandle nh;

#ifdef ORIGIN
	{
		int fd = open("/dev/ttyUSB0", O_RDWR);              //打开串口
		memset(queue_cycle.Recbuf, 0, MAXSIZE);            //初始化缓冲数组
		queue_cycle.tail = 0;                           //初始化缓冲数组指针
		queue_cycle.head = 0;
		if (fd == -1)
		{
		printf("open error.\n");
		return 0;
		}
		set_uart_baudrate(fd, 115200);                 //串口初始化
		//公告消息
		ros::Publisher nav40_pitch = nh.advertise<std_msgs::Float32>
				("nav40/pitch", 10);

		//设置循环频率
		ros::Rate rate(500.0);

		cout << "open success"<< endl;
		
		while(ros::ok())
		{
			//nav40_demo::nav40_msg Pitch;  //如需自定义消息类型可仿造
			std_msgs::Float32  Pitch;
		
			//循环队列读取串口数据
			unsigned char buf[1];
			int len = read(fd, buf, 1);
			memcpy(queue_cycle.Recbuf + queue_cycle.tail, buf, len);
			queue_cycle.tail = (queue_cycle.tail + 1) % MAXSIZE;
			
		//进入帧结构判断
		//循环队列大于等于2倍的长度，才进入帧结构的判断
			while ((queue_cycle.tail>queue_cycle.head && queue_cycle.tail - queue_cycle.head >= NAV_DL) || (queue_cycle.tail<queue_cycle.head && (MAXSIZE - queue_cycle.head + queue_cycle.tail) >= NAV_DL))
			{
				if (queue_cycle.Recbuf[queue_cycle.head] == NAV_DH1)   //校验帧头
				{
					queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
					if (queue_cycle.Recbuf[queue_cycle.head] == NAV_DH2)   //校验帧头
					{
						queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
						for (int k = 0; k <= 117; k++)
						{
							checksum += queue_cycle.Recbuf[queue_cycle.head];
							queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
						}
						checksum = checksum + 0xEB + 0x90;
						checkRes_L = checksum & 0x00ff;
						checkRes_H = (checksum >> 8) & 0x00ff;
						checksum = 0;       //必须清零
						//检验和
						if (queue_cycle.Recbuf[queue_cycle.head] == checkRes_L && queue_cycle.Recbuf[(queue_cycle.head + 1) % MAXSIZE] == checkRes_H)
						{   //校验和通过
							for (int j = 121; j>=0; j--)
							{
								temp_buf[121-j]= queue_cycle.Recbuf[(queue_cycle.head + MAXSIZE - j+1) % MAXSIZE];
							}
							memcpy(&APM, temp_buf,122 );  // 将一帧完整的数据帧拷贝到结构体                                                     
							//在这里访问结构体成员即可      
							printf("apm_counter:%d\r\n",APM.counter);
							Pitch.data=APM.pitch*180/3.1415926;
							nav40_pitch.publish(Pitch);//发布消息           
						}
					}
				}
				else queue_cycle.head = (queue_cycle.head + 1) % MAXSIZE;
			}
			// rate.sleep();
		}
	}
#else
	ImuDriverNode ImuDriverNode_;
	ImuDriverNode_.spin();
#endif
    return 0;
}

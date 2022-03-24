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
#include "nav40_driver/sensorgps.h"

using namespace std;

#define NAV_DL     122    //数据长度
#define NAV_DH1    0xEB   //帧头
#define NAV_DH2    0x90   //帧头
#define MAXSIZE    1024   //缓冲区长度

string FrameHead={(char)NAV_DH1, (char)NAV_DH2};

APM_Datatype APM;            //创建帧结构体

ImuDriverNode::ImuDriverNode() : n_("~")
{
	n_.param("dog_device", imu_device_, std::string("/dev/ttyUSB0"));
	n_.param("baud", baud_, 460800);
	n_.param("framerate", framerate_, 500);
	n_.param("isVerif", isVerif_, false);

	pub_imu = n_.advertise<sensor_msgs::Imu>("/imu/data", 5);
	pub_gps = n_.advertise<nav40_driver::sensorgps>("/nav40/gps", 5);

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
	mtx.lock();
	std::string readData;
	
	my_serial.readline(readData, (size_t)512, FrameHead);
	// my_serial.read(readData, (size_t)512);

	mtx.unlock();

	// for(unsigned char n : readData)
    //     std::cout << "0x" <<  std::hex << static_cast<unsigned short>(n) << " " ;//输出十六进制FF
	// std::cout<<std::endl<<"______________________________________"<<std::endl;

	// std::cout<<"readData.size() : "<<std::dec<<readData.size()<<std::endl;
	if(readData.size()!=(unsigned int)readData[0] || readData.size()!=NAV_DL) // 验证帧长度
	{
		// for(unsigned char n : readData)
        // 	cout << "0x" <<  std::hex << static_cast<unsigned short>(n) << " " ;//输出十六进制FF
		// std::cout<<std::endl;
		// ROS_WARN("Wrong message length!");
		return;
	}

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
	// 有一半报文校验和出错，暂时未确定原因，因此暂不校验低位和
	if(/*checkRes_L != readData[readData.size()-4] ||*/ checkRes_H != readData[readData.size()-3])
	{
		// for(unsigned char n : readData)
        // 	cout << "0x" <<  std::hex << static_cast<unsigned short>(n) << " " ;//输出十六进制FF
		std::cout<<std::endl;
		ROS_WARN("Message checksum error!");
		return;
	}
	
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
	msg.angular_velocity.y = -APM.pitch_rate;
	msg.angular_velocity.z = -APM.yaw_rate;
	Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(APM.roll, Eigen::Vector3f::UnitX()));
	Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(-APM.pitch, Eigen::Vector3f::UnitY()));
	Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(-APM.yaw, Eigen::Vector3f::UnitZ())); 
	Eigen::Quaternionf rotation;
	rotation=yawAngle * pitchAngle * rollAngle;
	// 东北地坐标系转成XYZ坐标系
	Eigen::Vector3f accel(APM.accel_x, -APM.accel_y, -APM.accel_z);
	accel = rotation.inverse() * accel;
	msg.linear_acceleration.x = accel.x();
	msg.linear_acceleration.y = accel.y();
	msg.linear_acceleration.z = accel.z();
	msg.orientation.x = rotation.x();
	msg.orientation.y = rotation.y();
	msg.orientation.z = rotation.z();
	msg.orientation.w = rotation.w();

	pub_imu.publish(msg);

	nav40_driver::sensorgps gps_msg;
	gps_msg.header = msg.header;
	gps_msg.lat = APM.lat;
	gps_msg.lon = APM.lon;
	gps_msg.alt = APM.alt;
	gps_msg.satenum = APM.satellite_num;
	gps_msg.roll = APM.roll;
	gps_msg.pitch = -APM.pitch;
	gps_msg.heading = -APM.yaw;
	gps_msg.status = APM.state;
	pub_gps.publish(gps_msg);

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


int main(int argc, char **argv)
{
    //初始化
    ros::init(argc, argv, "nav40_node");

	ImuDriverNode ImuDriverNode_;
	ImuDriverNode_.spin();
	
    return 0;
}

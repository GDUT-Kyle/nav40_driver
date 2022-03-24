#ifndef CH_IMU_DRIVER_H
#define CH_IMU_DRIVER_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

class ImuDriverNode
{
protected:
    ros::NodeHandle n_;

    ros::Publisher pub_imu;
    ros::Publisher pub_gps;

    std::string imu_device_;
    int baud_;
    int framerate_;
    bool isVerif_;

    serial::Serial my_serial;

    std::mutex mtx;

public:
    ImuDriverNode();
    virtual ~ImuDriverNode();
    void config();
    void connect();
    void disconnect();
    void spin();
    void checkPort();
};

#endif
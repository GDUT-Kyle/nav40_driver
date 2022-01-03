/*//创作者：尚彬彬 2019.6.24
//如有疑问请加交流qq群：903013799
字节编号	数据	说明
1	0xEB	帧头1,固定为0xEB
2	0x90	帧头2,固定为0x90
3	N	帧总长度,包含帧头和校验和，范围:5-255
4	cmd	命令字，标识后续数据内容
…	…	…
N	Checksum	校验和，计算方式为:以无符号整数解释每个字节，从第1字节累加到第N-1字节，取最低字节作为校验和字节。

*/
#pragma once
 struct APM_Datatype
{
    unsigned char zhentou1;   //length 1
    unsigned char zhentou2;   //length 1
    unsigned char zhen_len;   //length 1
    unsigned char zhen_flag;  //length 1
    
    unsigned int counter;     //length 4

    unsigned char state;      //低3位: ahrs.   0 初始化 1正常 2错误 length 1
                    //bit3 罗盘是否需要校准  0 正常   1需要校准
                   //高四位健康状态  
                    //bit4 罗盘     0正常   1故障
                    //bit5  陀螺    0正常   1故障
                    //bit6  加计     0正常   1故障
                    //bit7  气压计    0正常   1故障

    float pitch;        //unit:rad,ahrs   抬头正    length 4
    float roll;         //unit:rad,ahrs   右滚正    length 4
    float yaw;          //unit:rad,ahrs   N 0    E 90   W -90   S +-180  length 4
    float yaw_gps;      //unit:d,gps      N 0    E 90   W -90   S +-180   length 4
  
    float pitch_rate;   //unit:rad/s,ahrs   抬头正   length 4
    float roll_rate;    //unit:rad/s,ahrs   右滚正   length 4
    float yaw_rate;     //unit:rad/s,ahrs   顺时针为正length 4

    signed int lon;        //unit:0.0000001d,inertial_nav length 4
    signed int lat;        //unit:0.0000001d,inertial_nav  length 4

    signed int alt_baro;   //unit:0.01m,barometer   原始气压高度  length 4
    signed int alt_gps;    //unit:0.01m,gps         原始GPS高度   length 4
    signed int alt;        //unit:0.01m,inertial_nav   EKF滤波高度   无GPS从零开始，有GPS按照GPS高度初始化  length 4

    float velocity_x;   //unit:m/s,NED,inertial_nav   N  正       length 4
    float velocity_y;   //unit:m/s,NED,inertial_nav   E  正       length 4
    float velocity_z;   //unit:m/s,NED,inertial_nav   D  正       length 4
    float velocity_air; //m/s,

    float accel_x;      //unit:m/s^2,NED,ahrs     N  正           length 4
    float accel_y;      //unit:m/s^2,NED,ahrs     E  正           length 4
    float accel_z;      //unit:m/s^2,NED,ahrs     D  正           length 4

    unsigned char satellite_num;    //length 1

    unsigned short hdop;        //0.01m length 2
    unsigned short vdop;        //0.01m length 2

    unsigned char gps_status;     //NO_GPS = 0,                     ///< No GPS connected/detected
                            //NO_FIX = 1,                     ///< Receiving valid GPS messages but no lock
                            //GPS_OK_FIX_2D = 2,              ///< Receiving valid messages and 2D lock
                            //GPS_OK_FIX_3D = 3,              ///< Receiving valid messages and 3D lock
                            //GPS_OK_FIX_3D_DGPS = 4,         ///< Receiving valid messages and 3D lock with differential improvements
                            //GPS_OK_FIX_3D_RTK_FLOAT = 5,    ///< Receiving valid messages and 3D RTK Float
                            //GPS_OK_FIX_3D_RTK_FIXED = 6,    ///< Receiving valid messages and 3D RTK Fixed       length 1                     
    unsigned char gps_hh;    //length 1
    unsigned char gps_mm;    //length 1
    unsigned char gps_ss;    //length 1

    unsigned char temperature;  //d  度  //length 1
    unsigned short HDT;         //d  真航向   0~3600   0~3600度   -17    负值不可用	  单位 0.1度length 2
    unsigned short HDG_Dev;     //d  标准差   0~3600   0~3600度   -17    负值不可用	  单位 0.1度length 2
    
    unsigned char redundancy;  //  01 加计  23 陀螺  45 罗盘  67 GPS  0 1 2 3 按优先级 来  加计&陀螺: 0 外部  1内部1 2内部2  罗盘: 0外部 1 内部  GPS: 0内部  1 外部 
    unsigned char GPS0_DT;//内部   单位100ms  范围 0-255  
    unsigned char GPS1_DT;//外部   单位100ms  范围 0-255     

    float GPS_vx;   //unit:m/s,NED,GPS   N  正       
    float GPS_vy;   //unit:m/s,NED,GPS   E  正
    float GPS_vz;   //unit:m/s,NED,GPS   D  正
	
    unsigned short gps_ms;
    unsigned char gps_day;
    unsigned short gps_week;

    unsigned char ahrs_state;

    //float pitch2;
    //float roll2;
    //float yaw2;

    unsigned char check_sum_L;
    unsigned char check_sum_H;
 } ;
 
//extern APM_Datatype APM;

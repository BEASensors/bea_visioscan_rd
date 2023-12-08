// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.
#ifndef PROTOCOL_INFO_H
#define PROTOCOL_INFO_H
#include <vector>
#define ROS_RANGE_MAX  25.0

const unsigned char CMD_cWN_SendMDI[20]={0x02 ,0x02 ,0xBE ,0xA0 ,0x12 ,0x34 ,0x00 ,0x0B ,0x63 ,0x57 ,0x4E ,0x20 ,0x53 ,0x65 ,0x6E ,0x64 ,0x4D ,0x44 ,0x49 ,0x26};
const unsigned char CMD_cRN_GetResol[21]={0x02 ,0x02 ,0xBE ,0xA0 ,0x12 ,0x34 ,0x00 ,0x0C ,0x63 ,0x52 ,0x4E ,0x20 ,0x47 ,0x65 ,0x74 ,0x52 ,0x65 ,0x73 ,0x6F ,0x6C ,0x4E};
const unsigned char CMD_cRN_GetResol_HEX[14]={0x02 ,0x63 ,0x52 ,0x4E ,0x20 ,0x47 ,0x65 ,0x74 ,0x52 ,0x65 ,0x73 ,0x6F ,0x6C ,0x03};
//const unsigned char CMD_cRN_GetResol[20]={0x02,0x02 ,0xBE ,0xA0 ,0x12 ,0x34 ,0x00 ,0x0B ,0x63 ,0x52 ,0x4E ,0x20 ,0x47 ,0x65 ,0x74 ,0x50 ,0x6F ,0x72 ,0x74 ,0x30};

const unsigned char CMD_cRN_GetPort[13]  ={0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x50, 0x6F, 0x72, 0x74, 0x03};
const unsigned char CMD_cRN_GetRange[14] ={0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x52, 0x61, 0x6E, 0x67, 0x65, 0x03};
const unsigned char CMD_cRN_GetPType[14] ={0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x50, 0x54, 0x79, 0x70, 0x65, 0x03};
const unsigned char CMD_cRN_GetProto[14] ={0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x50, 0x72, 0x6F, 0x74, 0x6F, 0x03};
const unsigned char CMD_cRN_GetDir[12]   ={0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x44, 0x69, 0x72, 0x03};

#define TO_RADIAN                                                     0.017453292222222222
#define SYS_TIMEOUT                                                   1
#define SYS_CMD_REALTIME_NULL                                         0
#define SYS_CMD_REALTIME_RESP_NULL                                    0
#define SYS_CMD_REALTIME_GET_ANGULAR_RESOLUTION                       2001
#define SYS_CMD_REALTIME_GET_ANGLE_RANGE                              2002
#define SYS_CMD_REALTIME_GET_LIDAR_DATA_PACKET_TYPE                   2003
#define SYS_CMD_REALTIME_GET_PROTOCOL_TYPE                            2004
#define SYS_CMD_REALTIME_GET_DATA_DIRECTION                           2005


#define SYS_CMD_REALTIME_RESP_GET_ANGULAR_RESOLUTION_OK               3001
#define SYS_CMD_REALTIME_RESP_GET_ANGLE_RANGE_OK                      3002
#define SYS_CMD_REALTIME_RESP_GET_LIDAR_DATA_PACKET_TYPE_OK           3003
#define SYS_CMD_REALTIME_RESP_GET_PROTOCOL_TYPE_OK                    3004
#define SYS_CMD_REALTIME_RESP_GET_DATA_DIRECTION_OK                   3005


#define SYS_CMD_REALTIME_RESP_GET_ANGULAR_RESOLUTION_ERROR            4001
#define SYS_CMD_REALTIME_RESP_GET_ANGLE_RANGE_ERROR                   4002
#define SYS_CMD_REALTIME_RESP_GET_LIDAR_DATA_PACKET_TYPE_ERROR        4003
#define SYS_CMD_REALTIME_RESP_GET_PROTOCOL_TYPE_ERROR                 4004
#define SYS_CMD_REALTIME_RESP_GET_DATA_DIRECTION_ERROR                4005


//@Command Syntax  excluding the data
const std::string RESP_CMD_STR_GET_ANGLE_RANGE="cRA GetRange";
const std::string RESP_CMD_STR_GET_LIDAR_DATA_PACKET_TYPE="cRA GetPType";
const std::string RESP_CMD_STR_GET_PROTO_TYPE="cRA GetProto";
const std::string RESP_CMD_STR_GET_DATA_DIRECTION="cRA GetDir";

//@sets
const std::string SET_CMD_STR_LIDAR_DATA_PACKET_TYPE_DIS_INTEN="cWN SetPType 1";
const std::string SET_CMD_STR_LIDAR_DATA_PACKET_TYPE_DIS_ONLY="cWN SetPType 0";
const std::string SET_CMD_STR_SPIN_DIRECTION_CLOCKWISE="cWN SetDir 0";
const std::string SET_CMD_STR_SPIN_DIRECTION_COUNTERCLOCKWISE="cWN SetDir 1";



namespace bea_power {

typedef struct tag_BEA_PARAMETER_INFO
{
    int protocolType;         // 0-UDP  1-TCP
    int lidarDataPacketType;
    int scanDataDirection;    // 0-Clockwise  1-Countclockwise
    double startAngle;
    double stopAngle;
    double angularResolution;
    double freqHZ;
    double scan_time;
    double deltaAngle;
    bool isUpsideDown;        // 0-Normal  1-Upsidedown
}BEA_PARAMETER_INFO;
typedef struct tag_ROS_SCAN_INFO
{
    float ros_angle_min;
    float ros_angle_max;
    float ros_angle_increment;
    float ros_scan_time;
    float ros_range_min;
    float ros_range_max;
}ROS_SCAN_INFO;


//@ \class ProtocolInfo
//@ \brief Information about the HTTP/JSON protocol
struct ProtocolInfo
{
    std::string protocol_name;
    int version_major;
    int version_minor;
    std::vector< std::string > commands;
};

//@ \class HandleInfo
//@ \brief Encapsulates data about an etablished data connection
//@ 处理信息 结构体
struct HandleInfo
{
    static const int HANDLE_TYPE_TCP = 0;
    static const int HANDLE_TYPE_UDP = 1;
    int handle_type;
    std::string hostname;
    int port;
    std::string handle;
    char packet_type;
    int start_angle;
    bool watchdog_enabled;
    int watchdog_timeout;
};
}

#endif // PROTOCOL_INFO_H

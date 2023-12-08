// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.
#ifndef TCP_COMMAND_INTERFACE_H
#define TCP_COMMAND_INTERFACE_H
#include <string>
#include <map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/property_tree/ptree.hpp>
#include <protocol_info.h>
#include <common_libs.h>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <array>
#include <packet_structure.h>
#include <protocol_info.h>
#include <common_libs.h>
#include <ros/ros.h>

namespace bea_power {
class TcpCommandInterface
{
public:
    TcpCommandInterface(const std::string  http_host, int http_port=3050);
    TcpCommandInterface();
    ~TcpCommandInterface();
    void HandleTcpSocketRead(const boost::system::error_code& error, std::size_t bytes_transferred);
    void disconnect();
    double GetAngularResolution();
    double GetAngleRange();
    double GetLidarDataPacketType();
    int GetProtocolType();
    void SetCmdFactory(const std::string icmdSet);
    void SetLidarDataPacketType(int param);
    void SetScanDataDirection(int param);
    int GetScanDataDirection();
    void stopScanOutput();
    int sys_cmd_realtime_;
    int sys_cmd_resp_;
    const std::string& getHttpHost() const { return http_host_; }
    bool setParameter(const std::string name, const std::string value);
    boost::optional<std::string> getParameter(const std::string name);
    const BEA_PARAMETER_INFO GetParameters() const { return parameterInfo_; }
    std::map< std::string, std::string > getParameters( const std::vector< std::string >& names );
    std::vector< std::string > getParameterList();
    boost::optional<ProtocolInfo> getProtocolInfo();

private:
    boost::asio::ip::tcp::socket* tcp_socket_;
    boost::asio::streambuf inbuf_;
    std::istream instream_;
    boost::thread io_service_thread_;
    boost::asio::io_service io_service_;
    std::array< char, 65536 > tcp_buffer_;
    BEA_PARAMETER_INFO parameterInfo_;
    char setCmdBuf_[64];

    std::string http_host_;
    int http_port_;
    boost::property_tree::ptree pt_;
    int http_status_code_;
};
}

#endif

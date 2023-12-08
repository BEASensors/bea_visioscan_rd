// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.

#ifndef E449_DRIVER_H
#define E449_DRIVER_H

#include <string>
#include <vector>
#include <map>
#include <boost/optional.hpp>
#include <protocol_info.h>
#include <packet_structure.h>
#include "common_libs.h"

namespace bea_power {

class TcpCommandInterface;
class ScanDataReceiver;

class E449Driver
{
public:
    E449Driver();

    ~E449Driver();


    bool connect(const std::string hostname, int port=3050);
    void disconnect();
    bool isConnected() {return is_connected_; }
    bool startCapturingTCP(const std::string hostname, int port);
    bool startCapturingUDP(const std::string hostname, int port);
    bool stopCapturing();
    bool isCapturing();
    bool checkConnection();
    const ProtocolInfo& getProtocolInfo() { return protocol_info_; }
    const std::map< std::string, std::string >& getParameters();
    const std::map< std::string, std::string >& getParametersCached() const {return parameters_;}
    ScanData getScan();
    ScanData getFullScan();
    std::size_t getScansAvailable() const;
    std::size_t getFullScansAvailable() const;
    bool setScanFrequency( unsigned int frequency );
    bool setSamplesPerScan( unsigned int samples );
    bool rebootDevice();
    bool resetParameters( const std::vector<std::string>& names );
    bool setParameter( const std::string& name, const std::string& value );
    void configScanOutputDirection(int dir_);
    void configTopicID(std::string tid_, std::string fid_);
    void setMountDirection(bool dir_);

private:

    ScanDataReceiver* data_receiver_;
    TcpCommandInterface* command_interface_;
    BEA_PARAMETER_INFO parameterInfo_;
    bool is_connected_;
    bool is_capturing_;
    boost::optional<HandleInfo> handle_info_;
    ProtocolInfo protocol_info_;
    std::map< std::string, std::string > parameters_;
    int data_direction_;
    bool is_upsidedown_;
    std::string idTopic_;
    std::string idFrame_;
};

} // end namespace bea_power

#endif // E449_DRIVER_H

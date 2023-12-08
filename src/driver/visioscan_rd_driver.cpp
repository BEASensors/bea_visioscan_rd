// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.


#include <ctime>
#include <algorithm>
#include <visioscan_rd_driver.h>
#include <packet_structure.h>
#include <tcp_command_interface.h>
#include "scan_data_receiver.h"

namespace bea_power {

E449Driver::E449Driver()
{
    data_receiver_ = 0;
    is_connected_ = false;
    is_capturing_ = false;
    data_direction_ = 0;
    is_upsidedown_ = false;
}

bool E449Driver::connect(const std::string hostname, int port)
{
    std::string ilog;
    printf("@connect start\n");
    command_interface_ = new TcpCommandInterface(hostname, port);
    /* command_interface_->SetScanDataDirection(data_direction_); */
    command_interface_->GetScanDataDirection();
    command_interface_->GetAngularResolution();
    command_interface_->GetAngleRange();
    command_interface_->GetLidarDataPacketType();
    if(command_interface_->GetProtocolType()==1)
    {
        // Success to get protocol type
    }
    else
    {
        return false;
    }
    parameterInfo_=command_interface_->GetParameters();
    parameterInfo_.isUpsideDown=is_upsidedown_;
    if(parameterInfo_.protocolType==0)//@UDP
    {
        ilog=DEBUG_POS+" UDP start......";
        std::cout<<ilog<<std::endl;
        startCapturingUDP(hostname, port);
    }
    if(parameterInfo_.protocolType==1)//@TCP
    {
        startCapturingTCP(hostname, port);
    }
    is_connected_ = true;
    printf("@connect end\n");
    return true;
}

E449Driver::~E449Driver()
{
    disconnect();
}

bool E449Driver::startCapturingTCP(const std::string hostname, int port)
{
    data_receiver_ = new ScanDataReceiver(idTopic_,idFrame_,hostname,port,parameterInfo_);
    if( !data_receiver_->isConnected() )
    {
        return false;
    }
    is_capturing_ = true;
    return true;
}

bool E449Driver::startCapturingUDP(const std::string hostname, int port)
{
    data_receiver_ = new ScanDataReceiver(idTopic_,idFrame_,hostname,port,parameterInfo_,true);
    if( !data_receiver_->isConnected() )
    {
        return false;
    }

    is_capturing_ = true;
    return true;
}

bool E449Driver::stopCapturing()
{
    if( !is_capturing_ || !command_interface_ )
        return false;

    //bool return_val = checkConnection();
    command_interface_->stopScanOutput();
    return true;
}

bool E449Driver::checkConnection()
{
    /*
        TODO
    */
    return false;
}

ScanData E449Driver::getScan()
{
    if( data_receiver_ )
    {
        return data_receiver_->getScan();
    }
    else
    {
        std::cerr << "ERROR: No scan capturing started!" << std::endl;
        return ScanData();
    }
}

ScanData E449Driver::getFullScan()
{
    if( data_receiver_ )
    {
        return data_receiver_->getFullScan();
    }
    else
    {
        std::cerr << "@getFullScan ERROR: No scan capturing started!" << std::endl;
        return ScanData();
    }
}

std::size_t E449Driver::getScansAvailable() const
{
    if( data_receiver_ )
    {
        return data_receiver_->getScansAvailable();
    }
    else
    {
        std::cerr << "ERROR: No scan capturing started!" << std::endl;
        return 0;
    }
}

std::size_t E449Driver::getFullScansAvailable() const
{
    if( data_receiver_ )
    {
        return data_receiver_->getFullScansAvailable();
    }
    else
    {
        std::cerr << "ERROR: No scan capturing started!" << std::endl;
        return 0;
    }
}

void E449Driver::disconnect()
{
    if( isCapturing() )
        stopCapturing();

    delete data_receiver_;
    delete command_interface_;
    data_receiver_ = 0;
    command_interface_ = 0;

    is_capturing_ = false;
    is_connected_ = false;
}

bool E449Driver::isCapturing()
{
    return is_capturing_;
}

const std::map< std::string, std::string >& E449Driver::getParameters()
{
    return parameters_;
}

bool E449Driver::setScanFrequency(unsigned int frequency)
{
    /*
        TODO
    */
    return false;
}

bool E449Driver::setSamplesPerScan(unsigned int samples)
{
    /*
        TODO
    */
    return false;
}

bool E449Driver::rebootDevice()
{
    /*
        TODO
    */
    return false;
}

bool E449Driver::resetParameters(const std::vector<std::string> &names)
{
    /*
        TODO
    */
    return false;
}

bool E449Driver::setParameter(const std::string &name, const std::string &value)
{
    /*
        TODO
    */
    return false;
}

void E449Driver::configScanOutputDirection(int dir_)
{
    if(dir_ == 0 || dir_ == 1)
    {
        data_direction_ = dir_;
    }
}

void E449Driver::configTopicID(std::string tid_, std::string fid_)
{
    idTopic_ = tid_;
    idFrame_ = fid_;
}

void E449Driver::setMountDirection(bool dir_)
{
    is_upsidedown_ = dir_;
}

}

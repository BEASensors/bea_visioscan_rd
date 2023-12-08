// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.

#include "scan_data_receiver.h"
#include <chrono>
#include <ctime>

namespace bea_power {

//@TCP
ScanDataReceiver::ScanDataReceiver(const std::string topic, const std::string frame, const std::string hostname, const int port,BEA_PARAMETER_INFO bpi):inbuf_(4096),instream_(&inbuf_),ring_buffer_(65536),scan_data_()
{
    parameterInfo_=bpi;
    frame_id_ = frame;
    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>(topic,1000);
    printf("hostname=%s",hostname.c_str());
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf("startAngle=%f\n",parameterInfo_.startAngle);
    printf("stopAngle=%f\n",parameterInfo_.stopAngle);
    printf("angularResolution=%f\n",parameterInfo_.angularResolution);
    printf("lidarDataPacketType=%d\n",parameterInfo_.lidarDataPacketType);
    float _totalRadian=(parameterInfo_.stopAngle-parameterInfo_.startAngle)*TO_RADIAN;
    if(parameterInfo_.scanDataDirection==0) // Clockwise
    {
        rosScanInfo_.ros_angle_min=_totalRadian/2;
        rosScanInfo_.ros_angle_max=-_totalRadian/2;
        rosScanInfo_.ros_angle_increment=-parameterInfo_.angularResolution*TO_RADIAN;
    }
    else // Counterclockwise
    {
        rosScanInfo_.ros_angle_min=-_totalRadian/2;
        rosScanInfo_.ros_angle_max=_totalRadian/2;
        rosScanInfo_.ros_angle_increment=parameterInfo_.angularResolution*TO_RADIAN;
    }
    rosScanInfo_.ros_scan_time=parameterInfo_.scan_time;
    rosScanInfo_.ros_range_min=0;
    rosScanInfo_.ros_range_max=ROS_RANGE_MAX;
    printf("ros_angle_min=%f\n",rosScanInfo_.ros_angle_min);
    printf("ros_angle_max=%f\n",rosScanInfo_.ros_angle_max);
    printf("ros_angle_increment=%f\n",rosScanInfo_.ros_angle_increment);
    printf("ros_scan_time=%f\n",rosScanInfo_.ros_scan_time);
    printf("ros_range_min=%f\n",rosScanInfo_.ros_range_min);
    printf("ros_range_max=%f\n",rosScanInfo_.ros_range_max);
    is_reverse_ = parameterInfo_.isUpsideDown;
    last_data_time_ = std::time(0);
    tcp_socket_ = 0;
    udp_socket_ = 0;
    udp_port_ = -1;
    is_connected_ = false;
    std::cout << "Connecting to TCP data channel at " << hostname << ":" << port << " ... ";
    try
    {
        // Resolve hostname/ip
        boost::asio::ip::tcp::resolver resolver(io_service_);
        boost::asio::ip::tcp::resolver::query query(hostname, std::to_string(port));
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::ip::tcp::resolver::iterator end;

        tcp_socket_ = new boost::asio::ip::tcp::socket(io_service_);
        boost::system::error_code error = boost::asio::error::host_not_found;

        // Iterate over endpoints and etablish connection
        while (error && endpoint_iterator != end)
        {
            tcp_socket_->close();
            tcp_socket_->connect(*endpoint_iterator++, error);
        }
        if (error)
        {
            throw boost::system::system_error(error);
        }

        tcp_socket_->async_receive(boost::asio::buffer(&tcp_buffer_[0],tcp_buffer_.size()),\
                boost::bind(&ScanDataReceiver::handleSocketRead, this,\
                            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        is_connected_ = true;
        io_service_thread_.join();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}


//@ udp
ScanDataReceiver::ScanDataReceiver(const std::string topic, const std::string frame, const std::string hostname, int port,BEA_PARAMETER_INFO bpi,bool udpType):inbuf_(4096),instream_(&inbuf_),ring_buffer_(65536),scan_data_()
{
    parameterInfo_=bpi;
    frame_id_ = frame;
    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>(topic,1000);
    printf("hostname=%s",hostname.c_str());
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf("startAngle=%f\n",parameterInfo_.startAngle);
    printf("stopAngle=%f\n",parameterInfo_.stopAngle);
    printf("angularResolution=%f\n",parameterInfo_.angularResolution);
    printf("lidarDataPacketType=%d\n",parameterInfo_.lidarDataPacketType);
    float _totalRadian=(parameterInfo_.stopAngle-parameterInfo_.startAngle)*TO_RADIAN;
    if(parameterInfo_.scanDataDirection==0) // Clockwise
    {
        rosScanInfo_.ros_angle_min=_totalRadian/2;
        rosScanInfo_.ros_angle_max=-_totalRadian/2;
        rosScanInfo_.ros_angle_increment=-parameterInfo_.angularResolution*TO_RADIAN;
    }
    else // Counterclockwise
    {
        rosScanInfo_.ros_angle_min=-_totalRadian/2;
        rosScanInfo_.ros_angle_max=_totalRadian/2;
        rosScanInfo_.ros_angle_increment=parameterInfo_.angularResolution*TO_RADIAN;
    }
    rosScanInfo_.ros_scan_time=parameterInfo_.scan_time;
    rosScanInfo_.ros_range_min=0;
    rosScanInfo_.ros_range_max=ROS_RANGE_MAX;
    printf("ros_angle_min=%f\n",rosScanInfo_.ros_angle_min);
    printf("ros_angle_max=%f\n",rosScanInfo_.ros_angle_max);
    printf("ros_angle_increment=%f\n",rosScanInfo_.ros_angle_increment);
    printf("ros_scan_time=%f\n",rosScanInfo_.ros_scan_time);
    printf("ros_range_min=%f\n",rosScanInfo_.ros_range_min);
    printf("ros_range_max=%f\n",rosScanInfo_.ros_range_max);
    is_reverse_ = parameterInfo_.isUpsideDown;
    tcp_socket_ = 0;
    udp_socket_ = 0;
    udp_port_ = -1;
    is_connected_ = false;
    try
    {
        udp_socket_ = new boost::asio::ip::udp::socket(io_service_, boost::asio::ip::udp::v4());
        udp_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));
        udp_port_ = udp_socket_->local_endpoint().port();
        // Start async reading
        udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0],udp_buffer_.size()), udp_endpoint_,
                boost::bind(&ScanDataReceiver::handleSocketRead, this,
                            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        is_connected_ = true;
        io_service_thread_.join();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
    std::cout << "Receiving scanner data at local UDP port " << udp_port_ << " ... ";

}

ScanDataReceiver::~ScanDataReceiver()
{
    disconnect();
    delete udp_socket_;
    delete tcp_socket_;
}

void ScanDataReceiver::handleSocketRead(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    if (!error )
    {
        // Read all received data and write it to the internal ring buffer
        writeBufferBack(&udp_buffer_[0],bytes_transferred);
        // Handle (read and parse) packets stored in the internal ring buffer
        while( handleNextPacket() ) {}
        // Read data asynchronously
        udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0],udp_buffer_.size()), udp_endpoint_,
                boost::bind(&ScanDataReceiver::handleSocketRead, this,
                            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        if( error.value() != 995 )
            std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
        disconnect();
    }
    last_data_time_ = std::time(0);
}

bool ScanDataReceiver::handleNextPacket()
{
    // Search for a packet
    int packet_start = findPacketStart();
    if( packet_start<0 )
    {
        return false;
    }

    // Try to retrieve packet
    char buf[65536];
    PacketTypeC* p = (PacketTypeC*) buf;
    if( !retrievePacket(packet_start,p) )
    {
        return false;
    }
    // Lock internal outgoing data queue, automatically unlocks at end of function
    std::unique_lock<std::mutex> lock(data_mutex_);

    // Create new scan container if necessary
    if ( scan_data_.empty() ) //init
    {
        scan_data_.emplace_back();
        if( scan_data_.size()>100 )
        {
            scan_data_.pop_front();
            std::cerr << "Too many scans in receiver queue: Dropping scans!" << std::endl;
        }
        data_notifier_.notify_one();
    }

    // Parse payload of packet
    std::uint16_t* p_scan_data = (std::uint16_t*) &buf[MDI_PACKET_HEADER_SIZE];
    int num_scan_points = p->header.ScanPoints;
    if(p->header.SubNO==1)
    {
        ScanPointsSum_=0;
        std::vector<float>().swap(sacnData_);
        std::vector<float>().swap(intensitiesDtata_);
    }
    ScanPointsSum_+=p->header.ScanPoints;
    //@Type of scan data packet, 0 = distance only, 1= distance & intensity ;
    if(p->header.PacketType==1)//1= distance & intensity
    {
        for(int i=0; i<num_scan_points; i++ )
        {
            unsigned short data = ntohs(p_scan_data[i]);
            sacnData_.push_back(data);
        }
        for(int i=num_scan_points; i<num_scan_points*2; i++ )
        {
            unsigned short data1 = ntohs(p_scan_data[i]);
            intensitiesDtata_.push_back(data1);
        }
    }
    else
    {
        for( int i=0; i<num_scan_points; i++ )
        {
            unsigned short data = ntohs(p_scan_data[i]);
            sacnData_.push_back(data);
        }
    }

    if(p->header.SubNO==p->header.TotalNO)
    {
        //@start
        int _scanPointLen=sacnData_.size();
        sensor_msgs::LaserScan scanmsg;
        scanmsg.header.frame_id = frame_id_;
        scanmsg.header.stamp = ros::Time::now();
        if(!is_reverse_)
        {
            scanmsg.angle_min = rosScanInfo_.ros_angle_min;
            scanmsg.angle_max = rosScanInfo_.ros_angle_max;
            scanmsg.angle_increment = float(p->header.DeltaAngle)/1000.0f*TO_RADIAN;
        }
        else
        {
            scanmsg.angle_min = -rosScanInfo_.ros_angle_min;
            scanmsg.angle_max = -rosScanInfo_.ros_angle_max;
            scanmsg.angle_increment = -float(p->header.DeltaAngle)/1000.0f*TO_RADIAN;
        }
        scanmsg.scan_time = rosScanInfo_.ros_scan_time;
        scanmsg.time_increment = rosScanInfo_.ros_scan_time / float(_scanPointLen);
        scanmsg.range_min = 0.0;
        scanmsg.range_max = rosScanInfo_.ros_range_max;
        scanmsg.ranges.resize(_scanPointLen);
        scanmsg.intensities.resize(_scanPointLen);
        if(p->header.PacketType==1)//@distance&intensity
        {
            for( int i=0; i<_scanPointLen; i++ )
            {
                scanmsg.ranges[i] = float(sacnData_[i])/1000.0f;
                scanmsg.intensities[i]=float(intensitiesDtata_[i]);
            }
        }
        else//@distance only
        {
            for( int i=0; i<_scanPointLen; i++ )
            {
                scanmsg.ranges[i] = float(sacnData_[i])/1000.0f;
            }
        }
        pub_scan_.publish(scanmsg);
    }

    return true;
}


int ScanDataReceiver::findPacketStart()
{
    if( ring_buffer_.size()<60 )
    {
        return -1;
    }
    for( std::size_t i=0; i<ring_buffer_.size()-4; i++)
    {
        if(   ((unsigned char) ring_buffer_[i])      == 0xBE
              && ((unsigned char) ring_buffer_[i+1]) == 0xA0
              && ((unsigned char) ring_buffer_[i+2]) == 0x12
              && ((unsigned char) ring_buffer_[i+3]) == 0x34 )
        {
            return i;
        }
    }
    return -2;
}

void ScanDataReceiver::PacketHeaderToLittleEndian(PacketTypeC *pHead)
{
    pHead->header.PacketSize=ntohs(pHead->header.PacketSize);
    pHead->header.ReservedA=ntohs(pHead->header.ReservedA);
    pHead->header.ReservedB=ntohs(pHead->header.ReservedB);
    pHead->header.ReservedC=ntohs(pHead->header.ReservedC);
    pHead->header.PacketNO=ntohs(pHead->header.PacketNO);
    pHead->header.ScanFreq=ntohs(pHead->header.ScanFreq);
    pHead->header.ScanPoints=ntohs(pHead->header.ScanPoints);
    pHead->header.FirstAngle=ntohl(pHead->header.FirstAngle);
    pHead->header.DeltaAngle=ntohl(pHead->header.DeltaAngle);
    pHead->header.Timestamp=ntohs(pHead->header.Timestamp);
}

/**
 * @brief Find the complete packet according to the starting position
 * @param start,starting position index
 * @param p,packet
 * @return
 */
bool ScanDataReceiver::retrievePacket(std::size_t start, PacketTypeC *p)
{
    if( ring_buffer_.size()<MDI_PACKET_HEADER_SIZE )
    {
        printf("@retrievePacket error 1\n");
        return false;
    }

    //@从找到开始，开始之前全不要
    ring_buffer_.erase_begin(start);
    char* pp = (char*) p;
    // Read header
    readBufferFront(pp, MDI_PACKET_HEADER_SIZE);
    PacketHeaderToLittleEndian(p);

    if( ring_buffer_.size() < p->header.PacketSize )
    {
        ShowCharArray(pp, MDI_PACKET_HEADER_SIZE);
        printf("p->header.TotalNO=%d\n",p->header.TotalNO);
        printf("p->header.SubNO=%d\n",p->header.SubNO);
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!(%ld,%d)\n",ring_buffer_.size(),p->header.PacketSize);
        printf("@retrievePacket error 2\n");
        return false;
    }

    // Read header+payload data
    readBufferFront(pp,p->header.PacketSize);
    // Erase packet from ring buffer
    PacketHeaderToLittleEndian(p);

    //@erase
    ring_buffer_.erase_begin(p->header.PacketSize);
    return true;
}


void ScanDataReceiver::disconnect()
{
    is_connected_ = false;
    try
    {
        if( tcp_socket_ )
        {
            tcp_socket_->close();
        }
        if( udp_socket_ )
        {
            udp_socket_->close();
        }
        io_service_.stop();
        if( boost::this_thread::get_id() != io_service_thread_.get_id() )
        {
            io_service_thread_.join();
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}


bool ScanDataReceiver::checkConnection()
{
    if( !isConnected() )
        return false;
    if( (std::time(0)-last_data_time_) > 2 )
    {
        disconnect();
        return false;
    }
    return true;
}


ScanData ScanDataReceiver::getScan()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    ScanData data(std::move(scan_data_.front()));
    scan_data_.pop_front();
    return data;
}


ScanData ScanDataReceiver::getFullScan()
{
    std::unique_lock<std::mutex> lock(data_mutex_);

    printf("@------getFullScan scan_data_.size()=%ld\n",scan_data_.size());
    ScanData data;
    if( scan_data_.size() >= 1 && isConnected() )
    {
        data = ScanData(std::move(scan_data_.front()));
        scan_data_.pop_front();
    }
    return data;
}


std::size_t ScanDataReceiver::getFullScansAvailable() const
{
    if( scan_data_.size() == 0 )
        return 0;
    else
        return scan_data_.size()-1;
}

void ScanDataReceiver::writeBufferBack(char *src, std::size_t numbytes)
{
    if( ring_buffer_.size()+numbytes > ring_buffer_.capacity() )
    {
        printf("@writeBufferBack error!");
        throw std::exception();
    }
    ring_buffer_.resize(ring_buffer_.size()+numbytes);
    char* pone = ring_buffer_.array_one().first;             //起始
    std::size_t pone_size = ring_buffer_.array_one().second; //起始到尾大小
    char* ptwo = ring_buffer_.array_two().first;
    std::size_t ptwo_size = ring_buffer_.array_two().second;
    if( ptwo_size >= numbytes )
    {
        std::memcpy(ptwo+ptwo_size-numbytes, src, numbytes);
    }
    else
    {
        std::memcpy(pone+pone_size+ptwo_size-numbytes,
                    src,
                    numbytes-ptwo_size );
        std::memcpy(ptwo,
                    src+numbytes-ptwo_size,
                    ptwo_size );
    }
}


void ScanDataReceiver::readBufferFront(char *dst, std::size_t numbytes)
{
    if( ring_buffer_.size() < numbytes )
    {
        printf("@readBufferFront error!");
        throw std::exception();
    }
    char* pone = ring_buffer_.array_one().first;
    std::size_t pone_size = ring_buffer_.array_one().second;

    char* ptwo = ring_buffer_.array_two().first;

    if( pone_size >= numbytes )
    {
        std::memcpy( dst, pone, numbytes );
    }
    else
    {
        std::memcpy( dst, pone, pone_size );
        std::memcpy( dst+pone_size, ptwo, numbytes-pone_size);
    }
}

}

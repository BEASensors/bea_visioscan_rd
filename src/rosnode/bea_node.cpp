// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.


#include "bea_node.h"
#include <sensor_msgs/LaserScan.h>
#include <visioscan_rd_driver.h>
#include <signal.h>

namespace bea_power {
BEANode::BEANode():nh_("~")
{
    driver_ = 0;
    //@Reading and checking parameters
    nh_.param("frame_id", frame_id_, std::string("/laser"));
    nh_.param("laser_ip",laser_ip_,std::string("192.168.1.2"));
    nh_.param("laser_port",laser_port_,3050);
    nh_.param("scan_frequency",scan_frequency_,80);
    nh_.param("samples_per_scan",samples_per_scan_,1377);
    nh_.param("laser_direction",laser_direction_,0);
    nh_.param("scan_topic_id",scan_topic_id_, std::string("scan"));

    if( laser_ip_ == "" )
    {
        std::cerr << "IP of laser range finder not set!" << std::endl;
        return;
    }
    connect();

    sub_cmd_code_ = nh_.subscribe("control_command",100,&BEANode::cmdMsgCallback,this);
}


bool BEANode::connect()
{
    delete driver_;
    driver_ = new E449Driver();
    driver_->setMountDirection(laser_direction_);
    driver_->configTopicID(scan_topic_id_,frame_id_);
    std::cout << "Connecting to scanner at " << laser_ip_ << " ... "<<std::endl;
    if( driver_->connect(laser_ip_,laser_port_) )
    {
        std::cout << "@BEANode::connect OK" << std::endl;
    }
    else
    {
        std::cout << "FAILED!" << std::endl;
        std::cerr << "Connection to scanner at " << laser_ip_ << " failed!" << std::endl;
        return false;
    }

    return true;
}


void BEANode::getScanData(const ros::TimerEvent &e)
{
    auto scandata = driver_->getFullScan();

    if( scandata.distance_data.empty() )
    {
        ROS_ERROR("data NULL!");
        //sleep(10);
        return;
    }

    sensor_msgs::LaserScan scanmsg;
    scanmsg.header.frame_id = frame_id_;
    scanmsg.header.stamp = ros::Time::now();

    scanmsg.angle_min = -M_PI;
    scanmsg.angle_max = +M_PI;
    scanmsg.angle_increment = 2*M_PI/float(scandata.distance_data.size());
    scanmsg.time_increment = 1/35.0f/float(scandata.distance_data.size());

    scanmsg.scan_time = 1/std::atof(driver_->getParametersCached().at("scan_frequency").c_str());
    scanmsg.range_min = std::atof(driver_->getParametersCached().at("radial_range_min").c_str());
    scanmsg.range_max = std::atof(driver_->getParametersCached().at("radial_range_max").c_str());

    scanmsg.ranges.resize(scandata.distance_data.size());
    scanmsg.intensities.resize(scandata.amplitude_data.size());
    for( std::size_t i=0; i<scandata.distance_data.size(); i++ )
    {
        scanmsg.ranges[i] = float(scandata.distance_data[i])/1000.0f;
        scanmsg.intensities[i] = scandata.amplitude_data[i];
    }

    sensor_msgs::LaserScan _new_scan=scanmsg;
    int _rangeAngle=300;  //@角度
    double _halfRadian=_rangeAngle*0.01745329/2;
    _new_scan.angle_min=- _halfRadian;
    _new_scan.angle_max= _halfRadian;
    int _initLen=scanmsg.ranges.size();
    int _len=round(_rangeAngle*0.01745329/_new_scan.angle_increment);
    int _halfLen=(_initLen-_len)/2;
    std::vector<float>().swap(_new_scan.ranges);
    std::vector<float>().swap(_new_scan.intensities);
    std::vector <float> _range ;     //@清除容器并最小化它的容量
    std::vector <float> _intensities ;
    for(int i=_halfLen;i<_initLen-_halfLen;i++)
    {
        _range.push_back(scanmsg.ranges[i]);
        _intensities.push_back(scanmsg.intensities[i]);
    }
    _new_scan.ranges=_range;
    _new_scan.intensities=_intensities;
    pub_scan_.publish(_new_scan);
}

void BEANode::cmdMsgCallback(const std_msgs::StringConstPtr &msg)
{
    const std::string& cmd = msg->data;
    static const std::string set_scan_frequency_cmd("set scan_frequency=");
    static const std::string set_samples_per_scan_cmd("set samples_per_scan=");

    if( cmd.substr(0,set_scan_frequency_cmd.size()) == set_scan_frequency_cmd )
    {
        std::string value = cmd.substr(set_scan_frequency_cmd.size());
        int frequency = std::atoi(value.c_str());
        if(frequency>=10 && frequency<=50)
        {
            scan_frequency_ = frequency;
            driver_->setScanFrequency(frequency);
        }
    }
    if( cmd.substr(0,set_samples_per_scan_cmd.size()) == set_samples_per_scan_cmd )
    {
        std::string value = cmd.substr(set_samples_per_scan_cmd.size());
        int samples = std::atoi(value.c_str());
        if(samples>=72 && samples<=25200)
        {
            samples_per_scan_ = samples;
            driver_->setSamplesPerScan(samples);
        }
    }
}

} // NS

std::string convert_ASCII(std::string hex)
{
   std::string ascii = "";
   for (size_t i = 0; i < hex.length(); i += 2)
   {
      //taking two characters from hex string
      std::string part = hex.substr(i, 2);
      //changing it into base 16
      char ch = stoul(part, nullptr, 16);
      //putting it into the ASCII string
      ascii += ch;
   }
   return ascii;
}

void sigIntHandler(int sig)
{
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bea_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    std::cout<<"bea_node start"<<std::endl;

    new bea_power::BEANode();

    signal(SIGINT, sigIntHandler);

    ros::spin();

    return 0;
}

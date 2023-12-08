// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.

#ifndef BEA_NODE_H
#define BEA_NODE_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <deque>
#include <common_libs.h>


namespace bea_power {
class E449Driver;

//@ BEANode
class BEANode
{
public:
    BEANode();
    void cmdMsgCallback( const std_msgs::StringConstPtr& msg );

private:
    bool connect();
    void getScanData( const ros::TimerEvent& e);
    ros::NodeHandle nh_;
    ros::Timer get_scan_data_timer_;
    ros::Publisher pub_scan_;
    ros::Subscriber sub_cmd_code_;
    std::string frame_id_;
    std::string laser_ip_;
    int laser_port_;
    int scan_frequency_;
    int samples_per_scan_;
    int laser_direction_;
    E449Driver* driver_;
    std::string scan_topic_id_;
};
}

#endif // BEA_NODE_H

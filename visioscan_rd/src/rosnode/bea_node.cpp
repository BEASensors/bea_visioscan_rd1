// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.


#include "bea_node.h"
#include <sensor_msgs/LaserScan.h>
#include "visioscan_rd_driver.h"
#include <signal.h>

namespace bea_power {
BEANode::BEANode() : nh_("~")
{
    driver_ = 0;
    //@Reading and checking parameters
    nh_.param("frame_id", frame_id_, std::string("laser"));
    nh_.param("laser_ip", laser_ip_, std::string("192.168.1.2"));
    nh_.param("laser_port", laser_port_, 3050);
    nh_.param("scan_frequency", scan_frequency_, 80);
    nh_.param("samples_per_scan", samples_per_scan_, 1377);
    nh_.param("laser_direction", laser_direction_, 0);
    nh_.param("topic_id", topic_id_, std::string("scan"));

    if(laser_ip_ == "")
    {
        std::cerr << "IP of scanner not set!" << std::endl;
        return;
    }

    if(!connect())
        return;

    check_connection_timer = nh_.createTimer(ros::Duration(1.0f / (2.0f * float(driver_->GetParameters().freqHZ))), &BEANode::checkConnection, this);
    // get_scan_data_timer_ = nh_.createTimer(ros::Duration(1.0f/(4.0f*float(driver_->GetParameters().freqHZ))), &BEANode::getScanData, this);
    // pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>(topic_id_, 1000);
    //sub_cmd_code_ = nh_.subscribe("control_command",100,&BEANode::cmdMsgCallback,this);
}


bool BEANode::connect()
{
    delete driver_;
    
    driver_ = new VISIOSCANDriver();
    driver_->setMountDirection(laser_direction_);
    driver_->configTopicID(topic_id_, frame_id_);
    std::cout << "Connecting to scanner at " << laser_ip_ << " ... " << std::endl;
    if(driver_->connect(laser_ip_, laser_port_))
    {
        std::cout << "@BEANode::connect OK" << std::endl;
    }
    else
    {
        std::cout << "FAILED!" << std::endl;
        std::cerr << "Connection to scanner at " << laser_ip_ << ":" << laser_port_ << " failed!" << std::endl;
        return false;
    }
    
    // Setting, reading user parameters
    BEA_PARAMETER_INFO params = driver_->GetParameters();
    packetType_ = params.lidarDataPacketType;
    float angleMin;
    float angleMax;
    float angleInc = params.angularResolution * (params.skipSpots + 1) * TO_RADIAN;
    if(params.scanDataDirection == 0) // Clockwise
    {
        angleMin = (params.stopAngle - 90.0) * TO_RADIAN;
        angleMax = (params.startAngle - 90.0) * TO_RADIAN;
        angleInc = -angleInc;
    }
    else // Counterclockwise
    {
        angleMin = (params.startAngle - 90.0) * TO_RADIAN;
        angleMax = (params.stopAngle - 90.0) * TO_RADIAN;
        angleInc = angleInc;
    }
    if(laser_direction_ != 0)
    {
        angleMin = -angleMin;
        angleMax = -angleMax;
        angleInc = -angleInc;
    }
    rosScanInfo_.ros_angle_min = angleMin;
    rosScanInfo_.ros_angle_max = angleMax;
    rosScanInfo_.ros_angle_increment = angleInc;
    rosScanInfo_.ros_scan_time = params.scan_time;
    rosScanInfo_.ros_range_min = 0;
    rosScanInfo_.ros_range_max = ROS_RANGE_MAX;
    std::cout << "Scanner Parameter: " << std::endl;
    std::cout << "ros_angle_min=" << rosScanInfo_.ros_angle_min << std::endl;
    std::cout << "ros_angle_max=" << rosScanInfo_.ros_angle_max << std::endl;
    std::cout << "ros_angle_increment=" << rosScanInfo_.ros_angle_increment << std::endl;
    std::cout << "ros_scan_time=" << rosScanInfo_.ros_scan_time << std::endl;

    // Start capturing scanner data
    //-----------------------------------------
    std::cout << "Starting capturing: ";
    int proto = params.protocolType;
    if(proto == 0)
    {
        if(driver_->startCapturingUDP())
        {
            std::cout << "OK" << std::endl;
        }
        else
        {
            std::cout << "FAILED!" << std::endl;
            return false;
        }
    }
    else
    {
        if(driver_->startCapturingTCP())
        {
            std::cout << "OK" << std::endl;
        }
        else
        {
            std::cout << "FAILED!" << std::endl;
            return false;
        }
    }

    return true;
}


void BEANode::getScanData(const ros::TimerEvent& e)
{
    if(!driver_->isCapturing())
    {
        std::cout << "ERROR: Scanner disconnected. Trying to reconnect..." << std::endl;
        while(!connect())
        {
            std::cout << "ERROR: Reconnect failed. Trying again in 2 seconds..." << std::endl;
            usleep(2 * 1000000);
        }
    }
    
    int size_ = driver_->getFullScansAvailable();
    if(size_ > 0) std::cout << "WARN: There are " << size_ + 1 << " scans available at the time!!!" << std::endl;
#if 0
    for( int d = 0; d < size_; d++)
    {
        // If there are multiple scans, drop old scans
        driver_->getFullScan();
    }
#endif
    auto scandata = driver_->getFullScan();

    if(scandata.distance_data.empty())
    {
        return;
    }
    int scanDataSize = scandata.distance_data.size();

    sensor_msgs::LaserScan scanmsg;
    scanmsg.header.frame_id = frame_id_;
    scanmsg.header.stamp = ros::Time::now();
    scanmsg.angle_min = rosScanInfo_.ros_angle_min;
    scanmsg.angle_max = rosScanInfo_.ros_angle_max;
    scanmsg.angle_increment = rosScanInfo_.ros_angle_increment;
    scanmsg.scan_time = rosScanInfo_.ros_scan_time;
    scanmsg.time_increment = rosScanInfo_.ros_scan_time / float(scanDataSize);
    scanmsg.range_min = 0.0;
    scanmsg.range_max = rosScanInfo_.ros_range_max;
    //scanmsg.ranges.resize(scanDataSize);
    //scanmsg.intensities.resize(scanDataSize);
    //std::cout<<"Scan data size="<<scanDataSize<<std::endl;
    if(packetType_ == 1)
    {
        scanmsg.ranges.resize(scanDataSize);
        scanmsg.intensities.resize(scanDataSize);
        for(int i = 0; i < scanDataSize; i++)
        {
            scanmsg.ranges[i] = float(scandata.distance_data[i]) / 1000.0f;
            scanmsg.intensities[i] = float(scandata.amplitude_data[i]);
        }
    }
    else
    {
        scanmsg.ranges.resize(scanDataSize);
        for(int i = 0; i < scanDataSize; i++)
        {
            scanmsg.ranges[i] = float(scandata.distance_data[i]) / 1000.0f;
        }
    }

    pub_scan_.publish(scanmsg);
}

void BEANode::checkConnection(const ros::TimerEvent& e)
{
    if(!driver_->isCapturing())
    {
        std::cout << "ERROR: Scanner disconnected. Trying to reconnect..." << std::endl;
        while(!connect())
        {
            std::cout << "ERROR: Reconnect failed. Trying again in 2 seconds..." << std::endl;
            usleep(2 * 1000000);
        }
    }
}

void BEANode::cmdMsgCallback(const std_msgs::StringConstPtr& msg)
{
    // TODO
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
    std::cout << "bea_node start" << std::endl;

    new bea_power::BEANode();

    signal(SIGINT, sigIntHandler);

    ros::spin();

    return 0;
}

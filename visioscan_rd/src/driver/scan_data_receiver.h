// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.

#ifndef SCAN_DATA_RECEIVER_H
#define SCAN_DATA_RECEIVER_H

#define BOOST_CB_DISABLE_DEBUG
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <array>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include "packet_structure.h"
#include "protocol_info.h"
#include "common_libs.h"
#include "tcp_command_interface.h"

#define MDI_PACKET_HEADER_SIZE   (31)
#define MDI_PACKET_HEADER_SYNC_0 (0xBE)
#define MDI_PACKET_HEADER_SYNC_1 (0xA0)
#define MDI_PACKET_HEADER_SYNC_2 (0x12)
#define MDI_PACKET_HEADER_SYNC_3 (0x34)

namespace bea_power
{
class ScanDataReceiver
{
public:
    ScanDataReceiver(const std::string topic, const std::string frame, const std::string hostname, int port, BEA_PARAMETER_INFO bpi);
    ScanDataReceiver(const std::string topic, const std::string frame, const std::string hostname, int port, BEA_PARAMETER_INFO bpi, bool udpType);
    ~ScanDataReceiver();
    bool isConnected() const { return is_connected_; }
    void disconnect();
    ScanData getScan();
    ScanData getFullScan();
    std::size_t getScansAvailable() const { return scan_data_.size(); }
    std::size_t getFullScansAvailable() const;
    bool sendMDI();
    bool stopMDI();

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_scan_;
    std::vector<unsigned short> distanceData_;
    std::vector<unsigned short> intensitiesData_;
    //@ TCP interface of the scanner
    BEA_PARAMETER_INFO parameterInfo_;
    ROS_SCAN_INFO rosScanInfo_;
    int scanPointsSum_ = 0;
    void handleSocketRead(const boost::system::error_code& error);
    void handleSocketRead(const boost::system::error_code& error, std::size_t bytes_transferred);
    bool handleNextPacket();
    int findPacketStart();
    bool retrievePacket(std::size_t start, PacketTypeC *p);
    bool checkConnection();
    void readBufferFront(char *dst, std::size_t numbytes);
    void PacketHeaderToLittleEndian(PacketTypeC *pHead);
    void writeBufferBack(char *src, std::size_t numbytes);
    int udp_port_;
    bool is_connected_;
    boost::thread io_service_thread_;
    boost::asio::io_service io_service_;
    boost::asio::streambuf inbuf_;
    std::istream instream_;
    boost::asio::ip::tcp::socket *tcp_socket_;
    boost::asio::ip::udp::socket *udp_socket_;
    boost::asio::ip::udp::endpoint udp_endpoint_;
    std::array<char, 65536> udp_buffer_;
    std::array<char, 65536> tcp_buffer_;
    boost::circular_buffer<char> ring_buffer_;
    std::mutex data_mutex_;
    std::condition_variable data_notifier_;
    std::deque<ScanData> scan_data_;
    double last_data_time_;
    std::string frame_id_;
    bool is_reverse_;
};

}
#endif // end SCAN_DATA_RECEIVER_H

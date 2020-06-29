/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *	@author Tony Zhang
 */

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <rslidar_msgs/rslidarScan.h>
//#include "filter4s4.h"

namespace rslidar_rawdata
{

static const float ANGLE_RESOLUTION = 0.01f; /**< degrees */

static const float DISTANCE_MAX = 200.0f;       /**< meters */
static const float DISTANCE_MIN = 0.5f;         /**< meters */
static const float DISTANCE_RESOLUTION = 0.01f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);

/** Special Defines for MEMS support **/
static const int MEMS_BLOCKS_PER_PACKET = 25;
static const int MEMS_SCANS_PER_FIRING = 6;
static const int POINT_COUNT_PER_VIEWFIELD = 15750;
/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
// union two_bytes {
//  uint16_t uint;
//  uint8_t bytes[2];
//};

static const int PACKET_SIZE = 1400;
static const int PACKET_STATUS_SIZE = 80;

/** \brief Raw Rsldar packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */

// MEMS channel return
typedef struct raw_mems_channel
{
  uint16_t intensity_1;
  uint16_t distance_1;
  uint16_t intensity_2;
  uint16_t distance_2;
} raw_mems_channel_t;

// MEMS block
typedef struct raw_mems_block
{
  uint16_t pitch;
  uint16_t yaw;
  raw_mems_channel_t channel[MEMS_SCANS_PER_FIRING];
} raw_mems_block_t;

// MEMS timestamp
typedef struct raw_mems_timestamp
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
}raw_mems_timestamp_t;

// MEMS packet
typedef struct raw_mems_packet
{
  uint8_t sync[4];
  uint8_t cmd[4];
  int8_t temp;
  uint8_t reserved;
  raw_mems_timestamp_t timestamp;
  raw_mems_block_t blocks[MEMS_BLOCKS_PER_PACKET];
  uint8_t status[PACKET_STATUS_SIZE];
} raw_mems_packet_t;


/** \brief RSLIDAR data conversion class */
class RawData
{
public:
  RawData();

  ~RawData()
  {
    this->tan_lookup_table_.clear();
  }

  /*load the cablibrated files: ChannelNum, limit, look-up table for pitch*/
  void loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh);

  /*unpack the MEMS UDP packet and opuput PCL PointXYZI type*/
  void unpack_MEMS(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);

  /*convert the deg */
  float pitchConvertDeg(int deg); 
  float yawConvertDeg(int deg);
  
  /*calibrated the disctance for mems */
  float pixelToDistance(int distance, int dsr); 

private:
  int channel_num_[MEMS_SCANS_PER_FIRING];
  int real_slow_[27780];
  float pitch_rate_;
  float yaw_rate_;
  float pitch_offset_[MEMS_SCANS_PER_FIRING];
  float yaw_offset_[MEMS_SCANS_PER_FIRING];
  float yaw_limit_start_[MEMS_SCANS_PER_FIRING];
  float yaw_limit_end_[MEMS_SCANS_PER_FIRING];
  float distance_max_thd_;
  float distance_min_thd_;

  uint32_t point_idx_;
  int last_pitch_index_;
  int skip_block_;
  int last_pkt_index_;

  /* tan lookup table */
  std::vector<float> tan_lookup_table_;

  Eigen::Matrix<float, 3, 5> input_ref_;
  Eigen::Matrix<float, 3, 3> rotate_gal_;
  ros::Publisher temp_output_;
};
}  // namespace rslidar_rawdata

#endif  // __RAWDATA_H

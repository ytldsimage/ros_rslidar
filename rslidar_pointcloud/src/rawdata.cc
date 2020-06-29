/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  RSLIDAR 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw RSLIDAR LIDAR packets into useful
 *  formats.
 *
 */
#include "rawdata.h"
#include "std_msgs/Float32.h"

#define RS_Grabber_toRadians(x) ((x)*M_PI / 180.0)
#define RS_SWAP_HIGHLOW(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
// #define RS_SWAP_HIGHLOW(x) (((x) / 256) + (((x) % 256) * 256))

namespace rslidar_rawdata
{
RawData::RawData()
{
  point_idx_ = 0;
  last_pitch_index_ = 0;
  skip_block_ = 0;
  last_pkt_index_ = -1;
}

void RawData::loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string channelPath, limitPath, slowPath;

  private_nh.param("channel_path", channelPath, std::string(""));
  private_nh.param("limit_path", limitPath, std::string(""));
  private_nh.param("slow_path", slowPath, std::string(""));

  //=============================================================
  FILE* f_channel = fopen(channelPath.c_str(), "r");
  if (!f_channel)
  {
    ROS_ERROR_STREAM("channelPath: " << channelPath << " does not exist");
  }
  else
  {
    ROS_INFO_STREAM("Loading channelNum corrections file!");

    int loopm = 0;
    float tmpBuf[32];
    const float defaultTmpBuf[32] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // distance correction offset
                        1.0f, 1.0f, // pitch yaw rate
                        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // pitch offset
                        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // yaw offset
                        -6.75f, 6.75f, -6.75f, 6.75f, -6.75f, 6.75f, -6.75f, 6.75f, -6.75f, 6.75f, -6.75f, 6.75f};// yaw start angle
    while (!feof(f_channel))
    {
      if (1 != fscanf(f_channel, "%f%*[^\n]%*c\n", &tmpBuf[loopm]))
      {
        break;
      }
      loopm++;
      if (loopm >= 32)
      {
        break;
      }
    }
    if (loopm != 32)
    {
      ROS_ERROR_STREAM("the format of " << channelPath << " is not correct, use default data");
      for (int i = 0; i < 32; ++i)
      {
        tmpBuf[i] = defaultTmpBuf[i];
      }
    }

    for (int i = 0; i < MEMS_SCANS_PER_FIRING; i++)
    {
      channel_num_[i] = (int)(tmpBuf[i]);
      pitch_offset_[i] = tmpBuf[8 + i];
      yaw_offset_[i] = tmpBuf[14 + i];
      yaw_limit_start_[i] = tmpBuf[20 + 2 * i];
      yaw_limit_end_[i] = tmpBuf[21 + 2 * i];
    }
    pitch_rate_ = tmpBuf[6];
    yaw_rate_ = tmpBuf[7];

    fclose(f_channel);
  }

  //=============================================================
  FILE* f_limit = fopen(limitPath.c_str(), "r");
  if (!f_limit)
  {
    ROS_ERROR_STREAM("limitPath: " << limitPath <<" does not exist");
    distance_max_thd_ = 200.0;
    distance_min_thd_ = 0.0;
  }
  else
  {
    ROS_INFO_STREAM("Loading limit file!");
    float tmp_min;
    float tmp_max;

    if (1 == fscanf(f_limit, "%f\n", &tmp_min) &&
        1 == fscanf(f_limit, "%f\n", &tmp_max))
    {
      distance_max_thd_ = tmp_max / 100.0;
      distance_min_thd_ = tmp_min / 100.0;
    }
    else
    {
      ROS_ERROR_STREAM("the format of " << limitPath << " is not correct, use default data");
      distance_max_thd_ = 200.0;
      distance_min_thd_ = 0.0;
    }

    fclose(f_limit);
  }

  //=============================================================
  FILE* f_slow = fopen(slowPath.c_str(), "r");
  if (!f_slow)
  {
    ROS_ERROR_STREAM("slowPath: " << slowPath << " does not exist");
  }
  else
  {
    ROS_INFO_STREAM("Loading slow corrections file!");
    int i, j;
    int t[20];
    for (i = 0; i < 1389; i++)
    {
      if (20 == fscanf(f_slow, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n", &t[0], &t[1], &t[2], &t[3],
                       &t[4], &t[5], &t[6], &t[7], &t[8], &t[9], &t[10], &t[11], &t[12], &t[13], &t[14], &t[15], &t[16], &t[17],
                       &t[18], &t[19]))
      {
        for (j = 0; j < 20; j++)
        {
          real_slow_[i * 20 + j] = t[j];
        }
      }
      else
      {
        ROS_ERROR_STREAM("the format of " << slowPath << " is not correct, please check the file");
      }
    }

    fclose(f_slow);
  }

  // lookup table init, -10 ~ 10 deg, 0.01 resolution
  this->tan_lookup_table_.resize(2000);

  for (int i = -1000; i < 1000; i++)
  {
    double rad = RS_Grabber_toRadians(i / 100.0f);
    this->tan_lookup_table_[i + 1000] = std::tan(rad);
  }

  input_ref_ << 0.748956, 0.414693, -0.414693, -0.748956, 0,
           0.33131,  0.454981,  0.454981,  0.33131,  0.5,
          -0.573846,-0.78805,  -0.78805,  -0.573846,-0.866025;

  rotate_gal_ << 1, 0, 0, 0, 0.965926, -0.258819, 0, 0.258819, 0.965926;
  // std::cout << "input_ref_" << input_ref_ << std::endl;
  // std::cout << "rotate_gal_" << rotate_gal_ << std::endl;
  // advertise output temp info
  std::string temp_info_topic;
  private_nh.param("output_temp_topic", temp_info_topic, std::string("temperature"));
  temp_output_ = node.advertise<std_msgs::Float32>(temp_info_topic, 10);
}

//------------------------------------------------------------

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */

float RawData::pitchConvertDeg(int deg)  // convert deg into  -625 ~ +625
{
  float result_f;
  float deg_f = deg;

  result_f = (deg_f * (1250.0f / 65534.0f) - 625.0f);
  // printf("%d,%f;%f\n",deg,deg_f,result_f);
  return result_f;
}

float RawData::yawConvertDeg(int deg)  // convert deg into  -675 ~ 675
{
  float result_f;
  float deg_f = deg;

  result_f = (deg_f * (1350.0f / 65534.0f) - 675.0f);
  // printf("%d,%f;%f\n",deg,deg_f,result_f);
  return result_f;
}

float RawData::pixelToDistance(int distance, int dsr)
{
  float result;
  int cor = channel_num_[dsr];

  if (distance <= cor)
  {
    result = 0.0;
  }
  else
  {
    result = distance - cor;
  }
  return result;
}

void RawData::unpack_MEMS(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  float pitch, yaw;  // degree
  int pitch_temp, index_temp;
  pcl::PointXYZI point;

  const raw_mems_packet_t* raw = (const raw_mems_packet_t*)&pkt.data[0];
  
  // check if it is lost udp packet, and need to set nan data and increase index of point
  // normally, index of packet will increase one if it doesn't lose packet
  int pkt_index = 256 * raw->cmd[0] + raw->cmd[1];
  // lose packets limit: 1 ~ 630
  if (pkt_index - last_pkt_index_ > 1 && pkt_index - last_pkt_index_ <  POINT_COUNT_PER_VIEWFIELD / MEMS_BLOCKS_PER_PACKET 
      && last_pkt_index_ != -1)
  {
    int lose_pkt_count = pkt_index - last_pkt_index_ - 1;
    while (lose_pkt_count--)
    {
      for (int block = 0; block < MEMS_BLOCKS_PER_PACKET; block ++)
      {
        for (int dsr = 1; dsr < MEMS_SCANS_PER_FIRING; dsr++)
        {
          point.x = NAN;
          point.y = NAN;
          point.z = NAN;
          point.intensity = 0;
          pointcloud->at(point_idx_, dsr - 1) = point;
        }
        point_idx_ ++;
      }
    } 
  }
  last_pkt_index_ = pkt_index;
  
  // unpack block
  for (int block = skip_block_; block < MEMS_BLOCKS_PER_PACKET; block++)  // 1 packet:25 data blocks
  {
    raw_mems_block_t* pBlockPkt = (raw_mems_block_t*)(raw->blocks + block);
    index_temp = RS_SWAP_HIGHLOW(pBlockPkt->pitch);
    // pitch index will increase until new frame
    if (last_pitch_index_ > index_temp)
    {
      // printf("point_idx_: %d\n", point_idx_);
      point_idx_ = 0;
      skip_block_ = block;
      last_pitch_index_ = index_temp;

      //temperature publish
      std_msgs::Float32 tempMsg; // temperature value, Celsius degree
      tempMsg.data = raw->temp;
      temp_output_.publish(tempMsg);
      break;
    }
    last_pitch_index_ = index_temp;

    // pitch angle mapping
    pitch_temp = (int)real_slow_[index_temp];
    if (pitch_temp < -32767)
    {
      pitch_temp = -32767;
    }
    else if (pitch_temp > 32767)
    {
      pitch_temp = 32767;
    }

    pitch = pitchConvertDeg(pitch_temp + 32767) * ANGLE_RESOLUTION;
    yaw = yawConvertDeg(RS_SWAP_HIGHLOW(pBlockPkt->yaw)) * ANGLE_RESOLUTION;

    // unpack points
    Eigen::Matrix<float, 3, 1> n_gal;
    Eigen::Matrix<float, 3, 1> i_out;
    for (int dsr = 1; dsr < MEMS_SCANS_PER_FIRING; dsr++)  // 5 channels, channel index 1 ~ 5
    {
      float temp_a, temp_b, temp_c, tanax, tanay;
      int ax, ay;
      
      ax = (int)(100 * pitch_rate_ * (pitch + pitch_offset_[dsr]));  // ax: -1000 ~ 1000, 0.01deg
      ay = (int)(100 * yaw_rate_ * (yaw + yaw_offset_[dsr]));        // ay: -1000 ~ 1000, 0.01deg
      tanax = this->tan_lookup_table_[ax + 1000];
      tanay = this->tan_lookup_table_[ay + 1000];
      tanax = -tanax;
      tanay = -tanay;

      // calc i_out
      n_gal << tanay, -tanax, 1;
      n_gal.normalize();
      n_gal = rotate_gal_ * n_gal;
      i_out = input_ref_.col(dsr - 1) - 2 * n_gal * n_gal.transpose() * input_ref_.col(dsr - 1);

      float distance = pixelToDistance(RS_SWAP_HIGHLOW(pBlockPkt->channel[dsr].distance_1), dsr);
      distance = distance * DISTANCE_RESOLUTION;

      if (distance > distance_max_thd_ || distance < distance_min_thd_ || yaw < yaw_limit_start_[dsr] ||
          yaw > yaw_limit_end_[dsr])  // invalid data
      {
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = 0;
      }
      else
      {
        point.x = i_out(2) * distance;
        point.y = i_out(0) * distance;
        point.z = i_out(1) * distance;
        point.intensity = RS_SWAP_HIGHLOW(pBlockPkt->channel[dsr].intensity_1);
      }

      if (point_idx_ < POINT_COUNT_PER_VIEWFIELD)
        pointcloud->at(point_idx_, dsr - 1) = point;
    }
    point_idx_ ++;
    skip_block_ = 0;
  }
}
}  // namespace rslidar_rawdata

/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017, Velodyne LiDAR INC., Algorithms and Signal Processing Group
 *  Copyright (C) 2020, Tier IV, Inc., David Robert Wong
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "driver.h"

namespace velodyne_driver
{
  static double prev_frac_packet = 0;
  inline   std::string toBinary(int n)
  {
        std::string r;
        while(n!=0) {r=(n%2==0 ?"0":"1")+r; n/=2;}
        while (r.length() != 8){
          r = '0' + r;
        }
        return r;
  }

  inline   double convertBinaryToDecimal(std::string binaryString)
  {
      double value = 0;
      int indexCounter = 0;
      for(int i=binaryString.length()-1;i>=0;i--){

          if(binaryString[i]=='1'){
              value += pow(2, indexCounter);
          }
          indexCounter++;
      }
      return value;
  }

  inline   double computeTimeStamp(velodyne_msgs::VelodyneScanPtr scan, int index){

      std::string digit4 = toBinary(scan->packets[index].data[1203]);
      std::string digit3 = toBinary(scan->packets[index].data[1202]);
      std::string digit2 = toBinary(scan->packets[index].data[1201]);
      std::string digit1 = toBinary(scan->packets[index].data[1200]);
      std::string digit = digit4 + digit3 + digit2 + digit1; // string concatenation
      double value = convertBinaryToDecimal(digit);
      // compute the seconds from the beginning of that hour to when the data being captured
      double time_stamp = (double)value / 1000000;
      return time_stamp;
  }

/** Utility function for Velodyne Driver
 *  gets the number of laser beams fired concurrently
 *  for different sensor models
*/

inline int get_concurrent_beams(uint8_t sensor_model)
{
/*
Strongest 0x37 (55)   HDL-32E 0x21 (33)
Last Return 0x38 (56) VLP-16 0x22 (34)
Dual Return 0x39 (57) Puck LITE 0x22 (34)
         -- --        Puck Hi-Res 0x24 (36)
         -- --        VLP-32C 0x28 (40)
         -- --        Velarray 0x31 (49)
         -- --        VLS-128 0xA1 (161)
*/

  switch(sensor_model)
  {
    case 33:
        return(2); // hdl32e
    case 34:
        return(1); // vlp16 puck lite
    case 36:
        return(1); // puck hires  (same as vlp16 ?? need to check)
    case 40:
        return(2); // vlp32c
    case 49:
        return(2); // velarray
    case 161:
        return(8); // vls128
    case 99:
        return(8); // vls128
    default:
        ROS_WARN_STREAM("[Velodyne Ros driver]Default assumption of device id .. Defaulting to HDL64E with 2 simultaneous firings");
        return(2); // hdl-64e

  }
}

/** Utility function for Velodyne Driver
 *  gets the number of packet multiplier for dual return mode vs
 *  single return mode
*/

inline int get_rmode_multiplier(uint8_t sensor_model, uint8_t packet_rmode)
{
 /*
    HDL64E 2
    VLP32C 2
    HDL32E 2
    VLS128 3
    VLSP16 2
*/
  if(packet_rmode  == 57)
  {
    switch(sensor_model)
    {
      case 33:
          return(2); // hdl32e
      case 34:
          return(2); // vlp16 puck lite
      case 36:
          return(2); // puck hires
      case 40:
          return(2); // vlp32c
      case 49:
          return(2); // velarray
      case 161:
          return(3); // vls128
      case 99:
          return(3); // vls128
      default:
          ROS_WARN_STREAM("[Velodyne Ros driver]Default assumption of device id .. Defaulting to HDL64E with 2x number of packekts for Dual return");
          return(2); // hdl-64e
    }
   }
   else
   {
     return(1);
   }
}

/** Constructor for the Velodyne driver
 *
 *  provides a binding to ROS node for processing and
 *  configuration
 *  @returns handle to driver object
 */

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));
  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
  if ((config_.model == "64E_S2") ||
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
      packet_rate = 3472.17;            // 1333312 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "64E")
    {
      packet_rate = 2600.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "64E_S3") // generates 2222220 points per second (half for strongest and half for lastest)
    {                                 // 1 packet holds 384 points
      packet_rate = 5787.03;          // 2222220 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "32E")
    {
      packet_rate = 1808.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "32C")
    {
      packet_rate = 1507.0;
      model_full_name = std::string("VLP-") + config_.model;
    }
  else if (config_.model == "VLP16")
    {
      packet_rate = 754;              // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
      model_full_name = "VLP-16";
    }
  else if (config_.model == "VLS128")
    {
      packet_rate = 6030;             // Datasheet gives 6253.9 packets/second, but experimentally closer to this
      model_full_name = "VLS-128";
    }
  else
    {
      ROS_ERROR_STREAM("Unknown Velodyne LIDAR model: " << config_.model);
      packet_rate = 2600.0;
    }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  private_nh.getParam("rpm", config_.rpm);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  private_nh.param("sensor_phase", config_.sensor_phase, 0.0);
  private_nh.getParam("sensor_phase", config_.sensor_phase);
  ROS_INFO_STREAM("Scan will align to sensor phase set to " << config_.sensor_phase  << " degrees");

  private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_driver::
    VelodyneNodeConfig> > (private_nh);
  dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::
    CallbackType f;
  f = boost::bind (&VelodyneDriver::callback, this, _1, _2);
  srv_->setCallback (f); // Set callback function und call initially

  // Initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = frequency;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("Expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new velodyne_driver::InputPCAP(private_nh, udp_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
    }

  // raw packet output topic
  output_ =
    node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
}

/** poll the device
 *
 * poll is used by nodelet to bind to the ROS thread.
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);

  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  bool use_next_packet = true;
  uint processed_packets = 0;
  while (use_next_packet)
  {
    while (true)
    {
        // keep reading until full packet received
        velodyne_msgs::VelodynePacket new_packet;
        scan->packets.push_back(new_packet);
        int rc = input_->getPacket(&scan->packets.back(), config_.time_offset);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
    }
    processed_packets++;

    curr_packet_azm  = scan->packets.back().data[2]; // lower word of azimuth block 0
    curr_packet_azm |= scan->packets.back().data[3] << 8; // higher word of azimuth block 0
    curr_packet_rmode = scan->packets.back().data[1204];
    curr_packet_sensor_model = scan->packets.back().data[1205];

    // For correct pointcloud assembly, always stop the scan after passing the
    // zero phase point. The pointcloud assembler will remedy this after unpacking
    // the packets, by buffering the overshot azimuths for the next cloud.
    if (processed_packets > 1)
    {
      uint16_t phase = (uint16_t)round(config_.sensor_phase*100);
      uint16_t azimuth_gap = (36000 + curr_packet_azm - prev_packet_azm) % 36000;
      uint16_t phased_azimuth_next = ((36000 + curr_packet_azm - phase) % 36000) + azimuth_gap;

      if (phased_azimuth_next > 36000)
      {
        use_next_packet = false;
      }
    }
    prev_packet_azm = curr_packet_azm;
  }

  // average the time stamp from first package and last package
  double firstTimeStamp = computeTimeStamp(scan, 0);
  double lastTimeStamp = computeTimeStamp(scan, processed_packets - 1);
  double meanTimeStamp = (firstTimeStamp + lastTimeStamp)/2;

  time_t seconds;
  seconds = time (NULL);
  int gpsSeconds = ((int)(seconds/3600)) * 3600 + floor(meanTimeStamp);
  int nanSecs =  (meanTimeStamp - floor(meanTimeStamp)) * pow(10,9);
  scan->header.stamp = ros::Time(gpsSeconds, nanSecs);
  // std::cerr<< scan->header.stamp << std::endl;
  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);
  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  if (dump_file != "")                  // have PCAP file?
  {
    // determine packet rate over the scan
    if (lastTimeStamp < firstTimeStamp)
    {
      lastTimeStamp = lastTimeStamp + 3600;
    }
    double scan_packet_rate = (processed_packets - 1)/(lastTimeStamp - firstTimeStamp);
    input_->setPacketRate(scan_packet_rate);
  }
  return true;
}

void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
  config_.sensor_phase = config.sensor_phase;
}

} // namespace velodyne_driver

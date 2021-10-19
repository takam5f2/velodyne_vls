/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _VELODYNE_POINTCLOUD_INTERPOLATE_H_
#define _VELODYNE_POINTCLOUD_INTERPOLATE_H_ 1

#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <velodyne_pointcloud/pointcloudXYZIRADT.h>
#include "specialized_intra_process_comm/specialized_intra_process_comm.hpp"
#include "pcl/pcl_base.h"

namespace velodyne_pointcloud
{
class Interpolate : public rclcpp::Node 
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudSharedPtr = boost::shared_ptr<PointCloud>;
  using PointCloudSharedPtrUniquePtr = std::unique_ptr<PointCloudSharedPtr>;
  using PointCloudMessageT = std::unique_ptr<PointCloudSharedPtr>;

  Interpolate(const rclcpp::NodeOptions & options);
  ~Interpolate() {}

private:
  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

  void processPoints(
    PointCloudMessageT points_xyziradt);
  void processTwist(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);

  feature::Subscription<PointCloudSharedPtr>::SharedPtr velodyne_points_ex_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  feature::Publisher<PointCloudSharedPtr>::SharedPtr velodyne_points_interpolate_pub_;
  feature::Publisher<PointCloudSharedPtr>::SharedPtr velodyne_points_interpolate_ex_pub_;

  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;

  std::string base_link_frame_;
};

}  // namespace velodyne_pointcloud

#endif  // _VELODYNE_POINTCLOUD_INTERPOLATE_H_

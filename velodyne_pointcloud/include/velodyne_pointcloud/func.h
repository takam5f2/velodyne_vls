/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <deque>
#include <vector>

#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <velodyne_pointcloud/point_types.h>

#include <half.hpp>
#include <libfixmath/fixmath.h>

namespace velodyne_pointcloud
{
using half_float::half;
using namespace half_float::literal;

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractValidPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const double min_range, const double max_range);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractValidPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const double min_range, const double max_range,
  velodyne_pointcloud::VelodyneImplementType impl_type);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidNearPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::vector<float> & invalid_intensity_array, const size_t num_lasers);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidNearPointsFiltered(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::vector<float> & invalid_intensity_array, const size_t num_lasers,
  const size_t points_size_threshold);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidNearPointsFiltered(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::vector<float> & invalid_intensity_array, const size_t num_lasers,
  const size_t points_size_threshold, velodyne_pointcloud::VelodyneImplementType impl_type);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor,
  const velodyne_pointcloud::VelodyneImplementType impl_type);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_single_precision(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_half_precision(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_fixed_16_16_precision(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortRingNumber(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const size_t num_lasers);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortZeroIndex(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const size_t num_lasers);

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortZeroIndex(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const size_t num_lasers, velodyne_pointcloud::VelodyneImplementType impl_type);

pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr convert(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud);

struct VelodyneVector3f
{
  float x, y, z;

  VelodyneVector3f(float arg_x=0.0f, float arg_y=0.0f, float arg_z=0.0f)
  {
    x = arg_x;
    y = arg_y;
    z = arg_z;
  }
};

struct VelodyneVector3h
{
  half x, y, z;

  VelodyneVector3h(half arg_x=0.0_h, half arg_y=0.0_h, half arg_z=0.0_h)
  {
    x = arg_x;
    y = arg_y;
    z = arg_z;
  }
};

struct VelodyneVector3fx
{
  fix16_t x, y, z;

  VelodyneVector3fx(
    fix16_t arg_x=fix16_from_int(0), fix16_t arg_y=fix16_from_int(0),
    fix16_t arg_z=fix16_from_int(0))
  {
    x = arg_x;
    y = arg_y;
    z = arg_z;
  }
};

VelodyneVector3f multiplyTFTransformAndVector(
  const tf2::Transform & transform, const VelodyneVector3f & vector);

VelodyneVector3h multiplyTFTransformAndVector(
  const tf2::Transform & transform, const VelodyneVector3h & vector);

VelodyneVector3fx multiplyTFTransformAndVector(
  const tf2::Transform & transform, const VelodyneVector3fx & vector);

void setTFQuaternionRPY(
  tf2::Quaternion & quaternion, float roll, float pitch, float yaw);

void setTFQuaternionRPY(
  tf2::Quaternion & quaternion, half roll, half pitch, half yaw);

void setTFQuaternionRPY(
  tf2::Quaternion & quaternion, fix16_t roll, fix16_t pitch, fix16_t yaw);

void setTFTransformRotation(
  tf2::Transform & transform, const tf2::Quaternion & quaternion,
  velodyne_pointcloud::VelodyneImplementType impl_type);

}  // namespace velodyne_pointcloud

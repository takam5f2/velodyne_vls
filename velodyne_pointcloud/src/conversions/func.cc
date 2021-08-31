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

#include <velodyne_pointcloud/func.h>

#include <algorithm>
#include <iterator>

#include <opencv2/opencv.hpp>

namespace velodyne_pointcloud
{
pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractValidPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const double min_range, const double max_range)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  for (const auto & p : input_pointcloud->points) {
    if (p.distance >= min_range && p.distance <= max_range) {
      output_pointcloud->points.push_back(p);
    }
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractValidPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const double min_range, const double max_range,
  velodyne_pointcloud::VelodyneImplementType impl_type)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  for (const auto & p : input_pointcloud->points) {
    switch (impl_type) {
      case velodyne_pointcloud::VelodyneImplementType::HALF_PRECISION:
      {
        if (half_float::half_cast<half>(p.distance) >= half_float::half_cast<half>(min_range) &&
            half_float::half_cast<half>(p.distance) <= half_float::half_cast<half>(max_range)) {
          output_pointcloud->points.push_back(p);
        }
      }
      break;
      case velodyne_pointcloud::VelodyneImplementType::FIXED_POINT_16_16:
      {
        if (fix16_from_float(p.distance) >= fix16_from_dbl(min_range) &&
            fix16_from_float(p.distance) <= fix16_from_dbl(max_range)) {
          output_pointcloud->points.push_back(p);
        }
      }
      break;
      case velodyne_pointcloud::VelodyneImplementType::SINGLE_PRECISION:
      default:
      {
        if (p.distance >= min_range && p.distance <= max_range) {
          output_pointcloud->points.push_back(p);
        }
      }
      break;
    }
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  for (const auto & p : input_pointcloud->points) {
    if (p.distance == 0 && p.intensity <= 100) {
      output_pointcloud->points.push_back(p);
    }
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidNearPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::vector<float> & invalid_intensity_array, const size_t num_lasers)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  for (const auto & p : input_pointcloud->points) {
    if (p.distance == 0 && p.intensity <= 100 && p.intensity != invalid_intensity_array[p.ring]) {
      output_pointcloud->points.push_back(p);
    }
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidNearPointsFiltered(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::vector<float> & invalid_intensity_array, const size_t num_lasers,
  const size_t points_size_threshold)
{
  std::vector<uint16_t> ring_id_array;
  //NOTE support only VLP16 and VLP32C
  if (num_lasers == 16) {
    ring_id_array = {2, 4, 6, 8, 10, 12, 14, 0, 3, 5, 7, 9, 11, 13, 15, 1};
  } else if (num_lasers == 32) {
    ring_id_array = {30, 1,  2, 5, 6, 9, 10, 14, 13, 17, 18, 22, 21, 25, 26, 0,
                     29, 31, 4, 8, 3, 7, 12, 16, 11, 15, 20, 19, 24, 23, 27, 28};
  }
  else {
    // ROS_WARN_STREAM_THROTTLE(10, "support only VLP16 and VLP32C");
    pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = 0;
    return output_pointcloud;
  }

  velodyne_pointcloud::PointXYZIRADT tmp_p;
  cv::Mat image =
    cv::Mat::zeros(cv::Size(input_pointcloud->size() / num_lasers, num_lasers), CV_8UC1);

  for (size_t x = 0; x < image.cols; ++x) {
    for (size_t y = 0; y < image.rows; ++y) {
      tmp_p = input_pointcloud->points.at(ring_id_array.at(y) + x * image.rows);
      if (
        tmp_p.distance == 0 && tmp_p.intensity <= 100 &&
        tmp_p.intensity != invalid_intensity_array[tmp_p.ring]) {
        image.at<unsigned char>(y, x) = 255;
      } else {
        image.at<unsigned char>(y, x) = 0;
      }
    }
  }

  cv::Mat element(3, 3, CV_8UC1, cv::Scalar::all(255));
  cv::morphologyEx(image, image, cv::MORPH_OPEN, element, cv::Point(-1, -1), 3);
  cv::morphologyEx(image, image, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 3);

  // cv::imshow("aaa", image);
  // cv::waitKey(1);

  cv::Mat label_image(image.size(), CV_32S);
  cv::Mat stats;
  cv::Mat centroids;
  int label_n = cv::connectedComponentsWithStats(image, label_image, stats, centroids, 8);

  std::vector<int> stat_area;
  for (size_t label = 0; label < label_n; ++label) {
    int * param = stats.ptr<int>(label);
    stat_area.push_back(param[cv::ConnectedComponentsTypes::CC_STAT_AREA]);
    // std::cerr << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << " ";
  }
  // std::cerr << std::endl;

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  int label = 0;
  for (size_t x = 0; x < image.cols; ++x) {
    for (size_t y = 0; y < image.rows; ++y) {
      label = label_image.at<int>(y, x);
      tmp_p = input_pointcloud->points.at(ring_id_array.at(y) + x * image.rows);
      if (
        label != 0 && stat_area.at(label) >= points_size_threshold && tmp_p.distance == 0 &&
        tmp_p.intensity <= 100 && tmp_p.intensity != invalid_intensity_array[tmp_p.ring]) {
        output_pointcloud->points.push_back(tmp_p);
      }
    }
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractInvalidNearPointsFiltered(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::vector<float> & invalid_intensity_array, const size_t num_lasers,
  const size_t points_size_threshold, velodyne_pointcloud::VelodyneImplementType impl_type)
{
  std::vector<uint16_t> ring_id_array;
  //NOTE support only VLP16 and VLP32C
  if (num_lasers == 16) {
    ring_id_array = {2, 4, 6, 8, 10, 12, 14, 0, 3, 5, 7, 9, 11, 13, 15, 1};
  } else if (num_lasers == 32) {
    ring_id_array = {30, 1,  2, 5, 6, 9, 10, 14, 13, 17, 18, 22, 21, 25, 26, 0,
                     29, 31, 4, 8, 3, 7, 12, 16, 11, 15, 20, 19, 24, 23, 27, 28};
  }
  else {
    // ROS_WARN_STREAM_THROTTLE(10, "support only VLP16 and VLP32C");
    pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = 0;
    return output_pointcloud;
  }

  velodyne_pointcloud::PointXYZIRADT tmp_p;
  cv::Mat image =
    cv::Mat::zeros(cv::Size(input_pointcloud->size() / num_lasers, num_lasers), CV_8UC1);

  for (size_t x = 0; x < image.cols; ++x) {
    for (size_t y = 0; y < image.rows; ++y) {
      tmp_p = input_pointcloud->points.at(ring_id_array.at(y) + x * image.rows);
      switch (impl_type) {
        case velodyne_pointcloud::VelodyneImplementType::HALF_PRECISION:
        {
          if (
            half_float::half_cast<half>(tmp_p.distance) == 0.0_h &&
            half_float::half_cast<half>(tmp_p.intensity) <= 100.0_h &&
            half_float::half_cast<half>(tmp_p.intensity) !=
              half_float::half_cast<half>(invalid_intensity_array[tmp_p.ring])) {
            image.at<unsigned char>(y, x) = 255;
          } else {
            image.at<unsigned char>(y, x) = 0;
          }
        }
        break;
        case velodyne_pointcloud::VelodyneImplementType::FIXED_POINT_16_16:
        {
          if (
            fix16_from_float(tmp_p.distance) == fix16_from_int(0) &&
            fix16_from_float(tmp_p.intensity) <= fix16_from_int(100) &&
            fix16_from_float(tmp_p.intensity) !=
              fix16_from_float(invalid_intensity_array[tmp_p.ring])) {
            image.at<unsigned char>(y, x) = 255;
          } else {
            image.at<unsigned char>(y, x) = 0;
          }
        }
        break;
        case velodyne_pointcloud::VelodyneImplementType::SINGLE_PRECISION:
        default:
        {
          if (
            tmp_p.distance == 0 && tmp_p.intensity <= 100 &&
            tmp_p.intensity != invalid_intensity_array[tmp_p.ring]) {
            image.at<unsigned char>(y, x) = 255;
          } else {
            image.at<unsigned char>(y, x) = 0;
          }
        }
        break;
      }
    }
  }

  cv::Mat element(3, 3, CV_8UC1, cv::Scalar::all(255));
  cv::morphologyEx(image, image, cv::MORPH_OPEN, element, cv::Point(-1, -1), 3);
  cv::morphologyEx(image, image, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 3);

  // cv::imshow("aaa", image);
  // cv::waitKey(1);

  cv::Mat label_image(image.size(), CV_32S);
  cv::Mat stats;
  cv::Mat centroids;
  int label_n = cv::connectedComponentsWithStats(image, label_image, stats, centroids, 8);

  std::vector<int> stat_area;
  for (size_t label = 0; label < label_n; ++label) {
    int * param = stats.ptr<int>(label);
    stat_area.push_back(param[cv::ConnectedComponentsTypes::CC_STAT_AREA]);
    // std::cerr << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << " ";
  }
  // std::cerr << std::endl;

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  int label = 0;
  for (size_t x = 0; x < image.cols; ++x) {
    for (size_t y = 0; y < image.rows; ++y) {
      label = label_image.at<int>(y, x);
      tmp_p = input_pointcloud->points.at(ring_id_array.at(y) + x * image.rows);
      switch (impl_type) {
        case velodyne_pointcloud::VelodyneImplementType::HALF_PRECISION:
        {
          if (
            label != 0 && stat_area.at(label) >= points_size_threshold &&
            half_float::half_cast<half>(tmp_p.distance) == 0.0_h &&
            half_float::half_cast<half>(tmp_p.intensity) <= 100.0_h &&
            half_float::half_cast<half>(tmp_p.intensity) !=
              half_float::half_cast<half>(invalid_intensity_array[tmp_p.ring])) {
            output_pointcloud->points.push_back(tmp_p);
          }
        }
        break;
        case velodyne_pointcloud::VelodyneImplementType::FIXED_POINT_16_16:
        {
          if (
            label != 0 && stat_area.at(label) >= points_size_threshold &&
            fix16_from_float(tmp_p.distance) == fix16_from_int(0) &&
            fix16_from_float(tmp_p.intensity) <= fix16_from_int(100) &&
            fix16_from_float(tmp_p.intensity) !=
            fix16_from_float(invalid_intensity_array[tmp_p.ring])) {
            output_pointcloud->points.push_back(tmp_p);
          }
        }
        break;
        case velodyne_pointcloud::VelodyneImplementType::SINGLE_PRECISION:
        default:
        {
          if (
            label != 0 && stat_area.at(label) >= points_size_threshold && tmp_p.distance == 0 &&
            tmp_p.intensity <= 100 && tmp_p.intensity != invalid_intensity_array[tmp_p.ring]) {
            output_pointcloud->points.push_back(tmp_p);
          }
        }
        break;
      }
    }
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());

  if (input_pointcloud->points.empty() || twist_queue.empty()) {
    ROS_WARN_STREAM_THROTTLE(10, "input_pointcloud->points or twist_queue is empty.");
    *output_pointcloud = *input_pointcloud;
    return output_pointcloud;
  }

  float theta = 0;
  float x = 0, y = 0;

  auto twist_it = std::lower_bound(
    std::begin(twist_queue), std::end(twist_queue),
    ros::Time(input_pointcloud->points.front().time_stamp),
    [](const geometry_msgs::TwistStamped & x, ros::Time t) { return x.header.stamp < t; });
  twist_it = twist_it == std::end(twist_queue) ? std::end(twist_queue) - 1 : twist_it;

  const tf2::Transform tf2_base_link_to_sensor_inv = tf2_base_link_to_sensor.inverse();
  for (const auto & p : input_pointcloud->points) {
    for (; (twist_it != std::end(twist_queue) - 1 && p.time_stamp > twist_it->header.stamp.toSec());
         ++twist_it) {
      // std::cout << std::fixed << p.time_stamp << " " << twist_it->header.stamp.toSec() << std::endl;
    }

    float v = twist_it->twist.linear.x;
    float w = twist_it->twist.angular.z;

    if (std::fabs(p.time_stamp - twist_it->header.stamp.toSec()) > 0.1) {
      ROS_WARN_STREAM_THROTTLE(10, "Twist time_stamp is too late. Cloud not interpolate.");
      v = 0;
      w = 0;
    }

    static double prev_time_stamp = p.time_stamp;
    const float time_offset = static_cast<float>(p.time_stamp - prev_time_stamp);

    tf2::Vector3 sensorTF_point(p.x, p.y, p.z);

    tf2::Vector3 base_linkTF_point;
    base_linkTF_point = tf2_base_link_to_sensor_inv * sensorTF_point;

    theta += w * time_offset;
    tf2::Quaternion baselink_quat;
    baselink_quat.setRPY(0.0, 0.0, theta);
    const float dis = v * time_offset;
    x += dis * std::cos(theta);
    y += dis * std::sin(theta);

    tf2::Transform baselinkTF_odom;
    baselinkTF_odom.setOrigin(tf2::Vector3(x, y, 0));
    baselinkTF_odom.setRotation(baselink_quat);

    tf2::Vector3 base_linkTF_trans_point;
    base_linkTF_trans_point = baselinkTF_odom * base_linkTF_point;

    tf2::Vector3 sensorTF_trans_point;
    sensorTF_trans_point = tf2_base_link_to_sensor * base_linkTF_trans_point;

    velodyne_pointcloud::PointXYZIRADT tmp_p;
    tmp_p.x = sensorTF_trans_point.getX();
    tmp_p.y = sensorTF_trans_point.getY();
    tmp_p.z = sensorTF_trans_point.getZ();
    tmp_p.intensity = p.intensity;
    tmp_p.ring = p.ring;
    tmp_p.azimuth = p.azimuth;
    tmp_p.distance = p.distance;
    tmp_p.time_stamp = p.time_stamp;
    output_pointcloud->points.push_back(tmp_p);

    prev_time_stamp = p.time_stamp;
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor,
  const velodyne_pointcloud::VelodyneImplementType impl_type)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud;
  switch (impl_type) {
    case velodyne_pointcloud::VelodyneImplementType::HALF_PRECISION:
      output_pointcloud = interpolate_half_precision(input_pointcloud, twist_queue, tf2_base_link_to_sensor);
      break;
    case velodyne_pointcloud::VelodyneImplementType::FIXED_POINT_16_16:
      output_pointcloud = interpolate_fixed_16_16_precision(input_pointcloud, twist_queue, tf2_base_link_to_sensor);
      break;
    case velodyne_pointcloud::VelodyneImplementType::SINGLE_PRECISION:
      output_pointcloud = interpolate_single_precision(input_pointcloud, twist_queue, tf2_base_link_to_sensor);
      break;
    default:
      output_pointcloud = interpolate(input_pointcloud, twist_queue, tf2_base_link_to_sensor);
      break;
  }
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_single_precision(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());

  if (input_pointcloud->points.empty() || twist_queue.empty()) {
    ROS_WARN_STREAM_THROTTLE(10, "input_pointcloud->points or twist_queue is empty.");
    *output_pointcloud = *input_pointcloud;
    return output_pointcloud;
  }

  float theta = 0;
  float x = 0, y = 0;

  auto twist_it = std::lower_bound(
    std::begin(twist_queue), std::end(twist_queue),
    ros::Time(input_pointcloud->points.front().time_stamp),
    [](const geometry_msgs::TwistStamped & x, ros::Time t) { return x.header.stamp < t; });
  twist_it = twist_it == std::end(twist_queue) ? std::end(twist_queue) - 1 : twist_it;

  const tf2::Transform tf2_base_link_to_sensor_inv = tf2_base_link_to_sensor.inverse();
  for (const auto & p : input_pointcloud->points) {
    for (; (twist_it != std::end(twist_queue) - 1 && p.time_stamp > twist_it->header.stamp.toSec());
         ++twist_it) {
      // std::cout << std::fixed << p.time_stamp << " " << twist_it->header.stamp.toSec() << std::endl;
    }

    float v = twist_it->twist.linear.x;
    float w = twist_it->twist.angular.z;

    if (std::fabs(p.time_stamp - twist_it->header.stamp.toSec()) > 0.1) {
      ROS_WARN_STREAM_THROTTLE(10, "Twist time_stamp is too late. Cloud not interpolate.");
      v = 0;
      w = 0;
    }

    static double prev_time_stamp = p.time_stamp;
    const float time_offset = static_cast<float>(p.time_stamp - prev_time_stamp);

    // tf2::Vector3 sensorTF_point(p.x, p.y, p.z);
    VelodyneVector3f sensorTF_point(p.x, p.y, p.z);

    // tf2::Vector3 base_linkTF_point;
    // base_linkTF_point = tf2_base_link_to_sensor_inv * sensorTF_point;
    VelodyneVector3f base_linkTF_point;
    base_linkTF_point = multiplyTFTransformAndVector(tf2_base_link_to_sensor_inv, sensorTF_point);

    theta += w * time_offset;
    tf2::Quaternion baselink_quat;
    // baselink_quat.setRPY(0.0, 0.0, theta);
    setTFQuaternionRPY(baselink_quat, 0.0, 0.0, theta);
    const float dis = v * time_offset;
    x += dis * std::cos(theta);
    y += dis * std::sin(theta);

    tf2::Transform baselinkTF_odom;
    baselinkTF_odom.setOrigin(tf2::Vector3(x, y, 0));
    // baselinkTF_odom.setRotation(baselink_quat);
    setTFTransformRotation(baselinkTF_odom, baselink_quat,
      velodyne_pointcloud::VelodyneImplementType::SINGLE_PRECISION);

    // tf2::Vector3 base_linkTF_trans_point;
    // base_linkTF_trans_point = baselinkTF_odom * base_linkTF_point;
    VelodyneVector3f base_linkTF_trans_point;
    base_linkTF_trans_point = multiplyTFTransformAndVector(baselinkTF_odom, base_linkTF_point);

    // tf2::Vector3 sensorTF_trans_point;
    // sensorTF_trans_point = tf2_base_link_to_sensor * base_linkTF_trans_point;
    VelodyneVector3f sensorTF_trans_point;
    sensorTF_trans_point = multiplyTFTransformAndVector(tf2_base_link_to_sensor, base_linkTF_trans_point);

    velodyne_pointcloud::PointXYZIRADT tmp_p;
    // TODO:削除
    std::cout << "[esol:single]" << sensorTF_trans_point.x << ", " << sensorTF_trans_point.y << ", " << sensorTF_trans_point.z << std::endl;
    tmp_p.x = sensorTF_trans_point.x;
    tmp_p.y = sensorTF_trans_point.y;
    tmp_p.z = sensorTF_trans_point.z;
    tmp_p.intensity = p.intensity;
    tmp_p.ring = p.ring;
    tmp_p.azimuth = p.azimuth;
    tmp_p.distance = p.distance;
    tmp_p.time_stamp = p.time_stamp;
    output_pointcloud->points.push_back(tmp_p);

    prev_time_stamp = p.time_stamp;
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_half_precision(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());

  if (input_pointcloud->points.empty() || twist_queue.empty()) {
    ROS_WARN_STREAM_THROTTLE(10, "input_pointcloud->points or twist_queue is empty.");
    *output_pointcloud = *input_pointcloud;
    return output_pointcloud;
  }

  half theta = 0.0_h;
  half x = 0.0_h, y = 0.0_h;

  auto twist_it = std::lower_bound(
    std::begin(twist_queue), std::end(twist_queue),
    ros::Time(input_pointcloud->points.front().time_stamp),
    [](const geometry_msgs::TwistStamped & x, ros::Time t) { return x.header.stamp < t; });
  twist_it = twist_it == std::end(twist_queue) ? std::end(twist_queue) - 1 : twist_it;

  const tf2::Transform tf2_base_link_to_sensor_inv = tf2_base_link_to_sensor.inverse();
  for (const auto & p : input_pointcloud->points) {
    for (; (twist_it != std::end(twist_queue) - 1 && p.time_stamp > twist_it->header.stamp.toSec());
         ++twist_it) {
      // std::cout << std::fixed << p.time_stamp << " " << twist_it->header.stamp.toSec() << std::endl;
    }

    half v = half_float::half_cast<half>(twist_it->twist.linear.x);
    half w = half_float::half_cast<half>(twist_it->twist.angular.z);

    if (std::fabs(p.time_stamp - twist_it->header.stamp.toSec()) > 0.1) {
      ROS_WARN_STREAM_THROTTLE(10, "Twist time_stamp is too late. Cloud not interpolate.");
      v = 0.0_h;
      w = 0.0_h;
    }

    static double prev_time_stamp = p.time_stamp;
    const half time_offset = half_float::half_cast<half>(p.time_stamp - prev_time_stamp);

    // tf2::Vector3 sensorTF_point(p.x, p.y, p.z);
    VelodyneVector3h sensorTF_point(
      half_float::half_cast<half>(p.x), half_float::half_cast<half>(p.y), half_float::half_cast<half>(p.z));

    // tf2::Vector3 base_linkTF_point;
    // base_linkTF_point = tf2_base_link_to_sensor_inv * sensorTF_point;
    VelodyneVector3h base_linkTF_point;
    base_linkTF_point = multiplyTFTransformAndVector(tf2_base_link_to_sensor_inv, sensorTF_point);

    theta += w * time_offset;
    tf2::Quaternion baselink_quat;
    // baselink_quat.setRPY(0.0, 0.0, (float)theta);
    setTFQuaternionRPY(baselink_quat, 0.0_h, 0.0_h, theta);
    const half dis = v * time_offset;
    x += dis * half_float::cos(theta);
    y += dis * half_float::sin(theta);

    tf2::Transform baselinkTF_odom;
    baselinkTF_odom.setOrigin(tf2::Vector3((float)x, (float)y, 0));
    // baselinkTF_odom.setRotation(baselink_quat);
    setTFTransformRotation(baselinkTF_odom, baselink_quat,
      velodyne_pointcloud::VelodyneImplementType::HALF_PRECISION);

    // tf2::Vector3 base_linkTF_trans_point;
    // base_linkTF_trans_point = baselinkTF_odom * base_linkTF_point;
    VelodyneVector3h base_linkTF_trans_point;
    base_linkTF_trans_point = multiplyTFTransformAndVector(baselinkTF_odom, base_linkTF_point);

    // tf2::Vector3 sensorTF_trans_point;
    // sensorTF_trans_point = tf2_base_link_to_sensor * base_linkTF_trans_point;
    VelodyneVector3h sensorTF_trans_point;
    sensorTF_trans_point = multiplyTFTransformAndVector(tf2_base_link_to_sensor, base_linkTF_trans_point);

    velodyne_pointcloud::PointXYZIRADT tmp_p;
    tmp_p.x = (float)sensorTF_trans_point.x;
    tmp_p.y = (float)sensorTF_trans_point.y;
    tmp_p.z = (float)sensorTF_trans_point.z;
    tmp_p.intensity = p.intensity;
    tmp_p.ring = p.ring;
    tmp_p.azimuth = p.azimuth;
    tmp_p.distance = p.distance;
    tmp_p.time_stamp = p.time_stamp;
    output_pointcloud->points.push_back(tmp_p);

    prev_time_stamp = p.time_stamp;
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_fixed_16_16_precision(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const std::deque<geometry_msgs::TwistStamped> & twist_queue,
  const tf2::Transform & tf2_base_link_to_sensor)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());

  if (input_pointcloud->points.empty() || twist_queue.empty()) {
    ROS_WARN_STREAM_THROTTLE(10, "input_pointcloud->points or twist_queue is empty.");
    *output_pointcloud = *input_pointcloud;
    return output_pointcloud;
  }

  fix16_t theta = fix16_from_int(0);
  fix16_t x = fix16_from_int(0), y = fix16_from_int(0);

  auto twist_it = std::lower_bound(
    std::begin(twist_queue), std::end(twist_queue),
    ros::Time(input_pointcloud->points.front().time_stamp),
    [](const geometry_msgs::TwistStamped & x, ros::Time t) { return x.header.stamp < t; });
  twist_it = twist_it == std::end(twist_queue) ? std::end(twist_queue) - 1 : twist_it;

  const tf2::Transform tf2_base_link_to_sensor_inv = tf2_base_link_to_sensor.inverse();
  for (const auto & p : input_pointcloud->points) {
    for (; (twist_it != std::end(twist_queue) - 1 && p.time_stamp > twist_it->header.stamp.toSec());
         ++twist_it) {
      // std::cout << std::fixed << p.time_stamp << " " << twist_it->header.stamp.toSec() << std::endl;
    }

    fix16_t v = fix16_from_float(twist_it->twist.linear.x);
    fix16_t w = fix16_from_float(twist_it->twist.angular.z);

    if (std::fabs(p.time_stamp - twist_it->header.stamp.toSec()) > 0.1) {
      ROS_WARN_STREAM_THROTTLE(10, "Twist time_stamp is too late. Cloud not interpolate.");
      v = fix16_from_int(0);
      w = fix16_from_int(0);
    }

    static double prev_time_stamp = p.time_stamp;
    const fix16_t time_offset = fix16_from_dbl(p.time_stamp - prev_time_stamp);

    // tf2::Vector3 sensorTF_point(p.x, p.y, p.z);
    VelodyneVector3fx sensorTF_point(
      fix16_from_dbl(p.x), fix16_from_dbl(p.y), fix16_from_dbl(p.z));

    // tf2::Vector3 base_linkTF_point;
    // base_linkTF_point = tf2_base_link_to_sensor_inv * sensorTF_point;
    VelodyneVector3fx base_linkTF_point;
    base_linkTF_point = multiplyTFTransformAndVector(tf2_base_link_to_sensor_inv, sensorTF_point);

    theta = fix16_add(theta, fix16_mul(w, time_offset));
    tf2::Quaternion baselink_quat;
    // baselink_quat.setRPY(0.0, 0.0, fix16_to_float(theta));
    setTFQuaternionRPY(baselink_quat, fix16_from_int(0), fix16_from_int(0), theta);
    const fix16_t dis = fix16_mul(v, time_offset);
    x = fix16_add(x, fix16_mul(dis, fix16_cos(theta)));
    y = fix16_add(y, fix16_mul(dis, fix16_sin(theta)));

    tf2::Transform baselinkTF_odom;
    baselinkTF_odom.setOrigin(tf2::Vector3(fix16_to_float(x), fix16_to_float(y), 0));
    // baselinkTF_odom.setRotation(baselink_quat);
    setTFTransformRotation(baselinkTF_odom, baselink_quat,
      velodyne_pointcloud::VelodyneImplementType::FIXED_POINT_16_16);

    // tf2::Vector3 base_linkTF_trans_point;
    // base_linkTF_trans_point = baselinkTF_odom * base_linkTF_point;
    VelodyneVector3fx base_linkTF_trans_point;
    base_linkTF_trans_point = multiplyTFTransformAndVector(baselinkTF_odom, base_linkTF_point);

    // tf2::Vector3 sensorTF_trans_point;
    // sensorTF_trans_point = tf2_base_link_to_sensor * base_linkTF_trans_point;
    VelodyneVector3fx sensorTF_trans_point;
    sensorTF_trans_point = multiplyTFTransformAndVector(tf2_base_link_to_sensor, base_linkTF_trans_point);

    velodyne_pointcloud::PointXYZIRADT tmp_p;
    tmp_p.x = fix16_to_float(sensorTF_trans_point.x);
    tmp_p.y = fix16_to_float(sensorTF_trans_point.y);
    tmp_p.z = fix16_to_float(sensorTF_trans_point.z);
    tmp_p.intensity = p.intensity;
    tmp_p.ring = p.ring;
    tmp_p.azimuth = p.azimuth;
    tmp_p.distance = p.distance;
    tmp_p.time_stamp = p.time_stamp;
    output_pointcloud->points.push_back(tmp_p);
    // TODO:削除
    std::cout << "[esol:fixed]" << fix16_to_dbl(tmp_p.x) << ", " << fix16_to_dbl(tmp_p.y) << ", " << fix16_to_dbl(tmp_p.z) << std::endl;

    prev_time_stamp = p.time_stamp;
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortRingNumber(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const size_t num_lasers)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->points.resize(input_pointcloud->points.size());
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT> segment_pointcloud;
  for (size_t i = 0; i < input_pointcloud->points.size(); ++i) {
    segment_pointcloud.points.push_back(input_pointcloud->points.at(i));
    if (i % num_lasers == (num_lasers - 1)) {
      std::sort(
        std::begin(segment_pointcloud.points), std::end(segment_pointcloud.points),
        [](
          const velodyne_pointcloud::PointXYZIRADT & lhs,
          const velodyne_pointcloud::PointXYZIRADT & rhs) { return lhs.ring < rhs.ring; });
      output_pointcloud->points.insert(
        std::end(output_pointcloud->points), std::begin(segment_pointcloud.points),
        std::end(segment_pointcloud.points));
      segment_pointcloud.points.clear();
    }
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortZeroIndex(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const size_t num_lasers)
{
  size_t zero_index = 0;
  int last_azimuth = input_pointcloud->points.at(0).azimuth;
  for (size_t i = 0; i < input_pointcloud->points.size(); ++i) {
    if (input_pointcloud->points.at(i).ring == (num_lasers - 1) / 2) {
      if (last_azimuth <= 18000 && input_pointcloud->points.at(i).azimuth > 18000) {
        zero_index = i;
        break;
      }
      last_azimuth = input_pointcloud->points.at(i).azimuth;
    }
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  output_pointcloud->points.insert(
    std::end(output_pointcloud->points), std::begin(input_pointcloud->points) + zero_index,
    std::end(input_pointcloud->points));
  output_pointcloud->points.insert(
    std::end(output_pointcloud->points), std::begin(input_pointcloud->points),
    std::begin(input_pointcloud->points) + zero_index);

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortZeroIndex(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud,
  const size_t num_lasers, velodyne_pointcloud::VelodyneImplementType impl_type)
{
  size_t zero_index = 0;
  int last_azimuth = input_pointcloud->points.at(0).azimuth;
  for (size_t i = 0; i < input_pointcloud->points.size(); ++i) {
    if (input_pointcloud->points.at(i).ring == (num_lasers - 1) / 2) {
      switch (impl_type) {
        case velodyne_pointcloud::VelodyneImplementType::HALF_PRECISION:
        {
          if (last_azimuth <= 18000 &&
              half_float::half_cast<half>(input_pointcloud->points.at(i).azimuth) > 18000.0_h) {
            zero_index = i;
            break;
          }
        }
        break;
        case velodyne_pointcloud::VelodyneImplementType::FIXED_POINT_16_16:
        {
          if (last_azimuth <= 18000 &&
              fix16_from_float(input_pointcloud->points.at(i).azimuth * 0.1f) > fix16_from_int(1800)) {
            zero_index = i;
            break;
          }
        }
        break;
        case velodyne_pointcloud::VelodyneImplementType::SINGLE_PRECISION:
        default:
        {
          if (last_azimuth <= 18000 && input_pointcloud->points.at(i).azimuth > 18000) {
            zero_index = i;
            break;
          }
        }
        break;
      }
      last_azimuth = input_pointcloud->points.at(i).azimuth;
    }
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  output_pointcloud->points.insert(
    std::end(output_pointcloud->points), std::begin(input_pointcloud->points) + zero_index,
    std::end(input_pointcloud->points));
  output_pointcloud->points.insert(
    std::end(output_pointcloud->points), std::begin(input_pointcloud->points),
    std::begin(input_pointcloud->points) + zero_index);

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr convert(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & input_pointcloud)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr output_pointcloud(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  velodyne_pointcloud::PointXYZIR point;
  for (const auto & p : input_pointcloud->points) {
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point.ring = p.ring;
    output_pointcloud->points.push_back(point);
  }

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}

VelodyneVector3f multiplyTFTransformAndVector(
  const tf2::Transform & transform, const VelodyneVector3f & vector)
{
  VelodyneVector3f ret_val;
  auto dot = [](tf2::Vector3 a, VelodyneVector3f b) -> float {
    return (float)(a[0] * b.x + a[1] * b.y + a[2] * b.z);
  };

  const tf2::Matrix3x3 basis = transform.getBasis();
  const tf2::Vector3 origin = transform.getOrigin();
  ret_val.x = dot(basis[0], vector) + origin.x();
  ret_val.y = dot(basis[1], vector) + origin.y();
  ret_val.z = dot(basis[2], vector) + origin.z();

  return ret_val;
}

VelodyneVector3h multiplyTFTransformAndVector(
  const tf2::Transform & transform, const VelodyneVector3h & vector)
{
  VelodyneVector3h ret_val;
  auto dot = [](tf2::Vector3 a, VelodyneVector3h b) -> half {
    return (half_float::half_cast<half>(a[0]) * b.x + half_float::half_cast<half>(a[1]) * b.y +
            half_float::half_cast<half>(a[2]) * b.z);
  };

  const tf2::Matrix3x3 basis = transform.getBasis();
  const tf2::Vector3 origin = transform.getOrigin();
  ret_val.x = dot(basis[0], vector) + half_float::half_cast<half>(origin.x());
  ret_val.y = dot(basis[1], vector) + half_float::half_cast<half>(origin.y());
  ret_val.z = dot(basis[2], vector) + half_float::half_cast<half>(origin.z());

  return ret_val;
}

VelodyneVector3fx multiplyTFTransformAndVector(
  const tf2::Transform & transform, const VelodyneVector3fx & vector)
{
  VelodyneVector3fx ret_val;
  auto dot = [](tf2::Vector3 a, VelodyneVector3fx b) -> fix16_t {
    return (fix16_add(
      fix16_add(fix16_mul(fix16_from_dbl(a[0]), b.x), fix16_mul(fix16_from_dbl(a[1]), b.y)),
      fix16_mul(fix16_from_dbl(a[2]), b.z)));
  };

  const tf2::Matrix3x3 basis = transform.getBasis();
  const tf2::Vector3 origin = transform.getOrigin();
  ret_val.x = dot(basis[0], vector) + fix16_from_dbl(origin.x());
  ret_val.y = dot(basis[1], vector) + fix16_from_dbl(origin.y());
  ret_val.z = dot(basis[2], vector) + fix16_from_dbl(origin.z());

  return ret_val;
}

void setTFQuaternionRPY(
  tf2::Quaternion & quaternion, float roll, float pitch, float yaw)
{
  float halfYaw = yaw * float(0.5);
  float halfPitch = pitch * float(0.5);
  float halfRoll = roll * float(0.5);
  float cosYaw = std::cos(halfYaw);
  float sinYaw = std::sin(halfYaw);
  float cosPitch = std::cos(halfPitch);
  float sinPitch = std::sin(halfPitch);
  float cosRoll = std::cos(halfRoll);
  float sinRoll = std::sin(halfRoll);
  quaternion.setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                      cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                      cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                      cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
}

void setTFQuaternionRPY(
  tf2::Quaternion & quaternion, half roll, half pitch, half yaw)
{
  half halfYaw = yaw * half(0.5);
  half halfPitch = pitch * half(0.5);
  half halfRoll = roll * half(0.5);
  half cosYaw = half_float::cos(halfYaw);
  half sinYaw = half_float::sin(halfYaw);
  half cosPitch = half_float::cos(halfPitch);
  half sinPitch = half_float::sin(halfPitch);
  half cosRoll = half_float::cos(halfRoll);
  half sinRoll = half_float::sin(halfRoll);
  quaternion.setValue((double)(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw), //x
                      (double)(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw), //y
                      (double)(cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw), //z
                      (double)(cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw)); //formerly yzx
}

void setTFQuaternionRPY(
  tf2::Quaternion & quaternion, fix16_t roll, fix16_t pitch, fix16_t yaw)
{
  fix16_t halfYaw = yaw * fix16_from_float(0.5);
  fix16_t halfPitch = pitch * fix16_from_float(0.5);
  fix16_t halfRoll = roll * fix16_from_float(0.5);
  fix16_t cosYaw = fix16_cos(halfYaw);
  fix16_t sinYaw = fix16_sin(halfYaw);
  fix16_t cosPitch = fix16_cos(halfPitch);
  fix16_t sinPitch = fix16_sin(halfPitch);
  fix16_t cosRoll = fix16_cos(halfRoll);
  fix16_t sinRoll = fix16_sin(halfRoll);
  fix16_t sinRollcosPitch = fix16_mul(sinRoll, cosPitch);
  fix16_t cosRollsinPitch = fix16_mul(cosRoll, sinPitch);
  fix16_t cosRollcosPitch = fix16_mul(cosRoll, cosPitch);
  fix16_t sinRollsinPitch = fix16_mul(sinRoll, sinPitch);
  fix16_t x = fix16_sub(fix16_mul(sinRollcosPitch, cosYaw), fix16_mul(cosRollsinPitch, sinYaw));
  fix16_t y = fix16_add(fix16_mul(cosRollsinPitch, cosYaw), fix16_mul(sinRollcosPitch, sinYaw));
  fix16_t z = fix16_sub(fix16_mul(cosRollcosPitch, sinYaw), fix16_mul(sinRollsinPitch, cosYaw));
  fix16_t w = fix16_add(fix16_mul(cosRollcosPitch, cosYaw), fix16_mul(sinRollsinPitch, sinYaw));
  quaternion.setValue(fix16_to_dbl(x), //x
                      fix16_to_dbl(y), //y
                      fix16_to_dbl(z), //z
                      fix16_to_dbl(w)); //formerly yzx
}

void setTFTransformRotation(
  tf2::Transform & transform, const tf2::Quaternion & quaternion,
  velodyne_pointcloud::VelodyneImplementType impl_type)
{
  tf2::Matrix3x3& basis = transform.getBasis();
  switch (impl_type)
  {
    case velodyne_pointcloud::VelodyneImplementType::HALF_PRECISION:
    {
      half d = half_float::half_cast<half>(quaternion.x()) * half_float::half_cast<half>(quaternion.x()) +
               half_float::half_cast<half>(quaternion.y()) * half_float::half_cast<half>(quaternion.y()) +
               half_float::half_cast<half>(quaternion.z()) * half_float::half_cast<half>(quaternion.z()) +
               half_float::half_cast<half>(quaternion.w()) * half_float::half_cast<half>(quaternion.w());
      tf2FullAssert(d != half(0.0));
      half s = half(2.0) / d;
      half xs = half_float::half_cast<half>(quaternion.x()) * s,  ys = half_float::half_cast<half>(quaternion.y()) * s,
           zs = half_float::half_cast<half>(quaternion.z()) * s;
      half wx = half_float::half_cast<half>(quaternion.w()) * xs,  wy = half_float::half_cast<half>(quaternion.w()) * ys,
           wz = half_float::half_cast<half>(quaternion.w()) * zs;
      half xx = half_float::half_cast<half>(quaternion.x()) * xs,  xy = half_float::half_cast<half>(quaternion.x()) * ys,
           xz = half_float::half_cast<half>(quaternion.x()) * zs;
      half yy = half_float::half_cast<half>(quaternion.y()) * ys,  yz = half_float::half_cast<half>(quaternion.y()) * zs,
           zz = half_float::half_cast<half>(quaternion.z()) * zs;
      basis.setValue(double(1.0) - (yy + zz), xy - wz, xz + wy,
        xy + wz, double(1.0) - (xx + zz), yz - wx,
        xz - wy, yz + wx, double(1.0) - (xx + yy));
    }
    break;
    case velodyne_pointcloud::VelodyneImplementType::FIXED_POINT_16_16:
    {
      fix16_t d = fix16_add(
                    fix16_add(
                      fix16_mul(fix16_from_dbl(quaternion.x()), fix16_from_dbl(quaternion.x())),
                      fix16_mul(fix16_from_dbl(quaternion.y()), fix16_from_dbl(quaternion.y()))),
                    fix16_add(
                      fix16_mul(fix16_from_dbl(quaternion.z()), fix16_from_dbl(quaternion.z())),
                      fix16_mul(fix16_from_dbl(quaternion.w()), fix16_from_dbl(quaternion.w())))
                  );
      tf2FullAssert(d != fix16_from_int(0));
      fix16_t s = fix16_div(fix16_from_int(2), d);
      fix16_t xs = fix16_mul(fix16_from_dbl(quaternion.x()), s),  ys = fix16_mul(fix16_from_dbl(quaternion.y()), s),
              zs = fix16_mul(fix16_from_dbl(quaternion.z()), s);
      fix16_t wx = fix16_mul(fix16_from_dbl(quaternion.w()), xs),  wy = fix16_mul(fix16_from_dbl(quaternion.w()), ys),
              wz = fix16_mul(fix16_from_dbl(quaternion.w()), zs);
      fix16_t xx = fix16_mul(fix16_from_dbl(quaternion.x()), xs),  xy = fix16_mul(fix16_from_dbl(quaternion.x()), ys),
              xz = fix16_mul(fix16_from_dbl(quaternion.x()), zs);
      fix16_t yy = fix16_mul(fix16_from_dbl(quaternion.y()), ys),  yz = fix16_mul(fix16_from_dbl(quaternion.y()), zs),
              zz = fix16_mul(fix16_from_dbl(quaternion.z()), zs);
      basis.setValue(double(1.0) - fix16_to_dbl(fix16_add(yy, zz)), fix16_to_dbl(fix16_sub(xy, wz)), fix16_to_dbl(fix16_add(xz, wy)),
        fix16_to_dbl(fix16_add(xy, wz)), double(1.0) - fix16_to_dbl(fix16_add(xx, zz)), fix16_to_dbl(fix16_sub(yz, wx)),
        fix16_to_dbl(fix16_sub(xz, wy)), fix16_to_dbl(fix16_add(yz, wx)), double(1.0) - fix16_to_dbl(fix16_add(xx, yy)));
      basis = transform.getBasis();
      std::cout << "[esol:fixed]basis:" << std::endl << basis.getRow(0)[0] << ", " << basis.getRow(0)[1] << ", " << basis.getRow(0)[2] << std::endl
                                                     << basis.getRow(1)[0] << ", " << basis.getRow(1)[1] << ", " << basis.getRow(1)[2] << std::endl
                                                     << basis.getRow(2)[0] << ", " << basis.getRow(2)[1] << ", " << basis.getRow(2)[2] << std::endl;
    }
    break;
    case velodyne_pointcloud::VelodyneImplementType::SINGLE_PRECISION:
    default:
    {
      float d = quaternion.x() * quaternion.x() + quaternion.y() * quaternion.y() +
                quaternion.z() * quaternion.z() + quaternion.w() * quaternion.w();
      tf2FullAssert(d != float(0.0));
      float s = float(2.0) / d;
      float xs = quaternion.x() * s,   ys = quaternion.y() * s,   zs = quaternion.z() * s;
      float wx = quaternion.w() * xs,  wy = quaternion.w() * ys,  wz = quaternion.w() * zs;
      float xx = quaternion.x() * xs,  xy = quaternion.x() * ys,  xz = quaternion.x() * zs;
      float yy = quaternion.y() * ys,  yz = quaternion.y() * zs,  zz = quaternion.z() * zs;
      basis.setValue(double(1.0) - (yy + zz), xy - wz, xz + wy,
        xy + wz, double(1.0) - (xx + zz), yz - wx,
        xz - wy, yz + wx, double(1.0) - (xx + yy));
      basis = transform.getBasis();
      std::cout << "[esol:single]basis:" << std::endl << basis.getRow(0)[0] << ", " << basis.getRow(0)[1] << ", " << basis.getRow(0)[2] << std::endl
                                                     << basis.getRow(1)[0] << ", " << basis.getRow(1)[1] << ", " << basis.getRow(1)[2] << std::endl
                                                     << basis.getRow(2)[0] << ", " << basis.getRow(2)[1] << ", " << basis.getRow(2)[2] << std::endl;    }
    break;
  }
}

}  // namespace velodyne_pointcloud

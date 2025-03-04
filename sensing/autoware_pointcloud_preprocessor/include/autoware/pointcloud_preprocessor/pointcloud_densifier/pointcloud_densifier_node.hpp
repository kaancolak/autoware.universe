// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_
#define POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_

#include <deque>
#include <memory>
#include <string>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#include "autoware/pointcloud_preprocessor/pointcloud_densifier/occupancy_grid.hpp"

namespace autoware::pointcloud_preprocessor
{

// Structure to store point clouds with metadata
struct StoredPointCloud {
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  std_msgs::msg::Header header;
};

class PointCloudDensifierNode : public rclcpp::Node
{
public:
  explicit PointCloudDensifierNode() 
  : PointCloudDensifierNode(rclcpp::NodeOptions()) {}
  
  explicit PointCloudDensifierNode(const rclcpp::NodeOptions & options);

private:
  void onPointCloud(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & input_msg);

  // Helper methods
  sensor_msgs::msg::PointCloud2::SharedPtr filterPointCloudByROI(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud);
  
  void transformAndMergePreviousClouds(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& current_msg,
    const OccupancyGrid& occupancy_grid,
    sensor_msgs::msg::PointCloud2::SharedPtr& combined_cloud);
    
  void storeCurrentCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& filtered_cloud,
    const std_msgs::msg::Header& header);

  bool isValidTransform(const Eigen::Matrix4d& transform) const;
  void loadParameters();
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::deque<StoredPointCloud> previous_pointclouds_;
  int num_previous_frames_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double grid_resolution_;
};

// Node factory declaration
class PointCloudDensifierNodeFactory
{
public:
  static std::shared_ptr<rclcpp::Node> create(
    const rclcpp::NodeOptions& options);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_
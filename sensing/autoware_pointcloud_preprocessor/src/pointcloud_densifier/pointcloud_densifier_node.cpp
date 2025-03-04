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

#include "autoware/pointcloud_preprocessor/pointcloud_densifier/pointcloud_densifier_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <chrono>

namespace autoware::pointcloud_preprocessor
{

PointCloudDensifierNode::PointCloudDensifierNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_densifier", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  RCLCPP_INFO(get_logger(), "Node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "Node namespace: %s", get_namespace());
  
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", 
    rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&PointCloudDensifierNode::onPointCloud, this, std::placeholders::_1));

  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
  
  RCLCPP_INFO(get_logger(), "Subscription topic: %s", pointcloud_sub_->get_topic_name());
  RCLCPP_INFO(get_logger(), "Publishing topic: %s", pointcloud_pub_->get_topic_name());
  
  // Load parameters
  loadParameters();
}

void PointCloudDensifierNode::loadParameters()
{
  // Get parameters without declaring them if they're already declared
  if (!this->has_parameter("num_previous_frames")) {
    this->declare_parameter<int>("num_previous_frames", 1);
  }
  if (!this->has_parameter("x_min")) {
    this->declare_parameter<double>("x_min", 80.0);
  }
  if (!this->has_parameter("x_max")) {
    this->declare_parameter<double>("x_max", 200.0);
  }
  if (!this->has_parameter("y_min")) {
    this->declare_parameter<double>("y_min", -20.0);
  }
  if (!this->has_parameter("y_max")) {
    this->declare_parameter<double>("y_max", 20.0);
  }
  if (!this->has_parameter("grid_resolution")) {
    this->declare_parameter<double>("grid_resolution", 0.30);
  }

  // Get parameter values
  num_previous_frames_ = this->get_parameter("num_previous_frames").as_int();
  x_min_ = this->get_parameter("x_min").as_double();
  x_max_ = this->get_parameter("x_max").as_double();
  y_min_ = this->get_parameter("y_min").as_double();
  y_max_ = this->get_parameter("y_max").as_double();
  grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  
  // Validate parameters
  if (num_previous_frames_ < 0) {
    RCLCPP_WARN(get_logger(), "num_previous_frames must be non-negative. Setting to 0.");
    num_previous_frames_ = 0;
  }
  if (x_min_ >= x_max_ || y_min_ >= y_max_) {
    RCLCPP_ERROR(get_logger(), "Invalid ROI bounds: x_min must be less than x_max, and y_min must be less than y_max");
    throw std::invalid_argument("Invalid ROI bounds");
  }
  if (grid_resolution_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "grid_resolution must be positive");
    throw std::invalid_argument("Invalid grid resolution");
  }
}

void PointCloudDensifierNode::onPointCloud(
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & input_msg)
{
  // Measure time
  auto start_time = std::chrono::high_resolution_clock::now();

  // Filter points by ROI
  auto far_front_pointcloud_ptr = filterPointCloudByROI(input_msg);
  
  // Build occupancy grid from the current filtered cloud
  OccupancyGrid occupancy_grid(x_min_, x_max_, y_min_, y_max_, grid_resolution_);
  occupancy_grid.updateOccupancy(*far_front_pointcloud_ptr);

  // Create and initialize output message with same structure as input
  auto combined_pointcloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
  *combined_pointcloud_ptr = *input_msg;  // Copy metadata and structure

  // Transform and merge previous clouds
  transformAndMergePreviousClouds(input_msg, occupancy_grid, combined_pointcloud_ptr);

  // Store the current filtered point cloud for future use
  storeCurrentCloud(far_front_pointcloud_ptr, input_msg->header);

  // Publish the combined point cloud
  pointcloud_pub_->publish(*combined_pointcloud_ptr);

  
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time = end_time - start_time;
  RCLCPP_INFO(get_logger(), "Processing time: %f [s]", elapsed_time.count());
  
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudDensifierNode::filterPointCloudByROI(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud)
{
  auto filtered_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  
  // Initialize output with same format as input
  filtered_cloud->header = input_cloud->header;
  filtered_cloud->height = 1;  // Unorganized cloud
  filtered_cloud->fields = input_cloud->fields;
  filtered_cloud->is_bigendian = input_cloud->is_bigendian;
  filtered_cloud->point_step = input_cloud->point_step;
  filtered_cloud->is_dense = input_cloud->is_dense;
  
  // Pre-allocate data for the filtered cloud
  filtered_cloud->data.resize(input_cloud->data.size());
  
  // Set up iterators to access x,y coordinates
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_cloud, "y");
  
  // Counter for filtered points
  size_t output_size = 0;
  
  // Iterate through points and filter by ROI
  for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, i++) {
    float x = *iter_x;
    float y = *iter_y;
    
    if (x > x_min_ && x < x_max_ && y > y_min_ && y < y_max_) {
      // If point is in ROI, copy the entire point data
      size_t data_offset = i * input_cloud->point_step;
      std::memcpy(
        &filtered_cloud->data[output_size],
        &input_cloud->data[data_offset],
        input_cloud->point_step
      );
      output_size += input_cloud->point_step;
    }
  }
  
  // Set width based on actual number of points
  filtered_cloud->width = output_size / filtered_cloud->point_step;
  filtered_cloud->row_step = filtered_cloud->width * filtered_cloud->point_step;
  filtered_cloud->data.resize(output_size);
  
  return filtered_cloud;
}

void PointCloudDensifierNode::transformAndMergePreviousClouds(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& current_msg,
  const OccupancyGrid& occupancy_grid,
  sensor_msgs::msg::PointCloud2::SharedPtr& combined_cloud)
{
  for (const auto& previous_pointcloud : previous_pointclouds_) {
    if (!previous_pointcloud.cloud || previous_pointcloud.cloud->data.empty()) {
      continue;
    }
    
    // Get transform from previous cloud's frame to current frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      tf2::TimePoint current_time_point = tf2::TimePoint(
        std::chrono::nanoseconds(current_msg->header.stamp.nanosec) +
        std::chrono::seconds(current_msg->header.stamp.sec));
      tf2::TimePoint prev_time_point = tf2::TimePoint(
        std::chrono::nanoseconds(previous_pointcloud.header.stamp.nanosec) +
        std::chrono::seconds(previous_pointcloud.header.stamp.sec));
      transform_stamped = tf_buffer_->lookupTransform(
        current_msg->header.frame_id,
        current_time_point,
        previous_pointcloud.header.frame_id,
        prev_time_point,
        "map"  // fixed frame
      );
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Could not transform point cloud: %s", ex.what());
      continue;
    }
    
    // Validate and obtain the transformation
    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped);
    if (!isValidTransform(transform_eigen.matrix())) {
      RCLCPP_WARN(get_logger(), "Invalid transform matrix, skipping point cloud");
      continue;
    }
    
    // Transform previous point cloud to current frame
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*previous_pointcloud.cloud, transformed_cloud, transform_stamped);
    
    // Set up iterators for the transformed cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
    
    // Add previous points only if they fall into occupied grid cells
    for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, i++) {
      float x = *iter_x;
      float y = *iter_y;
      
      if (occupancy_grid.isOccupied(x, y)) {
        // Add this point to the combined cloud
        size_t data_offset = i * transformed_cloud.point_step;
        combined_cloud->data.insert(
          combined_cloud->data.end(),
          transformed_cloud.data.begin() + data_offset,
          transformed_cloud.data.begin() + data_offset + transformed_cloud.point_step
        );
      }
    }
  }
  
  // Update cloud metadata
  combined_cloud->width = combined_cloud->data.size() / combined_cloud->point_step;
  combined_cloud->row_step = combined_cloud->width * combined_cloud->point_step;
  combined_cloud->height = 1; 
}

void PointCloudDensifierNode::storeCurrentCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr& filtered_cloud,
  const std_msgs::msg::Header& header)
{
  StoredPointCloud current_pointcloud;
  current_pointcloud.cloud = filtered_cloud;
  current_pointcloud.header = header;
  if (previous_pointclouds_.size() >= static_cast<size_t>(num_previous_frames_)) {
    previous_pointclouds_.pop_front();
  }
  previous_pointclouds_.push_back(current_pointcloud);
}

bool PointCloudDensifierNode::isValidTransform(const Eigen::Matrix4d& transform) const
{
  return transform.allFinite() && 
         std::abs(transform.determinant() - 1.0) < 1e-3; // Check if it's a proper rigid transformation
}

// Factory method implementation
std::shared_ptr<rclcpp::Node> PointCloudDensifierNodeFactory::create(
  const rclcpp::NodeOptions& options)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<PointCloudDensifierNode>(options);
  return node;
}

}  // namespace autoware::pointcloud_preprocessor

// Register component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PointCloudDensifierNode)

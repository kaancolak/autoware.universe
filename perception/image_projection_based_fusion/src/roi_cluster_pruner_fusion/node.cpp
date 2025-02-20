#include "image_projection_based_fusion/roi_cluster_pruner_fusion/node.hpp"

#include <image_projection_based_fusion/utils/geometry.hpp>
#include <image_projection_based_fusion/utils/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

namespace image_projection_based_fusion
{
  ClusterRoiPrunerFusionNode::ClusterRoiPrunerFusionNode(const rclcpp::NodeOptions & options)
  : FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature, DetectedObjectsWithFeature>(
    "cluster_roi_pruner_fusion", options)
{

  RCLCPP_INFO(this->get_logger(), "ClusterRoiPrunerFusionNode started");
  // Declare separate thresholds per vehicle type.
  double threshold_car = declare_parameter<double>("merge_distance_threshold_car", 3.0);
  double threshold_bus = declare_parameter<double>("merge_distance_threshold_bus", 6.0);
  double threshold_truck = declare_parameter<double>("merge_distance_threshold_truck", 6.0);
  double threshold_trailer = declare_parameter<double>("merge_distance_threshold_trailer", 4.0);
  merge_distance_threshold_map_[autoware_perception_msgs::msg::ObjectClassification::CAR] = threshold_car;
  merge_distance_threshold_map_[autoware_perception_msgs::msg::ObjectClassification::BUS] = threshold_bus;
  merge_distance_threshold_map_[autoware_perception_msgs::msg::ObjectClassification::TRUCK] = threshold_truck;
  merge_distance_threshold_map_[autoware_perception_msgs::msg::ObjectClassification::TRAILER] = threshold_trailer;
  // New scope parameters.
  roi_min_x_ = declare_parameter<double>("roi_min_x", 80.0);
  roi_max_x_ = declare_parameter<double>("roi_max_x", 300.0);
  roi_min_y_ = declare_parameter<double>("roi_min_y", -20.0);
  roi_max_y_ = declare_parameter<double>("roi_max_y", 20.0);
  roi_min_z_ = declare_parameter<double>("roi_min_z", -10.0);
  roi_max_z_ = declare_parameter<double>("roi_max_z", 10.0);
  // NEW: Define roi_scale_factor as a config parameter.
  roi_scale_factor_ = declare_parameter<double>("roi_scale_factor", 1.1);

  valid_labels_ = {
    autoware_perception_msgs::msg::ObjectClassification::CAR,
    autoware_perception_msgs::msg::ObjectClassification::TRUCK,
    autoware_perception_msgs::msg::ObjectClassification::BUS,
    autoware_perception_msgs::msg::ObjectClassification::TRAILER
  };


}

void ClusterRoiPrunerFusionNode::preprocess(DetectedObjectsWithFeature & output_msg)
{
  output_msg.feature_objects.clear();
}

void ClusterRoiPrunerFusionNode::fuseOnSingleImage(
  const DetectedObjectsWithFeature & clusters_msg,
  const std::size_t image_id,
  const DetectedObjectsWithFeature & roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info,
  DetectedObjectsWithFeature & output_msg)
{
  Eigen::Matrix4d projection;
  projection << camera_info.p.at(0),  camera_info.p.at(1),  camera_info.p.at(2),  camera_info.p.at(3),
                camera_info.p.at(4),  camera_info.p.at(5),  camera_info.p.at(6),  camera_info.p.at(7),
                camera_info.p.at(8),  camera_info.p.at(9),  camera_info.p.at(10), camera_info.p.at(11);

  // get transform from cluster frame id to camera optical frame id
  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, camera_info.header.frame_id,
      clusters_msg.header.frame_id, camera_info.header.stamp);
    if (!transform_stamped_optional) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Failed to get transform from " << clusters_msg.header.frame_id << " to "
                                                      << camera_info.header.frame_id);
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }
  
  // Compute each cluster's ROI based on its transformed pointcloud.
  std::vector<sensor_msgs::msg::RegionOfInterest> cluster_rois(clusters_msg.feature_objects.size());
  std::vector<bool> valid_cluster(clusters_msg.feature_objects.size(), true);

  for (std::size_t i = 0; i < clusters_msg.feature_objects.size(); ++i) {
    const auto & obj = clusters_msg.feature_objects.at(i);

    if (obj.feature.cluster.data.empty() || out_of_scope(obj)) {
      valid_cluster[i] = false;
      continue;
    }

    sensor_msgs::msg::PointCloud2 transformed_cluster;
    tf2::doTransform(obj.feature.cluster, transformed_cluster, transform_stamped);
    
    int min_x(camera_info.width), min_y(camera_info.height), max_x(0), max_y(0);
    std::vector<Eigen::Vector2d> projected_points;
    projected_points.reserve(transformed_cluster.data.size());
    
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
         iter_y(transformed_cluster, "y"),
         iter_z(transformed_cluster, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      Eigen::Vector4d point(*iter_x, *iter_y, *iter_z, 1.0);
      Eigen::Vector4d projected_point = projection * point;
    
      // Ensure we don't divide by zero.
      if (std::abs(projected_point.z()) < 1e-6) {
        continue;
      }
      
      // Compute normalized projected coordinates.
      Eigen::Vector2d proj_point(projected_point.x() * projected_point.z(),
                                 projected_point.y() * projected_point.z());
    
      // Convert to integer pixel coordinates.
      const int proj_x = static_cast<int>(proj_point.x());
      const int proj_y = static_cast<int>(proj_point.y());
      const int max_width = static_cast<int>(camera_info.width) - 1;
      const int max_height = static_cast<int>(camera_info.height) - 1;
    
      if (proj_x >= 0 && proj_x <= max_width &&
          proj_y >= 0 && proj_y <= max_height)
      {
        min_x = std::min(proj_x, min_x);
        min_y = std::min(proj_y, min_y);
        max_x = std::max(proj_x, max_x);
        max_y = std::max(proj_y, max_y);
        projected_points.push_back(proj_point);
        if (debugger_) {
          debugger_->obstacle_points_.push_back(proj_point);
        }
      }
    }

    if (projected_points.empty()) {
      valid_cluster[i] = false;
      continue;
    }


    sensor_msgs::msg::RegionOfInterest cluster_roi;
    cluster_roi.x_offset = min_x;
    cluster_roi.y_offset = min_y;
    cluster_roi.width = max_x - min_x;
    cluster_roi.height = max_y - min_y;
    cluster_rois[i] = cluster_roi;

    if (debugger_) debugger_->obstacle_rois_.push_back(cluster_roi);
  }


  std::map<std::size_t, std::vector<std::size_t>> roi_associations;
  for (std::size_t r = 0; r < roi_msg.feature_objects.size(); ++r) {
    const auto & roi_obj = roi_msg.feature_objects.at(r);

    // Assume classification vector is not empty.
    const auto roi_label = roi_obj.object.classification.front().label;
    if (valid_labels_.find(roi_label) == valid_labels_.end()) {
      continue;
    }

    sensor_msgs::msg::RegionOfInterest image_roi = roi_obj.feature.roi;
    if (debugger_) {
      debugger_->image_rois_.push_back(image_roi);
    }

    for (std::size_t i = 0; i < cluster_rois.size(); ++i) {
      if (!valid_cluster[i]) {
        continue;
      }
      
      const sensor_msgs::msg::RegionOfInterest & cluster_roi = cluster_rois[i];
      if (is_inside(image_roi, cluster_roi, roi_scale_factor_)) {
        roi_associations[r].push_back(i);
      }
    }
  }
  

  std::vector<DetectedObjectWithFeature> output_objects;

  // Process each ROI association.
  for (const auto & assoc_pair : roi_associations) {
    const std::size_t roi_index = assoc_pair.first;
    const auto & cluster_indices = assoc_pair.second;
    if (cluster_indices.empty()) {
      continue;
    }
  
    const auto & image_roi = roi_msg.feature_objects.at(roi_index).feature.roi;
  
    // Select candidate based solely on maximum IoU.
    size_t best_idx = cluster_indices.front();
    double best_iou = calcIoU(cluster_rois.at(best_idx), image_roi);
    for (auto idx : cluster_indices) {
      double iou = calcIoU(cluster_rois.at(idx), image_roi);
      if (iou > best_iou) {
        best_iou = iou;
        best_idx = idx;
      }
    }
  
    // Retrieve the ROI label.
    uint8_t label = roi_msg.feature_objects.at(roi_index).object.classification.front().label;
  
    // Output best object.
    output_objects.push_back(clusters_msg.feature_objects.at(best_idx));
  
    // Retrieve merge distance threshold.
    double distance_threshold = merge_distance_threshold_map_[label];
  
    // Check each other candidate individually.
    const auto & best_pose =
        clusters_msg.feature_objects.at(best_idx).object.kinematics.pose_with_covariance.pose.position;
    Eigen::Vector2d best_pt(best_pose.x, best_pose.y);
    for (auto idx : cluster_indices) {
      if (idx == best_idx)
        continue;
      const auto & pose =
          clusters_msg.feature_objects.at(idx).object.kinematics.pose_with_covariance.pose.position;
      Eigen::Vector2d pt(pose.x, pose.y);
      if ((best_pt - pt).norm() >= distance_threshold) {
        output_objects.push_back(clusters_msg.feature_objects.at(idx));
      }
    }
  }

  for (std::size_t i = 0; i < clusters_msg.feature_objects.size(); ++i) {
    if (!valid_cluster[i]) {
      output_objects.push_back(clusters_msg.feature_objects.at(i));
    }
  }
  output_msg.feature_objects = output_objects;

  // note: debug objects are safely cleared in fusion_node.cpp
  if (debugger_) {
    debugger_->publishImage(image_id, roi_msg.header.stamp);
  }
}

bool ClusterRoiPrunerFusionNode::out_of_scope(const DetectedObjectWithFeature & obj)
{
  const auto & position = obj.object.kinematics.pose_with_covariance.pose.position;
  return position.x < roi_min_x_ || position.x > roi_max_x_ ||
         position.y < roi_min_y_ || position.y > roi_max_y_ ||
         position.z < roi_min_z_ || position.z > roi_max_z_;
}

}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::ClusterRoiPrunerFusionNode)
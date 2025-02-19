#ifndef IMAGE_PROJECTION_BASED_FUSION__CLUSTER_ROI_PRUNER_FUSION__NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__CLUSTER_ROI_PRUNER_FUSION__NODE_HPP_

#include "image_projection_based_fusion/fusion_node.hpp"
#include <unordered_map>

namespace image_projection_based_fusion
{
class ClusterRoiPrunerFusionNode
  : public FusionNode<
        DetectedObjectsWithFeature, DetectedObjectWithFeature, DetectedObjectsWithFeature>
{
public:
  explicit ClusterRoiPrunerFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(DetectedObjectsWithFeature & output_msg) override;
  //void postprocess(DetectedObjectsWithFeature & output_msg) override;
  void fuseOnSingleImage(
    const DetectedObjectsWithFeature & clusters_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info,
    DetectedObjectsWithFeature & output_msg) override;
  // Declare out_of_scope override (implementation in .cpp)
  bool out_of_scope(const DetectedObjectWithFeature & obj) override;

  double iou_merge_threshold_;
  double merge_distance_threshold_default_;
  std::unordered_map<uint8_t, double> merge_distance_threshold_map_;

  // New filter scope parameters.
  float roi_min_x_;
  float roi_max_x_;
  float roi_min_y_;
  float roi_max_y_;
  float roi_min_z_;
  float roi_max_z_;

  double roi_scale_factor_;
};
}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__CLUSTER_ROI_PRUNER_FUSION__NODE_HPP_

#ifndef POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__ROI_EXCLUDED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__ROI_EXCLUDED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODELET_HPP_

#include "pointcloud_preprocessor/filter.hpp"
#include "pointcloud_preprocessor/transform_info.hpp"
#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace pointcloud_preprocessor
{
class RoiExcludedVoxelGridDownsampleFilterComponent : public Filter
{
public:
  explicit RoiExcludedVoxelGridDownsampleFilterComponent(const rclcpp::NodeOptions & options);

protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;
  void faster_filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices,
    PointCloud2 & output, const TransformInfo & transform_info) override;

private:
  float voxel_size_x_;
  float voxel_size_y_;
  float voxel_size_z_;
  float roi_x_min_;
  float roi_x_max_;
  float roi_y_min_;
  float roi_y_max_;
  std::mutex mutex_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__ROI_EXCLUDED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODELET_HPP_
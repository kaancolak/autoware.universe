#ifndef BUILD_OBJECT_DETECTION_EVALUATOR_HPP
#define BUILD_OBJECT_DETECTION_EVALUATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

namespace object_detection_evaluator
{
    using autoware_auto_perception_msgs::msg::DetectedObjects;
    using tier4_perception_msgs::msg::DetectedObjectsWithFeature;

class ObjectDetectionEvaluator : public rclcpp::Node
{
    public:
    explicit ObjectDetectionEvaluator(const rclcpp::NodeOptions & node_options);

    private:
    rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr sub_;
    rclcpp::Publisher<DetectedObjects>::SharedPtr pub_;
    void objectCallback(const DetectedObjectsWithFeature::ConstSharedPtr input);
};
}

#endif //BUILD_OBJECT_DETECTION_EVALUATOR_HPP

#ifndef BUILD_OBJECT_DETECTION_EVALUATOR_HPP
#define BUILD_OBJECT_DETECTION_EVALUATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "object_detection_evaluator/evaluate_object.hpp"

namespace object_detection_evaluator {
    using autoware_auto_perception_msgs::msg::DetectedObjects;

    class ObjectDetectionEvaluator : public rclcpp::Node {
    public:
        explicit ObjectDetectionEvaluator(const rclcpp::NodeOptions &node_options);

    private:

        message_filters::Subscriber<DetectedObjects> sub_gt_objects_;
        message_filters::Subscriber<DetectedObjects> sub_predicted_objects_;
        typedef message_filters::sync_policies::ApproximateTime<DetectedObjects, DetectedObjects> SyncPolicy;
        typedef message_filters::Synchronizer<SyncPolicy> Sync;
        std::shared_ptr<Sync> sync_;

        void objectCallback(const DetectedObjects::ConstSharedPtr input_gt,
                            const DetectedObjects::ConstSharedPtr input_prediction);

        EvaluateObject _evaluate_object;

    };
}

#endif //BUILD_OBJECT_DETECTION_EVALUATOR_HPP

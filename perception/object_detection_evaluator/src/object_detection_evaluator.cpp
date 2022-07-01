#include <object_detection_evaluator/object_detection_evaluator.hpp>

#include <boost/optional.hpp>

namespace object_detection_evaluator {
    ObjectDetectionEvaluator::ObjectDetectionEvaluator(const rclcpp::NodeOptions &node_options)
            : Node("object_detection_evaluator", node_options) {
        rclcpp::QoS qos(10);
        auto rmw_qos_profile = qos.get_rmw_qos_profile();

        sub_gt_objects_.subscribe(this, "/gt_objects", rmw_qos_profile);
        sub_predicted_objects_.subscribe(this, "/objects", rmw_qos_profile);

        sync_ = std::make_shared<Sync>(SyncPolicy(10), sub_gt_objects_, sub_predicted_objects_);

        using std::placeholders::_1;
        using std::placeholders::_2;

        sync_->registerCallback(
                std::bind(&ObjectDetectionEvaluator::objectCallback, this, _1, _2));

        std::cout << "BIKBIK" << std::endl;
    }

    void ObjectDetectionEvaluator::objectCallback(
            const DetectedObjects::ConstSharedPtr input_gt,
            const DetectedObjects::ConstSharedPtr input_prediction) {
        (void) input_gt;
        (void) input_prediction;
        std::cout << "Coming callback..." << std::endl;
    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(object_detection_evaluator::ObjectDetectionEvaluator)

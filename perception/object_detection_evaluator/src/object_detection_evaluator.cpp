#include <object_detection_evaluator/object_detection_evaluator.hpp>

namespace object_detection_evaluator
{
    ObjectDetectionEvaluator::ObjectDetectionEvaluator(const rclcpp::NodeOptions & node_options)
            : Node("object_detection_evaluator", node_options)
    {
        using std::placeholders::_1;
        pub_ = this->create_publisher<DetectedObjects>("~/output", rclcpp::QoS(1));
        sub_ = this->create_subscription<DetectedObjectsWithFeature>(
                "~/input", 1, std::bind(&ObjectDetectionEvaluator::objectCallback, this, _1));
    }

    void ObjectDetectionEvaluator::objectCallback(
            const DetectedObjectsWithFeature::ConstSharedPtr input)
    {
        (void) input;
        std::cout<<"Coming callback..."<<std::endl;
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_detection_evaluator::ObjectDetectionEvaluator)

cmake_minimum_required(VERSION 3.8)
project(object_detection_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(evaluator_lib SHARED
        src/object_detection_evaluator.cpp
        src/evaluate_object.cpp
        )

rclcpp_components_register_node(evaluator_lib
        PLUGIN "object_detection_evaluator::ObjectDetectionEvaluator"
        EXECUTABLE object_detection_evaluator_node
        )

ament_auto_package(
        INSTALL_TO_SHARE
#        launch
)

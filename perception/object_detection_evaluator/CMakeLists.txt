cmake_minimum_required(VERSION 3.8)
project(object_detection_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(detected_object_feature_remover_node SHARED
        src/object_detection_evaluator.cpp
        )

rclcpp_components_register_node(detected_object_feature_remover_node
        PLUGIN "object_detection_evaluator::ObjectDetectionEvaluator"
        EXECUTABLE object_detection_evaluator
        )

ament_auto_package(
        INSTALL_TO_SHARE
#        launch
)

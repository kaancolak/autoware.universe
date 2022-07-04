#include <object_detection_evaluator/evaluate_object.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <utility>

//using autoware_auto_perception_msgs::msg::DetectedObjects;

bool EvaluateObject::eval(autoware_auto_perception_msgs::msg::DetectedObjects input_gt,
                          autoware_auto_perception_msgs::msg::DetectedObjects input_prediction) {

    std::cout << "Input gt frame: " << input_gt.header.frame_id << std::endl;
    std::cout << "input_prediction:" << input_prediction.header.frame_id << std::endl;

    initGlobals();

    // holds wether orientation similarity shall be computed (might be set to false while loading detections)
    // and which labels where provided by this submission
    bool compute_aos = true;
    std::vector<bool> eval_image(NUM_CLASS, false);
    std::vector<bool> eval_ground(NUM_CLASS, false);
    std::vector<bool> eval_3d(NUM_CLASS, true);


//    bool evaluate_2d_detections = false;
//    bool evaluate_bev = false;
    bool evaluate_3d_detections = true;

    // hold detections and ground truth in memory
    std::vector<tDetection> detections = EvaluateObject::loadDetections(std::move(input_prediction));
    std::vector<tGroundtruth> groundtruth = EvaluateObject::loadGroundtruth(std::move(input_gt));

    std::cout << "Detection size : " << detections.size() << std::endl;
    std::cout << "GT size : " << groundtruth.size() << std::endl;



// holds pointers for result files
//    FILE *fp_det = 0;
//    FILE *fp_ori = 0;

//// eval image 2D bounding boxes
//    for (int c = 0; c < NUM_CLASS; c++) {
//        CLASSES cls = (CLASSES) c;
////mail->msg("Checking 2D evaluation (%s) ...", CLASS_NAMES[c].c_str());
//        if (eval_image[c]) {
//            mail->msg("Starting 2D evaluation (%s) ...", CLASS_NAMES[c].c_str());
//            fp_det = fopen((result_dir + "/stats_" + CLASS_NAMES[c] + "_detection.txt").c_str(), "w");
//            if (compute_aos)
//                fp_ori = fopen((result_dir + "/stats_" + CLASS_NAMES[c] + "_orientation.txt").c_str(), "w");
//            std::vector<double> precision[3], aos[3];
//            if (!eval_class(fp_det, fp_ori, cls, groundtruth, detections, compute_aos, imageBoxOverlap,
//                            precision[0], aos[0], EASY, IMAGE)
//                || !eval_class(fp_det, fp_ori, cls, groundtruth, detections, compute_aos, imageBoxOverlap,
//                               precision[1], aos[1], MODERATE, IMAGE)
//                || !eval_class(fp_det, fp_ori, cls, groundtruth, detections, compute_aos, imageBoxOverlap,
//                               precision[2], aos[2], HARD, IMAGE)) {
//                mail->msg("%s evaluation failed.", CLASS_NAMES[c].c_str());
//                return false;
//            }
//            fclose(fp_det);
//            saveAndPlotPlots(plot_dir, CLASS_NAMES[c] + "_detection", CLASS_NAMES[c], precision, 0);
//            if (compute_aos) {
//                saveAndPlotPlots(plot_dir, CLASS_NAMES[c] + "_orientation", CLASS_NAMES[c], aos, 1);
//                fclose(fp_ori);
//            }
//            mail->msg("  done.");
//        }
//    }

//// don't evaluate AOS for birdview boxes and 3D boxes
//    compute_aos = false;
//
//// eval bird's eye view bounding boxes
//    for (int c = 0; c < NUM_CLASS; c++) {
//        CLASSES cls = (CLASSES) c;
//        //mail->msg("Checking bird's eye evaluation (%s) ...", CLASS_NAMES[c].c_str());
//        if (eval_ground[c]) {
//            mail->msg("Starting bird's eye evaluation (%s) ...", CLASS_NAMES[c].c_str());
//            fp_det = fopen((result_dir + "/stats_" + CLASS_NAMES[c] + "_detection_ground.txt").c_str(), "w");
//            std::vector<double> precision[3], aos[3];
//            if (!eval_class(fp_det, fp_ori, cls, groundtruth, detections, compute_aos, groundBoxOverlap,
//                            precision[0], aos[0], EASY, GROUND)
//                || !eval_class(fp_det, fp_ori, cls, groundtruth, detections, compute_aos, groundBoxOverlap,
//                               precision[1], aos[1], MODERATE, GROUND)
//                || !eval_class(fp_det, fp_ori, cls, groundtruth, detections, compute_aos, groundBoxOverlap,
//                               precision[2], aos[2], HARD, GROUND)) {
//                mail->msg("%s evaluation failed.", CLASS_NAMES[c].c_str());
//                return false;
//            }
//            fclose(fp_det);
//            saveAndPlotPlots(plot_dir, CLASS_NAMES[c] + "_detection_ground", CLASS_NAMES[c], precision, 0);
//            mail->msg("  done.");
//        }
//    }


    if(evaluate_3d_detections){
        // eval 3D bounding boxes
        for (int c = 0; c < NUM_CLASS; c++) {
            auto cls = (CLASSES) c;

            std::cout<<"aaaa"<<std::endl;

            //mail->msg("Checking 3D evaluation (%s) ...", CLASS_NAMES[c].c_str());
            if (eval_3d[c]) {
//                fp_det = fopen((result_dir + "/stats_" + CLASS_NAMES[c] + "_detection_3d.txt").c_str(), "w");
                std::vector<double> precision[3];
                std::vector<double> aos[3];
                if (!eval_class(cls, groundtruth, detections, compute_aos, precision[0],
                                aos[0], BOX3D)
//                    ||
//                    !eval_class(cls, groundtruth, detections, compute_aos, precision[1],
//                                aos[1], BOX3D)
//                    ||
//                    !eval_class(cls, groundtruth, detections, compute_aos, precision[2],
//                                aos[2], BOX3D)

                                ) {
                    std::cout << "Evaluation failed..." << std::endl;
                    return false;
                }
//                saveAndPlotPlots(plot_dir, CLASS_NAMES[c] + "_detection_3d", CLASS_NAMES[c], precision, 0);

                std::cout<<CLASS_NAMES[c]<<std::endl;
                std::cout<<precision->at(0)<<std::endl;
                std::cout<<precision->at(1)<<std::endl;
                std::cout<<precision->at(2)<<std::endl;

            }

        }
    }



// success
    return true;
}

std::vector<EvaluateObject::tGroundtruth> EvaluateObject::loadGroundtruth(
        autoware_auto_perception_msgs::msg::DetectedObjects ground_truth) {

    // holds all ground truth (ignored ground truth is indicated by an index vector
    std::vector<EvaluateObject::tGroundtruth> groundtruth;

    for (auto gt_object: ground_truth.objects) {
        EvaluateObject::tGroundtruth g;


        if (gt_object.classification[0].label == autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
            g.box.type = "Pedestrian";
        } else if (gt_object.classification[0].label == autoware_auto_perception_msgs::msg::ObjectClassification::CAR) {
            g.box.type = "Car";
        } else if (gt_object.classification[0].label ==
                   autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE) {
            g.box.type = "Cyclist";
        }

        tf2::Quaternion q(
                gt_object.kinematics.pose_with_covariance.pose.orientation.x,
                gt_object.kinematics.pose_with_covariance.pose.orientation.y,
                gt_object.kinematics.pose_with_covariance.pose.orientation.z,
                gt_object.kinematics.pose_with_covariance.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        g.t1 = gt_object.kinematics.pose_with_covariance.pose.position.x;
        g.t2 = gt_object.kinematics.pose_with_covariance.pose.position.y;
        g.t3 = gt_object.kinematics.pose_with_covariance.pose.position.z;

        g.l = gt_object.shape.dimensions.x;
        g.w = gt_object.shape.dimensions.y;
        g.h = gt_object.shape.dimensions.z;

        g.ry = yaw;

        groundtruth.push_back(g);
    }

    return groundtruth;
}


std::vector<EvaluateObject::tDetection> EvaluateObject::loadDetections(
        autoware_auto_perception_msgs::msg::DetectedObjects input_detections) {

//    // holds all detections (ignored detections are indicated by an index vector
    std::vector<EvaluateObject::tDetection> detections;
    for (auto gt_object: input_detections.objects) {
        EvaluateObject::tDetection g;

        if (gt_object.classification[0].label == autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
            g.box.type = "Pedestrian";
        } else if (gt_object.classification[0].label == autoware_auto_perception_msgs::msg::ObjectClassification::CAR) {
            g.box.type = "Car";
        } else if (gt_object.classification[0].label ==
                   autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE) {
            g.box.type = "Cyclist";
        }

        tf2::Quaternion q(
                gt_object.kinematics.pose_with_covariance.pose.orientation.x,
                gt_object.kinematics.pose_with_covariance.pose.orientation.y,
                gt_object.kinematics.pose_with_covariance.pose.orientation.z,
                gt_object.kinematics.pose_with_covariance.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        g.t1 = gt_object.kinematics.pose_with_covariance.pose.position.x;
        g.t2 = gt_object.kinematics.pose_with_covariance.pose.position.y;
        g.t3 = gt_object.kinematics.pose_with_covariance.pose.position.z;

        g.l = gt_object.shape.dimensions.x;
        g.w = gt_object.shape.dimensions.y;
        g.h = gt_object.shape.dimensions.z;

        g.ry = yaw;

        detections.push_back(g);
    }

    return detections;
}



bool EvaluateObject::eval_class(CLASSES current_class,
                const std::vector<tGroundtruth> &groundtruth,
                const std::vector<tDetection> &detections,
                bool compute_aos,
                std::vector<double> &precision, std::vector<double> &aos,
                METRIC metric) {

//    assert(groundtruth.size() == detections.size());

    // init
    int n_gt = 0;                                     // total no. of gt (denominator of recall)
    std::vector<double> v, thresholds;                       // detection scores, evaluated for recall discretization
    std::vector <std::vector<tGroundtruth>> dontcare;            // index of dontcare areas, included in ground truth


    // holds ignored ground truth, ignored detections and dontcare areas for current frame
    std::vector <int> i_gt, i_det;
    std::vector <tGroundtruth> dc;

    // only evaluate objects of current class and ignore occluded, truncated objects
    cleanData(current_class, groundtruth, detections, i_gt, dc, i_det, n_gt);

    dontcare.push_back(dc);

    // compute statistics to get recall values
    tPrData pr_tmp = tPrData();
    pr_tmp = computeStatistics(current_class, groundtruth, detections, dc, i_gt, i_det, false,
                               metric);

    // add detection scores to vector over all images
    for (std::size_t j = 0; j < pr_tmp.v.size(); j++)
        v.push_back(pr_tmp.v[j]);

    // get scores that must be evaluated for recall discretization
    thresholds = getThresholds(v, n_gt);

    // compute TP,FP,FN for relevant scores
    std::vector <tPrData> pr;
    pr.assign(thresholds.size(), tPrData());

    // for all scores/recall thresholds do:
    for (std::size_t t = 0; t < thresholds.size(); t++) {
        tPrData tmp = tPrData();
        tmp = computeStatistics(current_class, groundtruth, detections, dontcare[0],
                                i_gt, i_det, true, metric,
                                compute_aos, thresholds[t]);

        // add no. of TP, FP, FN, AOS for current frame to total evaluation for current threshold
        pr[t].tp += tmp.tp;
        pr[t].fp += tmp.fp;
        pr[t].fn += tmp.fn;
        if (tmp.similarity != -1)
            pr[t].similarity += tmp.similarity;
    }

    // compute recall, precision and AOS
    std::vector<double> recall;
    precision.assign(N_SAMPLE_PTS, 0);
    if (compute_aos)
        aos.assign(N_SAMPLE_PTS, 0);
    double r = 0;
    for (std::size_t i = 0; i < thresholds.size(); i++) {
        r = pr[i].tp / (double) (pr[i].tp + pr[i].fn);
        recall.push_back(r);
        precision[i] = pr[i].tp / (double) (pr[i].tp + pr[i].fp);
        if (compute_aos)
            aos[i] = pr[i].similarity / (double) (pr[i].tp + pr[i].fp);
    }

    // filter precision and AOS using max_{i..end}(precision)
    for (std::size_t i = 0; i < thresholds.size(); i++) {
        precision[i] = *max_element(precision.begin() + i, precision.end());
        if (compute_aos)
            aos[i] = *max_element(aos.begin() + i, aos.end());
    }

    // save statisics and finish with success
//    saveStats(precision, aos, fp_det, fp_ori);

    for (std::size_t i = 0; i < precision.size(); i++)
        std::cout<<" " << precision[i];
    std::cout<<std::endl;
    for (std::size_t i = 0; i < aos.size(); i++)
        std::cout<<" " << aos[i];
    std::cout<<std::endl;

    return true;
}




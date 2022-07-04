#ifndef BUILD_EVALUATE_OBJECT_HPP
#define BUILD_EVALUATE_OBJECT_HPP

#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <strings.h>
#include <assert.h>

#include <dirent.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>



BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)

typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > Polygon;

using autoware_auto_perception_msgs::msg::DetectedObjects;


class EvaluateObject {
public:

    EvaluateObject(){
        std::cout<<"Const.."<< std::endl;
    }

    enum DIFFICULTY {
        EASY = 0, MODERATE = 1, HARD = 2
    };
    enum METRIC {
        IMAGE = 0, GROUND = 1, BOX3D = 2
    };
    // evaluation parameter
    const int MIN_HEIGHT[3] = {40, 25, 25};     // minimum height for evaluated groundtruth/detections
    const int MAX_OCCLUSION[3] = {0, 1, 2};        // maximum occlusion level of the groundtruth used for evaluation
    const double MAX_TRUNCATION[3] = {0.15, 0.3,
                                      0.5}; // maximum truncation level of the groundtruth used for evaluation

    // evaluated object classes
    enum CLASSES {
        CAR = 0, PEDESTRIAN = 1, CYCLIST = 2
    };
    const int NUM_CLASS = 3;

    // parameters varying per class
    std::vector<std::string> CLASS_NAMES;
    std::vector<std::string> CLASS_NAMES_CAP;
    // the minimum overlap required for 2D evaluation on the image/ground plane and 3D evaluation
    const double MIN_OVERLAP[3][3] = {{0.7, 0.5, 0.5},
                                      {0.7, 0.5, 0.5},
                                      {0.7, 0.5, 0.5}};

    // no. of recall steps that should be evaluated (discretized)
    const double N_SAMPLE_PTS = 41;

    // initialize class names
    void initGlobals() {
        CLASS_NAMES.push_back("car");
        CLASS_NAMES.push_back("pedestrian");
        CLASS_NAMES.push_back("cyclist");
        CLASS_NAMES_CAP.push_back("Car");
        CLASS_NAMES_CAP.push_back("Pedestrian");
        CLASS_NAMES_CAP.push_back("Cyclist");
    }

    /*=======================================================================
    DATA TYPES FOR EVALUATION
    =======================================================================*/

    // holding data needed for precision-recall and precision-aos
    struct tPrData {
        std::vector<double> v;           // detection score for computing score thresholds
        double similarity;  // orientation similarity
        int tp;          // true positives
        int fp;          // false positives
        int fn;          // false negatives
        tPrData() :
                similarity(0), tp(0), fp(0), fn(0) {}
    };

    // holding bounding boxes for ground truth and detections
    struct tBox {
        std::string type;     // object type as car, pedestrian or cyclist,...
        double x1;      // left corner
        double y1;      // top corner
        double x2;      // right corner
        double y2;      // bottom corner
        double alpha;   // image orientation
        tBox(std::string type, double x1, double y1, double x2, double y2, double alpha) :
                type(type), x1(x1), y1(y1), x2(x2), y2(y2), alpha(alpha) {}
    };

    // holding ground truth data
    struct tGroundtruth {
        tBox box;        // object type, box, orientation
        double truncation; // truncation 0..1
        int occlusion;  // occlusion 0,1,2 (non, partly, fully)
        double ry;
        double t1, t2, t3;
        double h, w, l;

        tGroundtruth() :
                box(tBox("invalild", -1, -1, -1, -1, -10)), truncation(-1), occlusion(-1) {}

        tGroundtruth(tBox box, double truncation, int occlusion) :
                box(box), truncation(truncation), occlusion(occlusion) {}

        tGroundtruth(std::string type, double x1, double y1, double x2, double y2, double alpha, double truncation,
                     int occlusion) :
                box(tBox(type, x1, y1, x2, y2, alpha)), truncation(truncation), occlusion(occlusion) {}
    };

    // holding detection data
    struct tDetection {
        tBox box;    // object type, box, orientation
        double thresh; // detection score
        double ry;
        double t1, t2, t3;
        double h, w, l;

        tDetection() :
                box(tBox("invalid", -1, -1, -1, -1, -10)), thresh(-1000) {}

        tDetection(tBox box, double thresh) :
                box(box), thresh(thresh) {}

        tDetection(std::string type, double x1, double y1, double x2, double y2, double alpha, double thresh) :
                box(tBox(type, x1, y1, x2, y2, alpha)), thresh(thresh) {}
    };




    /*=======================================================================
    FUNCTIONS TO LOAD DETECTION AND GROUND TRUTH DATA ONCE, SAVE RESULTS
    =======================================================================*/
    std::vector<EvaluateObject::tDetection> loadDetections(autoware_auto_perception_msgs::msg::DetectedObjects detections);

    std::vector<EvaluateObject::tGroundtruth> loadGroundtruth(autoware_auto_perception_msgs::msg::DetectedObjects ground_truth);


    void saveStats(const std::vector<double> &precision, const std::vector<double> &aos, FILE *fp_det, FILE *fp_ori) {

        // save precision to file
        if (precision.empty())
            return;
        for (std::size_t i = 0; i < precision.size(); i++)
            fprintf(fp_det, "%f ", precision[i]);
        fprintf(fp_det, "\n");

        // save orientation similarity, only if there were no invalid orientation entries in submission (alpha=-10)
        if (aos.empty())
            return;
        for (std::size_t i = 0; i < aos.size(); i++)
            fprintf(fp_ori, "%f ", aos[i]);
        fprintf(fp_ori, "\n");
    }


    inline double imageBoxOverlap(tDetection a, tGroundtruth b, int criterion = -1) {
        return imageBoxOverlap(a.box, b.box, criterion);
    }

    inline double imageBoxOverlap(tBox a, tBox b, int32_t criterion=-1){

        // overlap is invalid in the beginning
        double o = -1;

        // get overlapping area
        double x1 = std::max(a.x1, b.x1);
        double y1 = std::max(a.y1, b.y1);
        double x2 = std::min(a.x2, b.x2);
        double y2 = std::min(a.y2, b.y2);

        // compute width and height of overlapping area
        double w = x2-x1;
        double h = y2-y1;

        // set invalid entries to 0 overlap
        if(w<=0 || h<=0)
            return 0;

        // get overlapping areas
        double inter = w*h;
        double a_area = (a.x2-a.x1) * (a.y2-a.y1);
        double b_area = (b.x2-b.x1) * (b.y2-b.y1);

        // intersection over union overlap depending on users choice
        if(criterion==-1)     // union
            o = inter / (a_area+b_area-inter);
        else if(criterion==0) // bbox_a
            o = inter / a_area;
        else if(criterion==1) // bbox_b
            o = inter / b_area;

        // overlap
        return o;
    }

// compute polygon of an oriented bounding box
    template<typename T>
    Polygon toPolygon(const T &g) {
        using namespace boost::numeric::ublas;
        using namespace boost::geometry;

        matrix<double> mref(2, 2);
        mref(0, 0) = cos(g.ry);
        mref(0, 1) = sin(g.ry);
        mref(1, 0) = -sin(g.ry);
        mref(1, 1) = cos(g.ry);

        matrix<double> corners(2, 4);
        double data[] = {g.l / 2, g.l / 2, -g.l / 2, -g.l / 2,
                         g.w / 2, -g.w / 2, -g.w / 2, g.w / 2};
        std::copy(data, data + 8, corners.data().begin());
        matrix<double> gc = prod(mref, corners);
        for (int i = 0; i < 4; ++i) {
            gc(0, i) += g.t1;
            gc(1, i) += g.t3;
        }

        double points[][2] = {{gc(0, 0), gc(1, 0)},
                              {gc(0, 1), gc(1, 1)},
                              {gc(0, 2), gc(1, 2)},
                              {gc(0, 3), gc(1, 3)},
                              {gc(0, 0), gc(1, 0)}};
        Polygon poly;
        append(poly, points);
        return poly;
    }

// measure overlap between bird's eye view bounding boxes, parametrized by (ry, l, w, tx, tz)
    inline double groundBoxOverlap(tDetection d, tGroundtruth g, int criterion = -1) {
        using namespace boost::geometry;
        Polygon gp = toPolygon(g);
        Polygon dp = toPolygon(d);

        std::vector<Polygon> in, un;
        intersection(gp, dp, in);
        union_(gp, dp, un);

        double inter_area = in.empty() ? 0 : area(in.front());
        double union_area = area(un.front());
        double o;
        if (criterion == -1)     // union
            o = inter_area / union_area;
        else if (criterion == 0) // bbox_a
            o = inter_area / area(dp);
        else if (criterion == 1) // bbox_b
            o = inter_area / area(gp);

        return o;
    }

// measure overlap between 3D bounding boxes, parametrized by (ry, h, w, l, tx, ty, tz)
    inline double box3DOverlap(tDetection d, tGroundtruth g, int criterion = -1) {
        using namespace boost::geometry;
        Polygon gp = toPolygon(g);
        Polygon dp = toPolygon(d);

        std::vector<Polygon> in, un;
        intersection(gp, dp, in);
        union_(gp, dp, un);

        double ymax = std::min(d.t2, g.t2);
        double ymin = std::max(d.t2 - d.h, g.t2 - g.h);

        double inter_area = in.empty() ? 0 : area(in.front());
        double inter_vol = inter_area * std::max(0.0, ymax - ymin);

        double det_vol = d.h * d.l * d.w;
        double gt_vol = g.h * g.l * g.w;

        double o = 1.0;
        if (criterion == -1)     // union
            o = inter_vol / (det_vol + gt_vol - inter_vol);
        else if (criterion == 0) // bbox_a
            o = inter_vol / det_vol;
        else if (criterion == 1) // bbox_b
            o = inter_vol / gt_vol;

        return o;
    }

    std::vector<double> getThresholds(std::vector<double> &v, double n_groundtruth) {

        // holds scores needed to compute N_SAMPLE_PTS recall values
        std::vector<double> t;

        // sort scores in descending order
        // (highest score is assumed to give best/most confident detections)
        sort(v.begin(), v.end(), std::greater<double>());

        // get scores for linearly spaced recall
        double current_recall = 0;
        for (std::size_t i = 0; i < v.size(); i++) {

            // check if right-hand-side recall with respect to current recall is close than left-hand-side one
            // in this case, skip the current detection score
            double l_recall;
            double r_recall;
//            double recall;
            l_recall = (double) (i + 1) / n_groundtruth;
            if (i < (v.size() - 1))
                r_recall = (double) (i + 2) / n_groundtruth;
            else
                r_recall = l_recall;

            if ((r_recall - current_recall) < (current_recall - l_recall) && i < (v.size() - 1))
                continue;

            // left recall is the best approximation, so use this and goto next recall step for approximation
//            recall = l_recall;

            // the next recall step was reached
            t.push_back(v[i]);
            current_recall += 1.0 / (N_SAMPLE_PTS - 1.0);
        }
        return t;
    }

    void cleanData(CLASSES current_class, const std::vector <tGroundtruth> &gt, const std::vector <tDetection> &det,
                   std::vector <int> &ignored_gt, std::vector <tGroundtruth> &dc, std::vector <int> &ignored_det,
                   int &n_gt) {

        // extract ground truth bounding boxes for current evaluation class
        for (std::size_t i = 0; i < gt.size(); i++) {

            // only bounding boxes with a minimum height are used for evaluation
//            double height = gt[i].box.y2 - gt[i].box.y1;

            // neighboring classes are ignored ("van" for "car" and "person_sitting" for "pedestrian")
            // (lower/upper cases are ignored)
            int valid_class;

            // all classes without a neighboring class
            if (!strcasecmp(gt[i].box.type.c_str(), CLASS_NAMES[current_class].c_str()))
                valid_class = 1;

                // classes with a neighboring class
            else if (!strcasecmp(CLASS_NAMES[current_class].c_str(), "Pedestrian") &&
                     !strcasecmp("Person_sitting", gt[i].box.type.c_str()))
                valid_class = 0;
            else if (!strcasecmp(CLASS_NAMES[current_class].c_str(), "Car") &&
                     !strcasecmp("Van", gt[i].box.type.c_str()))
                valid_class = 0;

                // classes not used for evaluation
            else
                valid_class = -1;

            // ground truth is ignored, if occlusion, truncation exceeds the difficulty or ground truth is too small
            // (doesn't count as FN nor TP, although detections may be assigned)

            // set ignored vector for ground truth
            // current class and not ignored (total no. of ground truth is detected for recall denominator)
            if (valid_class == 1) {
                ignored_gt.push_back(0);
                n_gt++;
            }
                // neighboring class, or current class but ignored
            else if (valid_class == 0 )
                ignored_gt.push_back(1);

                // all other classes which are FN in the evaluation
            else
                ignored_gt.push_back(-1);
        }

        // extract dontcare areas
        for (std::size_t i = 0; i < gt.size(); i++)
            if (!strcasecmp("DontCare", gt[i].box.type.c_str()))
                dc.push_back(gt[i]);

        // extract detections bounding boxes of the current class
        for (std::size_t i = 0; i < det.size(); i++) {

            // neighboring classes are not evaluated
            int valid_class;
            if (!strcasecmp(det[i].box.type.c_str(), CLASS_NAMES[current_class].c_str()))
                valid_class = 1;
            else
                valid_class = -1;

            if (valid_class == 1)
                ignored_det.push_back(0);
            else
                ignored_det.push_back(-1);
        }
    }

    tPrData computeStatistics(CLASSES current_class, const std::vector <tGroundtruth> &gt,
                              const std::vector <tDetection> &det, const std::vector <tGroundtruth> &dc,
                              const std::vector <int> &ignored_gt, const std::vector <int> &ignored_det,
                              bool compute_fp,
                              METRIC metric, bool compute_aos = false, double thresh = 0) {

        tPrData stat = tPrData();
        const double NO_DETECTION = -10000000;
        std::vector<double> delta;            // holds angular difference for TPs (needed for AOS evaluation)
        std::vector<bool> assigned_detection; // holds wether a detection was assigned to a valid or ignored ground truth
        assigned_detection.assign(det.size(), false);
        std::vector<bool> ignored_threshold;
        ignored_threshold.assign(det.size(),
                                 false); // holds detections with a threshold lower than thresh if FP are computed

        // detections with a low score are ignored for computing precision (needs FP)
        if (compute_fp)
            for (std::size_t i = 0; i < det.size(); i++)
                if (det[i].thresh < thresh)
                    ignored_threshold[i] = true;

        // evaluate all ground truth boxes
        for (std::size_t i = 0; i < gt.size(); i++) {

            // this ground truth is not of the current or a neighboring class and therefore ignored
            if (ignored_gt[i] == -1)
                continue;

            /*=======================================================================
            find candidates (overlap with ground truth > 0.5) (logical len(det))
            =======================================================================*/
            int det_idx = -1;
            double valid_detection = NO_DETECTION;
            double max_overlap = 0;

            // search for a possible detection
            bool assigned_ignored_det = false;
            for (std::size_t j = 0; j < det.size(); j++) {

                // detections not of the current class, already assigned or with a low threshold are ignored
                if (ignored_det[j] == -1)
                    continue;
                if (assigned_detection[j])
                    continue;
                if (ignored_threshold[j])
                    continue;

                // find the maximum score for the candidates and get idx of respective detection
                double overlap = box3DOverlap(det[j], gt[i], -1);

                // for computing recall thresholds, the candidate with highest score is considered
                if (!compute_fp && overlap > MIN_OVERLAP[metric][current_class] && det[j].thresh > valid_detection) {
                    det_idx = j;
                    valid_detection = det[j].thresh;
                }

                    // for computing pr curve values, the candidate with the greatest overlap is considered
                    // if the greatest overlap is an ignored detection (min_height), the overlapping detection is used
                else if (compute_fp && overlap > MIN_OVERLAP[metric][current_class] &&
                         (overlap > max_overlap || assigned_ignored_det) && ignored_det[j] == 0) {
                    max_overlap = overlap;
                    det_idx = j;
                    valid_detection = 1;
                    assigned_ignored_det = false;
                } else if (compute_fp && overlap > MIN_OVERLAP[metric][current_class] &&
                           valid_detection == NO_DETECTION && ignored_det[j] == 1) {
                    det_idx = j;
                    valid_detection = 1;
                    assigned_ignored_det = true;
                }
            }

            /*=======================================================================
            compute TP, FP and FN
            =======================================================================*/

            // nothing was assigned to this valid ground truth
            if (valid_detection == NO_DETECTION && ignored_gt[i] == 0) {
                stat.fn++;
            }

                // only evaluate valid ground truth <=> detection assignments (considering difficulty level)
            else if (valid_detection != NO_DETECTION && (ignored_gt[i] == 1 || ignored_det[det_idx] == 1))
                assigned_detection[det_idx] = true;

                // found a valid true positive
            else if (valid_detection != NO_DETECTION) {

                // write highest score to threshold vector
                stat.tp++;
                stat.v.push_back(det[det_idx].thresh);

                // compute angular difference of detection and ground truth if valid detection orientation was provided
                if (compute_aos)
                    delta.push_back(gt[i].box.alpha - det[det_idx].box.alpha);

                // clean up
                assigned_detection[det_idx] = true;
            }
        }

        // if FP are requested, consider stuff area
        if (compute_fp) {

            // count fp
            for (std::size_t i = 0; i < det.size(); i++) {

                // count false positives if required (height smaller than required is ignored (ignored_det==1)
                if (!(assigned_detection[i] || ignored_det[i] == -1 || ignored_det[i] == 1 || ignored_threshold[i]))
                    stat.fp++;
            }

            // do not consider detections overlapping with stuff area
            int nstuff = 0;
            for (std::size_t i = 0; i < dc.size(); i++) {
                for (std::size_t j = 0; j < det.size(); j++) {

                    // detections not of the current class, already assigned, with a low threshold or a low minimum height are ignored
                    if (assigned_detection[j])
                        continue;
                    if (ignored_det[j] == -1 || ignored_det[j] == 1)
                        continue;
                    if (ignored_threshold[j])
                        continue;

                    // compute overlap and assign to stuff area, if overlap exceeds class specific value
                    double overlap = box3DOverlap(det[j], dc[i], 0);
                    if (overlap > MIN_OVERLAP[metric][current_class]) {
                        assigned_detection[j] = true;
                        nstuff++;
                    }
                }
            }

            // FP = no. of all not to ground truth assigned detections - detections assigned to stuff areas
            stat.fp -= nstuff;

            // if all orientation values are valid, the AOS is computed
            if (compute_aos) {
                std::vector<double> tmp;

                // FP have a similarity of 0, for all TP compute AOS
                tmp.assign(stat.fp, 0);
                for (std::size_t i = 0; i < delta.size(); i++)
                    tmp.push_back((1.0 + cos(delta[i])) / 2.0);

                // be sure, that all orientation deltas are computed
                assert(tmp.size() == stat.fp + stat.tp);
                assert(delta.size() == stat.tp);

                // get the mean orientation similarity for this image
                if (stat.tp > 0 || stat.fp > 0)
                    stat.similarity = accumulate(tmp.begin(), tmp.end(), 0.0);

                    // there was neither a FP nor a TP, so the similarity is ignored in the evaluation
                else
                    stat.similarity = -1;
            }
        }
        return stat;
    }

/*=======================================================================
EVALUATE CLASS-WISE
=======================================================================*/
    bool eval_class(CLASSES current_class,
                                    const std::vector<tGroundtruth> &groundtruth,
                                    const std::vector<tDetection> &detections,
                                    bool compute_aos,
                                    std::vector<double> &precision, std::vector<double> &aos,
                                    METRIC metric);

    void saveAndPlotPlots(std::string dir_name, std::string file_name, std::string obj_type, std::vector<double> vals[], bool is_aos) {

        char command[1024];

        // save plot data to file
        FILE *fp = fopen((dir_name + "/" + file_name + ".txt").c_str(), "w");
        printf("save %s\n", (dir_name + "/" + file_name + ".txt").c_str());
        for (int i = 0; i < (int) N_SAMPLE_PTS; i++)
            fprintf(fp, "%f %f %f %f\n", (double) i / (N_SAMPLE_PTS - 1.0), vals[0][i], vals[1][i], vals[2][i]);
        fclose(fp);

        // create png + eps
        for (int j = 0; j < 2; j++) {

            // open file
            FILE *fp = fopen((dir_name + "/" + file_name + ".gp").c_str(), "w");

            // save gnuplot instructions
            if (j == 0) {
                fprintf(fp, "set term png size 450,315 font \"Helvetica\" 11\n");
                fprintf(fp, "set output \"%s.png\"\n", file_name.c_str());
            } else {
                fprintf(fp, "set term postscript eps enhanced color font \"Helvetica\" 20\n");
                fprintf(fp, "set output \"%s.eps\"\n", file_name.c_str());
            }

            // set labels and ranges
            fprintf(fp, "set size ratio 0.7\n");
            fprintf(fp, "set xrange [0:1]\n");
            fprintf(fp, "set yrange [0:1]\n");
            fprintf(fp, "set xlabel \"Recall\"\n");
            if (!is_aos) fprintf(fp, "set ylabel \"Precision\"\n");
            else fprintf(fp, "set ylabel \"Orientation Similarity\"\n");
            obj_type[0] = toupper(obj_type[0]);
            fprintf(fp, "set title \"%s\"\n", obj_type.c_str());

            // line width
            int lw = 5;
            if (j == 0) lw = 3;

            // plot error curve
            fprintf(fp, "plot ");
            fprintf(fp, "\"%s.txt\" using 1:2 title 'Easy' with lines ls 1 lw %d,", file_name.c_str(), lw);
            fprintf(fp, "\"%s.txt\" using 1:3 title 'Moderate' with lines ls 2 lw %d,", file_name.c_str(), lw);
            fprintf(fp, "\"%s.txt\" using 1:4 title 'Hard' with lines ls 3 lw %d", file_name.c_str(), lw);

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command, "cd %s; gnuplot %s", dir_name.c_str(), (file_name + ".gp").c_str());
            system(command);
        }

        // create pdf and crop
        sprintf(command, "cd %s; ps2pdf %s.eps %s_large.pdf", dir_name.c_str(), file_name.c_str(), file_name.c_str());
        system(command);
        sprintf(command, "cd %s; pdfcrop %s_large.pdf %s.pdf", dir_name.c_str(), file_name.c_str(), file_name.c_str());
        system(command);
        sprintf(command, "cd %s; rm %s_large.pdf", dir_name.c_str(), file_name.c_str());
        system(command);
    }

    bool eval(autoware_auto_perception_msgs::msg::DetectedObjects input_gt,
              autoware_auto_perception_msgs::msg::DetectedObjects input_prediction);

private:

};

#endif //BUILD_EVALUATE_OBJECT_HPP

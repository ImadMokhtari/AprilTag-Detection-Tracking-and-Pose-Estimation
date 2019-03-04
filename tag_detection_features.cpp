#include "tag_detection_features.h"



Tag_Detection_Features::
Tag_Detection_Features(){}

void Tag_Detection_Features::
Tag_Define(getopt_t *getopt,apriltag_family_t *tf,apriltag_detector_t *td)
{

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "2", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    tf = tag36h11_create();
    tf->black_border = getopt_get_int(getopt, "border");
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");
}

void Tag_Detection_Features::
Tag_Destroy(getopt_t *getopt,apriltag_family_t *tf,apriltag_detector_t *td,zarray_t *detections)
{
    zarray_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    getopt_destroy(getopt);
}



vector<Point2f> Tag_Detection_Features::
Tag_Calculate_Features(Mat gray,vector<Point2f> tag_points)
{
    vector<Point> vertex;

    for(int i=0;i<tag_points.size();i++)
        vertex.push_back(tag_points[i]);

    Mat mask(gray.rows, gray.cols, CV_8U, Scalar(0));
    fillConvexPoly(mask, vertex, Scalar(1));

    goodFeaturesToTrack( gray,
                         crns,
                         500,
                         0.4,
                         5,
                         mask,
                         3,
                         false,
                         0.04 );
    return  crns;
}


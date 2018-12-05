#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>


#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/getopt.h"


#ifndef TAG_DETECTION_FEATURES_H
#define TAG_DETECTION_FEATURES_H

using namespace std;
using namespace cv;

class Tag_Detection_Features
{
public:
    void Tag_Define(getopt_t *getopt,apriltag_family_t *tf,apriltag_detector_t *td);
    void Tag_Destroy(getopt_t *getopt,apriltag_family_t *tf,apriltag_detector_t *td,zarray_t *detections);
    vector<Point2f> Tag_Calculate_Features(Mat gray,vector<Point> tag_points);
    Tag_Detection_Features();
};

#endif // TAG_DETECTION_FEATURES_H

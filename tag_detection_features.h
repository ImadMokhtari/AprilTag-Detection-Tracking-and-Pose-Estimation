#include <iostream>
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

private:
    vector<Point2f> crns;
    int maxCorners = 500;
    double qualityLevel = 0.4;
    double minDistance = 3;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

public:
    void Tag_Define(getopt_t *getopt,apriltag_family_t *tf,apriltag_detector_t *td);
    void Tag_Destroy(getopt_t *getopt,apriltag_family_t *tf,apriltag_detector_t *td,zarray_t *detections);
    vector<Point2f> Tag_Calculate_Features(Mat gray,vector<Point2f> tag_points);
    Tag_Detection_Features();

};

#endif // TAG_DETECTION_FEATURES_H

#ifndef PDAT_H
#define PDAT_H

#include <pthread.h>
#include "tag_detection_features.h"
#include "features_tracking.h"
#include "pose_estimation.h"
#include<deque>

#include <ctime>


class pdat
{

private:
    vector<Point2f> box_edges,corners,next_corners,nedges,corners0,_next_corners_previous,_nedges_previous;
    Mat H,rotation,translation,src,frame;
    bool Find_detec, detection_start, detection_finished, prev_track_finished, end_detection;
    struct StampedImg
    {
        Mat Img;
        uint64_t ID;
    };

public:
    uint64_t CurId,previous_id,Detection_ID;
    deque<StampedImg> Previous_Imgs;
    StampedImg  current_image;

    void * detection();
    void * image_thread();
    void * tracking_current();
    void * tracking_previous();
    void pdat_start();
    pdat();

};

#endif // PDAT_H

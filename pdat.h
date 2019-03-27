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
    vector<Point2f> box_edges,corners,next_corners,nedges,corners0;
    Mat H,rotation,translation,src,frame;

    bool Find_detec=false, detection_start, detection_finished,push_current;
    bool prev_track_finished=false,track_current_init=false,current_track_init=false;
    Features_Tracking Track;
    vector<Point2f> _next_corners_previous,_nedges_previous;
    Mat detec_img;
    struct StampedImg
    {
        Mat Img;
        uint64_t ID;
    };


public:
    uint64_t previous_id=0;

    uint64_t CurId=1;
    deque<StampedImg> Previous_Imgs, Prev_detection;

    StampedImg  current_image;
    uint64_t Detection_ID;
    bool end_detection=false;

    void * detection();
    void * image_thread();

    void * image_show();
    void * tracking_current();
    void * tracking_previous();
    void * tracking_previous_img();

    void pdat_start();
    pdat();

};

#endif // PDAT_H

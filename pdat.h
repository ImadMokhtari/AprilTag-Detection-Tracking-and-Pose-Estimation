#ifndef PDAT_H
#define PDAT_H

#include <pthread.h>
#include "tag_detection_features.h"
#include "features_tracking.h"
#include "pose_estimation.h"



class pdat
{

private:
    vector<Point2f> box_edges,corners,next_corners,nedges,corners0;
    Mat current_image,H,rotation,translation,src;
    bool Find_detec;
    Features_Tracking Track;

public:
    void * detection();
    void * image_thread();
    void * tracking_thread();
    void pdat_start();
    pdat();

};

#endif // PDAT_H

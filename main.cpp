#include "features_tracking.h"
#include "tag_detection_features.h"
#include <iostream>
#include <pthread.h>
#include "apriltag.h"
#include "tag36h11.h"
#include "common/getopt.h"

using namespace cv;
using namespace std;

Mat src,src_gray,prevgray;
vector<Point> tag_points;
vector<Point2f> corners;
vector<Point2f> corners0;
vector<Point2f> next_corners;
bool Find_detec=0;

VideoCapture cap(1);

void *detectionv(void *i);
void *trackingv(void *i);

int main()
{
    int rc,rct;
    pthread_t detection,tracking;
    cap.open(1);
    rc = pthread_create(&detection, NULL, detectionv, (void*)1);
    if (rc)
    {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }

    rct = pthread_create(&tracking, NULL, trackingv, (void*)1);
    if (rct) {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }

    pthread_join(tracking,NULL);
    pthread_exit(NULL);
    return(0);
}


void *detectionv(void *i)
{

    Tag_Detection_Features Tag;
    Mat frame, gray;
    zarray_t *detections;
    apriltag_detection_t *det;
    bool detec=0;

    if (!cap.isOpened())
        cerr << "Couldn't open video capture device12" << endl;

    getopt_t *getopt = getopt_create();
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    Tag.Tag_Define(getopt,tf,td);

    while(1)
    {
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        image_u8_t im = { .width = gray.cols,
                          .height = gray.rows,
                          .stride = gray.cols,
                          .buf = gray.data  };
        detections = apriltag_detector_detect(td, &im);
        int siz=zarray_size(detections);
        for (int i = 0; i < siz; i++)
        {
            zarray_get(detections, i, &det);
            tag_points.clear();
            tag_points.push_back(Point(det->p[0][0], det->p[0][1]));
            tag_points.push_back(Point(det->p[1][0], det->p[1][1]));
            tag_points.push_back(Point(det->p[2][0], det->p[2][1]));
            tag_points.push_back(Point(det->p[3][0], det->p[3][1]));
            if(siz!=0)
            {
                Find_detec=1;
            }
        }
        if(siz==0)
        {
            detec=1;
        }

        while (detec)
        {
            cap >> frame;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            image_u8_t im = { .width = gray.cols,
                              .height = gray.rows,
                              .stride = gray.cols,
                              .buf = gray.data
                            };
            detections = apriltag_detector_detect(td, &im);
            siz=zarray_size(detections);
            for (int i = 0; i < siz; i++)
            {
                zarray_get(detections, i, &det);
                tag_points.clear();
                tag_points.push_back(Point(det->p[0][0], det->p[0][1]));
                tag_points.push_back(Point(det->p[1][0], det->p[1][1]));
                tag_points.push_back(Point(det->p[2][0], det->p[2][1]));
                tag_points.push_back(Point(det->p[3][0], det->p[3][1]));
                if(siz!=0)
                {
                    Find_detec=1;
                    detec=0;
                }
            }
        }
        corners =Tag.Tag_Calculate_Features(gray,tag_points);
        corners0.resize(corners.size());
        corners0=corners;
    }
    Tag.Tag_Destroy(getopt,tf,td,detections);
}

void *trackingv(void *i)
{
    Features_Tracking Track;
    Mat H;

    while(1)
    {
        cap>>src;
        cvtColor( src, src_gray, CV_BGR2GRAY );
        if(prevgray.empty())
            src_gray.copyTo(prevgray);
        next_corners= Track.OpticalFlow_Homograhpy(prevgray,src_gray,corners,corners0,H);
        cout<<H<<endl;
        if(Find_detec)
        {
            Track.Show_Detection(src,tag_points);
            Track.Show_Tracking(src,tag_points,H);
            Find_detec=0;
        }
            Track.Show_OpticalFlow(2,src,corners,next_corners);
            corners=next_corners;
            swap(prevgray,src_gray);
        }
}

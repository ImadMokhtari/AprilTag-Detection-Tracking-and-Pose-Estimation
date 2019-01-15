#include "features_tracking.h"
#include "tag_detection_features.h"
#include <iostream>
#include <fstream>
#include <pthread.h>
#include "apriltag.h"
#include "tag36h11.h"
#include "common/getopt.h"
#include "pose_estimation.h"

using namespace cv;
using namespace std;

Mat src_gray,prevgray;
vector<Point> tag_points;
vector<Point2f> corners,corners0,next_corners;
bool Find_detec=0,flag,not_detect=false,begin_detect=true;;
int size_c;
int siz;
VideoCapture cap;

void *detectionv(void *i);
void *trackingv(void *i);

int main()
{
    int rc,rct;
    pthread_t detection,tracking;
    rc = pthread_create(&detection, NULL, detectionv, (void*)1);
    cap.open(0);
    if (!cap.isOpened())
    {
        cout<<"cap can't be opened\n";
        exit(-1);
    }
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

vector<Point2f> corners1,box_edges;
void *detectionv(void *i)
{

    Tag_Detection_Features Tag;
    Mat  gray,frame;
    zarray_t *detections;
    apriltag_detection_t *det;
    bool detec=0;

    getopt_t *getopt = getopt_create();
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    Tag.Tag_Define(getopt,tf,td);

    while(1)
    {
        detec=1;
        if(begin_detect)
        {
            while (detec)
            {
                cap>>frame;
                if(!frame.empty())
                {
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

                        box_edges.clear();
                        box_edges.push_back(Point2f(det->p[0][0], det->p[0][1]));
                        box_edges.push_back(Point2f(det->p[1][0], det->p[1][1]));
                        box_edges.push_back(Point2f(det->p[2][0], det->p[2][1]));
                        box_edges.push_back(Point2f(det->p[3][0], det->p[3][1]));

                        if(siz!=0)
                        {
                            Find_detec=1;
                            detec=0;
                        }

                        else
                            detec=1;
                    }
                    if(Find_detec){
                        corners1 =Tag.Tag_Calculate_Features(gray,tag_points);
                        not_detect=false;
                    }
                    else
                    {
                        corners.clear();
                        corners0.clear();
                        not_detect=true;
                    }
                    size_c=corners1.size();
                    corners.resize(size_c);

                    corners=corners1;
                    corners0.resize(corners.size());
                    corners0=corners;
                    flag=true;
                }
                else
                    break;

                begin_detect=false;
            }
        }
    }
    Tag.Tag_Destroy(getopt,tf,td,detections);
}

vector<Point2f> nedges;
void *trackingv(void *i)
{
 //   ofstream myfile_detections,translate,translatedetect;
    vector<Mat>next_edges;
    Features_Tracking Track;
    Pose_Estimation pose;
    Mat H,rotation,translation,src;
    vector<Point2f> corners_t,corners0_t;
    vector<Point3f> camera_pose;

   // translate.open("/home/imad/Desktop/Mini_projet/evaluation/translation.csv");
    //translatedetect.open("/home/imad/Desktop/Mini_projet/evaluation/translationdetection.csv");

    int n=1,nf;
    while(1)
    {
        nf=0;
        while(nf!=n)
        {
            nf++;
            cap>>src;
            if(!src.empty())
            {
                if(flag)
                {
                    corners_t.resize(size_c);
                    corners_t=corners;
                    corners0_t.resize(size_c);
                    corners0_t=corners0;
                    next_corners.resize(size_c);
                    flag=false;
                }
                cvtColor( src, src_gray, CV_BGR2GRAY );
                if(prevgray.empty())
                    src_gray.copyTo(prevgray);

                if(!not_detect)
                {
                    next_corners= Track.OpticalFlow_Homograhpy(prevgray,src_gray,corners_t,corners0_t,H);
                    nedges=Track.OpticalFlow_tracking_box(src,prevgray,src_gray,box_edges);
                    if(box_edges.size()>0)
                    {
                        Track.Show_OpticalFlow(2,src,corners_t,next_corners);
                        corners_t.resize(next_corners.size());

                        camera_pose=pose.using_solvepnp(src,box_edges,rotation,translation);
                        pose.show_pose_xyz(src,translation);
                        pose.show_pose_rotation(src,rotation);
                        //translation from the algorithme
                        //translate<<translation.at<double>(0,0)<<","<<translation.at<double>(0,1)<<","<<translation.at<double>(0,2)<<","<<endl;
                        if(Find_detec)
                        {
                            //translation from the detection
                          //  translatedetect<<translation.at<double>(0,0)<<","<<translation.at<double>(0,1)<<","<<translation.at<double>(0,2)<<","<<endl;
                            Track.Show_Detection(src,tag_points);
                            Find_detec=0;
                        }
                        not_detect=false;
                    }
                    //else
                    //  translate<<0<<","<<0<<","<<0<<","<<endl;
                }
                else
                {
                    corners_t.clear();
                    corners0_t.clear();
                    next_corners.clear();
                    nedges.clear();
                }
                box_edges=nedges;
                corners_t=next_corners;
                namedWindow( "OpticalFlow", CV_WINDOW_AUTOSIZE);
                imshow( "OpticalFlow", src );
                waitKey(1);
                swap(prevgray,src_gray);
            }
            else
                break;
        }
        begin_detect=true;
    }
  //  translate.close();
   // translatedetect.close();
}

#include "features_tracking.h"
#include "tag_detection_features.h"
#include <iostream>
#include <fstream>
#include <pthread.h>
#include "apriltag.h"
#include "tag36h11.h"
#include "common/getopt.h"
#include "pose_estimation.h"
#include <mutex>
#include <ctime>


using namespace cv;
using namespace std;

Mat src_gray,prevgray;
vector<Point> tag_points;
vector<Point2f> corners,corners0,next_corners;
bool Find_detec=0,flag,not_detect=false,begin_detect=true;;
int size_c;
mutex cap_mutex;
int siz;


int ni=1,nii=ni;
int nf=260;

void *detectionv(void *i);
void *trackingv(void *i);



struct StampedImg
{
    Mat Img;
    uint64_t ID;
};

deque<StampedImg> Imgs_vector;
StampedImg image;
uint64_t CurId = 1;


int main()
{
    int rc,rct;
    pthread_t detection,tracking;


    for(int i=ni;i<=nf;i++)
    {
        auto result ="/home/imad/Desktop/video_pfe/images_pc5*/" +to_string(i)+".png";
        image.Img = imread(result);
        image.ID = CurId;
        Imgs_vector.push_back(image);
        CurId++;
        cout<<"vector im size  "<< Imgs_vector.size() << endl;
    }
    /*
    for(int i=0; i <Imgs_vector.size(); i++)
    {
        imshow("pdat",Imgs_vector[i].Img);
        waitKey(30);
        cout<<"ID is "<<Imgs_vector[i].ID<<endl;
    }*/

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

bool Detection_finished=false;

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
    int k=1;

    while(1)
    {
        //  usleep(33333);
        //    cout<<"detec  "<<k++<<endl;
        detec=1;
        if(begin_detect)
        {
            while (detec)
            {
                //cap_mutex.lock();
              //  frame=image[ni-nii].clone();
                Imgs_vector[ni-nii].Img.copyTo(frame);
                //cap_mutex.unlock();
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
                        not_detect=true;

                        corners.clear();
                        corners0.clear();
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
                Detection_finished=true;
                begin_detect=false;
            }
        }
    }
    Tag.Tag_Destroy(getopt,tf,td,detections);
}

vector<Point2f> nedges;
void *trackingv(void *i)
{
    ofstream Time,myfile_detections,translate,translatedetect;
    vector<Mat>next_edges;
    Pose_Estimation pose;
    Mat H,rotation,translation,src;
    vector<Point2f> corners_t,corners0_t;
    vector<Point3f> camera_pose;
    clock_t start,end;
    Features_Tracking Track;
/*
    translate.open("/home/imad/Desktop/video_pfe/evaluation results/Tracking/translation.csv");
    translatedetect.open("/home/imad/Desktop/video_pfe/evaluation results/Tracking/translationdetection.csv");
    Time.open("/home/imad/Desktop/video_pfe/evaluation results/Tracking/Time.csv");
  */
    // PDAT
    translate.open("/home/imad/Desktop/video_pfe/evaluation results/PDAT/translation.csv");
    translatedetect.open("/home/imad/Desktop/video_pfe/evaluation results/PDAT/translationdetection.csv");
    Time.open("/home/imad/Desktop/video_pfe/evaluation results/PDAT/Time.csv");

  int n=4,nf;

    while(1)
    {
            nf=0;
            if(Detection_finished)
                while(nf!=n)
                {
                    Imgs_vector[ni-nii].Img.copyTo(src);

                    waitKey(1);

                    nf++;
                    start=clock();

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

                        if(!not_detect )
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

                                translate<<Imgs_vector[ni-nii].ID<<","<<translation.at<double>(0,0)<<","<<translation.at<double>(0,1)<<","<<translation.at<double>(0,2)<<","<<endl;
                                if(Find_detec)
                                {
                                    translatedetect<<translation.at<double>(0,0)<<","<<translation.at<double>(0,1)<<","<<translation.at<double>(0,2)<<","<<endl;
                                    Track.Show_Detection(src,tag_points);
                                    Find_detec=0;
                                }
                                not_detect=false;
                            }
                            else
                                translate<<Imgs_vector[ni-nii].ID<<","<<0<<","<<0<<","<<0<<","<<endl;
                        }
                        else
                        {
                            translate<<Imgs_vector[ni-nii].ID<<","<<0<<","<<0<<","<<0<<","<<endl;

                            corners_t.clear();
                            corners0_t.clear();
                            next_corners.clear();
                            nedges.clear();
                        }
                        box_edges=nedges;
                        corners_t=next_corners;
                        end=clock()-start;

                        imshow( "OpticalFlow", src );
                        //  auto result ="/home/imad/Desktop/Mini_projet/src/sequence/13 MASTER Algo/" +to_string( ni )+".png";
                        //  imwrite(result,src);
                        waitKey(15);
                        swap(prevgray,src_gray);
                        ni++;
                    }
                    end=clock()-start;
                    Time<<2.5<<','<<(float)end/CLOCKS_PER_SEC<<','<<endl;
                }
        begin_detect=true;

    }
    Time.close();
    translate.close();
    translatedetect.close();
}

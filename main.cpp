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
bool Find_detec=0,flag,not_detect=false,begin_detect=true;
int size_c;
mutex cap_mutex;
int siz;


void *detectionv(void *i);
void *trackingv(void *i);

int frame_number=260;
int iff=1;   //index first frame.


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
    int rc,rct,i=1;
    pthread_t detection,tracking;
    for(int i=iff;i<=frame_number;i++)
    {
        auto result ="/home/imad/Desktop/video_pfe/images_pc5*/" +to_string(i)+".png";
        image.Img = imread(result);
        image.ID = CurId;
        Imgs_vector.push_back(image);
        CurId++;
        cout<<"vector im size  "<< Imgs_vector.size() << endl;
    }

    rc = pthread_create(&detection, NULL, detectionv, (void*)1);
    if (rc)
    {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }
    pthread_join(detection,NULL);
    pthread_exit(NULL);
    return(0);
}

vector<Point2f> corners1,box_edges;
void *detectionv(void *i)
{
    clock_t start_time,end_time;
    Features_Tracking Track;
    Pose_Estimation pose;
    Mat rotation,translation;
    vector<Point3f> camera_pose;
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
    ofstream myfile_detections,translate,myfile_indexNotDetected_time;
    myfile_detections.open("/home/imad/Desktop/video_pfe/evaluation results/Detection/translation.csv");
    myfile_indexNotDetected_time.open("/home/imad/Desktop/video_pfe/evaluation results/Detection/Time.csv");
    int k=0;
    int nff=iff-2;
    while(1)
    {
        nff++;


        detec=1;

        while (detec)
        {
            // usleep(150000);
            //frame= image[nff].clone();
            Imgs_vector[k].Img.copyTo(frame);

            start_time=clock();

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
                    Track.Show_Detection(frame,tag_points);

                }
                if(siz!=0)
                {
                    Find_detec=1;
                    detec=0;

                    camera_pose=pose.using_solvepnp(frame,box_edges,rotation,translation);

                    end_time=clock()-start_time;

                    myfile_detections<<Imgs_vector[k].ID<<","<<translation.at<double>(0,0)<<","<<translation.at<double>(0,1)<<","<<translation.at<double>(0,2)<<","<<endl;
                    cout<<"nf= "<<nff<<endl;

                    myfile_indexNotDetected_time<<2.5<<','<<(float)end_time/CLOCKS_PER_SEC<<','<<endl;
                }

                else
                {
                    myfile_detections<<Imgs_vector[k].ID<<","<<0<<","<<0<<","<<0<<","<<endl;
                    cout<<"nf111= "<<nff<<endl;

                    myfile_indexNotDetected_time<<nff<<','<<0<<endl;
                    detec=0;

                }

                if(Find_detec)
                {
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
            {
                cout<<"frame empty"<<endl;
                //break;
            }
            begin_detect=true;
        }
        k++;


    }
    myfile_detections.close();
    myfile_indexNotDetected_time.close();

    Tag.Tag_Destroy(getopt,tf,td,detections);
}

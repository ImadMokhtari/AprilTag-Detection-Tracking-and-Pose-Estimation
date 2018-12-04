#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<opencv2/videoio.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/video/tracking.hpp>
#include<time.h>

#include "features_tracking.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/getopt.h"
#include<stack>

using namespace cv;
using namespace std;
#include <sstream>
stack<Mat> pic;
int argc;
char** argv;
int edge11x=0,edge11y=0,edge12x=0,edge21y=0,px1,py1,px2,py2,px3,py3,px4,py4;
Mat src,src_gray,prevgray;
vector<Point2f> corners;
vector<Point2f> corners0;
vector<Point2f> next_corners;
bool Find_detec=0;
//Good features to track parameters
int maxCorners = 1000;
double qualityLevel = 0.3;
double minDistance = 1;
int blockSize = 3;
bool useHarrisDetector = false;
double k = 0.04;

//Mat copy;
// Initialize camera
cv:: VideoCapture cap(0);

bool D_tra=0;
int retard=0;
RNG rng(12345);//random numbers for circles color

void *detectionv(void *i);
void *trackingv(void *i);
void *image(void *i);
void show_OpticalFlow(int r , Mat src , vector<Point2f> corners , vector<Point2f> nextcorners);


int main()
{
    std::ostringstream oss;
        int Imgnum=1; //Picture number
        string  Folder="/home/med/Bureau/image1/t"; //chemin des images
        string  Suffix=".jpg";    //picture extansion
        string  FileName;
     cap.open(0);
    pthread_t detection;
    pthread_t tracking;
    pthread_t capture;
    int rc,rct;

   rc = pthread_create(&detection, NULL, detectionv, (void*)1);
    if (rc) {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }

    do{
        retard=0;
    }while (D_tra!=1);


    rct = pthread_create(&tracking, NULL, trackingv, (void*)1);
    if (rct) {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }
    pthread_join(tracking,NULL);

    //pthread_join(detection,NULL);

    return(0);
    pthread_exit(NULL);
}

void *image(void *i)

    {
        Mat image;


        if(cap.isOpened())
        {
            cout << "Capture is opened" << endl;
        }
        else
        {
            cout << "No capture" << endl;
            image = Mat::zeros(480, 640, CV_8UC1);
            waitKey(0.1);
            exit;
        }

D_tra=1;
        while ( 1 )
        {  clockid_t tstart=clock();
            cap >> image;
            pic.push(image);
            if(image.empty())
               break;
            if(waitKey(10) >= 0)
                break;
     }
}

void *detectionv(void *i)

{
    vector<Point> tag_points;

    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device12" << endl;
    }
    cout<<"debut"<<endl;

        getopt_t *getopt = getopt_create();

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

        if (!getopt_parse(getopt, argc, argv, 1) ||
                getopt_get_bool(getopt, "help")) {
            printf("Usage: %s [options]\n", argv[0]);
            getopt_do_usage(getopt);
            exit(0);
        }


        // Initialize tag detector with options
        apriltag_family_t *tf = NULL;
        const char *famname = getopt_get_string(getopt, "family");
        if (!strcmp(famname, "tag36h11"))
            tf = tag36h11_create();
        else if (!strcmp(famname, "tag36h10"))
            tf = tag36h10_create();
        else if (!strcmp(famname, "tag36artoolkit"))
            tf = tag36artoolkit_create();
        else if (!strcmp(famname, "tag25h9"))
            tf = tag25h9_create();
        else if (!strcmp(famname, "tag25h7"))
            tf = tag25h7_create();
        else {
            printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
            exit(-1);
        }
        tf->black_border = getopt_get_int(getopt, "border");

        apriltag_detector_t *td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
        td->quad_decimate = getopt_get_double(getopt, "decimate");
        td->quad_sigma = getopt_get_double(getopt, "blur");
        td->nthreads = getopt_get_int(getopt, "threads");
        td->debug = getopt_get_bool(getopt, "debug");
        td->refine_edges = getopt_get_bool(getopt, "refine-edges");
        td->refine_decode = getopt_get_bool(getopt, "refine-decode");
        td->refine_pose = getopt_get_bool(getopt, "refine-pose");

        std::vector<cv::Point2f> imagePoints;
        Mat frame, gray;
        zarray_t *detections;
        apriltag_detection_t *det;
        Size s;
        bool detec=0;


        while(1)
        {

                cout<<"***********************\n";
                cout<<"here0"<<endl;
                cap >> frame;
                cvtColor(frame, gray, COLOR_BGR2GRAY);
                image_u8_t im = { .width = gray.cols,
                                  .height = gray.rows,
                                  .stride = gray.cols,
                                  .buf = gray.data  };
                detections = apriltag_detector_detect(td, &im);
                int siz=zarray_size(detections);
                cout<<"here1"<<endl;

                for (int i = 0; i < siz; i++)
                {
                    zarray_get(detections, i, &det);
                    tag_points.clear();
                    tag_points.push_back(Point(det->p[0][0], det->p[0][1]));
                    tag_points.push_back(Point(det->p[1][0], det->p[1][1]));
                    tag_points.push_back(Point(det->p[2][0], det->p[2][1]));
                    tag_points.push_back(Point(det->p[3][0], det->p[3][1]));
                    px1=det->p[0][0];  py1=det->p[0][1];
                    px2=det->p[1][0];  py2=det->p[1][1];
                    px3=det->p[2][0];  py3=det->p[2][1];
                    px4=det->p[3][0];  py4=det->p[3][1];

                    if(siz!=0)
                    {
                        Find_detec=1;
                    }
                }
                cout<<"here2"<<endl;

                if(siz==0)
                {
                    detec=1;
                }
                cout<<"here3"<<endl;

                while (detec)
                {
                    cout<<"here4"<<endl;

                    cap >> frame;
                    cvtColor(frame, gray, COLOR_BGR2GRAY);
                    image_u8_t im = { .width = gray.cols,
                        .height = gray.rows,
                        .stride = gray.cols,
                        .buf = gray.data};
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
                        px1=det->p[0][0];  py1=det->p[0][1];
                        px2=det->p[1][0];  py2=det->p[1][1];
                        px3=det->p[2][0];  py3=det->p[2][1];
                        px4=det->p[3][0];  py4=det->p[3][1];

                        if(siz!=0)
                        {
                            Find_detec=1;
                        }
                        if(siz!=0)
                        {
                            detec=0;
                        }
                    }
                }

                cout<<"here5"<<endl;

                Mat mask(frame.rows, frame.cols, CV_8U, Scalar(0));
                fillConvexPoly(mask, tag_points, Scalar(1));
                vector<Point2f> crns;
                goodFeaturesToTrack( gray,
                                     crns,
                                     maxCorners,
                                     qualityLevel,
                                     minDistance,
                                     mask,
                                     blockSize,
                                     useHarrisDetector,
                                     k );
                corners.resize(crns.size());
                corners0.resize(corners.size());
                corners=crns;
                corners0=corners;
                zarray_destroy(detections);
                D_tra=1;

        }
     //   zarray_destroy(detections);
        cout<<"fin detection"<<endl;
        apriltag_detector_destroy(td);
        if (!strcmp(famname, "tag36h11"))
            tag36h11_destroy(tf);
        else if (!strcmp(famname, "tag36h10"))
            tag36h10_destroy(tf);
        else if (!strcmp(famname, "tag36artoolkit"))
            tag36artoolkit_destroy(tf);
        else if (!strcmp(famname, "tag25h9"))
            tag25h9_destroy(tf);
        else if (!strcmp(famname, "tag25h7"))
            tag25h7_destroy(tf);
        getopt_destroy(getopt);
}


void *trackingv(void *i)
{
        Features_Tracking Track;

        cap>>src;
        cvtColor( src, src_gray, CV_BGR2GRAY);
        while(1)
        {
                cap>>src;
                cvtColor( src, src_gray, CV_BGR2GRAY );
                if(prevgray.empty()) src_gray.copyTo(prevgray);
                next_corners= Track.OpticalFlow_Homograhpy(prevgray,src_gray,corners,corners0);
               // Track.Show_OpticalFlow(2,src,corners,next_corners);
                for( int i = 0; i < next_corners.size(); i++ )
                {

                    arrowedLine(src,           //picture where to draw
                                corners[i],     //begin arrow points
                                next_corners[i], //end arrow points
                                Scalar(0,0,220),//arrow color
                                2,              //Arrow thickness
                                8,              //line Type
                                0,              //int shift
                                0.2);           //Arrow head length


                    circle( src,
                            next_corners[i],
                            2,
                            Scalar(0,255,0),
                            2,
                            8,
                            0 );
                }
                if(Find_detec)
                {
                line(src,Point(px1,py1),Point(px2,py2),Scalar(255,255,0),2);
                line(src,Point(px2,py2),Point(px3,py3),Scalar(255,255,0),2);
                line(src,Point(px3,py3),Point(px4,py4),Scalar(255,255,0),2);
                line(src,Point(px4,py4),Point(px1,py1),Scalar(255,255,0),2);
                Find_detec=0;
                }

                cout<<"display"<<endl;
                namedWindow( "OpticalFlow", CV_WINDOW_AUTOSIZE );
                imshow( "OpticalFlow", src );
                waitKey(1);

                corners=next_corners;
                swap(prevgray,src_gray);



        }
}

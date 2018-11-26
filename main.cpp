#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<opencv2/videoio.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/video/tracking.hpp>
#include<time.h>


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

//Good features to track parameters
int maxCorners = 1000;
double qualityLevel = 0.3;
double minDistance = 1;
int blockSize = 3;
bool useHarrisDetector = false;
double k = 0.04;

Mat copy;
// Initialize camera
cv:: VideoCapture cap(0);

bool D_tra=0;
int retard=0;
RNG rng(12345);//random numbers for circles color


void *detectionv(void *i);
void *trackingv(void *i);
void *image(void *i);


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
   /* rc = pthread_create(&capture, NULL, image, (void*)1);
cout<<"sleep bedore"<<endl;
    sleep(4);
cout<<"sleep 10"<<endl;

    while(1){
    oss.str("");
                oss<<Folder<<Imgnum<<Suffix; //Picture target
                FileName=oss.str();
                cout<<FileName<<endl;
                if(pic.empty())
                {cout<<"empty"<<endl;}
                if(!pic.empty()) {
                  imwrite(FileName,pic.top());     // verifier si la pile est vide
                  pic.pop();
                Imgnum=Imgnum+1;}

                }*/
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
            //drawText(image);
            //imshow("Sample", image);
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
            //drawText(image);
         // imshow("Sample", image);
            if(waitKey(10) >= 0)
                break;
cout<<"capture time="<<(double)(clock() - tstart)/CLOCKS_PER_SEC<<endl;
        }




    }

void *detectionv(void *i)

{
    vector<Point> tag_points;

    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device12" << endl;
    }
    cout<<"debut"<<endl;
   // while(1)
    //{
        getopt_t *getopt = getopt_create();

        getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
        getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
        getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
        getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
        getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
        getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
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

        int Cs;
        int a=0;
        int detection11=0;
        while(1)
        {if(D_tra==1)
            {    a=0;
            }
            else{


        clockid_t   tstart=clock();
        cap >> frame;
        cout<<"capture detection"<<endl;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        image_u8_t im = { .width = gray.cols,
                          .height = gray.rows,
                          .stride = gray.cols,
                          .buf = gray.data  };
        cout<<"detectio 1"<<endl;
        detection11++;

cout<<"detectio11="<<detection11<<endl;
        detections = apriltag_detector_detect(td, &im);
        cout<<"detectio 2"<<endl;
        // Draw detection outlines
int siz=zarray_size(detections);
        for (int i = 0; i < siz; i++) {
            //apriltag_detection_t *det;   it was declaratedoutside the loop.
            cout<<"detectio 3"<<endl;
            zarray_get(detections, i, &det);
cout<<"detectio 4"<<endl;
            //Detected apriltag Edges
tag_points.clear();
tag_points.push_back(Point(det->p[0][0], det->p[0][1]));
tag_points.push_back(Point(det->p[1][0], det->p[1][1]));
tag_points.push_back(Point(det->p[2][0], det->p[2][1]));
tag_points.push_back(Point(det->p[3][0], det->p[3][1]));
           /* edge11x=det->p[3][0];
            edge11y=det->p[3][1];
            edge12x=det->p[2][0];
            edge21y=det->p[0][1];*/

}
        cout<<"detection not while"<<endl;
//vector<Point> tag_points;
//        /edge11x<=0 || edge11y<=0 || edge12x<=0 || edge21y<=0
        if(siz==0)
        {detec=1;}

        while (detec)
       {cout<<"detection while"<<endl;
 cout<<"zarra size="<<zarray_size(detections)<<endl;

 cap >> frame;

            cvtColor(frame, gray, COLOR_BGR2GRAY);

            // Make an image_u8_t header for the Mat data
            image_u8_t im = { .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
            };

            detections = apriltag_detector_detect(td, &im);
             siz=zarray_size(detections);
            // Draw detection outlines
            for (int i = 0; i < siz; i++) {
                //apriltag_detection_t *det;   it was declaratedoutside the loop.
                zarray_get(detections, i, &det);
/*
                line(frame, Point(det->p[0][0], det->p[0][1]),
                         Point(det->p[1][0], det->p[1][1]),
                         Scalar(0, 0xff, 0), 2);
                line(frame, Point(det->p[0][0], det->p[0][1]),
                         Point(det->p[3][0], det->p[3][1]),
                         Scalar(0, 0, 0xff), 2);
                line(frame, Point(det->p[1][0], det->p[1][1]),
                         Point(det->p[2][0], det->p[2][1]),
                         Scalar(0xff, 0, 0), 2);
                line(frame, Point(det->p[2][0], det->p[2][1]),
                         Point(det->p[3][0], det->p[3][1]),
                         Scalar(0xff, 0, 0), 2);

               /* stringstream ss;
                ss << det->id;
                String text = ss.str();
                int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                Size textsize = getTextSize(text, fontface, fontscale, 2,
                                                &baseline);
                putText(frame, text, Point(det->c[0]-textsize.width/2,
                                           det->c[1]+textsize.height/2),
                        fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
                */

            //    vector<Point> tag_points;
                tag_points.clear();
                tag_points.push_back(Point(det->p[0][0], det->p[0][1]));
                tag_points.push_back(Point(det->p[1][0], det->p[1][1]));
                tag_points.push_back(Point(det->p[2][0], det->p[2][1]));
                tag_points.push_back(Point(det->p[3][0], det->p[3][1]));
if(siz!=0)
{detec=0;}

  /*              //Find the corresponding ROI based on the poitns
                convexHull(tag_points, tag_points);                         //to assure correct point order
                Mat mask(frame.rows, frame.cols, CV_8U, Scalar(0));  //black image
                fillConvexPoly(mask, tag_points, Scalar(1));
*/

                //Detected apriltag Edges
               /* edge11x=det->p[3][0];
                edge11y=det->p[3][1];
                edge12x=det->p[2][0];
                edge21y=det->p[0][1];
                //sprintf("%d  %d  %d  %d \n",edge11x,edge11y,edge12x,edge21y);*/

            }

            cout<<"no tag detected"<<endl;
          //  zarray_destroy(detections);




        }
        /*edge11x=0;
        edge11y=0;
        edge12x=0;
        edge21y=0;*/
     //   printf("mazel1");
       // printf("sortir de la boucle");
        //Declaration of the mask for Good features to track algorithm
        //s=frame.size();
        cout<<tag_points;//src

        //Mat mask(s,CV_8U,Scalar(0));
        //mask(Rect(edge11x+1,edge11y+1,edge12x-edge11x+1,edge21y-edge11y+1))=1;
        //Find the corresponding ROI based on the poitns
        //convexHull(tag_points, tag_points);
        cout<<"convxhull"<<endl;
        //to assure correct point order
        Mat mask(frame.rows, frame.cols, CV_8U, Scalar(0));
       // cout<<mask;//black image
        fillConvexPoly(mask, tag_points, Scalar(1));
       // cout<<mask;
/*
        while (!tag_points.empty())
          {
            //sum=sum+tag_points.back();
            tag_points.pop_back();
          }
*/
        //end of the mask Declaration
        goodFeaturesToTrack( gray,  //src_gray
                             corners,
                             maxCorners,
                             qualityLevel,
                             minDistance,
                             mask,
                             blockSize,
                             useHarrisDetector,
                             k );
cout<<"features"<<endl;
        //First features detectedminEigThreshold

       // corners0=corners;
       // printf("mazel");
        Cs=corners.size();
        if(Cs>0){
            corners0.resize(corners.size());
            // corners0=corners;
            for (int i=0;i<=corners0.size();i++){
                corners0[i].x=corners[i].x;
                corners0[i].y=corners[i].y;
                //printf("x=%f  y=%f \n",corners0[i].x,corners0[i].y);
            }
        }


        D_tra=1;
        cout<<"time taken detection"<<(double)(clock() - tstart)/CLOCKS_PER_SEC<<endl;
        //printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC)
            }
        }
        zarray_destroy(detections);

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

    //}

}

void *trackingv(void *i)
{
    {
        cout<<"debut tracking"<<endl;
        int r = 3;//radius of circles of features

        Mat copy;
        vector<Point2f> nextcorners;
        // vector<Point2f> OutLiersMask;

        vector<int> OutLiersMask;

        vector<uchar> status;
        vector<float> err;
        TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
        Size  winSize(31,31);


        Mat Homography;
     /*   for (int i=0;i<=corners.size();i++){
            corners0[i].x=corners[i].x;
            corners0[i].y=corners[i].y;
        }
*/
        //First frame to detect
        cap>>src;
        cvtColor( src, src_gray, CV_BGR2GRAY );
        copy = src.clone();
        cout<<"before loop"<<endl;

        //if there in no features,features=1
        if( maxCorners < 1 ) { maxCorners = 1; }

      //  if (edge12x>0 && edge11x>0 && edge21y>0 && edge11y> 00)
        //{
            //begin of tracking
            for(;;)
               {int Nf=5;
                int Nframe=0;
                while(Nframe!=Nf)
                {Nframe++;
                    clock_t tstart2=clock();
                cap>>src;
                cout<<"capture tracking"<<endl;
                cvtColor( src, src_gray, CV_BGR2GRAY ); //RGB to Gray
                //copy = src.clone(); //Copy of the original image

                if(prevgray.empty()) //first etiration
                    src_gray.copyTo(prevgray);


                cout<<"calculopticflow"<<endl;
                if(corners.size()>3 )//&& nextcorners.size()>0)
                 {
                     calcOpticalFlowPyrLK(prevgray,   //first frame
                                     src_gray,   //next frame
                                     corners,    //coordinates of the features to track
                                     nextcorners,//coordiantes of the features in the next frame
                                     status,     //vector of status of the detection(0:detected failed ; 1:detected with success)
                                     err,        //error of detection
                                     winSize,    //size of the search window at each pyramid level
                                     3,          //0-based maximal pyramid level number; if set to 0, pyramids are not used (single level), if set to 1, two levels are used, and so on
                                     termcrit,   //parameter, specifying the termination criteria of the iterative search algorithm (after the specified maximum number of iterations
                                     0,          //flags
                                     0.001);       //minEigThreshold


                Homography = findHomography (corners0,
                                             nextcorners,
                                             RANSAC,         //method to use
                                             10,
                                             OutLiersMask,   //OutputArray mask
                                             1000,           //const int maxIters
                                             0.995 );
                }//const double 	confidence

                int j=0;
                for (int i=0;i<OutLiersMask.size();i++)
                {
                    // printf("%d\n",OutLiersMask[i]);

                    if (OutLiersMask[i])
                    {
                        nextcorners[j]=nextcorners[i];
                        corners[j]=corners[i];
                        corners0[j]=corners0[i];
                        j++;

                    }

                }

                nextcorners.resize(j);
                corners.resize(j);
                corners0.resize(j);


                for( int i = 0; i < corners.size(); i++ )
                {
                    arrowedLine(src,           //picture where to draw
                                corners[i],     //begin arrow points
                                nextcorners[i], //end arrow points
                                Scalar(0,0,220),//arrow color
                                2,              //Arrow thickness
                                8,              //line Type
                                0,              //int shift
                                0.2);           //Arrow head length


                    circle( src,nextcorners[i],
                            r,
                            Scalar(0,255,0),
                            2,
                            8,
                            0 );

                }
                cout<<"dislay"<<endl;
                //Display
                namedWindow( "OpticalFlow", CV_WINDOW_AUTOSIZE );
                imshow( "OpticalFlow", src );

                corners=nextcorners;
                swap(prevgray,src_gray);
                waitKey(1);
               //cout<<"time taken tracking"<<(double)(clock() - tstart2)/CLOCKS_PER_SEC<<endl;
                }

               D_tra=0;
               while(D_tra==0)
               {retard=0;}


        }
    }
}

#include "pdat.h"

pdat::pdat(){}

void * pdat::
image_thread()
{
    VideoCapture cap(1);
    if(cap.isOpened())
        while(1)
            cap >> current_image;

    return NULL;
}

void * pdat::
detection()
{
    Tag_Detection_Features Tag;
    Mat  gray,frame;
    zarray_t *detections;
    apriltag_detection_t *det;
    int detected_tags_number;
    getopt_t *getopt = getopt_create();
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    Tag.Tag_Define(getopt,tf,td);

    while(1)
    {
        if(!current_image.empty())
        {
            frame=current_image.clone();
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            image_u8_t im = { .width = gray.cols,
                              .height = gray.rows,
                              .stride = gray.cols,
                              .buf = gray.data};

            detections = apriltag_detector_detect(td, &im);
            detected_tags_number=zarray_size(detections);
            if(detected_tags_number != 0)
                for (int i = 0; i < detected_tags_number; i++)
                {
                    zarray_get(detections, i, &det);
                    box_edges.clear();
                    box_edges.push_back(Point2f(det->p[0][0], det->p[0][1]));
                    box_edges.push_back(Point2f(det->p[1][0], det->p[1][1]));
                    box_edges.push_back(Point2f(det->p[2][0], det->p[2][1]));
                    box_edges.push_back(Point2f(det->p[3][0], det->p[3][1]));

                    corners =Tag.Tag_Calculate_Features(gray,box_edges);
                    corners0=corners;
                    Find_detec=true;
                }
            else
            {
                box_edges.clear();
                corners.clear();
                Find_detec=false;
            }
        }
    }
    Tag.Tag_Destroy(getopt,tf,td,detections);
    return NULL;
}

void * pdat::
tracking_thread()
{
    vector<Mat>next_edges;
    Pose_Estimation pose;
    Mat src_gray,prevgray;
    vector<Point2f> _corners,_box_edges,_nedges;//corners0;
    vector<Point3f> camera_pose;

    while(1)
    {
        if(Find_detec)
        {
            _corners = corners;
            _box_edges = box_edges;
            Find_detec = false;
        }
        if(!current_image.empty())
        {
            src=current_image.clone();
            cvtColor( src, src_gray, CV_BGR2GRAY );
            if(prevgray.empty())
                src_gray.copyTo(prevgray);

            next_corners= Track.OpticalFlow_Homograhpy(prevgray,src_gray,_corners,_corners,H);
            cout<<next_corners<<endl;
            cout<<"size : "<<next_corners.size()<<endl;
            nedges=Track.OpticalFlow_tracking_box(src,prevgray,src_gray,_box_edges);
            Track.Show_OpticalFlow(2,src,_corners,next_corners);
            if(_box_edges.size()>0)
            {
                camera_pose=pose.using_solvepnp(src,_box_edges,rotation,translation);
                pose.show_pose_xyz(src,translation);
                pose.show_pose_rotation(src,rotation);
                if(Find_detec)
                {
                    Track.Show_Detection(src,box_edges);
                }
            }
            if( box_edges.empty() && next_corners.size() < 7)
            {
                _box_edges.clear();
                _corners.clear();
                corners0.clear();
                next_corners.clear();
                nedges.clear();
                camera_pose.clear();
            }
            _box_edges=nedges;
            _corners=next_corners;

            namedWindow( "OpticalFlow", CV_WINDOW_AUTOSIZE);
            imshow( "OpticalFlow", src );
            waitKey(1);

            swap(prevgray,src_gray);
        }
    }
    return NULL;
}



void pdat::
pdat_start()
{
    typedef void * (*THREADFUNCPTR)(void *);
    pthread_t image,detection,tracking;
    int im,det,track;

    im = pthread_create(&image, NULL,(THREADFUNCPTR) &pdat::image_thread,this);
    if (im)
    {
        cout << "Thread creation failed : " << strerror(im);
        exit(-1);
    }

    det = pthread_create(&detection, NULL, (THREADFUNCPTR) &pdat::detection,this);
    if (det)
    {
        cout << "Thread creation failed : " << strerror(det);
        exit(-1);
    }

    track = pthread_create(&tracking, NULL, (THREADFUNCPTR) &pdat::tracking_thread,this);
    if (track)
    {
        cout << "Thread creation failed : " << strerror(track);
        exit(-1);
    }

    pthread_join(image,NULL);
}

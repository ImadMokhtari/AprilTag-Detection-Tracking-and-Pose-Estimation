#include "pdat.h"
#include<mutex>
#include <chrono>

pdat::pdat(){}




void * pdat::
image_thread()
{
    VideoCapture cap(0);

    if(cap.isOpened())
        while(1)
        {
            cap >> current_image.Img;
            if(!current_image.Img.empty())
            {
                current_image.ID=CurId;
                Previous_Imgs.push_back (current_image);
                ////cout<<"prev stampedimg   "<<Previous_Imgs.size()<<endl;
                CurId++;
            }
        }
    return NULL;
}

void * pdat::
detection()
{

    Tag_Detection_Features Tag;
    Mat  gray;
    zarray_t *detections;
    apriltag_detection_t *det;
    int detected_tags_number;
    getopt_t *getopt = getopt_create();
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    Tag.Tag_Define(getopt,tf,td);
    detection_start=false;
    uint64_t id_det;

    while(1)
    {

        if(!current_image.Img.empty())
        {
            //   detection_start=true;
            //  detection_finished=false;

            /*   if(!Previous_Imgs.empty())
            {
                id_det = Previous_Imgs.back().ID;
                cout<<"---Detectoin_START_ID = " << Detection_ID<<endl;
                detec_img = Previous_Imgs.back().Img;
                frame = detec_img.clone();
            }
            else
            {
                id_det=0;
                Detection_ID = 0;
                frame = current_image.Img.clone();
            }*/

            id_det = current_image.ID;

            cout<<"---Detectoin_START_ID = " << id_det<<endl;

            cvtColor(current_image.Img, gray, COLOR_BGR2GRAY);
            image_u8_t im = { .width = gray.cols,
                              .height = gray.rows,
                              .stride = gray.cols,
                              .buf = gray.data};

            detections = apriltag_detector_detect(td, &im);
            detected_tags_number=zarray_size(detections);

            if(detected_tags_number != 0)
            {

                Detection_ID = id_det;
                Find_detec=true;
                cout<<"-------------------------------------------------------------------Detectoin_END_ID = " << Detection_ID<<endl;

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
                    end_detection=true;
                    detection_finished=true;
                }
            }
            else
            {
                id_det=0;
                Detection_ID = 0 ;
                box_edges.clear();
                corners.clear();
                Find_detec=false;
            }
            //detection_start=false;
        }
    }
    Tag.Tag_Destroy(getopt,tf,td,detections);
    return NULL;
}

void * pdat::
tracking_current()
{
    vector<Mat>next_edges;
    Pose_Estimation pose;
    Mat src_gray,prevgray;
    vector<Point2f> _corners,_box_edges,_nedges;
    vector<Point3f> camera_pose;
    uint64_t previous_id;

    while(1)
    {
        if(Find_detec)
        {
            previous_id = current_image.ID;
            _corners=corners;
            _box_edges=box_edges;
            break;
        }
    }

    while(1)
    {
        if(prev_track_finished)
        {
            cout<<"????????Current Track init?????????\n";
            _corners=_next_corners_previous;
            _box_edges=_nedges_previous;
            prev_track_finished=false;
            //   current_track_init=true;
        }


        if(!current_image.Img.empty() && current_image.ID != previous_id)
        {

            previous_id = current_image.ID;

            src = current_image.Img.clone();
            cout<<"**********************************current_trac_ID = " << current_image.ID<<endl;
            cvtColor( src, src_gray, CV_BGR2GRAY );

            if(prevgray.empty())
                src_gray.copyTo(prevgray);

            next_corners= Track.OpticalFlow_Homograhpy(prevgray,src_gray,_corners,_corners,H);
            nedges=Track.OpticalFlow_tracking_box(src,prevgray,src_gray,_box_edges);
            Track.Show_OpticalFlow(2,src,_corners,next_corners);
            if(_box_edges.size()>0)
            {
                camera_pose=pose.using_solvepnp(src,_box_edges,rotation,translation);
                if(!pose.pose_estimation_failed)
                {
                    pose.show_pose_xyz(src,translation);
                    pose.show_pose_rotation(src,rotation);
                }
                if(Find_detec)
                {
                    Track.Show_Detection(src,box_edges);
                    Find_detec=false;
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

            imshow( "OpticalFlow", src );
            waitKey(1);
            swap(prevgray,src_gray);

        }
    }
    return NULL;
}


void * pdat::
tracking_previous()
{
    Mat img , src_gray,prevgray;
    vector <Point2f> _box_edges_previous,_corners_previous;
    bool Find_ID=false ;
    int previous_index;
    while (1)
    {
        if(!Previous_Imgs.empty())
        {
            for(int j=0;j < Previous_Imgs.size();j++)
            {
                if(Previous_Imgs[j].ID == Detection_ID && j>0)
                {
                    Previous_Imgs.erase(Previous_Imgs.begin(),Previous_Imgs.begin()+(j));
                    //cout << "size after " << Previous_Imgs.size()<<endl;
                    cout<<"previous serach for ID="<<Detection_ID<<endl;

                    //if(!box_edges.empty() && !corners.empty())//detection_finished)
                    if(detection_finished)
                    {
                        //cout<<"box edges assertion with success\n";
                        _box_edges_previous=box_edges;
                        _corners_previous=corners;
                        Find_ID=true;
                        detection_finished=false;
                        break;
                    }
                    else
                    {
                        Find_ID=false;
                        break;
                    }
                }
            }
            previous_index=0;
            while(1)
            {
                if(Find_detec)
                {
                    prev_track_finished=true;

                    Find_detec=false;
                    break;
                }
             //   cout<<"DETECTION FINISHED  "<<detection_finished<<endl;
                if(!Previous_Imgs.empty()  && Find_ID && !_corners_previous.empty() && !_box_edges_previous.empty())
                {
                    img = Previous_Imgs[previous_index].Img.clone();
                    cout<<"Previous_trac_ID="<<Previous_Imgs[previous_index].ID<<endl;

                    previous_index++;

                    //Previous_Imgs.erase(Previous_Imgs.begin());
                    // cout<<"size after optical flow "<< Previous_Imgs.size()<<endl;

                    cvtColor(img , src_gray, CV_BGR2GRAY );

                    if(prevgray.empty())
                        src_gray.copyTo(prevgray);



                    _next_corners_previous = Track.OpticalFlow_Homograhpy(prevgray,src_gray,_corners_previous,_corners_previous,H);
                    _nedges_previous=Track.OpticalFlow_tracking_box_previous(prevgray,src_gray,_box_edges_previous);
                    _box_edges_previous =_nedges_previous;
                    _corners_previous =_next_corners_previous;

                    swap(prevgray,src_gray);

                    //cout<< "(current_image.ID-previous_index].ID   =   "<<current_image.ID <<" ;; "<< Previous_Imgs[previous_index].ID<<endl;

                    if( (current_image.ID - Previous_Imgs[previous_index].ID) < 2  )
                    {

                        cout<<"!!!!!Prev Finish!!!!!!!!!!!!!!!!!!!!\n";
                        prev_track_finished=true;
                        Find_ID=false;
                        break;
                    }
                }
                else
                {
                    //   prev_track_finished = false;
                    //  current_track_init=false;
                    Find_ID=false;
                    break;
                }
            }
        }
    }
    return NULL;
}


void pdat::
pdat_start()
{
    typedef void * (*THREADFUNCPTR)(void *);
    pthread_t image,detection,tracking,tracking_prev;
    int im,det,track,track_prev,track_prev_img;

    im = pthread_create(&image, NULL,(THREADFUNCPTR) &pdat::image_thread,this);
    if (im)
    {
        ////cout << "Thread creation failed : " << strerror(im);
        exit(-1);
    }

    det = pthread_create(&detection, NULL, (THREADFUNCPTR) &pdat::detection,this);
    if (det)
    {
        ////cout << "Thread creation failed : " << strerror(det);
        exit(-1);
    }

    track = pthread_create(&tracking, NULL, (THREADFUNCPTR) &pdat::tracking_current,this);
    if (track)
    {
        ////cout << "Thread creation failed : " << strerror(track);
        exit(-1);
    }

    track_prev = pthread_create(&tracking_prev, NULL, (THREADFUNCPTR) &pdat::tracking_previous ,this);
    if (track_prev)
    {
        ////cout << "Thread creation failed : " << strerror(track_prev);
        exit(-1);
    }
    pthread_join(image,NULL);
    pthread_join(detection,NULL);
    pthread_join(tracking,NULL);
    pthread_join(tracking_prev,NULL);
}

/*
uint64_t unix_timestamp()
{
    chrono::high_resolution_clock m_clock;
    return chrono::duration_cast<chrono::microseconds>(m_clock.now().time_since_epoch()).count();
}*/

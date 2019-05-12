#include <chrono>
#include "pdat.h"

pdat::
pdat()
{
    this->previous_id = 0;
    this->Detection_ID = 0;
    this->CurId = 1;
    this->end_detection = false;
    this->Find_detec=false;
    this->detection_start=false;
    this->detection_finished=false;
    this->prev_track_finished=false;
}

vector<Mat> image;
int ni=1,nii=ni;
int nf=310;

void * pdat::
image_thread()
{
    //string video = "/home/imad/Desktop/Mini_projet/src/Best Videos/14.webm";

    // string video = "/home/imad/Desktop/video_pfe/pc5.webm";

    // VideoCapture cap(video);
    /*
    VideoCapture cap;

    if(!cap.open(1))
    {
        cap.open(0);
        cout << "Opening the default camera !!! \n";
    }
*/

    image.clear();

    for (int i=ni;i<=nf;i++)
    {
        auto result ="/home/imad/Desktop/video_pfe/videos/images_pc13/" +to_string(i)+".png";
        Mat outImg;
        resize(imread(result), outImg, cv::Size(), 0.75, 0.75);
        image.push_back(outImg);
        cout<<"imgs  "<<image.size()<<"   size of image is "<<outImg.size()<<endl;
    }

    //    Mat frame;
    // if(cap.isOpened())
   // usleep(1500000);
    while(1)
    {
        //cap >> frame ;
        Mat im;

        image[ni-nii].copyTo(current_image.Img);
        ni++;

        if(!current_image.Img.empty())
        {
            current_image.Img.copyTo(im);
            current_image.ID = CurId;
            Previous_Imgs.push_back({im,current_image.ID});
            CurId++;
        }
        else
            exit(-1);

        usleep(33333);
        // usleep(20000);

    }
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
    detection_start=false;
    uint64_t id_det;

    while(1)
    {
        if(!current_image.Img.empty())
        {

            current_image.Img.copyTo(frame);
            id_det = current_image.ID ;
            detection_start=true;
            //cout<<"\n----****----Detectoin_START_ID = " << id_det << endl;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
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
                    cout<<"---------------------------------------------------Detectoin_END_ID = " << Detection_ID<<endl;

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
        }
    }
    Tag.Tag_Destroy(getopt,tf,td,detections);
    return NULL;
}

void * pdat::
tracking_current()
{
    Pose_Estimation pose;
    Mat src_gray,prevgray , image;
    vector<Point2f> _corners,_box_edges,_nedges;
    vector<Point3f> camera_pose;
    Features_Tracking Track;
    int k;
    ofstream Time,translate,translatedetect;
    clock_t start,end;


    translate.open("/home/imad/Desktop/video_pfe/evaluation results/pdat_a/translation_pdata.csv");
    Time.open("/home/imad/Desktop/video_pfe/evaluation results/pdat_a/Time_pdata.csv");

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
    k=0;
    while(1)
    {
        //k++;
        if(prev_track_finished)
        {
            cout<<"????????????__Current Track init__????????????\n";
            _corners=_next_corners_previous;
            _box_edges=_nedges_previous;
            prev_track_finished=false;
        }

        if(!current_image.Img.empty() && current_image.ID != previous_id)
        {
            current_image.Img.copyTo(image);
            previous_id = current_image.ID;

            src = image.clone();

            cout<<"**********************************current_trac_ID = " << current_image.ID<<endl;

            cvtColor( src, src_gray, CV_BGR2GRAY );
            if(prevgray.empty())
                src_gray.copyTo(prevgray);

            start=clock();

            next_corners= Track.OpticalFlow_Homograhpy(prevgray,src_gray,_corners,corners0,H);
            nedges=Track.OpticalFlow_tracking_box(src,prevgray,src_gray,_box_edges);

            end=clock()-start;

            Track.Show_OpticalFlow(2,src,_corners,next_corners);

            if(_box_edges.size() > 0)
            {
                camera_pose = pose.using_solvepnp(src,_box_edges,rotation,translation);
                translate <<current_image.ID<<","<< translation.at<double>(0,0)<<","<<translation.at<double>(0,1)<<","<<translation.at<double>(0,2)<<","<<endl;

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
            Time<<2.5<<','<<(float)end/CLOCKS_PER_SEC<<','<<endl;

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
    uint64_t previous_index;
    Features_Tracking Track;

    while (1)
    {
        if(!Previous_Imgs.empty() && !prev_track_finished)
        {
            for(uint64_t j = 0 ; j < Previous_Imgs.size() ; j++)
            {
                if(detection_finished)
                {
                    if(Previous_Imgs[j].ID == Detection_ID)
                    {
                        Previous_Imgs.erase(Previous_Imgs.begin(),Previous_Imgs.begin()+int (j+1));
                        // cout<<"******\n";
                        // cout<<"\n previous serach for ID =  "<< Detection_ID <<endl;
                        // cout<<"box edges assertion with success\n";
                        _box_edges_previous = box_edges;
                        _corners_previous = corners;
                        Find_ID = true;
                        detection_finished = false;
                        break;
                    }
                    else
                        Find_ID = false;
                }
            }

            previous_index = 0;

            while(Find_ID)
            {
                cvtColor(Previous_Imgs[previous_index].Img , src_gray, CV_BGR2GRAY );
                cout<<"Previous_track_ID="<<Previous_Imgs[previous_index].ID<<endl;

                if(prevgray.empty())
                    src_gray.copyTo(prevgray);

                _next_corners_previous = Track.OpticalFlow_Homograhpy(prevgray,src_gray,_corners_previous,_corners_previous,H);
                _nedges_previous=Track.OpticalFlow_tracking_box_previous(prevgray,src_gray,_box_edges_previous);
                _box_edges_previous =_nedges_previous;
                _corners_previous =_next_corners_previous;
                // cout<< "_nedges_previous  \n"<<_nedges_previous<<endl;
                swap(prevgray,src_gray);
                if(previous_id - Previous_Imgs[previous_index].ID < 1 )
                {
                    //   cout<<"!!!!!!!!!!!!!!!! Prev_Finish !!!!!!!!!!!!!!!!\n";
                    //  cout<<"Previous_track_ID break ="<< Previous_Imgs[previous_index].ID << endl;
                    prev_track_finished=true;
                    Find_ID=false;
                }
                previous_index++;
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
        cout << "Thread creation failed : " << strerror(im);
        exit(-1);
    }

    det = pthread_create(&detection, NULL, (THREADFUNCPTR) &pdat::detection,this);
    if (det)
    {
        cout << "Thread creation failed : " << strerror(det);
        exit(-1);
    }

    track = pthread_create(&tracking, NULL, (THREADFUNCPTR) &pdat::tracking_current,this);
    if (track)
    {
        cout << "Thread creation failed : " << strerror(track);
        exit(-1);
    }

    track_prev = pthread_create(&tracking_prev, NULL, (THREADFUNCPTR) &pdat::tracking_previous ,this);
    if (track_prev)
    {
        cout << "Thread creation failed : " << strerror(track_prev);
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

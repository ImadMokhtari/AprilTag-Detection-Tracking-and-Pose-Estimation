#include "features_tracking.h"

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size  winSize(21,21);
vector<int> OutLiersMask;
vector<uchar> status;
vector<float> err;
vector<Point2f> nexttcorners;

Features_Tracking::Features_Tracking(void){}

vector<Point2f> Features_Tracking::OpticalFlow_Homograhpy(Mat prevgray,Mat src_gray,vector<Point2f> corners,vector<Point2f> corners0,Mat& H)
{	
    if(corners.size()>0)
    {
        nexttcorners.resize(corners.size());
        calcOpticalFlowPyrLK(prevgray,
                             src_gray,
                             corners,
                             nexttcorners,
                             status,
                             err,
                             winSize,
                             3,
                             termcrit,
                             0,
                             0.05);

        if(corners0.size()==nexttcorners.size())
        {
            H =findHomography (corners0,
                               nexttcorners,
                               RANSAC,         //method to use
                               5,
                               OutLiersMask,   //OutputArray mask
                               1000,           //const int maxIters
                               0.5 );
            int j=0;
            for (int i=0;i<OutLiersMask.size();i++)
            {
                if (OutLiersMask[i]==1)
                {
                    nexttcorners[j]=nexttcorners[i];
                    corners[j]=corners[i];
                    corners0[j]=corners0[i];
                    j++;
                }
            }
            nexttcorners.resize(j);
            corners.resize(j);
            corners0.resize(j);
        }
        return nexttcorners;
    }
    
}

void Features_Tracking::Show_OpticalFlow(int r, Mat src , vector<Point2f> corners, vector<Point2f> nextcorners)
{
    for( int i = 0; i < nextcorners.size(); i++ )
    {
        if(corners.size()==nextcorners.size())
        {
            arrowedLine(src,           //picture where to draw
                        corners[i],     //begin arrow points
                        nextcorners[i], //end arrow points
                        Scalar(0,0,255),//arrow color
                        2,              //Arrow thickness
                        8,              //line Type
                        0,              //int shift
                        0.2);           //Arrow head length


            circle( src,
                    nextcorners[i],
                    r,
                    Scalar(0,255,0),
                    1,
                    8,
                    0 );

        }
    }
    /*
    cout<<"display"<<endl;
    namedWindow( "OpticalFlow", CV_WINDOW_AUTOSIZE );
    imshow( "OpticalFlow", src );
    waitKey(1);*/
}

void Features_Tracking::Show_Detection(Mat src,vector<Point> tag_Points)
{
    line(src,Point(tag_Points[0].x,tag_Points[0].y),Point(tag_Points[1].x,tag_Points[1].y),Scalar(255,0,0),3);
    line(src,Point(tag_Points[1].x,tag_Points[1].y),Point(tag_Points[2].x,tag_Points[2].y),Scalar(255,0,0),3);
    line(src,Point(tag_Points[2].x,tag_Points[2].y),Point(tag_Points[3].x,tag_Points[3].y),Scalar(255,0,0),3);
    line(src,Point(tag_Points[3].x,tag_Points[3].y),Point(tag_Points[0].x,tag_Points[0].y),Scalar(255,0,0),3);
}
/*
void Features_Tracking::Show_Tracking(Mat src,vector<Point> tag_points,Mat H)
{
    vector<Point> track_h;
    if(!H.empty())
    {
        track_h.clear();
        Mat pt1 = (Mat_<double>(3,1) << tag_points[0].x, tag_points[0].y, 1);
        Mat pt2 = H * pt1;
        // Continuer le travail........

        pt2 /= pt2.at<double>(2);
        Point end( (int) (pt2.at<double>(0)), (int) pt2.at<double>(1));
        Point end0=end;
        track_h.push_back(end);
        for (size_t i = 1; i < tag_points.size(); i++)
        {
            Mat pt1 = (Mat_<double>(3,1) << tag_points[i].x, tag_points[i].y, 1);
            Mat pt2 = H * pt1;
            // Continuer le travail........
            cout<<"pt1:\n"<<pt1<<endl;
            cout<<"pt2:\n"<<pt1<<endl;

            pt2 /= pt2.at<double>(2);
            Point end1( (int) (pt2.at<double>(0)), (int) pt2.at<double>(1) );
            track_h.push_back(end1);
            line(src, end, end1, Scalar(0,0,255), 2);
            end=end1;
            if(i==3)
            {
                line(src, end1, end0,Scalar(0,0,255), 2);
            }

            //namedWindow( "OpticalFlow", CV_WINDOW_AUTOSIZE );
            //imshow( "OpticalFlow", src );
           // waitKey(0);
        }
        cout<<"track_H:\n"<<track_h<<endl;
    }
}*/

vector<Mat> Features_Tracking::Show_Tracking(Mat src,vector<Point> tag_points,Mat H)//,vector<Point>next_edges)
{

    vector <Point3f> pred_edge;
    Mat pred;
    vector<Mat> next_points;
    vector<Point>next_edges;
    if(!H.empty())
    {
        pred_edge.clear();
        next_edges.clear();
        next_points.clear();

        for (int i=0;i<4;i++)
        {
            pred.push_back(Point3f(tag_points[i].x,tag_points[i].y,1));
            Mat edge1 = (Mat_<double>(3,1)<<pred.at<float>(i,0),pred.at<float>(i,1),1);
            Mat edge11=H*edge1;
            next_points.push_back(edge11);
            Point e1((int) edge11.at<double>(0), (int) edge11.at<double>(1));
            next_edges.push_back(Point((int) edge11.at<double>(0), (int) edge11.at<double>(1)));
        }

        line(src,next_edges[0],next_edges[1],Scalar(0,0,255), 2);
        line(src,next_edges[1],next_edges[2],Scalar(0,0,255), 2);
        line(src,next_edges[2],next_edges[3],Scalar(0,0,255), 2);
        line(src,next_edges[3],next_edges[0],Scalar(0,0,255), 2);
        return next_points;
    }
}


vector<Point2f> next_edges;
vector<Point2f> Features_Tracking::OpticalFlow_tracking_box(Mat src,Mat prevgray,Mat src_gray,vector<Point2f> edges)
{
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size  winSize(21,21);
    vector<int> OutLiersMask;
    vector<uchar> status;
    vector<float> err;
    if(edges.size()>0)
    {
        next_edges.resize(edges.size());
        calcOpticalFlowPyrLK(prevgray,
                             src_gray,
                             edges,
                             next_edges,
                             status,
                             err,
                             winSize,
                             3,
                             termcrit,
                             0,
                             0.01);


        vector< Point> contour;
        contour.push_back(Point(next_edges[0].x,next_edges[0].y));
        contour.push_back(Point(next_edges[1].x,next_edges[1].y));
        contour.push_back(Point(next_edges[2].x,next_edges[2].y));
        contour.push_back(Point(next_edges[3].x,next_edges[3].y));

        const Point *pts;
        int npts;

        pts = (const cv::Point*) Mat(contour).data;
        npts = Mat(contour).rows;
        polylines(src, &pts, &npts, 1, false, Scalar(0,255, 0));
        /*
        if(next_edges.size()>0)

            polylines 	( 	src,
                            next_edges,
                            true,
                            Scalar(255,0,0),
                            2,//thikness
                            LINE_8
                            );
        */
        line(src,Point(next_edges[0].x,next_edges[0].y),Point(next_edges[1].x,next_edges[1].y),Scalar(0,0,255),2);
        line(src,Point(next_edges[1].x,next_edges[1].y),Point(next_edges[2].x,next_edges[2].y),Scalar(0,0,255),2);
        line(src,Point(next_edges[2].x,next_edges[2].y),Point(next_edges[3].x,next_edges[3].y),Scalar(0,0,255),2);
        line(src,Point(next_edges[3].x,next_edges[3].y),Point(next_edges[0].x,next_edges[0].y),Scalar(0,0,255),2);
        return next_edges;
    }
}

#include "features_tracking.h"

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size  winSize(31,31);
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
                             0.001);

        if(corners0.size()==nexttcorners.size())
        {
            H =findHomography (corners0,
                               nexttcorners,
                               RANSAC,         //method to use
                               3,
                               OutLiersMask,   //OutputArray mask
                               1000,           //const int maxIters
                               0.995 );
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
    }
    return nexttcorners;
}

void Features_Tracking::Show_OpticalFlow(int r, Mat src , vector<Point2f> corners, vector<Point2f> nextcorners)
{
    for( int i = 0; i < nextcorners.size(); i++ )
    {

        arrowedLine(src,           //picture where to draw
                    corners[i],     //begin arrow points
                    nextcorners[i], //end arrow points
                    Scalar(0,0,220),//arrow color
                    2,              //Arrow thickness
                    8,              //line Type
                    0,              //int shift
                    0.2);           //Arrow head length


        circle( src,
                nextcorners[i],
                r,
                Scalar(0,255,0),
                2,
                8,
                0 );

        }

    cout<<"display"<<endl;
    namedWindow( "OpticalFlow", CV_WINDOW_AUTOSIZE );
    imshow( "OpticalFlow", src );
    waitKey(1);
}

void Features_Tracking::Show_Detection(Mat src,vector<Point> tag_Points)
{
    line(src,Point(tag_Points[0].x,tag_Points[0].y),Point(tag_Points[1].x,tag_Points[1].y),Scalar(255,0,0),4);
    line(src,Point(tag_Points[1].x,tag_Points[1].y),Point(tag_Points[2].x,tag_Points[2].y),Scalar(255,0,0),4);
    line(src,Point(tag_Points[2].x,tag_Points[2].y),Point(tag_Points[3].x,tag_Points[3].y),Scalar(255,0,0),4);
    line(src,Point(tag_Points[3].x,tag_Points[3].y),Point(tag_Points[0].x,tag_Points[0].y),Scalar(255,0,0),4);
}

void Features_Tracking::Show_Tracking(Mat src,vector<Point> tag_points,Mat H)
{
    if(!H.empty())
    {
        Mat pt1 = (Mat_<double>(3,1) << tag_points[0].x, tag_points[0].y, 1);
        Mat pt2 = H * pt1;
        // Continuer le travail........

        pt2 /= pt2.at<double>(2);
        Point end( (int) (pt2.at<double>(0)), (int) pt2.at<double>(1));
        Point end0=end;
        for (size_t i = 1; i < tag_points.size(); i++)
        {
            Mat pt1 = (Mat_<double>(3,1) << tag_points[i].x, tag_points[i].y, 1);
            Mat pt2 = H * pt1;
            // Continuer le travail........

            pt2 /= pt2.at<double>(2);
            Point end1( (int) (pt2.at<double>(0)), (int) pt2.at<double>(1) );
            line(src, end, end1, Scalar(0,0,255), 2);
            end=end1;
            if(i==3)
            {
                line(src, end1, end0,Scalar(0,0,255), 2);
            }
        };
    }
}



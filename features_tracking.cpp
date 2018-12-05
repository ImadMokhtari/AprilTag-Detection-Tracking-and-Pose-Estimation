#include "features_tracking.h"

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size  winSize(31,31);
vector<int> OutLiersMask;
vector<uchar> status;
vector<float> err;
vector<Point2f> nexttcorners;

Features_Tracking::Features_Tracking(void){}

vector<Point2f> Features_Tracking::OpticalFlow_Homograhpy(Mat prevgray,Mat src_gray,vector<Point2f> corners,vector<Point2f> corners0)
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
        findHomography (corners0,
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

void Features_Tracking::Show_Detection(Mat src,int px11,int py11,int px12,int py12,int px22,int py22,int px21,int py21)
{
    line(src,Point(px11,py11),Point(px12,py12),Scalar(255,255,0),2);
    line(src,Point(px12,py12),Point(px22,py22),Scalar(255,255,0),2);
    line(src,Point(px22,py22),Point(px21,py21),Scalar(255,255,0),2);
    line(src,Point(px21,py21),Point(px11,py11),Scalar(255,255,0),2);
}

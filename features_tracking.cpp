#include "features_tracking.h"


TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size  winSize(31,31);
vector<int> OutLiersMask;
vector<uchar> status;
vector<float> err;
vector<Point2f> nextcorners;


Features_Tracking::Features_Tracking(void)
    {
        cout<<"constructor\n";
    }

vector<Point2f> Features_Tracking::OpticalFlow_Homograhpy(Mat prevgray,Mat src_gray,vector<Point2f> corners,vector<Point2f> corners0)
{
    calcOpticalFlowPyrLK(prevgray,
                         src_gray,
                         corners,
                         nextcorners,
                         status,
                         err,
                         winSize,
                         3,
                         termcrit,
                         0,
                         0.001);

    if(corners0.size()==nextcorners.size())
    {
    findHomography (corners0,
                    nextcorners,
                    RANSAC,         //method to use
                    5,
                    OutLiersMask,   //OutputArray mask
                    1000,           //const int maxIters
                    0.995 );
    int j=0;
    for (int i=0;i<OutLiersMask.size();i++)
    {
        if (OutLiersMask[i]==1)
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
    }

    return nextcorners;
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
        else
        {
            break;
        }
    }
    cout<<"display"<<endl;
    namedWindow( "OpticalFlow", CV_WINDOW_AUTOSIZE );
    imshow( "OpticalFlow", src );
    waitKey(1);

}

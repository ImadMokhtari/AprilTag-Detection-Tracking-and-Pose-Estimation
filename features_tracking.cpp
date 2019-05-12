#include "features_tracking.h"


Features_Tracking::
Features_Tracking(void){}


vector<Point2f> Features_Tracking::
OpticalFlow_Homograhpy(Mat prevgray,Mat src_gray,vector<Point2f> corners,vector<Point2f> corners0,Mat& H)
{
    vector<Point2f> next_corners;
    if(corners.size()>0)
    {
        next_corners.resize(corners.size());
        calcOpticalFlowPyrLK(prevgray,
                             src_gray,
                             corners,
                             next_corners,
                             status,
                             err,
                             Size(21,21),
                             4,
                             TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03),
                             0,
                             0.05);
/*
        if(corners0.size()==next_corners.size())
        {
            calc_homography = true;
          //  cout << "calc _ homography \n";
            H =findHomography (corners0,
                               next_corners,
                               RANSAC,         //method to use
                               4,
                               OutLiersMask,   //OutputArray mask
                               1000,           //const int maxIters
                               0.9);
            int j=0;
            for (int i=0;i<OutLiersMask.size();i++)
            {
                if (OutLiersMask[i]==1)
                {
                    next_corners[j]=next_corners[i];
                    corners[j]=corners[i];
                    corners0[j]=corners0[i];
                    j++;
                }
            }
            next_corners.resize(j);
            corners.resize(j);
            corners0.resize(j);
        }
        else
            calc_homography = false;*/
        return next_corners;
    }
}



void Features_Tracking::
Show_OpticalFlow(int r, Mat src , vector<Point2f> corners, vector<Point2f> nextcorners)
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
                        0.3);           //Arrow head length


            circle( src,
                    nextcorners[i],
                    r,
                    Scalar(0,255,0),
                    1,
                    8,
                    0 );

        }
    }

}

void Features_Tracking::
Show_Detection(Mat src,vector<Point2f> tag_Points)
{
    line(src,Point2f(tag_Points[0].x,tag_Points[0].y),Point2f(tag_Points[1].x,tag_Points[1].y),Scalar(255,0,0),2);
    line(src,Point2f(tag_Points[1].x,tag_Points[1].y),Point2f(tag_Points[2].x,tag_Points[2].y),Scalar(255,0,0),2);
    line(src,Point2f(tag_Points[2].x,tag_Points[2].y),Point2f(tag_Points[3].x,tag_Points[3].y),Scalar(255,0,0),2);
    line(src,Point2f(tag_Points[3].x,tag_Points[3].y),Point2f(tag_Points[0].x,tag_Points[0].y),Scalar(255,0,0),2);
}


vector<Point2f> Features_Tracking::
Show_Tracking_Homography(Mat src,vector<Point2f> tag_points,Mat H)
{
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
            next_edges.push_back(Point2f((int) edge11.at<double>(0), (int) edge11.at<double>(1)));
        }

        line(src,next_edges[0],next_edges[1],Scalar(0,255,0), 2);
        line(src,next_edges[1],next_edges[2],Scalar(0,255,0), 2);
        line(src,next_edges[2],next_edges[3],Scalar(0,255,0), 2);
        line(src,next_edges[3],next_edges[0],Scalar(0,255,0), 2);
        return next_edges;
    }
}


vector<Point2f> Features_Tracking::
OpticalFlow_tracking_box(Mat src,Mat prevgray,Mat src_gray,vector<Point2f> edges)
{   vector<int> Mask;
    vector<Point2f> next_edges;
    Mat H;
    if(edges.size()>0)
    {
        next_edges.resize(edges.size());
        calcOpticalFlowPyrLK(prevgray,
                             src_gray,
                             edges,
                             next_edges,
                             status,
                             err,
                             Size(21,21),
                             4,
                             TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03),
                             0,
                             0.01);
        /*        cout<<"******************************************"<<endl;
  if(edges.size()==next_edges.size())
        {
            cout<<" ------------------------------------------"<<endl;
            cout<<"next_edges size : "<<next_edges.size()<<endl;

            H =findHomography (edges,
                               next_edges,
                               RANSAC,         //method to use
                               3,
                               Mask,   //OutputArray mask
                               500,           //const int maxIters
                               0.9);
            int j=0;
            for (int i=0;i<Mask.size();i++)
            {
                if (Mask[i]==1)
                {
                    next_edges[j]=next_edges[i];
                    edges[j]=edges[i];

                    j++;
                }
            }
            next_edges.resize(j);
            edges.resize(j);
        }*/
        line(src,Point(next_edges[0].x,next_edges[0].y),Point(next_edges[1].x,next_edges[1].y),Scalar(0,0,255),2);
        line(src,Point(next_edges[1].x,next_edges[1].y),Point(next_edges[2].x,next_edges[2].y),Scalar(0,0,255),2);
        line(src,Point(next_edges[2].x,next_edges[2].y),Point(next_edges[3].x,next_edges[3].y),Scalar(0,0,255),2);
        line(src,Point(next_edges[3].x,next_edges[3].y),Point(next_edges[0].x,next_edges[0].y),Scalar(0,0,255),2);
        return next_edges;
    }
}

vector<Point2f> Features_Tracking::
OpticalFlow_tracking_box_previous(Mat prevgray,Mat src_gray,vector<Point2f> edges)
{   vector<int> Mask;
    vector<Point2f> next_edges;
    Mat H;
    if(edges.size()>0)
    {
        next_edges.resize(edges.size());
        calcOpticalFlowPyrLK(prevgray,
                             src_gray,
                             edges,
                             next_edges,
                             status,
                             err,
                             Size(21,21),
                             4,
                             TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03),
                             0,
                             0.01);
        return next_edges;
    }
}

vector<Point2f> Features_Tracking::
Next_with_Homography(Mat src, vector<Point2f> tag_points,Mat H)//,vector<Point>next_edges)
{
    vector<Point2f> next_edges,next_corners;

    if(!H.empty() && !tag_points.empty())
    {
        if(calc_homography)
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
         //   cout << "calc _ homography*********** \n";
            line(src,Point(next_edges[0].x,next_edges[0].y),Point(next_edges[1].x,next_edges[1].y),Scalar(0,255,0),2);
            line(src,Point(next_edges[1].x,next_edges[1].y),Point(next_edges[2].x,next_edges[2].y),Scalar(0,255,0),2);
            line(src,Point(next_edges[2].x,next_edges[2].y),Point(next_edges[3].x,next_edges[3].y),Scalar(0,255,0),2);
            line(src,Point(next_edges[3].x,next_edges[3].y),Point(next_edges[0].x,next_edges[0].y),Scalar(0,255,0),2);

            return next_edges;
        }
    }
}

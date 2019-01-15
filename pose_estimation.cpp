#include "pose_estimation.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Pose_Estimation::Pose_Estimation(){
}

vector<Point3f> Pose_Estimation::using_solvepnp(Mat src,vector<Point2f> tag_image_points,Mat &rotation, Mat &translation )
{
    vector<Point3f> tag_3d_points;
    vector<Point3f> camera_pose;
    Mat rotation1;
    tag_3d_points.clear();
    tag_3d_points.push_back(Point3d(0.0f, 0.0f, 0.0f));
    tag_3d_points.push_back(Point3d(0.0f, 17.2f, 0.0f));
    tag_3d_points.push_back(Point3d(17.2f, 17.2f, 0.0f));
    tag_3d_points.push_back(Point3d(17.2f, 0.0f, 0.0f));

    Point2d center = Point2d(328.1430,233.6942);
    Mat camera_matrix = (Mat_<double>(3,3) << 712.0027, 0, center.x, 0 , 711.2302, center.y, 0, 0, 1);
    Mat dist_coeffs = (Mat_<double>(4,1)<< 0.1115,-0.3290,-0.0016,-0.0014);

    if(!tag_image_points.empty()){
        solvePnP(tag_3d_points, tag_image_points, camera_matrix, dist_coeffs, rotation, translation,false,SOLVEPNP_ITERATIVE);
        Rodrigues(rotation,rotation1);//vector to matrix
        camera_pose.clear();

        for (int i=0;i<tag_3d_points.size();i++)
        {
            double x =rotation1.at<double>(0,0)*tag_3d_points[i].x+rotation1.at<double>(0,1)*tag_3d_points[i].y+rotation1.at<double>(0,2)*tag_3d_points[i].z+translation.at<double>(0,0);
            double y =rotation1.at<double>(1,0)*tag_3d_points[i].x+rotation1.at<double>(1,1)*tag_3d_points[i].y+rotation1.at<double>(1,2)*tag_3d_points[i].z+translation.at<double>(0,1);
            double z =rotation1.at<double>(2,0)*tag_3d_points[i].x+rotation1.at<double>(2,1)*tag_3d_points[i].y+rotation1.at<double>(2,2)*tag_3d_points[i].z+translation.at<double>(0,2);
            camera_pose.push_back(Point3f(x,y,z));

        }
        return camera_pose;
    }
    else
        cout<<"image points are empty"<<endl;
}

double rad2deg (double angle);

void Pose_Estimation::show_pose_xyz(Mat src,Mat translation)
{
    if (!translation.empty())
    {
        stringstream stream;
        double x=translation.at<double>(0,0);
        double y=translation.at<double>(1,0);
        double z=translation.at<double>(2,0);

        stream <<"X= " << x <<"  ";
        stream <<"Y= " << y <<"  ";
        stream <<"Z= " << z <<"  ";

        string concat = stream.str();
        putText (src,
                 concat,
                 Point(0,src.size().height-30),
                 FONT_HERSHEY_TRIPLEX,
                 0.7,
                 Scalar(0,0,0),
                 2,
                 LINE_AA,
                 false);
    }
}


void Pose_Estimation::show_pose_rotation(Mat src,Mat rotation)
{
    if (!rotation.empty())
    {
        stringstream stream;
        double x=rotation.at<double>(0,0);
        double y=rotation.at<double>(1,0);
        double z=rotation.at<double>(2,0);
        x=rad2deg(x);
        y=rad2deg(y);
        z=rad2deg(z);
        stream <<"phi= " << x <<"  ";
        stream <<"theta= " << y <<"  ";
        stream <<"Psi= " << z <<"  ";

        string concat = stream.str();
      //  cout<<concat<<endl;
        putText (src,
                 concat,
                 Point(0,src.size().height-10),
                 FONT_HERSHEY_TRIPLEX,
                 0.7,
                 Scalar(0,0,0),
                 2,
                 LINE_AA,
                 false);
    }
}


const double pi=3.14159265358979323846;

double rad2deg (double angle)
{
    angle=(angle * 180)/pi;
    return angle;
}


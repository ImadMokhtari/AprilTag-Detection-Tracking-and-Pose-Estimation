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

    tag_3d_points.clear();
    tag_3d_points.push_back(Point3d(0.0f, 0.0f, 0.0f));
    tag_3d_points.push_back(Point3d(0.0f, 17.3f, 0.0f));
    tag_3d_points.push_back(Point3d(17.3f, 17.3f, 0.0f));
    tag_3d_points.push_back(Point3d(17.3f, 0.0f, 0.0f));

    // Camera internals
   // double focal_length =717.909 ; // Approximate focal length.
    Point2d center = Point2d(331.8828,234.9087);
    Mat camera_matrix = (Mat_<double>(3,3) << 709.5849, 0, center.x, 0 , 682.9863, center.y, 0, 0, 1);
    Mat dist_coeffs = (Mat_<double>(4,1)<< 0.0295,0.2532,0.0019,-0.0016); // Assuming no lens distortion
//(4,1,DataType<double>::type);
    // Solve for pose
    if(!tag_image_points.empty()){
        solvePnP(tag_3d_points, tag_image_points, camera_matrix, dist_coeffs, rotation, translation,false,SOLVEPNP_ITERATIVE);
        Rodrigues(rotation,rotation);//vector to matrix
        camera_pose.clear();

        for (int i=0;i<tag_3d_points.size();i++)
        {
            double x =rotation.at<double>(0,0)*tag_3d_points[i].x+rotation.at<double>(0,1)*tag_3d_points[i].y+rotation.at<double>(0,2)*tag_3d_points[i].z+translation.at<double>(0,0);
            double y =rotation.at<double>(1,0)*tag_3d_points[i].x+rotation.at<double>(1,1)*tag_3d_points[i].y+rotation.at<double>(1,2)*tag_3d_points[i].z+translation.at<double>(0,1);
            double z =rotation.at<double>(2,0)*tag_3d_points[i].x+rotation.at<double>(2,1)*tag_3d_points[i].y+rotation.at<double>(2,2)*tag_3d_points[i].z+translation.at<double>(0,2);
            camera_pose.push_back(Point3f(x,y,z));

        }
        return camera_pose;
        //cout<<"camera pose is :\n"<< camera_pose<<endl;
    }
    else
        cout<<"image points are empty"<<endl;


}


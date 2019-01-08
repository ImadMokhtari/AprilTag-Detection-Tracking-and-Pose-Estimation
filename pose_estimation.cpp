#include "pose_estimation.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Pose_Estimation::Pose_Estimation(){
}

void Pose_Estimation::using_solvepnp(Mat src,vector<Point2f> tag_image_points,Mat &rotation, Mat &translation )
{
    vector<Point3f> tag_3d_points;
    vector<Point3f> camera_pose;

    tag_3d_points.clear();
    tag_3d_points.push_back(Point3d(0.0f, 0.0f, 0.0f));
    tag_3d_points.push_back(Point3d(0.0f, 17.0f, 0.0f));
    tag_3d_points.push_back(Point3d(16.0f, 17.0f, 0.0f));
    tag_3d_points.push_back(Point3d(16.0f, 0.0f, 0.0f));

    // Camera internals
    double focal_length =717.909 ; // Approximate focal length.
    Point2d center = Point2d(315,163);
    Mat camera_matrix = (Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    Mat dist_coeffs = Mat::zeros(4,1,DataType<double>::type); // Assuming no lens distortion

    //cout << "Camera Matrix " << endl << camera_matrix << endl ;
    // Output rotation and translation
    /*cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;
     */

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
            //cout<<"corner"<<i<<"is :\n"<<endl;
            /*cout<<camera_pose<<endl;

            cout<<"x is : \n"<<camera_pose[i].x<<endl;
            cout<<"y is : \n"<<camera_pose[i].y<<endl;
            cout<<"z is : \n"<<camera_pose[i].z<<endl;
       */ }
        cout<<"camera pose is :\n"<< camera_pose<<endl;
    }
    else
        cout<<"image points are empty"<<endl;

    // Project a 3D point (0, 0, 1000.0) onto the image plane.
    // We use this to draw a line sticking out of the nose

    /*pour estimer la pose de la camera
     *
    vector<Point3d> nose_end_point3D;
    vector<Point2d> nose_end_point2D;
    nose_end_point3D.push_back(Point3d(0,0,1000.0));

    projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
     */
    /*
    for(int i=0; i < image_points.size(); i++)
    {
        circle(im, image_points[i], 3, Scalar(0,0,255), -1);
    }

    cv::line(im,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);

    cout << "Rotation Vector " << endl << rotation_vector << endl;
    cout << "Translation Vector" << endl << translation_vector << endl;

    cout <<  nose_end_point2D << endl;

    // Display image.
    cv::imshow("Output", im);
    cv::waitKey(0);
 */
}


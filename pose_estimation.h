#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

class Pose_Estimation
{
private:
    vector<Point3f> tag_3d_points;
    vector<Point3f> camera_pose;
    Mat rotation_matrix;
public:

    bool pose_estimation_failed;
    vector<Point3f> using_solvepnp(Mat src,vector<Point2f> tag_image_points,Mat &rotation,Mat &translation );
    void show_pose_xyz(Mat,Mat);
    void show_pose_rotation(Mat src,Mat rotation );
    Pose_Estimation();
};

#endif // POSE_ESTIMATION_H

#include "pose_estimation.h"


Pose_Estimation::
Pose_Estimation(){
    pose_estimation_failed=true;
}

vector<Point3f> Pose_Estimation::
using_solvepnp(Mat src,vector<Point2f> tag_image_points,Mat &rotation, Mat &translation )
{
    tag_3d_points.clear();
    tag_3d_points.push_back(Point3d(0.0f, 0.0f, 0.0f));
    tag_3d_points.push_back(Point3d(0, 0.233f, 0.0f));
    tag_3d_points.push_back(Point3d(0.233f, 0.233f, 0.0f));
    tag_3d_points.push_back(Point3d(0.233f, 0.0f, 0.0f));

/*
    tag_3d_points.push_back(Point3d(0.0f, 0.0f, 0.0f));
    tag_3d_points.push_back(Point3d(0, 23.3f, 0.0f));
    tag_3d_points.push_back(Point3d(23.3f, 23.3f, 0.0f));
    tag_3d_points.push_back(Point3d(23.3f, 0.0f, 0.0f));
  */
    Point2d center = Point2d(252.8849,191.9539);
    Mat camera_matrix = (Mat_<double>(3,3) << 681.6268, 0, center.x, 0 , 676.5015, center.y, 0, 0, 1);
    Mat dist_coeffs = (Mat_<double>(4,1)<< 0.2930,-1.2787,-0.0396,-0.0593);

    if(tag_image_points.size()>3)
    {
        solvePnP(tag_3d_points, tag_image_points, camera_matrix, dist_coeffs, rotation, translation,false,SOLVEPNP_ITERATIVE);
        Rodrigues(rotation,rotation_matrix);//vector to matrix
        camera_pose.clear();

        for (int i=0;i<tag_3d_points.size();i++)
        {
            double x =rotation_matrix.at<double>(0,0)*tag_3d_points[i].x+rotation_matrix.at<double>(0,1)*tag_3d_points[i].y+rotation_matrix.at<double>(0,2)*tag_3d_points[i].z+translation.at<double>(0,0);
            double y =rotation_matrix.at<double>(1,0)*tag_3d_points[i].x+rotation_matrix.at<double>(1,1)*tag_3d_points[i].y+rotation_matrix.at<double>(1,2)*tag_3d_points[i].z+translation.at<double>(0,1);
            double z =rotation_matrix.at<double>(2,0)*tag_3d_points[i].x+rotation_matrix.at<double>(2,1)*tag_3d_points[i].y+rotation_matrix.at<double>(2,2)*tag_3d_points[i].z+translation.at<double>(0,2);
            camera_pose.push_back(Point3f(x,y,z));

        }
        vector<Point3d> image_point3D;
        vector<Point2d> image_point2D;

        image_point3D.push_back(Point3d(0,0,50.0));


        projectPoints(image_point3D, rotation, translation, camera_matrix, dist_coeffs, image_point2D);

        line(src,tag_image_points[0], image_point2D[0],Scalar(0,255,0), 2);
        pose_estimation_failed=false;
        return camera_pose;
    }
    else
    {
        pose_estimation_failed=true;
        cout<<"image points are empty"<<endl;
    }
}

double rad2deg (double angle);

void Pose_Estimation::
show_pose_xyz(Mat src,Mat translation)
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


void Pose_Estimation::
show_pose_rotation(Mat src,Mat rotation)
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

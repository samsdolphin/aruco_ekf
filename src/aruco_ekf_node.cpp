#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/posetracker.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>

using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;

MarkerDetector MDetector;
MarkerMap MarkerMapConfig;
ros::Publisher odom_pub;
cv::Mat K, D;
Matrix3d w_R_tag = Quaterniond(0.707107, 0, 0.707107, 0).toRotationMatrix(); // TODO
Matrix3d imu_R_cam = Quaterniond(0.5, -0.5, 0.5, -0.5).toRotationMatrix(); //TODO
Vector3d imu_T_cam(0.08, 0, 0.03); //TODO
Vector3d tag_T_w(-1, 0, 0); //TODO
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd X = MatrixXd::Zero(15,1);
MatrixXd cov = MatrixXd::Identity(15,15);
Vector3d g(0,0,9.78);

double t = 0;
bool flag = false;
float MarkerSize = 0.04;
float MarkerWithMargin = 0.048;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double cur_t = msg->header.stamp.toSec();
    if(!flag)
    {
        t = cur_t;
        flag = true;
        return;
    }
    double delta_t = cur_t - t;
    t = cur_t;

    Vector3d ang_v(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Vector3d acc_raw(msg->linear_acceleration.x, msg->linear_acceleration.y,msg->linear_acceleration.z);

    MatrixXd At = MatrixXd::Zero(15,15);
    MatrixXd Ut = MatrixXd::Zero(15,12);
    At.block<3,3>(0,6) = MatrixXd::Identity(3,3);

    double phi = X(3,0);
    double theta = X(4,0);
    double psi = X(5,0);

    Vector3d y_temp = ang_v;
    Vector3d w = y_temp - X.block<3,1>(9,0);
    Vector3d acc = acc_raw - X.block<3,1>(12,0);
    Matrix3d G, Ginv, Rot, R_acc_dot, Ginv_w_dot;
    G <<    cos(theta),    0,     -cos(phi)*sin(theta),
            0,             1,           sin(phi),
            sin(theta),    0,      cos(phi)*cos(theta);

    Ginv<<   cos(theta),                       0,               sin(theta),
            (sin(phi)*sin(theta))/cos(phi),    1,              -(cos(theta)*sin(phi))/cos(phi),
            -sin(theta)/cos(phi),              0,               cos(theta)/cos(phi);
    Ginv_w_dot << 0,                                                                w(2)*cos(theta) - w(0)*sin(theta),                      0,
                 -(w(2)*cos(theta) - w(0)*sin(theta))/pow(cos(phi),2),              (sin(phi)*(w(0)*cos(theta) + w(2)*sin(theta)))/cos(phi), 0,
                 (sin(phi)*(w(2)*cos(theta) - w(0)*sin(theta)))/pow(cos(phi),2),  -(w(0)*cos(theta) + w(2)*sin(theta))/cos(phi),            0;
    R_acc_dot  << sin(psi)*(acc(1)*sin(phi) + acc(2)*cos(phi)*cos(theta) - acc(0)*cos(phi)*sin(theta)), acc(2)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - acc(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)),  -acc(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(2)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - acc(1)*cos(phi)*cos(psi),
                  -cos(psi)*(acc(1)*sin(phi) + acc(2)*cos(phi)*cos(theta) - acc(0)*cos(phi)*sin(theta)), acc(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(0)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)),   acc(0)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + acc(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc(1)*cos(phi)*sin(psi),
                  acc(1)*cos(phi) - acc(2)*cos(theta)*sin(phi) + acc(0)*sin(phi)*sin(theta),           -cos(phi)*(acc(0)*cos(theta) + acc(2)*sin(theta)),                                                                            0;
    At.block<3,3>(3,3) = Ginv_w_dot;
    At.block<3,3>(3,9) = -G.inverse();
    At.block<3,3>(6,3) = R_acc_dot;
    Ut.block<3,3>(3,0) = -G.inverse();

    Rot << cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
           cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),
           -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);

    At.block<3,3>(6,12) = -Rot;
    Ut.block<3,3>(6,3) = -Rot;
    Ut.block<6,6>(9,6) = MatrixXd::Identity(6,6);

    MatrixXd Ft = MatrixXd::Identity(15,15) + delta_t * At;

    MatrixXd Vt = delta_t * Ut;
    cov = Ft * cov * Ft.transpose() + Vt * Q * Vt.transpose();
    VectorXd f_value = MatrixXd::Zero(15,1);
    f_value.block<3,1>(0,0) = X.block<3,1>(6,0);
    f_value.block<3,1>(3,0) = Ginv * w;
    f_value.block<3,1>(6,0) = g + Rot * acc;
    X += delta_t * f_value;

    nav_msgs::Odometry odom;
    Matrix3d w_R_cam;
    w_R_cam << 0, 1, 0,
               0, 0, 1,
               1, 0, 0;
    Vector3d w_p = w_R_cam * X.block<3,1>(0,0);
    Vector3d w_v = w_R_cam * X.block<3,1>(6,0);
    odom.header.frame_id = "world";
    odom.header.stamp = msg->header.stamp;
    odom.pose.pose.position.x = w_p(0,0);
    odom.pose.pose.position.y = w_p(1,0);
    odom.pose.pose.position.z = w_p(2,0);

    odom.twist.twist.linear.x = w_v(0);
    odom.twist.twist.linear.y = w_v(1);
    odom.twist.twist.linear.z = w_v(2);

    double phi_t = X(3,0);
    double theta_t = X(4,0);
    double psi_t = X(5,0);
    Matrix3d R_rot;
    R_rot << cos(psi_t)*cos(theta_t)-sin(phi_t)*sin(psi_t)*sin(theta_t), -cos(phi_t)*sin(psi_t), cos(psi_t)*sin(theta_t)+cos(theta_t)*sin(phi_t)*sin(psi_t),
              cos(theta_t)*sin(psi_t)+cos(psi_t)*sin(phi_t)*sin(theta_t), cos(phi_t)*cos(psi_t), sin(psi_t)*sin(theta_t)-cos(psi_t)*cos(theta_t)*sin(phi_t),
              -cos(phi_t)*sin(theta_t), sin(phi_t), cos(phi_t)*cos(theta_t);
    R_rot = w_R_cam * R_rot;
    Quaterniond ori(R_rot);

    odom.pose.pose.orientation.x = ori.x();
    odom.pose.pose.orientation.y = ori.y();
    odom.pose.pose.orientation.z = ori.z();
    odom.pose.pose.orientation.w = ori.w();
    odom_pub.publish(odom);
}

void process(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d cam_R_tag;
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            cam_R_tag(i,j) = r.at<double>(i, j);
    Vector3d cam_T_tag(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));
    Matrix3d w_R_imu = w_R_tag * w_R_tag * cam_R_tag.transpose() * imu_R_cam.transpose();
    Vector3d w_T_imu = -w_R_tag * (tag_T_w + cam_R_tag.transpose() * cam_T_tag + cam_R_tag.transpose() * imu_R_cam.transpose() * imu_T_cam);

    double phi = asin(w_R_imu(2,1));
    double psi = atan2(-w_R_imu(0,1)/cos(phi),w_R_imu(1,1)/cos(phi));
    double theta = atan2(-w_R_imu(2,0)/cos(phi),w_R_imu(2,2)/cos(phi));

    Vector3d w_Ori_imu(phi, theta, psi);
    MatrixXd Ct = MatrixXd::Zero(6,15);
    Ct.block<6,6>(0,0) = MatrixXd::Identity(6,6);
    MatrixXd Wt = MatrixXd::Identity(6,6);

    MatrixXd temp = Ct * cov * Ct.transpose() + Wt * Rt * Wt.transpose();
    MatrixXd Kt = cov * Ct.transpose() * temp.inverse();
    VectorXd imu_pos(6,1);
    imu_pos.block<3,1>(0,0) = w_T_imu;
    imu_pos.block<3,1>(3,0) = w_Ori_imu;

    VectorXd odom_diff = imu_pos - Ct * X;
    if(odom_diff(5) > M_PI)
        odom_diff(5) -= 2*M_PI;

    else if(odom_diff(5) < -M_PI)
        odom_diff(5) += M_PI*2;

    if(odom_diff(4) > M_PI)
        odom_diff(4) -= 2*M_PI;

    else if(odom_diff(4) < -M_PI)
        odom_diff(4) += M_PI*2;

    if(odom_diff(3) > M_PI)
        odom_diff(3) -= 2*M_PI;

    else if(odom_diff(3) < -M_PI)
        odom_diff(3) += M_PI*2;

    X += Kt*odom_diff;
    cov -= Kt*Ct*cov;
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 4, idx_y = idx / 4;
    double p_x = idx_x * MarkerWithMargin - (2 + 1.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 0 || nth == 1) * MarkerSize, 0.0);
}

void odom_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    try
    {
        Mat InImage = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        vector<cv::Point3f> pts_3;
        vector<cv::Point2f> pts_2;

        for(auto m:MDetector.detect(InImage))
        {
            int idx = MarkerMapConfig.getIndexOfMarkerId(m.id); // idx starts from 0, right buttom

            char str[100];
            for (int i=0; i<4; i++)
            {
                pts_3.push_back(getPositionFromIndex(idx, i));
                pts_2.push_back(m[i]);
                sprintf(str, "%d", i);
                cv::putText(InImage, str, m[i], CV_FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0,0,255,255));
            }

            //m.draw(InImage);
        }

        if (pts_3.size() > 20)
            process(pts_3, pts_2, img_msg->header.stamp);

        imshow("view", InImage);
        waitKey(10);
    }
    catch(cv_bridge::Exception& ex)
    {
        ROS_ERROR("'%s'", ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_ekf");
    ros::NodeHandle nh;
    MDetector.setDictionary("ARUCO_MIP_36h12");
    namedWindow("view", 0); // 1:autosize, 0:resize
    resizeWindow("view", 640, 360);

    ros::Subscriber odom_sub = nh.subscribe("image_raw", 100, odom_callback);
    ros::Subscriber imu_sub = nh.subscribe("djiros/imu", 1000, imu_callback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom_ref", 10);

    MarkerMapConfig.readFromFile("/home/sam/catkin_ws/src/aruco_ekf/config/map.yml");
    FileStorage fs("/home/sam/catkin_ws/src/aruco_ekf/config/camera.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;

    ros::spin();
    destroyWindow("view");
}

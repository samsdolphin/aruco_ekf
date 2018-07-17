#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
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

CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
MarkerMap MarkerMapConfig;
ros::Publisher pub_odom;
cv::Mat K, D;
float MarkerSize = 0.04;
float MarkerMargin = 0.008;

void process(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0; i<3; i++)
    	for(int j=0; j<3; j++)
    		R_ref(i,j) = r.at<double>(i, j);
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom.publish(odom_ref);
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 4, idx_y = idx / 4;
    double p_x = idx_x * (MarkerMargin + MarkerSize);
    double p_y = idx_y * (MarkerMargin + MarkerSize);
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 0 || nth == 1) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
	try
	{
		Mat InImage = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
		vector<cv::Point3f> pts_3;
    	vector<cv::Point2f> pts_2;

		for(auto m:MDetector.detect(InImage))
		{
			int idx = MarkerMapConfig.getIndexOfMarkerId(m.id);

			char str[100];
	        for (unsigned int j = 0; j < 4; j++)
	        {
	        	pts_3.push_back(getPositionFromIndex(idx, j));
            	pts_2.push_back(m[j]);
	            sprintf(str, "%d", j);
	            cv::putText(InImage, str, m[j], CV_FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0,0,255,255));
	        }

			m.draw(InImage);
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
    ros::init(argc, argv, "aruco_pos");
    ros::NodeHandle nh;
    MDetector.setDictionary("ARUCO_MIP_36h12");
    namedWindow("view", 0); // 1:autosize, 0:resize
    resizeWindow("view", 640, 360);
    ros::Subscriber sub_img = nh.subscribe("image_raw", 100, img_callback);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom_ref", 10);
	
    MarkerMapConfig.readFromFile("/home/sam/catkin_ws/src/aruco_pos/config/map.yml");
    FileStorage fs("/home/sam/catkin_ws/src/aruco_pos/config/camera.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;

    ros::spin();
    destroyWindow("view");
}
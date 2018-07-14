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

//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace std;
using namespace cv;
using namespace aruco;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
MarkerMap TheMarkerMapConfig;
ros::Publisher pub_odom;
cv::Mat K, D;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
	try
	{
		Mat InImage = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
		for(auto m:MDetector.detect(InImage))
		{
			cout << m << endl;
			m.draw(InImage);
		}
		imshow("view", InImage);
		waitKey(30);
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
    namedWindow("view");	//subscribe to the image_raw topic, call img_callback once a msg received
    ros::Subscriber sub_img = nh.subscribe("image_raw", 100, img_callback);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom_ref",10);

    //init aruco detector
    string cam_config, map_config;
//    nh.getParam("cam_config_file", cam_config);
//    nh.getParam("map_config_file", map_config);
//    CamParam.readFromXMLFile("cam_config_file");
//    TheMarkerMapConfig.readFromFile(map_config);

    //init intrinsic parameters
    //cv::FileStorage param_reader(cam_config, cv::FileStorage::READ);
//    param_reader["camera_matrix"] >> K;
//    param_reader["distortion_coefficients"] >> D;

    ros::spin();
    destroyWindow("view");
}

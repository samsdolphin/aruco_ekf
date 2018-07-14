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
ros::Publisher pub_odom_ref;
cv::Mat K, D;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
/*
    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheMarkerMapConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp, rms);
*/
    cout<<"callback"<<endl;
    ROS_INFO("%s","callback!!");
    cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_pos");
    ros::NodeHandle n("~");

	//subscribe to the image_raw topic, call img_callback once a msg received
    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
//    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    
    //init aruco detector
    string cam_config, map_config;
    n.getParam("cam_config_file", cam_config);
    n.getParam("map_config_file", map_config);
    CamParam.readFromXMLFile(cam_config);
    TheMarkerMapConfig.readFromFile(map_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_config, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}

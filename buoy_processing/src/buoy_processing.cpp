#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

ros::Publisher coordinates_pub;
// calibration details for approximate depth estimation
float focal_length;
float known_width;
// thresholding paramters
int low_bgr[3];
int high_bgr[3];
// camera frame name
std::string camera_frame = "front_cam_link";

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_img_ptr;
  try
  {
    cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat src = cv_img_ptr->image;

    if(!src.empty())
    {
      //// pre-processing image
      // applying gaussian blur for smoothening
      cv::blur(src, src, cv::Size(3,3));
      cv::resize(src, src, cv::Size(320, 240), 0,0,cv::INTER_CUBIC);
      cv::imshow("rgb", src);
//      cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
//      cv::imshow("src", src);
      // thresholding image
      cv::Mat thresholded;
      cv::inRange(src, cv::Scalar(low_bgr[0], low_bgr[1], low_bgr[2]), cv::Scalar(high_bgr[0], high_bgr[1], high_bgr[2]), thresholded);
      // identifying contour
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      //// finding contour of largest area
      cv::findContours(thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      cv::imshow("thresholded", thresholded);
      ROS_INFO("contours size = %d", contours.size());

      if (contours.size() != 0)
      {
        int index = 0;
        float max_area = 0.0;
        float area = 0.0;
        for( int i = 0; i< contours.size(); i++ )
        {
          area = cv::contourArea(contours[i]);
          if(area > max_area)
          {
            index = i;
     	      max_area =area;
          }
        }
        try{
          // calculating center of mass of contour using moments
          std::vector<cv::Moments> mu(1);
          mu[0] = moments(contours[index], false);
          std::vector<cv::Point2f> mc(1);
          mc[0] = cv::Point2f( mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00 );
          if(contours[index].size()>=5){
            cv::Rect bounding_rectangle = cv::boundingRect(cv::Mat(contours[index]));
            // publish coordinates message
            geometry_msgs::PointStamped buoy_point_message;
            buoy_point_message.header.stamp = ros::Time();
            buoy_point_message.header.frame_id = camera_frame.c_str();
            buoy_point_message.point.x = (bounding_rectangle.br().x + bounding_rectangle.tl().x)/2 - (src.size().width)/2;
            buoy_point_message.point.y = ((float)src.size().height)/2 - (bounding_rectangle.br().y + bounding_rectangle.tl().y)/2;
            buoy_point_message.point.z = (known_width * focal_length) / (bounding_rectangle.br().x + bounding_rectangle.tl().x);

            ROS_INFO("Buoy Location (x, y, z) = (%.2f, %.2f, %.2f)", buoy_point_message.point.x, buoy_point_message.point.y, buoy_point_message.point.z);

            coordinates_pub.publish(buoy_point_message);

          }
        }
        catch(...){
          ROS_INFO("Hagg diya");
        }
      }
    }
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  // initiliazing ROS node
  ros::init(argc, argv, "buoy_processing");
  ros::NodeHandle nh;

  ROS_INFO("buoy_processing node initialized");

  // reading parameters from the paramter server
  //ros::param::get("buoy_processing/focal_length", focal_length);
  //ros::param::get("buoy_processing/known_width", known_width);
  //ros::param::get("buoy_processing/low_b", low_bgr[0]);
  //ros::param::get("buoy_processing/low_g", low_bgr[1]);
  //ros::param::get("buoy_processing/low_r", low_bgr[2]);
  //ros::param::get("buoy_processing/high_b", high_bgr[0]);
  //ros::param::get("buoy_processing/high_g", high_bgr[1]);
  //ros::param::get("buoy_processing/high_r", high_bgr[2]);
  low_bgr[0] = 107;
  low_bgr[1] = 0;
  low_bgr[2] = 21;
  high_bgr[0] = 255;
  high_bgr[1] = 255;
  high_bgr[2] = 116;
  ROS_INFO("Thresholding BGR with range: (%d, %d, %d) - (%d, %d, %d)", low_bgr[0], low_bgr[1], low_bgr[2], high_bgr[0], high_bgr[1], high_bgr[2]);

  // initializing publishers
  coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/buoy_processing/buoy_coordinates", 1000);

  //initializing subscribers
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_raw_sub = it.subscribe("/hardware_camera/cam_lifecam/image_raw", 1, imageCallback);

  ros::spin();

  return 0;
}

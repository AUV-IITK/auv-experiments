// Copyright 2018 AUV-IITKs
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <vision_buoy/buoyConfig.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float32MultiArray.h"
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>
#include <geometry_msgs/PointStamped.h>

int red_min, red_max, blue_min, blue_max, green_min, green_max;  // Default Params

cv::Mat frame;
cv::Mat newframe;
cv::Mat dst;

int count = 0;
int count_avg = 0;
void callback(vision_buoy::buoyConfig &config, uint32_t level)
{
  red_min = config.red_min_param;
  red_max = config.red_max_param;
  blue_min = config.blue_min_param;
  blue_max = config.blue_max_param;
  green_min = config.green_min_param;
  green_max = config.green_max_param;
  ROS_INFO("Buoy_Reconfigure Request:New params : %d %d %d %d %d %d", red_min, red_max, blue_min, blue_max, green_min, green_max);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    newframe.copyTo(frame);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
  }
}

void balance_white(cv::Mat mat) {
  double discard_ratio = 0.05;
  int hists[3][256];
  memset(hists, 0, 3*256*sizeof(int));

  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        hists[j][ptr[x * 3 + j]] += 1;
      }
    }
  }

  // cumulative hist
  int total = mat.cols*mat.rows;  
  int vmin[3], vmax[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 255; ++j) {
      hists[i][j + 1] += hists[i][j];
    }
    vmin[i] = 0;
    vmax[i] = 255;
    while (hists[i][vmin[i]] < discard_ratio * total)
      vmin[i] += 1;
    while (hists[i][vmax[i]] > (1 - discard_ratio) * total)
      vmax[i] -= 1;
    if (vmax[i] < 255 - 1)
      vmax[i] += 1;
  }


  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        int val = ptr[x * 3 + j];
        if (val < vmin[j])
          val = vmin[j];
        if (val > vmax[j])
          val = vmax[j];
        ptr[x * 3 + j] = static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
      }
    }
  }
}


int main(int argc, char *argv[])
{
  int height, width, step, channels;  // parameters of the image we are working on

  ros::init(argc, argv, "buoy_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PointStamped>("/threshold/center_coordinates", 1000);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  dynamic_reconfigure::Server<vision_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<vision_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  std::vector<cv::Point2f> center_ideal(5);

  float r[5];

  for (int m = 0; m++; m < 5)
    r[m] = 0;
    
  cv::Mat lab_image, balanced_image1, dstx, thresholded, image_clahe, dst;
  std::vector<cv::Mat> lab_planes(3);

  while (ros::ok())
  {
    // std_msgs::Float64MultiArray array;
    geometry_msgs::PointStamped point_coord;
    loop_rate.sleep();

    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    cv::Scalar bgr_min = cv::Scalar(blue_min, green_min, red_min, 0);
    cv::Scalar bgr_max = cv::Scalar(blue_max, green_max, red_max, 0);
    
    cv::cvtColor(frame, lab_image, CV_BGR2Lab);

    // Extract the L channel
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

    // convert back to RGB
    cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);
    
    for (int i=0; i < 6; i++)
    {
      bilateralFilter(image_clahe, dstx, 6, 8, 8);
      bilateralFilter(dstx, image_clahe, 6, 8, 8);
    }
    
    image_clahe.copyTo(balanced_image1);
    balance_white(image_clahe);
    
    for (int i=0; i < 2; i++)
    {
      bilateralFilter(image_clahe, dstx, 6, 8, 8);
      bilateralFilter(dstx, image_clahe, 6, 8, 8);
    }

    cv::inRange(image_clahe, bgr_min, bgr_max, thresholded);
    
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
    sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();

    pub2.publish(msg2);
    pub3.publish(msg3);
    
    std::cout << frame.cols << " " << frame.rows << std::endl;

    if (1)
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;
      double second_largest_area = 0, second_largest_contour_index = 0;

      if (contours.empty())
      {
        int x_cord = center_ideal[0].x - 320;
        int y_cord = 240 - center_ideal[0].y;
        if (x_cord < -290)
        {
          point_coord.point.x = -2;
          point_coord.point.y = -2;
          point_coord.point.z = -2;
          std::cerr << "BUOY on the left edge of the screen." << std::endl;
          pub.publish(point_coord);
        }
        else if (x_cord > 290)
        {
          point_coord.point.x = -1;
          point_coord.point.y = -1;
          point_coord.point.z = -1;
          std::cerr << "BUOY on the right edge of the screen." << std::endl;
          pub.publish(point_coord);
        }
        else if (y_cord > 210)
        {
          point_coord.point.x = -3;
          point_coord.point.y = -3;
          point_coord.point.z = -3;
          std::cerr << "BUOY on the top edge of the screen." << std::endl;
          pub.publish(point_coord);
        }
        else if (y_cord < -210)
        {
          point_coord.point.x = -4;
          point_coord.point.y = -4;
          point_coord.point.z = -4;
          std::cerr << "BUOY on the bottom edge of the screen." << std::endl;
          pub.publish(point_coord);
        }
        ros::spinOnce();
        continue;
      }

      for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
      {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > largest_area)
        {
          largest_area = a;
          largest_contour_index = i;  // Store the index of largest contour
        }
      }

      second_largest_contour_index = largest_contour_index;

      for (int i = 0; i < contours.size(); i++)
      {
        double a = contourArea(contours[i], false);
        if ((a > second_largest_area) && (a != largest_area))
        {
          second_largest_area = a;
          second_largest_contour_index = i;
        }
      }

      cv::Point2f center;
      float radius;

      cv::Point2f temp_center_1;
      cv::Point2f temp_center_2;

      float temp_radius_1;
      float temp_radius_2;

      cv::minEnclosingCircle(contours[largest_contour_index], temp_center_1, temp_radius_1);
      cv::minEnclosingCircle(contours[second_largest_contour_index], temp_center_2, temp_radius_2);

      if (temp_center_1.y > temp_center_2.y) {
        center.x = temp_center_1.x;
        center.y = temp_center_1.y;
        radius = temp_radius_1;
      }

      else {
        center.x = temp_center_2.x;
        center.y = temp_center_2.y;
        radius = temp_radius_2;
      }

      cv::Point2f pt;
      pt.x = 320;  // size of my screen
      pt.y = 240;

      float r_avg = (r[0] + r[1] + r[2] + r[3] + r[4]) / 5;
      if ((radius < (r_avg + 10)) && (count_avg >= 5))
      {
        r[4] = r[3];
        r[3] = r[2];
        r[2] = r[1];
        r[1] = r[0];
        r[0] = radius;
        center_ideal[4] = center_ideal[3];
        center_ideal[3] = center_ideal[2];
        center_ideal[2] = center_ideal[1];
        center_ideal[1] = center_ideal[0];
        center_ideal[0] = center;
        count_avg++;
      }
      else if (count_avg <= 5)
      {
        r[count_avg] = radius;
        center_ideal[count_avg] = center;
        count_avg++;
      }
      else
      {
        count_avg = 0;
      }

      cv::Mat circles = frame;
      circle(circles, center_ideal[0], r[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      circle(circles, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center made on the screen
      circle(circles, pt, 4, cv::Scalar(250, 0, 0), -1, 8, 0);             // center screen

      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", circles).toImageMsg();
      pub1.publish(msg1);

      int net_x_cord = center_ideal[0].x - 320;
      int net_y_cord = 240 - center_ideal[0].y;
      if (net_x_cord > 290)
      {
        point_coord.point.x = -1;
        point_coord.point.y = -1;
        point_coord.point.z = -1;
        std::cerr << "BUOY on the right edge of the screen." << std::endl;
        pub.publish(point_coord);
      }
      else if (net_x_cord < -290)
      {
        point_coord.point.x = -2;
        point_coord.point.y = -2;
        point_coord.point.z = -2;
        std::cerr << "BUOY on the left edge of the screen." << std::endl;
        pub.publish(point_coord);
      }
      else if (net_y_cord > 210)
      {
        point_coord.point.x = -3;
        point_coord.point.y = -3;
        point_coord.point.z = -3;
        std::cerr << "BUOY on the top edge of the screen." << std::endl;
        pub.publish(point_coord);
      }
      else if (net_y_cord < -210)
      {
        point_coord.point.x = -4;
        point_coord.point.y = -4;
        point_coord.point.z = -4;
        std::cerr << "BUOY on the bottom edge of the screen." << std::endl;
        pub.publish(point_coord);
      }
      else if (r[0] > 200)
      {
        point_coord.point.x = -5;
        point_coord.point.y = -5;
        point_coord.point.z = -5;
        std::cerr << "BUOY too near to the screen." << std::endl;
        pub.publish(point_coord);
      }
      else
      {
        float distance;
        distance = pow(radius / 7526.5, -.92678);  // function found using experiment
        point_coord.point.x = distance;
        point_coord.point.y = (center_ideal[0].x - 320);
        point_coord.point.z = (240 - center_ideal[0].y);
        
        pub.publish(point_coord);
      }

      std::cerr << "center coordinates x: " << center_ideal[0].x << " y: " << center_ideal[0].y << std::endl;

      ros::spinOnce();
    }
    else
    {
      ros::spinOnce();
    }
  }
  return 0;
}

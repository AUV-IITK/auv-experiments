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
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/buoy", 1000);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture/buoy", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture/buoy", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture/buoy", 1);

  dynamic_reconfigure::Server<vision_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<vision_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cvNamedWindow("BuoyDetection:circle", CV_WINDOW_NORMAL);
  cvNamedWindow("BuoyDetection:AfterThresholding", CV_WINDOW_NORMAL);
  cvNamedWindow("BuoyDetection:AfterEnhancing",CV_WINDOW_NORMAL);

  std::vector<cv::Point2f> center_ideal(5);

  float r[5];

  for (int m = 0; m++; m < 5)
    r[m] = 0;
    
  cv::Mat lab_image, balanced_image1, dstx, thresholded, image_clahe, dst;
  std::vector<cv::Mat> lab_planes(3);

  while (ros::ok())
  {
    double intial_loop_time = ros::Time::now().toSec();
    std_msgs::Float64MultiArray array;
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
    
    for (int i=0; i < 7; i++)
    {
      bilateralFilter(image_clahe, dstx, 6, 8, 8);
      bilateralFilter(dstx, image_clahe, 6, 8, 8);
    }
    // balance_white(dst2);
    
    image_clahe.copyTo(balanced_image1);
    balance_white(balanced_image1);
    
    for (int i=0; i < 2; i++)
    {
      bilateralFilter(balanced_image1, dstx, 6, 8, 8);
      bilateralFilter(dstx, balanced_image1, 6, 8, 8);
    }

    cv::inRange(balanced_image1, bgr_min, bgr_max, thresholded);
    
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    // cv::imshow("BuoyDetection:AfterEnhancing", balanced_image1);
    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
    pub2.publish(msg2);
    // cv::imshow("BuoyDetection:AfterThresholding", thresholded);

    sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();
    pub3.publish(msg3);


    if (1)
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;
      if (contours.empty())
      { 
        ROS_INFO("NO Contours detected");
        int x_cord = -320 + center_ideal[0].x;
        int y_cord = 240 - center_ideal[0].y;
        if (x_cord < -270)
        {
          array.data.push_back(-2);  // left edge of the screen
          array.data.push_back(-2);
          array.data.push_back(-2);
          array.data.push_back(-2);
          pub.publish(array);
	  ROS_INFO("Buoy was detected at left edge of the screen, last time");
        }
        else if (x_cord > 270)
        {
          array.data.push_back(-1);  // right edge of the screen
          array.data.push_back(-1);
          array.data.push_back(-1);
          array.data.push_back(-1);
          pub.publish(array);
	  ROS_INFO("Buoy was detected at right edge of the screen, last time");
        }
        else if (y_cord > 200)
        {
          array.data.push_back(-3);  // top edge of the screen
          array.data.push_back(-3);
          array.data.push_back(-3);
          array.data.push_back(-3);
          pub.publish(array);
 	  ROS_INFO("Buoy was detected at top edge of the screen, last time");
        }
        else if (y_cord < -200)
        {
          array.data.push_back(-4);  // bottom edge of the screen
          array.data.push_back(-4);
          array.data.push_back(-4);
          array.data.push_back(-4);
          pub.publish(array);
	  ROS_INFO("Buoy was detected at bottom edge of the screen, last time");
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

      std::vector<cv::Point2f> center(1);
      std::vector<float> radius(1);
      cv::minEnclosingCircle(contours[largest_contour_index], center[0], radius[0]);
      cv::Point2f pt;
      pt.x = 320;  // size of my screen
      pt.y = 240;

      float r_avg = (r[0] + r[1] + r[2] + r[3] + r[4]) / 5;
      if ((radius[0] < (r_avg + 10)) && (count_avg >= 5))
      {
        r[4] = r[3];
        r[3] = r[2];
        r[2] = r[1];
        r[1] = r[0];
        r[0] = radius[0];
        center_ideal[4] = center_ideal[3];
        center_ideal[3] = center_ideal[2];
        center_ideal[2] = center_ideal[1];
        center_ideal[1] = center_ideal[0];
        center_ideal[0] = center[0];
        count_avg++;
      }
      else if (count_avg <= 5)
      {
        r[count_avg] = radius[0];
        center_ideal[count_avg] = center[0];
        count_avg++;
      }
      else
      {
        count_avg = 0;
      }

      cv::Mat circles = frame;
      circle(circles, center_ideal[0], r[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      circle(circles, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center is made on the screen
      circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);             // center of screen

      int net_x_cord = -320 + center_ideal[0].x + r[0];
      int net_y_cord = 240 - center_ideal[0].y + r[0];
      if (net_x_cord > 310)
      {
        array.data.push_back(-2);  // right edge of the frame
        array.data.push_back(-2);
        array.data.push_back(-2);
        array.data.push_back(-2);
        pub.publish(array);
	ROS_INFO("Contour detected at right edge of the screen");
      }
      else if (net_x_cord < -310)
      {
        array.data.push_back(-1);  // left edge of the frame
        array.data.push_back(-1);
        array.data.push_back(-1);
        array.data.push_back(-1);
        pub.publish(array);
        ros::spinOnce();
	ROS_INFO("Contour detected at left edge of the screen");
      }
      else if (net_y_cord > 230)
      {
        array.data.push_back(-3);  // top edge of the frame
        array.data.push_back(-3);
        array.data.push_back(-3);
        array.data.push_back(-3);
        pub.publish(array);
	ROS_INFO("Contour detected at top edge of the screen");
      }
      else if (net_y_cord < -230)
      {
        array.data.push_back(-4);  // bottom edge of the frame
        array.data.push_back(-4);
        array.data.push_back(-4);
        array.data.push_back(-4);
        pub.publish(array);
	ROS_INFO("Contour detected at bottom edge of the screen");
      }
      else if (r[0] > 110)
      {
        array.data.push_back(-5); // too near of the buoy
        array.data.push_back(-5);
        array.data.push_back(-5);
        array.data.push_back(-5);
        pub.publish(array);
	ROS_INFO("Contour detected too near to the screen");
      }
      else
      {
        float distance;
        distance = pow(radius[0] / 7526.5, -.92678);  // function found using experiment
        array.data.push_back(r[0]);                   // publish radius
        array.data.push_back((-320 + center_ideal[0].x));
        array.data.push_back((240 - center_ideal[0].y));
        array.data.push_back(distance);
        pub.publish(array);
	ROS_INFO("Contour detected, Data published, Distance: %f, X_cord: %f, Y_cord: %f, Radius: %f", distance, center_ideal[0].x - 320, 240 - center_ideal[0].y, r[0]);
      }
      // cv::imshow("BuoyDetection:circle", circles);  // Original stream with detected ball overlay
	
	sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

	pub1.publish(msg1);

      ros::spinOnce();
    }
    else
    {
      ros::spinOnce();
    }

    double end_loop_time = ros::Time::now().toSec();
    double total_loop_time = end_loop_time - intial_loop_time;

    std::cout << "Total Loop time: " << total_loop_time << std::endl;	

  }
  return 0;
}

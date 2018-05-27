#include "../include/lib.hpp"
#include <vision_buoy/buoyConfig.h>
#include "../include/pre_processing.h"

int red_min, red_max, blue_min, blue_max, green_min, green_max;  // Default Params
int count = 0;
cv::Mat frame;
cv::Mat newframe;
cv::Mat dst;
cv::Mat detected_buoy;

cv::Mat lab_image, after_image_correction, thresholded_image, image_clahe, after_white_balance;

std::vector<cv::Point2f> center_ideal(5);
cv::Point2f current_center;
float current_radius;

std::vector<float> ideal_radius_stream(5);

std::vector<std::vector<cv::Point> > contours_image;

std_msgs::Float64MultiArray array;

ros::NodeHandle n;
ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/buoy", 1000);

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
        count++;
        newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
        newframe.copyTo(frame);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
    }

    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
    
    }

    image_clahe = pre_processing::color_correction(frame, 4);

    pre_processing::denoise(image_clahe, 7); // for removing the colored noise

    image_clahe.copyTo(after_image_correction);
    after_white_balance = pre_processing::balance_white(after_image_correction, 0.05);

    pre_processing::denoise(after_white_balance, 2);

    cv::Scalar bgr_min = cv::Scalar(blue_min, green_min, red_min, 0);
    cv::Scalar bgr_max = cv::Scalar(blue_max, green_max, red_max, 0);

    // thresholding all the colors according to their thresholding values
    cv::inRange(after_white_balance, bgr_min, bgr_max, thresholded_image);

    findContours(thresholded_image, contours_image, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours

    if (contours_image.empty())
    {
        array = post_processing::empty_contour_handler(center_ideal[0]);
        pub.publish(array);
        ros::spinOnce();
    
    }

    double largest_contour_area = 0, largest_contour_index = 0;

    for (int i = 0; i < contours_image.size(); i++)  // iterate through each contour.
    {
        double a = contourArea(contours_image[i], false);  //  Find the area of contour
        if (a > largest_contour_area)
        {
            largest_contour_area = a;
            largest_contour_index = i;  // Store the index of largest contour
        }
    }

    cv::minEnclosingCircle(contours_image[largest_contour_index], current_center, current_radius);
    cv::Point2f screen_center;
    screen_center.x = 320;  // size of my screen
    screen_center.y = 240;

    float radius_avg = (ideal_radius_stream[0] + ideal_radius_stream[1] + ideal_radius_stream[2] + ideal_radius_stream[3] + ideal_radius_stream[4]) / 5;
    
    if ((current_radius < (radius_avg + 10)) && (count >= 5))
    {
        ideal_radius_stream[4] = ideal_radius_stream[3];
        ideal_radius_stream[3] = ideal_radius_stream[2];
        ideal_radius_stream[2] = ideal_radius_stream[1];
        ideal_radius_stream[1] = ideal_radius_stream[0];
        ideal_radius_stream[0] = current_radius;
        center_ideal[4] = center_ideal[3];
        center_ideal[3] = center_ideal[2];
        center_ideal[2] = center_ideal[1];
        center_ideal[1] = center_ideal[0];
        center_ideal[0] = current_center;
        count++;
    }
    else if (count <= 5)
    {
        ideal_radius_stream[count] = current_radius;
        center_ideal[count] = current_center;
        count++;
    }
    else
    {
        count = 0;
    }

    detected_buoy = frame;
    circle(detected_buoy, center_ideal[0], ideal_radius_stream[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
    circle(detected_buoy, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center is made on the screen
    circle(detected_buoy, screen_center, 4, cv::Scalar(150, 150, 150), -1, 8, 0);             // center of screen

    array = post_processing::edge_case_handler(center_ideal[0], ideal_radius_stream[0]);
    pub.publish(array);
    ros::spinOnce();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "buoy_detection");

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);

  dynamic_reconfigure::Server<vision_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<vision_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cvNamedWindow("BuoyDetection: AfterWhiteBalance", CV_WINDOW_NORMAL);
  cvNamedWindow("BuoyDetection: AfterThresholding", CV_WINDOW_NORMAL);
  cvNamedWindow("BuoyDetection: AfterColorCorrection",CV_WINDOW_NORMAL);
  cvNamedWindow("BuoyDetection: AfterContours", CV_WINDOW_NORMAL);

  cv::waitKey(0);
    
  return 0;
}

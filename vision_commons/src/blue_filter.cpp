#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/xphoto/white_balance.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

image_transport::Publisher white_balanced_pub;
image_transport::Publisher histogram_equalized_pub;
image_transport::Publisher blue_filtered_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
	cv_bridge::CvImagePtr cv_img_ptr;
	try {
		cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_img_ptr->image;
		if(!image.empty()){

			cv::resize(image, image, cv::Size(640, 480));

			cv::Mat white_balanced;
			cv::xphoto::createGrayworldWB()->balanceWhite(image, white_balanced);
			cv_bridge::CvImage white_balanced_ptr;
			white_balanced_ptr.header = msg->header;
			white_balanced_ptr.encoding = msg->encoding;
			white_balanced_ptr.image = white_balanced;

			cv::Mat lab_image;
			cv::cvtColor(image, lab_image, CV_BGR2Lab);
			std::vector<cv::Mat> lab_planes(3);
			cv::split(lab_image, lab_planes);
			cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4, cv::Size(3, 3));
			clahe->setClipLimit(8);
			cv::Mat l0dst;
			clahe->apply(lab_planes[0], l0dst);
			cv::Mat l1dst;
			clahe->apply(lab_planes[1], l1dst);
			cv::Mat l2dst;
			clahe->apply(lab_planes[2], l2dst);
			l0dst.copyTo(lab_planes[0]);
			l1dst.copyTo(lab_planes[1]);
			l2dst.copyTo(lab_planes[2]);
			cv::merge(lab_planes, lab_image);
			cv::Mat histogram_equalized;
			cv::cvtColor(lab_image, histogram_equalized, CV_Lab2BGR);

			cv_bridge::CvImage histogram_equalized_ptr;
			histogram_equalized_ptr.header = msg->header;
			histogram_equalized_ptr.encoding = msg->encoding;
			histogram_equalized_ptr.image = histogram_equalized;

			cv::Mat blue_filtered;
			cv::addWeighted(white_balanced, 0.5, histogram_equalized, 0.5, 0.0, blue_filtered);
			cv::fastNlMeansDenoisingColored(blue_filtered, blue_filtered, 4.0, 4.0, 5, 11);
			cv_bridge::CvImage blue_filtered_ptr;
			blue_filtered_ptr.header = msg->header;
			blue_filtered_ptr.encoding = msg->encoding;
			blue_filtered_ptr.image = blue_filtered;

			white_balanced_pub.publish(white_balanced_ptr.toImageMsg());
			histogram_equalized_pub.publish(histogram_equalized_ptr.toImageMsg());
			blue_filtered_pub.publish(blue_filtered_ptr.toImageMsg());
		}
	}
	catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

int main(int argc, char **argv) {
	cv::Mat image;
	ros::init(argc, argv, "blue_filter");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	white_balanced_pub = it.advertise("/input1", 1);
	histogram_equalized_pub = it.advertise("/input2", 1);
	blue_filtered_pub = it.advertise("/blue_filtered", 1);
	image_transport::Subscriber image_raw_sub = it.subscribe("/hardware_camera/cam_lifecam/image_raw", 1, imageCallback);
	ros::spin();
	return 0;
}

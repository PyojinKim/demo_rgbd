#include <math.h>
#include <typeinfo>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "cameraParameters.h"
#include "pointDefinition.h"

int imageCounter = 0;
const int imagePixelNum = imageHeight * imageWidth;
CvSize imgSize = cvSize(imageWidth, imageHeight);


void imageMonoCallback(const sensor_msgs::ImageConstPtr& msgImageMono)
{
  // get current mono image as CV_8UC1 (user can define image datatype)
  cv::Mat monoImage;
  try {
    monoImage = cv_bridge::toCvShare(msgImageMono, sensor_msgs::image_encodings::TYPE_8UC1)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  // plot current mono image
  cv::namedWindow("current mono image", cv::WINDOW_AUTOSIZE);// Create a window for display.
  cv::imshow("current mono image", monoImage);
  cv::waitKey(30);


  // for debug
  std::cout << (monoImage.type() == CV_8UC1) << std::endl;
  std::cout << "msgImageMono->height : " << msgImageMono->height << std::endl;
  std::cout << "msgImageMono->width : " << msgImageMono->width << std::endl;
  printf("%d \n", monoImage.at<unsigned char>(240,320));
  std::cout << "msgImageMono->header.stamp.toSec() : " << msgImageMono->header.stamp.toSec() << std::endl;
}


void imageDepthCallback(const sensor_msgs::ImageConstPtr& msgImageDepth)
{
  // get current depth image as CV_64FC1 (user can define image datatype)
  cv::Mat depthImage;
  try {
    depthImage = cv_bridge::toCvShare(msgImageDepth, sensor_msgs::image_encodings::TYPE_64FC1)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  // plot current depth image
  cv::namedWindow("current depth image", cv::WINDOW_AUTOSIZE);// Create a window for display.
  cv::imshow("current depth image", depthImage);
  cv::waitKey(30);


  // for debug
  std::cout << (depthImage.type() == CV_64FC1) << std::endl;
  std::cout << "msgImageDepth->height : " << msgImageDepth->height << std::endl;
  std::cout << "msgImageDepth->width : " << msgImageDepth->width << std::endl;
  std::cout << depthImage.at<double>(240,320) << std::endl;
  std::cout << "msgImageDepth->header.stamp.toSec() : " << msgImageDepth->header.stamp.toSec() << std::endl;
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "realsenseTopicsSubscriber");
  ros::NodeHandle nh;

  // subscribe mono and depth image
  ros::Subscriber monoImageDataSub = nh.subscribe("/camera/rgb/image_rect_mono", 1, imageMonoCallback);
  ros::Subscriber depthImageDataSub = nh.subscribe("/camera/depth_aligned/image_rect", 1, imageDepthCallback);


  ros::spin();

  return 0;
}

/*
/camera/rgb/image_rect_mono          uint8_t(CV_8UC1)
*/

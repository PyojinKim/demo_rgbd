#include <math.h>
#include <typeinfo>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "cameraParameters.h"
#include "pointDefinition.h"

ros::Publisher *imageMonoPubPointer;
ros::Publisher *imageDepthPubPointer;

void imageColorCallback(const sensor_msgs::ImageConstPtr& msgImageColor)
{
  // Get current color image as CV_8UC3
  cv::Mat colorImage;
  try {
    colorImage = cv_bridge::toCvShare(msgImageColor, sensor_msgs::image_encodings::TYPE_8UC3)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  // Convert to mono (CV_8UC1) image
  cv::Mat monoImage;
  cvtColor(colorImage, monoImage, CV_BGR2GRAY);
  monoImage.convertTo(monoImage, CV_8UC1);


  // Create image messages
  sensor_msgs::ImagePtr msgImageMono = cv_bridge::CvImage(std_msgs::Header(), "mono8", monoImage).toImageMsg();
  msgImageMono->header.stamp = ros::Time(msgImageColor->header.stamp.toSec());


  // Publish images
  imageMonoPubPointer->publish(msgImageMono);
}


void imageDepthCallback(const sensor_msgs::ImageConstPtr& msgImageDepth)
{
  // Get current depth image as CV_64FC1 (user can define image datatype)
  cv::Mat depthImage;
  try {
    depthImage = cv_bridge::toCvShare(msgImageDepth, sensor_msgs::image_encodings::TYPE_64FC1)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  // Convert to depth meter (1000) level (TUM depthImage is [m] unit.)
  cv::Mat imageDepth;
  imageDepth = depthImage * 1000;
  imageDepth.convertTo(imageDepth, CV_16UC1);


  // Create image messages
  sensor_msgs::ImagePtr msgImageDepth_temp = cv_bridge::CvImage(std_msgs::Header(), "mono16", imageDepth).toImageMsg();
  msgImageDepth_temp->header.stamp = ros::Time(msgImageDepth->header.stamp.toSec());


  // Publish images
  imageDepthPubPointer->publish(msgImageDepth_temp);
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "TUMdatasetTopicConverter");
  ros::NodeHandle nh;

  // subscribe color and depth image
  ros::Subscriber colorImageDataSub = nh.subscribe("/camera/rgb/image_color", 1, imageColorCallback);
  ros::Subscriber depthImageDataSub = nh.subscribe("/camera/depth/image", 1, imageDepthCallback);

  // publish mono and depth image
  ros::Publisher imageMonoPub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_rect_mono", 1);
  imageMonoPubPointer = &imageMonoPub;

  ros::Publisher imageDepthPub = nh.advertise<sensor_msgs::Image>("/camera/depth_aligned/image_rect", 1);
  imageDepthPubPointer = &imageDepthPub;


  ros::spin();

  return 0;
}

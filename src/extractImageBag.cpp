#include <math.h>
#include <typeinfo>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "cameraParameters.h"
#include "pointDefinition.h"

std::string rgbFolderName, depthFolderName;
uint32_t rgbImageCounter, depthImageCounter;
std::string rgbImageFileName, depthImageFileName;

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

  // change current rgb image file name
  char rgbImageIndex[255];
  sprintf(rgbImageIndex, "%010d.png", rgbImageCounter);
  rgbImageFileName = rgbFolderName + rgbImageIndex;
  std::cout << rgbImageFileName << std::endl;
  rgbImageCounter++;

  // save the current rgb image
  imwrite(rgbImageFileName, monoImage);
}


void imageDepthCallback(const sensor_msgs::ImageConstPtr& msgImageDepth)
{
  // get current depth image as CV_16UC1 (user can define image datatype)
  cv::Mat depthImage;
  try {
    depthImage = cv_bridge::toCvShare(msgImageDepth, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // change current depth image file name
  char depthImageIndex[255];
  sprintf(depthImageIndex, "%010d.png", depthImageCounter);
  depthImageFileName = depthFolderName + depthImageIndex;
  std::cout << depthImageFileName << std::endl;
  depthImageCounter++;

  // save the current depth image
  imwrite(depthImageFileName, depthImage);
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "extractImageBag");
  ros::NodeHandle nh;

  // make folders for saving ROS topic images
  rgbFolderName = "bagImages/rgb/";
  depthFolderName = "bagImages/depth/";
  std::string rgbFolderCreateCommand = "mkdir -p " + rgbFolderName;
  std::string depthFolderCreateCommand = "mkdir -p " + depthFolderName;
  system(rgbFolderCreateCommand.c_str());
  system(depthFolderCreateCommand.c_str());

  rgbImageCounter = 0;
  depthImageCounter = 0;

  // subscribe mono and depth image
  ros::Subscriber monoImageDataSub = nh.subscribe("/camera/rgb/image_rect_mono", 1, imageMonoCallback);
  ros::Subscriber depthImageDataSub = nh.subscribe("/camera/depth_aligned/image_rect", 1, imageDepthCallback);


  ros::spin();

  return 0;
}

/*
/camera/rgb/image_rect_mono          uint8_t(CV_8UC1)
*/

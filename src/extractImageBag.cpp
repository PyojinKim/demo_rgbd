#include <math.h>
#include <typeinfo>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "cameraParameters.h"
#include "pointDefinition.h"

std::string folderName, rgbImageFileName, depthImageFileName;
FILE *rgbTextStrm;
FILE *depTextStrm;

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

  // save rgb image time and name in text file
  double rgbImageTime = msgImageMono->header.stamp.toSec();
  char rgbImageIndex[255];
  sprintf(rgbImageIndex, "rgb/%15.5lf.png", rgbImageTime);
  fprintf(rgbTextStrm, "%15.5lf \t %s \n", rgbImageTime, rgbImageIndex);

  // save the current rgb image
  rgbImageFileName = folderName + rgbImageIndex;
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

  // save rgb image time and name in text file
  double depthImageTime = msgImageDepth->header.stamp.toSec();
  char depthImageIndex[255];
  sprintf(depthImageIndex, "depth/%15.5lf.png", depthImageTime);
  fprintf(depTextStrm, "%15.5lf \t %s \n", depthImageTime, depthImageIndex);

  // save the current depth image
  depthImageFileName = folderName + depthImageIndex;
  imwrite(depthImageFileName, depthImage);
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "extractImageBag");
  ros::NodeHandle nh;

  // make folders for saving ROS topic images
  folderName = "bagImages/";
  std::string rgbFolderCreateCommand = "mkdir -p " + folderName + "rgb/";
  std::string depthFolderCreateCommand = "mkdir -p " + folderName + "depth/";
  system(rgbFolderCreateCommand.c_str());
  system(depthFolderCreateCommand.c_str());

  // make text files for time sync
  std::string rgbTextFileName = folderName + "rgb.txt";
  std::string depthTextFileName = folderName + "depth.txt";
  rgbTextStrm = fopen(rgbTextFileName.c_str(),"w");
  depTextStrm = fopen(depthTextFileName.c_str(),"w");

  // subscribe mono and depth image
  ros::Subscriber monoImageDataSub = nh.subscribe("/camera/rgb/image_rect_mono", 1, imageMonoCallback);
  ros::Subscriber depthImageDataSub = nh.subscribe("/camera/depth_aligned/image_rect", 1, imageDepthCallback);


  ros::spin();

  return 0;
}

/*
/camera/rgb/image_rect_mono          uint8_t(CV_8UC1)
*/

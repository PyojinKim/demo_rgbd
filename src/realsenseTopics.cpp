#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense/rs.hpp>


int main(int argc, char** argv)
{
  // Create a context object. This object owns the handles to all connected realsense devices.
  rs::context ctx;
  ROS_INFO("There are %d connected RealSense devices.", ctx.get_device_count());
  if(ctx.get_device_count() == 0) {
    ROS_ERROR("No devece is connected");
    return EXIT_FAILURE;
  }


  // Connect Intel RealSense device 0 (access only a single device, but it is trivial to extend to multiple devices)
  rs::device * dev = ctx.get_device(0);
  float depth_scale = dev->get_depth_scale();
  ROS_INFO("Using device 0, an %s", dev->get_name());
  ROS_INFO("Serial number: %s", dev->get_serial());
  ROS_INFO("Firmware version: %s", dev->get_firmware_version());
  ROS_INFO("Depth scale: %f", depth_scale);


  // Various options for RealSense R200 camera.
  //dev->set_option(rs::option::r200_lr_exposure, 666);
  //dev->set_option(rs::option::color_exposure, 666);
  //dev->set_option(rs::option::color_gain, 12);
  //dev->set_option(rs::option::color_saturation, 120);
  //dev->set_option(rs::option::color_enable_auto_exposure, 1);
  //dev->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);
  //dev->set_option(rs::option::frames_queue_size, 10);


  // Configure depth to run at VGA resolution at 60 frames per second
  ROS_INFO("start Intel RealSense streaming");
  dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 60);
  dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
  dev->start();


  // Create ROS node to publish images
  ros::init(argc, argv, "realsenseTopics");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher imageColorPub = it.advertise("/camera/rgb/image_rect", 1);
  image_transport::Publisher imageDepthPub = it.advertise("/camera/depth_aligned/image_rect", 1);
  image_transport::Publisher imageMonoPub = it.advertise("/camera/rgb/image_rect_mono", 1);
  ros::Time timeColorCapture = ros::Time::now();
  ros::Time timeDepthCapture = ros::Time::now();


  // main loop
  ros::Rate loop_rate(60);
  while(nh.ok()) {
    dev->wait_for_frames();
    // Get color and depth frame and Change the format to Mat files
    const void * colorFrame = dev->get_frame_data(rs::stream::rectified_color);
    timeColorCapture = ros::Time::now();
    const void * depthFrame = dev->get_frame_data(rs::stream::depth_aligned_to_rectified_color);
    timeDepthCapture = ros::Time::now();
    cv::Mat imageColor(480, 640, CV_8UC3, (void*)colorFrame);
    cv::Mat imageDepth(480, 640, CV_16UC1, (void*)depthFrame);

    // Convert to mono (CV_8UC1) image
    cv::Mat imageMono;
    cvtColor(imageColor, imageMono, CV_BGR2GRAY);
    imageMono.convertTo(imageMono, CV_8UC1);

    // Create image messages
    sensor_msgs::ImagePtr msgImageColor = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageColor).toImageMsg();
    msgImageColor->header.stamp = ros::Time(timeColorCapture);
    sensor_msgs::ImagePtr msgImageDepth = cv_bridge::CvImage(std_msgs::Header(), "mono16", imageDepth).toImageMsg();
    msgImageDepth->header.stamp = ros::Time(timeDepthCapture);
    sensor_msgs::ImagePtr msgImageMono = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageMono).toImageMsg();
    msgImageMono->header.stamp = ros::Time(timeColorCapture);

    // Publish images
    imageColorPub.publish(msgImageColor);
    imageDepthPub.publish(msgImageDepth);
    imageMonoPub.publish(msgImageMono);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}


/* enum class stream : int32_t
{
depth                           ,  ///< Native stream of depth data produced by RealSense device
color                           ,  ///< Native stream of color data captured by RealSense device
infrared                        ,  ///< Native stream of infrared data captured by RealSense device
infrared2                       ,  ///< Native stream of infrared data captured from a second viewpoint by RealSense device
fisheye                         ,
points                          ,  ///< Synthetic stream containing point cloud data generated by deprojecting the depth image
rectified_color                 ,  ///< Synthetic stream containing undistorted color data with no extrinsic rotation from the depth stream
color_aligned_to_depth          ,  ///< Synthetic stream containing color data but sharing intrinsic of depth stream
infrared2_aligned_to_depth      ,  ///< Synthetic stream containing second viewpoint infrared data but sharing intrinsic of depth stream
depth_aligned_to_color          ,  ///< Synthetic stream containing depth data but sharing intrinsic of color stream
depth_aligned_to_rectified_color, ///< Synthetic stream containing depth data but sharing intrinsic of rectified color stream
depth_aligned_to_infrared2        ///< Synthetic stream containing depth data but sharing intrinsic of second viewpoint infrared stream
}; */
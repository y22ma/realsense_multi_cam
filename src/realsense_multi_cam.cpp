// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>

#include <iostream>
#include <algorithm>

int main(int argc, char * argv[]) try
{
  ros::init(argc, argv, "kcf_tracker");
  ros::NodeHandle nh;

  int32_t fps;
  ros::param::param<int32_t>("fps", fps, 30);

  rs::context ctx;
  if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

  // Enumerate all devices
  std::vector<rs::device *> devices;
  std::vector<ros::Publisher> depth_pubs(ctx.get_device_count());
  std::vector<ros::Publisher> rgb_pubs(ctx.get_device_count());
  for(int i = 0; i < ctx.get_device_count(); ++i)
  {
    devices.push_back(ctx.get_device(i));
    depth_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" + std::to_string(i) + "/depth/image_raw", 1, true);
    rgb_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" + std::to_string(i) + "/rgb/image_raw", 1, true);
  }

  // Configure and start our devices
  for(auto dev : devices)
  {
    std::cout << "Starting " << dev->get_name() << "... ";
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, fps);
    dev->enable_stream(rs::stream::color, 1920, 1080, rs::format::bgr8, fps);
    dev->start();
    std::cout << "done." << std::endl;
  }

  // Depth and color
  cv::Mat depth_img(cv::Size(640, 480), CV_16UC1);
  cv::Mat rgb_img(cv::Size(1920, 1080), CV_8UC3);
  cv_bridge::CvImage cv_img;

  std_msgs::Header header;
  while (ros::ok())
  {
    int i = 0;
    for (auto dev : devices)
    {
      header.stamp = ros::Time::now();
      dev->wait_for_frames();
      const uint16_t* depth_frame = reinterpret_cast<const uint16_t*>(
          dev->get_frame_data(rs::stream::depth));
      memcpy(depth_img.data, depth_frame, depth_img.cols*depth_img.rows*sizeof(uint16_t));
      header.frame_id = "depth_frame";
      cv_img.header = header;
      cv_img.encoding = "mono16";
      cv_img.image = depth_img;
      depth_pubs[i].publish(cv_img.toImageMsg());

      const uint8_t* rgb_frame = reinterpret_cast<const uint8_t*>(
          dev->get_frame_data(rs::stream::color));
      memcpy(rgb_img.data, rgb_frame, rgb_img.cols*rgb_img.rows*sizeof(uint8_t)*rgb_img.channels());
      header.frame_id = "rgb_frame";
      cv_img.header = header;
      cv_img.encoding = "bgr8";
      cv_img.image = rgb_img;
      rgb_pubs[i++].publish(cv_img);
    }
    ++header.seq;
  }

  return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
        << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

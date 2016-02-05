// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>

#include <iostream>
#include <algorithm>

void rsIntrinsics2CameraInfo(const rs::intrinsics& intrinsics, sensor_msgs::CameraInfo& cam_info)
{
  cam_info.width  = intrinsics.width;
  cam_info.height = intrinsics.height;
  cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  cam_info.D.resize(5);
  cam_info.D[0] = intrinsics.coeffs[0];
  cam_info.D[1] = intrinsics.coeffs[1];
  cam_info.D[2] = intrinsics.coeffs[2];
  cam_info.D[3] = intrinsics.coeffs[3];
  cam_info.D[4] = intrinsics.coeffs[4];
  cam_info.K[0] = intrinsics.fx; cam_info.K[1] = 0; cam_info.K[2] = intrinsics.ppx;
  cam_info.K[3] = 0; cam_info.K[4] = intrinsics.fy; cam_info.K[5] = intrinsics.ppy;
  cam_info.K[6] = 0; cam_info.K[7] = 0; cam_info.K[8] = 1;
  cam_info.binning_x = 1;
  cam_info.binning_y = 1;
  cam_info.roi.x_offset = 0;
  cam_info.roi.y_offset = 0;
  cam_info.roi.width = cam_info.width;
  cam_info.roi.height = cam_info.height;
  cam_info.roi.do_rectify = false;
}


void publishPoints(const std_msgs::Header& header, const cv::Mat& depth_img,
    const ros::Publisher& publisher, rs::intrinsics& depth_intrinsics)
{
  sensor_msgs::PointCloud points_msg;
  points_msg.header = header;

  for(int dy = 0; dy < depth_intrinsics.height; ++dy)
  {
    for(int dx = 0; dx < depth_intrinsics.width; ++dx)
    {
      // Retrieve the 16-bit depth value and map it into a depth in meters
      uint16_t depth_value = depth_img.at<uint16_t>(dy, dx);
      float depth_in_meters = depth_value * 1000;

      // Skip over pixels with a depth value of zero, which is used to indicate no data
      if(depth_value == 0) continue;

      // Map from pixel coordinates in the depth image to pixel coordinates in the color image
      rs::float2 depth_pixel = {(float)dx, (float)dy};
      rs::float3 depth_point = depth_intrinsics.deproject(depth_pixel, depth_in_meters);

      geometry_msgs::Point32 point;
      point.x = depth_point.x;
      point.y = depth_point.y;
      point.z = depth_point.z;

      points_msg.points.push_back(point);
    }
  }
  publisher.publish(points_msg);
}


int main(int argc, char * argv[]) try
{
  ros::init(argc, argv, "realsense_multi_cam");
  ros::NodeHandle nh;

  int32_t skip_frame_num;
  bool pub_points;
  ros::param::param<int32_t>("~skip_frame_num", skip_frame_num, 5);
  ros::param::param<bool>("~pub_points", pub_points, false);

  int32_t rgb_width, rgb_height, depth_width, depth_height;
  ros::param::param<int32_t>("~rgb_width", rgb_width, 1920);
  ros::param::param<int32_t>("~rgb_height", rgb_height, 1080);
  ros::param::param<int32_t>("~depth_width", depth_width, 640);
  ros::param::param<int32_t>("~depth_height", depth_height, 480);

  rs::context ctx;
  if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

  // Enumerate all devices
  std::vector<rs::device *> devices;
  std::vector<ros::Publisher> depth_pubs(ctx.get_device_count());
  std::vector<ros::Publisher> rgb_pubs(ctx.get_device_count());
  std::vector<ros::Publisher> depth_info_pubs(ctx.get_device_count());
  std::vector<ros::Publisher> rgb_info_pubs(ctx.get_device_count());
  std::vector<ros::Publisher> point_pubs(ctx.get_device_count());
  std::vector<sensor_msgs::CameraInfo> depth_infos(ctx.get_device_count());
  std::vector<sensor_msgs::CameraInfo> rgb_infos(ctx.get_device_count());
  for(int i = 0; i < ctx.get_device_count(); ++i)
  {
    devices.push_back(ctx.get_device(i));
    depth_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" + std::to_string(i) +
        "/depth/image_raw", 1, true);
    rgb_pubs[i] = nh.advertise<sensor_msgs::Image>("camera" + std::to_string(i) +
        "/rgb/image_raw", 1, true);
    depth_info_pubs[i] = nh.advertise<sensor_msgs::CameraInfo>("camera" +
        std::to_string(i) + "/depth/camera_info", 1, true);
    rgb_info_pubs[i] = nh.advertise<sensor_msgs::CameraInfo>("camera" +
        std::to_string(i) + "/rgb/camera_info", 1, true);
    point_pubs[i] = nh.advertise<sensor_msgs::PointCloud>("camera" +
        std::to_string(i) + "/depth/points", 1, true);
  }

  // Configure and start our devices
  int i = 0;
  for(auto dev : devices)
  {
    ROS_INFO("Starting %s...", dev->get_name());
    dev->enable_stream(rs::stream::depth, depth_width, depth_height, rs::format::z16, 30);
    dev->enable_stream(rs::stream::color, rgb_width, rgb_height, rs::format::bgr8, 30);
    rs::intrinsics depth_intrinsics = dev->get_stream_intrinsics(rs::stream::depth);
    rs::intrinsics color_intrinsics = dev->get_stream_intrinsics(rs::stream::color);

    rsIntrinsics2CameraInfo(depth_intrinsics, depth_infos[i]);
    rsIntrinsics2CameraInfo(color_intrinsics, rgb_infos[i]);

    dev->start();
    ROS_INFO("done.");
    ++i;
  }

  // Depth and color
  cv::Mat depth_img(cv::Size(depth_width, depth_height), CV_16UC1);
  cv::Mat rgb_img(cv::Size(rgb_width, rgb_height), CV_8UC3);
  cv_bridge::CvImage cv_img;

  std_msgs::Header header;
  int skip_count = 0;
  while (ros::ok())
  {
    int i = 0;
    if (skip_count == skip_frame_num)
    {
      skip_count = 0;
    }
    else
    {
      ++skip_count;
    }

    for (auto dev : devices)
    {
      header.stamp = ros::Time::now();
      dev->wait_for_frames();

      if (skip_count != skip_frame_num)
      {
        continue;
      }

      const uint16_t* depth_frame = reinterpret_cast<const uint16_t*>(
          dev->get_frame_data(rs::stream::depth));
      memcpy(depth_img.data, depth_frame, depth_img.cols*depth_img.rows*sizeof(uint16_t));
      header.frame_id = "depth_frame" + std::to_string(i);
      cv_img.header = header;
      cv_img.encoding = "mono16";
      cv_img.image = depth_img;
      depth_pubs[i].publish(cv_img.toImageMsg());
      depth_infos[i].header = header;
      depth_info_pubs[i].publish(depth_infos[i]);
      if (pub_points)
      {
        rs::intrinsics depth_intrinsics = dev->get_stream_intrinsics(rs::stream::depth);
        publishPoints(header, depth_img, point_pubs[i], depth_intrinsics);
      }

      const uint8_t* rgb_frame = reinterpret_cast<const uint8_t*>(
          dev->get_frame_data(rs::stream::color));
      memcpy(rgb_img.data, rgb_frame, rgb_img.cols*rgb_img.rows*sizeof(uint8_t)*rgb_img.channels());
      header.frame_id = "rgb_frame" + std::to_string(i);
      cv_img.header = header;
      cv_img.encoding = "bgr8";
      cv_img.image = rgb_img;
      rgb_pubs[i].publish(cv_img);
      rgb_infos[i].header = header;
      rgb_info_pubs[i].publish(rgb_infos[i]);
      ++i;
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

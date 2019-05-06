/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "StereoDepthDnnRosWrapper.hpp"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"


namespace isaac {
namespace rosbridge {

void StereoDepthDnnRosWrapper::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacStereoDnn", ros::init_options::NoSigintHandler);
  }
  
  // Initialize message handlers
  ros::NodeHandle nh("");
  pub_ = nh.advertise<std_msgs::Float32MultiArray>(
      get_publisher_channel_name(), get_publisher_queue_size());
  
  // Listen for Isaac messages
  tickOnMessage(rx_depth());
}

void StereoDepthDnnRosWrapper::tick() {
  if (!ros::ok()) {
    LOG_ERROR("An error has occurred within ROS.");
    return;
  }
  
  // Only publish if someone is listening to the topic
  if (pub_.getNumSubscribers() == 0)
    return;
  
  
  ImageConstView1f image;
  bool ok = FromProto(rx_depth().getProto().getDepthImage(), rx_depth().buffers(), image);
  
  if (!ok) {
    LOG_ERROR("Error while receiving depth image");
    return;
  }
  
  // Create message
  std::vector<float> data(image.num_pixels());
  for (size_t i = 0; i < image.rows(); i++) {
    for (size_t j = 0; j < image.cols(); j++) {
      data[i*image.cols() + j] = image(i, j);
    }
  }
  
  std_msgs::MultiArrayLayout layout;
  std_msgs::MultiArrayDimension dim1;
  std_msgs::MultiArrayDimension dim2;

  dim1.label = "rows";
  dim1.size = image.rows();
  dim1.stride = 1;
  dim2.label = "cols";
  dim2.size = image.cols();
  dim2.stride = 1;
  layout.dim.push_back(dim1);
  layout.dim.push_back(dim2);
  
  std_msgs::Float32MultiArray msg;
  msg.data = data;
  msg.layout = layout;
  pub_.publish(msg);
  
  /*
  auto depth_cam = rx_depth().getProto();
  
  auto depth_image = depth_cam.getDepthImage();
  
  LOG_INFO("Rows: %d, Cols: %d, Channels: %d",
           image.rows(),
           image.cols(),
           depth_image.getChannels() );
  
  LOG_INFO("Depth range: %f to %f meters",
           depth_cam.getMinDepth(),
           depth_cam.getMaxDepth() );
  */
}

void StereoDepthDnnRosWrapper::stop() {
  pub_.shutdown();

}

}  // namespace rosbridge
}  // namespace isaac

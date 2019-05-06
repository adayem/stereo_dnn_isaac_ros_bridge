/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"
#include "messages/camera.hpp"

#include "ros/callback_queue.h"
#include "ros/ros.h"

/*
#include "engine/core/image/image.hpp"
#include "engine/core/tensor/tensor.hpp"
*/

namespace isaac {
namespace rosbridge {

// A simple C++ codelet that prints periodically
class StereoDepthDnnRosWrapper : public alice::Codelet {
  public:
    void start() override;
    void tick() override;
    void stop() override;

    // ROS publisher queue depth
    ISAAC_PARAM(int, publisher_queue_size, 10);
    // ROS publisher channel. Used to broadcast messages to ROS
    ISAAC_PARAM(std::string, publisher_channel_name, "stereoDnn/depth");
    
    ISAAC_PROTO_RX(DepthCameraProto, depth);
    
    
  private:
    // Hide the ROS implementation details
    ros::Publisher pub_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::StereoDepthDnnRosWrapper);

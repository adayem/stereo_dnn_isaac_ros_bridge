"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_binary", "isaac_cc_module")

isaac_app(
    name = "stereo_depth_isaac_tx",
    app_json_file = "stereo_depth_isaac_tx.app.json",
    data = [
        "@redtail//:resnet18_2D_weights_and_plan",
    ],
    modules = [
        "perception",
        "stereo_depth:disparity_dnn",
        "stereo_depth:utils",
        "viewers",
        "zed",
        "//apps/ros_bridge/stereo_depth_isaac_tx:stereo_dnn_ros_wrapper",
    ],
)

isaac_cc_module(
    name = "stereo_dnn_ros_wrapper",
    srcs = ["StereoDepthDnnRosWrapper.cpp"],
    hdrs = ["StereoDepthDnnRosWrapper.hpp"],
    deps = ["//third_party:ros"],
)

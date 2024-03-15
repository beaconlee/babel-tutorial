# load("@rules_cc//cc:defs.bzl", "cc_proto_library")
# load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])
cc_library(
    name = "app",
    srcs = ["main.cc"],
    deps = [
        "//src/common_msgs/user:user_data_cc_proto",
    ],
)

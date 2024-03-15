load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "user_data_cc_proto",
    deps = [":user_data_proto"],
)

proto_library(
    name = "user_data_proto",
    srcs = ["user_data.proto"],
)

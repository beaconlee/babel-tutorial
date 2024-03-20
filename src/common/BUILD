load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
        "frame.h",
        "para.h",
        "status.h",
        "utils.h",
        "vehicle_config.h",
    ],
    deps=[
        "//src/common/datastruct:kd_tree"
    ]
)

cc_library(
    name = "matplotlib",
    hdrs = [
        "matplotlibcpp.h",
    ],
    # copts（编译参数）中添加编译器将要使用到的头文件"-I/usr/include/python3.6/"
    copts = ["-I/usr/include/python3.10/"],
    # deps（依赖文件）中加入“@system_python//:python36-lib"（系统中存在的python文件，并已经在下述的WORKSPACE中引入（即将系统的python放入沙盒中））
    deps = [
        "@system_python//:python310-lib",
    ],
)

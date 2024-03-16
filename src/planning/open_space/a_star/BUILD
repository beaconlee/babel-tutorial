# load("@bazel_tools//tools/build_defs/repo:local_repository.bzl", "local_repository")

package(default_visibility = ["//visibility:public"])

# # 添加本地库路径
# local_repository(
#     name = "fmt",
#     path = "/lib/x86_64-linux-gnu/fmt",
# )

cc_library(
    name = "a_star",
    srcs = [
        "a_star.cc",
        "a_star.h",
    ],
    copts = ["-fPIC"],  # Add -fPIC option here
    deps = [
        "//src/common",
        "//src/common/datastruct:kd_tree",
        "//src/common/datastruct:priority",
    ],
)

cc_binary(
    name = "a_star_test",
    srcs = [
        "a_star_test.cc",
    ],
    deps = [
        ":a_star",
        # ":fmt",
    ],
)

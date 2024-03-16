package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kd_tree",
    srcs = [
        "kd_tree.cc",
        "kd_tree.h",
    ],
)

cc_library(
    name = "priority",
    srcs = [
        "priority_queue.h",
    ],
)

cc_binary(
    name = "kd_tree_test",
    srcs = [
        "kd_tree_test.cc",
    ],
    deps = [":kd_tree"],
)

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "hybrid_a_star",
    srcs = [
        "hybrid_a_star.cc",
        "hybrid_a_star.h",
        "reeds_sheep_path.cc",
        "reeds_sheep_path.h",
        "trajectory_node.cc",
        "trajectory_node.h",
        "//src/common:matplotlibcpp.h",
    ],
    copts = ["-I/usr/include/python3.10/"],
    deps = [
        "//src/planning/open_space/a_star:a_star",
        "//src/common:matplotlib",
    ],
)

cc_binary(
    name = "hybrid_a_star_test",
    srcs = [
        "hybrid_a_star_test.cc",
        "//src/common:matplotlibcpp.h",
    ],
    copts = ["-I/usr/include/python3.10/"],
    deps = [
        ":hybrid_a_star",
        "//src/common:matplotlib",
    ],
)



cc_binary(
    name = "rs_test",
    srcs = [
        "reeds_sheep_test.cc",
        "//src/common:matplotlibcpp.h",
    ],
    copts = ["-I/usr/include/python3.10/"],
    deps = [
        ":hybrid_a_star",
        "//src/common:matplotlib",
    ],
)
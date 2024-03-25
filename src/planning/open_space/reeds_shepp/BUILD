package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reeds_sheep",
    srcs = [
        "reeds_shepp.cc",
        "reeds_shepp.h",
        "//src/common:matplotlibcpp.h",
    ],
    # copts = ["-fPIC"],  # Add -fPIC option here
    copts = ["-I/usr/include/python3.10/"],
    deps = [
        "//src/common",
        "//src/common:matplotlib",
    ],
)

cc_binary(
    name = "rs_test",
    srcs = [
        "reeds_shepp_test.cc",
        "//src/common:matplotlibcpp.h",
        "//src/common:utils.h",
    ],
    copts = ["-I/usr/include/python3.10/"],
    deps = [
        ":reeds_sheep",
        "//src/common:matplotlib",
    ],
)

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cubic",
    srcs = [
        "main.cc",
        "mcubic_spline.cc",
        "mcubic_spline.h",
        "//src/common:matplotlibcpp.h",
    ],
    copts = [
        "-I/usr/include/python3.8/",
        "-O0",
        "-I/usr/local/include/fmt",
    ],
    deps = [
        "//src/common",
        "//src/common:matplotlib",
        # "//src/planning/open_space/a_star",
    ],
)

# cc_binary(
#     name = "hybrid_a_star_test",
#     srcs = [
#         "hybrid_a_star_test.cc",
#         "//src/common:matplotlibcpp.h",
#     ],
#     copts = ["-I/usr/include/python3.10/"],
#     deps = [
#         ":hybrid_a_star",
#         "//src/common:matplotlib",
#     ],
# )

# cc_binary(
#     name = "rs_test",
#     srcs = [
#         "reeds_sheep_test.cc",
#         "//src/common:matplotlibcpp.h",
#     ],
#     copts = ["-I/usr/include/python3.10/"],
#     deps = [
#         ":hybrid_a_star",
#         "//src/common:matplotlib",
#     ],
# )

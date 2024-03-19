load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# rules_cc defines rules for generating C++ code from Protocol Buffers.

# WORKSPACE中可以为bazel引入外部依赖。当WORKSPACE为空时，将不引入外部依赖。由于在编译时，bazel会将编译所需要用到的内容放入一个沙盒（sandbox）中。所以我们需要告知bazel我们也需要系统中的python，方式如下：

new_local_repository(
    name = "system_python",
    # 有个很奇怪的问题，如果下面换行了，将会失败
    build_file_content = """
cc_library(
name = "python310-lib",
srcs = ["libpython3.10.so"],
visibility = ["//visibility:public"],
)
""",
    # Figure out where it is on your system, this is where it is on mine
    path = "/usr/lib/python3.10/config-3.10-x86_64-linux-gnu",
)

new_local_repository(
    name = "fmt",
    # 有个很奇怪的问题，如果下面换行了，将会失败
    build_file_content = """
cc_library(
name = "fmt_lib",
srcs = ["libfmt.so"],
visibility = ["//visibility:public"],
)
""",
    # Figure out where it is on your system, this is where it is on mine
    path = "/usr/lib/x86_64-linux-gnu/",
)

http_archive(
    name = "rules_cc",
    sha256 = "35f2fb4ea0b3e61ad64a369de284e4fbbdcdba71836a5555abb5e194cf119509",
    strip_prefix = "rules_cc-624b5d59dfb45672d4239422fa1e3de1822ee110",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_cc/archive/624b5d59dfb45672d4239422fa1e3de1822ee110.tar.gz",
        "https://github.com/bazelbuild/rules_cc/archive/624b5d59dfb45672d4239422fa1e3de1822ee110.tar.gz",
    ],
)

http_archive(
    name = "com_google_absl",
    strip_prefix = "abseil-cpp-98eb410c93ad059f9bba1bf43f5bb916fc92a5ea",
    urls = ["https://github.com/abseil/abseil-cpp/archive/98eb410c93ad059f9bba1bf43f5bb916fc92a5ea.zip"],
)

# 生成 compile_commands.json
http_archive(
    name = "hedron_compile_commands",
    strip_prefix = "bazel-compile-commands-extractor-ed994039a951b736091776d677f324b3903ef939",

    # 建议把下面两处 commit hash 换成 github 上最新的版本
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/ed994039a951b736091776d677f324b3903ef939.tar.gz",
)

# rules_proto defines abstract rules for building Protocol Buffers.
http_archive(
    name = "rules_proto",
    sha256 = "2490dca4f249b8a9a3ab07bd1ba6eca085aaf8e45a734af92aad0c42d9dc7aaf",
    strip_prefix = "rules_proto-218ffa7dfa5408492dc86c01ee637614f8695c45",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_proto/archive/218ffa7dfa5408492dc86c01ee637614f8695c45.tar.gz",
        "https://github.com/bazelbuild/rules_proto/archive/218ffa7dfa5408492dc86c01ee637614f8695c45.tar.gz",
    ],
)

load("@rules_cc//cc:repositories.bzl", "rules_cc_dependencies")

rules_cc_dependencies()

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

rules_proto_dependencies()

rules_proto_toolchains()

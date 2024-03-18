load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")
# load("@rules_cc//cc:defs.bzl", "cc_proto_library")
# load("@rules_proto//proto:defs.bzl", "proto_library")

cc_binary(
    name = "main",
    deps = [
        "//src/planning/app:app",
    ],
)


refresh_compile_commands(
    name = "refresh_compile_commands",
    # 指定目标 target 及其编译选项/参数（.bazelrc 中已有的参数/选项无需重复添加）
    targets = {
        "//src/planning/open_space/hybrid_a_star:hybrid_a_star_test": "",
    },
)


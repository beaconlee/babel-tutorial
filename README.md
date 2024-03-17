# babel-tutorial

bazel 生成 compile_commands.json 的命令

```
bazel run :refresh_compile_commands
```


vscode 控制是否显示代码缩进线： indent guides
vscode 控制是否渲染出空格： renderWhitespace



记录一个很奇怪的 bug： 

  static constexpr std::array<DIRS, 8> motion_set_={};
  这样进行初始化之后，在进行编译时回报错。


  uan@beacon:~/workspace/bazel_tutorial$ bazel build //src/planning/open_space/a_star:a_star
INFO: Analyzed target //src/planning/open_space/a_star:a_star (0 packages loaded, 0 targets configured).
ERROR: /home/uan/workspace/bazel_tutorial/src/planning/open_space/a_star/BUILD:3:11: Linking src/planning/open_space/a_star/liba_star.so failed: (Exit 1): gcc failed: error executing CppLink command (from target //src/planning/open_space/a_star:a_star) /usr/bin/gcc @bazel-out/k8-fastbuild/bin/src/planning/open_space/a_star/liba_star.so-2.params

Use --sandbox_debug to see verbose messages from the sandbox and retain the sandbox build root for debugging
/usr/bin/ld.gold: error: bazel-out/k8-fastbuild/bin/src/planning/open_space/a_star/_objs/a_star/a_star.pic.o: requires dynamic R_X86_64_PC32 reloc against '_ZN6beacon5AStar11motion_set_E' which may overflow at runtime; recompile with -fPIC
collect2: error: ld returned 1 exit status
Target //src/planning/open_space/a_star:a_star failed to build
Use --verbose_failures to see the command lines of failed build steps.
INFO: Elapsed time: 0.545s, Critical Path: 0.51s
INFO: 4 processes: 2 internal, 2 linux-sandbox.
ERROR: Build did NOT complete successfully
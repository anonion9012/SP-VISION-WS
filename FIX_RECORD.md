# 开发记录
## <div align = "center">M1</div>
## 第一次编译尝试与环境依赖解决
将sp_vision整理成一个包，并尝试构建，解决环境依赖问题。
### 所遇问题与解决方案
#### 问题一：首次构建命令被工具超时中断

执行 `colcon build --executor sequential` 后，命令在 1 秒前台执行限制内被中断，只输出了 `Starting >>> sp_vision`。

解决方案：使用更长的执行超时时间重新运行同一命令，确认实际构建结果，而不是将工具超时误判为编译失败。

验证结果：确认 `sp_vision` 随后确实进入编译阶段，并暴露出真实的依赖版本问题。

#### 问题二：链接阶段出现 fmt v8 未定义引用

构建过程中出现大量 `undefined reference to fmt::v8` 错误。项目源码和系统 `spdlog 1.9.2` 使用 `fmt 8.1.1` ABI，但 CMake 缓存中的 `fmt::fmt` 被解析到了 Anaconda 的 `/media/usr/Data/anaconda3/lib/libfmt.so.11.2.0`。

解决方案：重新配置构建时，将 `fmt_DIR` 和 `spdlog_DIR` 固定为系统 CMake 包目录，避免链接 Anaconda 的 `fmt 11`。

验证结果：链接命令改为使用 `/usr/lib/x86_64-linux-gnu/libfmt.so.8.1.1`，不再出现 `fmt::v8` 未定义引用。

#### 问题三：编译阶段混入 Anaconda 的 fmt v11 头文件

即使链接库已切换到系统版本，编译仍出现 `fmt::basic_runtime` 不存在以及 `spdlog` 格式字符串相关错误。检查生成的 `flags.make` 后发现编译参数包含 `-isystem /media/usr/Data/anaconda3/include`，导致 Anaconda 的 `fmt 11.2.0` 头文件覆盖了系统 `fmt 8.1.1` 头文件。

解决方案：继续检查 CMake 缓存和传递依赖，发现 Ceres 相关依赖仍使用 Anaconda 的 TBB、glog、gflags 配置，并将 Anaconda include 目录传递给目标。将 `TBB_DIR`、`gflags_DIR` 和 `glog_DIR` 也固定到系统 CMake 包目录。

验证结果：编译参数不再包含 Anaconda include；编译器使用系统 `fmt 8.1.1` 头文件，`spdlog` 相关错误消失。

#### 问题四：多个依赖同时被 Anaconda CMake 配置选中

构建缓存还曾使用 Anaconda 的 `yaml-cpp` 和 `nlohmann_json` 配置，造成头文件和库来源混杂，增加 ABI 不一致风险。

解决方案：将 `yaml-cpp_DIR` 固定为 `/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp`，将 `nlohmann_json_DIR` 固定为 `/usr/lib/cmake/nlohmann_json`，并与系统 `fmt`、`spdlog`、TBB、glog、gflags 配置一起重新配置构建。

验证结果：构建链接命令使用系统 `libyaml-cpp.so`、系统 `fmt`、系统 `glog`、系统 `gflags` 和系统 TBB，依赖来源统一。

#### 问题五：最终构建验证

修复后需要确认用户最初要求的原始命令可以直接运行，而不是只验证带额外 CMake 参数的命令。

解决方案：重新执行 `colcon build --executor sequential`。

验证结果：`sp_vision` 构建成功，耗时约 4.89 秒，最终摘要为 `1 package finished`。仅保留 ROS2 未找到和没有 `install` target 的警告，不影响编译结果。

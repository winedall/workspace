// MujocoSimBase.cpp
#include "mj_sim/MujocoSimBase.h"

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

#include "simulate/glfw_adapter.h"

namespace {
const int kErrorLength = 1024;

// UI 字体档位上限：Simulate 的 "Font" 选择项提供 50%~300%，对应档位 0..5。
constexpr int kMaxFontIndex = 5;

// 把目标实时倍率（百分比）吸附到 Simulate 的档位表上。与
// Simulate::LoadOnRenderThread 里"依据 XML vis/global realtime 选档"用同一套
// 对数尺度度量，保证 YAML 给 50 时落到的就是 50% 那一档。
int NearestRealtimeIndex(double percent) {
  constexpr int kNumClicks =
      static_cast<int>(sizeof(mj::Simulate::percentRealTime) / sizeof(mj::Simulate::percentRealTime[0]));
  const double desired = std::log(percent);
  int best = 0;
  double min_error = std::numeric_limits<double>::max();
  for (int i = 0; i < kNumClicks; ++i) {
    const double error = std::abs(std::log(static_cast<double>(mj::Simulate::percentRealTime[i])) - desired);
    if (error < min_error) {
      min_error = error;
      best = i;
    }
  }
  return best;
}
}  // namespace

MujocoSimBase* MujocoSimBase::controller_instance_ = nullptr;

MujocoSimBase::MujocoSimBase() : m_(nullptr), d_(nullptr), is_record_(false), running_(false) {
  // MuJoCo 控制回调无法携带用户数据，这里保存当前实例，供 StaticComputeControl 转发。
  controller_instance_ = this;
}

MujocoSimBase::~MujocoSimBase() {
  std::cout << "sim t4 end." << std::endl;
  running_ = false;
  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  if (controller_instance_ == this) {
    controller_instance_ = nullptr;
  }

  if (d_)
    mj_deleteData(d_);
  if (m_)
    mj_deleteModel(m_);
}

void MujocoSimBase::Run() {
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    // 头文件和动态库版本不一致会导致结构体布局不匹配，必须直接终止。
    mju_error("Headers and library have different versions");
  }

  // 扫描与可执行文件同目录下的 MuJoCo 插件，支持后续扩展自定义 actuator/sensor。
  ScanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  sim_ = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam, &opt, &pert, false);
  // 必须在 RenderLoop() 之前应用：UI 主题/字体/垂直同步由 RenderLoop 启动时读入。
  ApplyStartupOptions();

  const char* filename = xml_path.c_str();
  std::cout << "It will load xml file at path: " << filename << std::endl;
  running_ = true;
  // 物理线程负责加载模型和推进仿真；当前线程进入 GUI 渲染循环。
  physics_thread_ = std::thread(&MujocoSimBase::RunPhysicsThread, this, filename);
  // 线程名便于用 /proc 或调试器分辨物理/渲染/ROS 线程（同名进程下只有一个主线程名）。
  pthread_setname_np(physics_thread_.native_handle(), "mj_physics");

  sim_->RenderLoop();
  running_ = false;

  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  Cleanup();
}

void MujocoSimBase::ApplyStartupOptions() {
  if (sim_ == nullptr) {
    return;
  }

  // ---- viewer：只影响窗口与界面显示 ----
  const SimViewerOptions& v = viewer_options_;
  sim_->ui0_enable = v.ui0_enable ? 1 : 0;
  sim_->ui1_enable = v.ui1_enable ? 1 : 0;
  sim_->spacing = v.spacing;
  sim_->color = v.color;
  sim_->help = v.help ? 1 : 0;
  sim_->info = v.info ? 1 : 0;
  sim_->profiler = v.profiler ? 1 : 0;
  sim_->sensor = v.sensor ? 1 : 0;
  sim_->vsync = v.vsync ? 1 : 0;
  sim_->pause_update = v.pause_update ? 1 : 0;
  if (v.font >= 0) {
    // 显式配置字体档位；-1（默认）时保持 RenderLoop 的 DPI 自动推断。
    if (v.font > kMaxFontIndex) {
      std::printf("viewer.font=%d 超出 0..%d，按 %d（300%%）处理\n", v.font, kMaxFontIndex, kMaxFontIndex);
    }
    sim_->font = std::min(v.font, kMaxFontIndex);
  }

  // ---- dynamics：只影响物理线程的推进节奏与运行状态 ----
  const SimDynamicsOptions& d = dynamics_options_;
  sim_->run = d.start_paused ? 0 : 1;
  sim_->busywait = d.busywait ? 1 : 0;
  sim_->refresh_rate = d.refresh_rate;
}

void MujocoSimBase::ApplyPostLoadOptions() {
  if (sim_ == nullptr) {
    return;
  }
  const double percent = dynamics_options_.realtime_percent;
  if (percent <= 0.0) {
    return;  // 未配置：沿用 XML 里 vis/global realtime 决定的档位。
  }

  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  sim_->real_time_index = NearestRealtimeIndex(percent);
  // 通知物理线程重新对齐同步基准，避免换档瞬间按新的目标倍率追赶历史步数。
  sim_->speed_changed = true;
  std::printf("dynamics.realtime_percent=%.1f%% -> Simulate 档位 %g%%\n", percent,
              static_cast<double>(mj::Simulate::percentRealTime[sim_->real_time_index]));
}

void MujocoSimBase::RunPhysicsThread(const char* filename) {
  if (filename != nullptr) {
    sim_->LoadMessage(filename);
    m_ = LoadModel(filename);
    if (m_) {
      // mjData 保存当前仿真状态，包括 qpos/qvel/ctrl/sensordata。
      d_ = mj_makeData(m_);
    }
    if (d_) {
      InitRobotState(d_);
      sim_->Load(m_, d_, filename);
      // 加载完成后才能覆盖 LoadOnRenderThread 依据 XML 选出的实时倍率档位。
      ApplyPostLoadOptions();
      // 必须等模型加载成功后才能解析关节、执行器和传感器索引。
      InitInterface();
      InitCamera();
      // 根据初始状态做一次正向动力学，确保传感器和派生量有效。
      mj_forward(m_, d_);
    } else {
      sim_->LoadMessageClear();
    }
  }

  RunPhysicsLoop();
}

void MujocoSimBase::StaticComputeControl(const mjModel* m, mjData* d) {
  // MuJoCo 调用的是全局函数指针，这里再转发到当前实例的虚函数。
  if (controller_instance_) {
    controller_instance_->ComputeControl(m, d);
    // 紧跟在写 ctrl 之后，赶在 MuJoCo 读取 d->ctrl 之前做后处理（执行延迟等）。
    controller_instance_->PostComputeControl(m, d);
  }
}

void MujocoSimBase::RunPhysicsLoop() {
  using Seconds = std::chrono::duration<double>;
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  double sync_sim = 0;

  // 注册 MuJoCo 控制回调。每次 mj_step 内部都会调用 StaticComputeControl。
  mjcb_control = StaticComputeControl;

  while (!sim_->exitrequest.load()) {
    if (sim_->uiloadrequest.load()) {
      // GUI 中重新加载模型时，重新创建 mjModel/mjData，并重建派生类接口索引。
      sim_->uiloadrequest.fetch_sub(1);
      sim_->LoadMessage(sim_->filename);
      mjModel* mnew = LoadModel(sim_->filename);
      mjData* dnew = nullptr;
      if (mnew)
        dnew = mj_makeData(mnew);
      if (dnew) {
        sim_->Load(mnew, dnew, sim_->filename);
        // GUI 重载同样会重置实时倍率档位，这里再应用一次。
        ApplyPostLoadOptions();
        mj_deleteData(d_);
        mj_deleteModel(m_);
        m_ = mnew;
        d_ = dnew;
        InitRobotState(d_);
        // 新模型的关节/执行器地址可能变化，必须重新初始化接口映射。
        InitInterface();
        InitCamera();
        mj_forward(m_, d_);
      } else {
        sim_->LoadMessageClear();
      }
    }

    // 根据 GUI busywait 设置选择让出 CPU 或短暂睡眠。
    if (sim_->run && sim_->busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      if (m_) {
        if (sim_->run) {
          // 在步进前给派生类机会发布状态、读取 ROS 命令或同步外部数据。
          ExchangeMessages(sync_sim);

          bool stepped = false;
          const auto startCPU = mj::Simulate::Clock::now();
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d_->time - sync_sim;
          double slowdown = 100 / sim_->percentRealTime[sim_->real_time_index];

          // 当仿真时间和 CPU 时间明显错位时，重置同步基准，避免追赶过多历史步。
          bool misaligned = std::abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim_->speed_changed) {
            syncCPU = startCPU;
            sync_sim = d_->time;
            sim_->speed_changed = false;
            stepped = StepAndRecord();
          } else {
            bool measured = false;
            double prevSim = d_->time;
            double refreshTime = simRefreshFraction / sim_->refresh_rate;

            // 在单个显示刷新周期内尽量推进足够多的物理步，使仿真接近目标实时倍率。
            while (Seconds((d_->time - sync_sim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              if (!measured && elapsedSim) {
                sim_->measured_slowdown = std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }
              stepped = StepAndRecord();
              if (d_->time < prevSim)
                break;
            }
          }

          if (stepped) {
            // 保存历史轨迹，供 MuJoCo GUI 中的可视化/曲线工具使用。
            sim_->AddToHistory();
          }
        } else {
          // 暂停时仍做 forward，保证拖拽、扰动或 UI 修改后传感器显示是最新的。
          mj_forward(m_, d_);
          sim_->speed_changed = true;
        }
      }
    }
  }
}

mjModel* MujocoSimBase::LoadModel(const char* file) {
  char load_err[kErrorLength] = "";
  mjModel* mnew = nullptr;

  // 支持加载二进制 MJB 或 XML。当前 robot_sim 默认使用 XML，便于调试模型配置。
  if (std::strlen(file) > 4 && !std::strncmp(file + std::strlen(file) - 4, ".mjb", 4)) {
    mnew = mj_loadModel(file, nullptr);
    if (!mnew) {
      std::strcpy(load_err, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(file, nullptr, load_err, kErrorLength);
  }

  if (load_err[0]) {
    std::printf("Model loading warning: %s\n", load_err);
  }

  return mnew;
}

void MujocoSimBase::ScanPluginLibraries() {
  // 先打印已经静态链接到可执行文件中的插件，便于确认插件注册状态。
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }
  const std::string sep = "/";

  // 再尝试扫描 ${EXECDIR}/mujoco_plugin。${EXECDIR} 是当前可执行文件所在目录。
  const std::string executable_dir = GetExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = GetExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

std::string MujocoSimBase::GetExecutableDir() {
  constexpr char kPathSep = '/';
  const char* path = "/proc/self/exe";

  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      ssize_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path 已经不是符号链接，直接使用。
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // 缓冲区不够，扩容后重试。
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }
  // 找不到可执行文件目录时返回空，避免从文件系统根目录开始扫描插件。
  return "";
}

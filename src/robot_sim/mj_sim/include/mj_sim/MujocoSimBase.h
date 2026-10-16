// MujocoSimBase.h
#ifndef MUJOCOSIMBASE_H
#define MUJOCOSIMBASE_H

#include <mujoco/mujoco.h>

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "simulate/simulate.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

namespace mj = ::mujoco;

// 仿真运行期选项：与机器人型号无关、但需要按仿真包配置的量。
// 由派生类在构造阶段从 ROS 参数填入（见 sim_common/SimRuntimeOptions.hpp），
// 由 MujocoSimBase 在 Run() 创建 Simulate 之后统一应用。
//
// 分组按"影响面"划分，避免所有参数堆在 YAML 的同一层：
//   viewer   —— 只影响窗口与界面显示；不影响物理推进，也不影响任何话题；
//   dynamics —— 只影响物理线程的推进节奏与运行状态；不改变模型本身。
// 字段默认值即"不配置时的行为"：除 ui0_enable（默认隐藏左侧面板）与 font
// （-1 = 按屏幕自动推断）外，其余与 Simulate 的原默认值一致。
struct SimViewerOptions {
  // 左侧面板（File/Option/Simulation/Watch/Physics）。
  // 默认关闭：多机场景用不到；单机本地调试想看面板时在 YAML 里置 true。
  bool ui0_enable{false};
  // 右侧面板（关节/执行器滑块）。关节数或执行器数 > 100 的模型本来也不会构建
  // 该面板（mjui 的 offscreen 缓冲高度受 OpenGL 上限约束）。
  bool ui1_enable{false};
  // UI 密度：0 = Tight，1 = Wide。
  int spacing{0};
  // 配色：0 = Default，1 = Orange，2 = White，3 = Black。
  int color{0};
  // 字体档位 0..5（对应 50%..300%）；-1 = 按屏幕 DPI 自动推断（ComputeFontScale）。
  int font{-1};
  // 四个叠加面板：帮助(F1)、左下信息、CPU 曲线、传感器曲线。
  bool help{false};
  bool info{false};
  bool profiler{false};
  bool sensor{false};
  // 垂直同步；关掉后渲染线程不再被显示器刷新率节流。
  bool vsync{true};
  // 仿真暂停时是否继续刷新 profiler/sensor 曲线。
  bool pause_update{true};
};

struct SimDynamicsOptions {
  // 启动后立即暂停（先看初始姿态、等控制侧就绪）。
  bool start_paused{false};
  // 物理线程的等待方式：true = yield 忙等（步进更密、更贴近实时）；
  // false = 睡 1 ms（省 CPU，多机时不与渲染线程抢核）。
  bool busywait{false};
  // 物理线程每轮步进的时间预算 = simRefreshFraction / refresh_rate（见 RunPhysicsLoop）。
  int refresh_rate{60};
  // 目标实时倍率（百分比，如 50 = 0.5 倍速）。<= 0 = 用 XML 里 vis/global realtime 的值。
  // 多机 + 多路深度相机跑不动 100% 时用它降速，不必在窗口里手按 - 键。
  double realtime_percent{0.0};
};

// MuJoCo 仿真的公共基类。
//
// 该类只封装与具体机器人型号无关的仿真生命周期：
// - 加载 XML/MJB 模型；
// - 创建 MuJoCo mjModel/mjData；
// - 启动渲染循环和物理线程；
// - 注册 MuJoCo 控制回调；
// - 在合适的仿真节拍调用派生类发布/接收消息。
//
// 具体机器人如何初始化姿态、如何把 ROS 命令转成 d->ctrl、如何发布状态，
// 由 SimZ01Edm 等派生类实现。
class MujocoSimBase {
  private:
    // MuJoCo C API 的控制回调是静态函数指针，因此需要保存当前实例指针，
    // 再通过 StaticComputeControl 转发到派生类的 ComputeControl()。
    static MujocoSimBase* controller_instance_;
    static void StaticComputeControl(const mjModel* m, mjData* d);

  public:
    MujocoSimBase();
    virtual ~MujocoSimBase();

    // 主运行函数：启动物理线程，并在当前线程进入 MuJoCo GUI 渲染循环。
    void Run();

    // 获取仿真状态，供外部或派生类诊断使用。
    bool IsRunning() const { return running_; }
    double GetSimTime() const { return d_ ? d_->time : 0.0; }

  protected:
    // 派生类必须实现的机器人相关钩子。
    // InitRobotState：设置初始 qpos/qvel；
    // ComputeControl：每个 MuJoCo 控制步写入执行器控制量；
    // ExchangeMessages：按照仿真时间发布 ROS 状态或读取外部消息。
    virtual void InitRobotState(mjData* d) = 0;
    virtual void ComputeControl(const mjModel* m, mjData* d) = 0;
    // 每次 ComputeControl 之后立刻调用，此时 d->ctrl 刚写好、MuJoCo 还没读它算
    // actuator_force（mjcb_control 在 mj_fwdActuation 之前触发）。执行延迟一类的
    // "每步一次"后处理放在这里：mjcb_control 是全局函数指针，这是唯一的单点位置。
    // 注意 Euler 积分器下每步调用一次（RK4 会调用 4 次）。
    virtual void PostComputeControl(const mjModel* /*m*/, mjData* /*d*/) {}
    virtual void ExchangeMessages(double sync_sim) = 0;

    // 可选钩子：模型加载完成后解析索引、设置相机、退出清理、离线记录。
    virtual void InitInterface() {}
    virtual void InitCamera() {}
    virtual void Cleanup() {}
    virtual void RecordData() {}

    std::string xml_path;
    // MuJoCo 原始模型和运行时数据指针。由基类创建和释放，派生类只使用不拥有。
    mjModel* m_;
    mjData* d_;
    // MuJoCo 官方 simulate UI 封装，用于窗口渲染、暂停/运行和模型重载。
    std::unique_ptr<mj::Simulate> sim_;

    // 运行期选项：派生类在构造阶段从 ROS 参数填入，Run() 里统一应用。
    SimViewerOptions viewer_options_;
    SimDynamicsOptions dynamics_options_;

    bool is_record_;

    // 仿真时间和 CPU 时间允许的最大偏差，超过后重新对齐同步基准。
    const double syncMisalign = 0.1;
    // 每个刷新周期中用于物理步进的时间比例，避免渲染线程被物理循环长期占用。
    const double simRefreshFraction = 0.7;

  private:
    void RunPhysicsThread(const char* filename);
    void RunPhysicsLoop();
    mjModel* LoadModel(const char* file);
    void ScanPluginLibraries();
    std::string GetExecutableDir();

    // 应用 viewer/dynamics 选项。必须在 RenderLoop() 之前调用：spacing/color/font/vsync
    // 由 RenderLoop 启动时一次性读入 UI 主题与渲染上下文，之后再改不生效。
    void ApplyStartupOptions();
    // 模型加载完成后调用。只有与模型相关的项需要等加载完成：realtime_percent 要覆盖
    // LoadOnRenderThread 依据 XML vis/global realtime 选出的档位。
    void ApplyPostLoadOptions();

    bool running_;
    std::thread physics_thread_;

    bool StepAndRecord() {
        // 单步推进 MuJoCo 物理。RecordData 默认关闭，由派生类按需要启用。
        mj_step(m_, d_);
        // if (is_record_) {
        //   RecordData();
        // }
        return true;
    }
};

#endif

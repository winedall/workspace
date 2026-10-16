# AI Development Rules

本文件是 AI 编程代理进入本仓库后的最高优先级项目规则。修改代码前必须阅读并遵守；更详细的架构、接口和发布流程见文末参考文档。

## Project Context

- 本仓库是 ROS 2 Humble / Ubuntu 22.04 的机器人控制系统，根目录是 colcon 工作区。
- 主要代码位于 `src/`，配置位于 `config/`，脚本位于 `scripts/`，设计和操作文档位于 `docs/`。
- 系统包含真实硬件控制、MuJoCo 仿真、RL 推理、状态机、命令路由、关节控制和硬件驱动。涉及运动输出、控制权、安全保护、状态切换、硬件驱动或 RL 推理的改动默认按高风险处理。
- 优先保持现有 ROS 2 包边界、节点职责、launch/YAML 配置方式和可测试的核心逻辑结构。

## Hard Invariants

未经用户明确授权，不得改变以下内容的对外语义：

- ROS 2 Topic、Service、Action 名称与类型，消息字段语义，QoS，参数名称与默认值。
- 状态机、命令路由、控制权边界、故障检测、故障复位、安全降级和系统状态权威职责。
- 急停、阻尼、限幅、超时、失联保护、关节变化率判断、准备插补、NaN/Inf 拒绝等控制安全机制。
- 关节名称、顺序、单位、索引、硬件映射、限位和坐标空间转换。
- RL 模型输入输出语义、观测历史、动作缩放、模型加载格式、模型切换规则和推理频率。
- 控制频率、线程模型、锁、callback group、Executor、队列深度、超时机制和真实硬件启停策略。

## Required Workflow

- 先阅读目标包的 `README.md`、`package.xml`、`CMakeLists.txt` 或 `setup.py`，以及相关 launch/YAML 配置。
- 修改前确认影响范围，特别是接口、状态机、控制链路、关节映射、RL 推理、并发和硬件安全。
- 行为保持型重构、性能优化、接口升级和功能变更必须分开处理，不要混在一次改动里。
- 优先小步修改，保持现有架构和代码风格，避免无关格式化和大规模重写。
- 业务逻辑应尽量与 ROS 节点封装分离，便于单元测试；运行参数应通过 parameter、YAML 或 launch 管理。
- 对缺少覆盖的既有行为，优先补 characterization tests，再改实现。
- 不覆盖、回退或删除用户已有的无关改动。
- 不能把未执行的检查描述为已通过；硬件不可用时必须明确标记未验证范围。

## Build And Test

常用环境与验证命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select <package_name>
colcon test --packages-select <package_name>
colcon test-result --verbose --test-result-base build/<package_name>
```

选择验证范围时遵循：

- 修改单个 ROS 2 包：至少构建并测试该包。
- 修改 `robot_interfaces`：重建依赖该接口的相关包。
- 修改 launch、bringup、配置、控制链路或安全路径：至少做启动级或仿真/dry-run 验证；无法执行时说明原因。
- 修改真实硬件相关逻辑：默认只做编译、单元测试、仿真或 dry-run；不要在未经用户确认时运行会驱动真实硬件的命令。

## Forbidden

- 不得删除或弱化急停、阻尼、限幅、超时、失联保护、故障降级、关节变化率判断等安全机制，除非用户明确要求并提供替代方案。
- 不得让真实硬件控制路径默认启用，或把调试 topic、硬编码路径、临时参数留在主路径。
- 不得随意引入大型第三方依赖；新增依赖必须同步更新 `package.xml`、`CMakeLists.txt`、`setup.py` 或等效配置。
- 不得提交或依赖 `build/`、`install/`、`log/`、bag 文件、大体积临时数据等生成产物。
- 不得为了局部问题引入全局架构重写。

## Delivery Requirements

最终回复应简要说明：

- 修改目标和实际改动文件。
- 与本次改动相关的接口、安全、状态或时序兼容性判断。
- 实际执行的构建/测试命令和结果。
- 未验证事项、硬件相关风险和建议回滚方式。

## References

- `docs/zl_robot控制系统设计说明.md`：当前系统架构、运行入口、数据流和状态机。
- `docs/zl_robot仓库开发测试发布操作指导.md`：分支、测试、RC/RELEASE/HOTFIX 和发布流程。
- `src/robot_interfaces/README.md`：全局消息/服务契约。
- `src/robot_router/README.md`：命令仲裁和准入规则。
- `src/robot_state/README.md`：系统状态机和故障管理。
- `src/robot_control/README.md`：控制过滤、准备插补和踝关节转换。
- `src/robot_driver/README.md`：硬件驱动、实时线程和故障处理。
- `src/robot_rl_manager/README.md`：RL 模型管理、推理和模型切换。

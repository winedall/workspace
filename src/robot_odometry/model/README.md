# 里程计模型

本目录平铺放置 ONNX 图，文件名与注册表里的 `model_path` 一致：

| 文件 | 观测维度 | 说明 |
|---|---|---|
| `z01_std_odom_no_acc.onnx` | 88 | 不含基座加速度 |
| `z01_std_odom_with_acc.onnx` | 91 | 含基座加速度（默认） |

来源：`zl_robot_odometry` 仓库 `proprio_odometry/checkpoints/`
（Zoomlion T4-27DOF 本体感觉里程计 MLP）。两个图都是**单文件**，
不依赖 ONNX external data 分片。

## 图接口

| 方向 | 名称 | 类型 | 形状 | 含义 |
|---|---|---|---|---|
| 输入 | `obs_history` | float32 | `[batch, 50, ODIM]` | 未归一化的本体感觉历史窗口，按时间**从旧到新** |
| 输入 | `previous_odom_norm` | float32 | `[batch, 2]` | **归一化空间**的上一步预测；首帧为全 0 |
| 输出 | `pred_delta` | float32 | `[batch, 2]` | 当前步的世界系位移增量，单位**米** |
| 输出 | `next_previous_odom_norm` | float32 | `[batch, 2]` | 归一化空间的当前步预测，下一步回填给 `previous_odom_norm` |

`ODIM` 为 88 或 91，由模型的 `use_base_acceleration` 决定。

观测归一化（`obs_mean`/`obs_std`）与目标反归一化（`target_mean`/`target_std`）
都已折进图的常量里，运行时不需要 `normalization.npz`。

自回归反馈用**归一化空间**的值，与训练仓库 `odometry/inference.py` 中
`self.previous_odom = pred_norm` 的语义完全一致。

两个输出之间存在固定的仿射关系
`pred_delta = target_std * next_previous_odom_norm + target_mean`
（当前这批模型约 `target_std ≈ 0.009 m`、`target_mean ≈ 0`）。
`test/test_model_integration.py::test_feedback_is_affine_in_delta` 逐个模型校验——
若导出时漏掉反归一化，或两个输出对不上，该测试立刻失败。

## 观测布局（schema v3）

```
88 维（use_base_acceleration=false）
    joint_pos_current(27)    [0:27]
    joint_vel(27)            [27:54]
    base_quat(4)             [54:58]    世界系 (w, x, y, z)
    base_ang_vel(3)          [58:61]    body 系角速度，rad/s
    joint_pos_previous(27)   [61:88]    上一帧的 joint_pos

91 维（use_base_acceleration=true）
    joint_pos_current(27)    [0:27]
    joint_vel(27)            [27:54]
    base_quat(4)             [54:58]
    base_ang_vel(3)          [58:61]
    base_acc(3)              [61:64]    body 系线加速度，m/s^2
    joint_pos_previous(27)   [64:91]
```

两个必须注意的点：

1. **`base_acc` 插在 `base_ang_vel` 与 `joint_pos_previous` 之间**，88 维版本
   并不是简单截取 91 维的前 88 列，而是跳过 `[61:64]`。列索引由
   `joint_order.py` 统一给出，两个版本共用同一份定义。
2. **action 不在观测里**。更早的一代 88 维 `[61:88]` 是 action，现在同样的
   区间是 `joint_pos_previous` —— 维度相同、含义完全不同，喂错会静默出错。
   因此每份模型配置都必须逐字写出 `obs_layout`（**必填**），注册表加载时与
   代码里的布局字符串逐字比对；复制配置加新模型时，这里是唯一会暴露语义
   错配的关卡。

   顺带一提，训练仓库历史上还有过 schema 2（`train_prev_joint_pos`，88 维）
   与 schema 3（`train_prev_joint_pos_acc`）两代。schema 3 的 88 维视图跳过
   `base_acc` 三列后与 schema 2 **逐列相同**，所以同一个 adapter 基类服务
   两代，代码里不需要区分它们。

`joint_pos_previous` 在首帧（或刚 reset）时用当前帧自身填充，与训练仓库
`Trajectory.build_full_observation` 的约定一致。

## 自检

```bash
colcon test --packages-select robot_odometry
```

覆盖：注册表与配置一致性、配置与 ONNX 图维度一致、契约校验、逐模型前向、
反归一化仿射关系、adapter 的帧间语义。

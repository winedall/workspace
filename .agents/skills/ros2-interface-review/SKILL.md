---
name: ros2-interface-review
description: Use when 用户说“帮我审查接口”、“接口审查”、“ROS接口检查”、“接口兼容”、“topic检查”、“service检查”、“action检查”、“参数检查”、“QoS检查”、“launch检查”、“完整重构检查”，或 zl_robot 改动可能影响 ROS 2 话题、服务、消息、参数、QoS、namespace、remap、依赖。
---

# ROS 2 接口审查

## 目的

用于在实现前、代码审查中或交付前，检查 zl_robot 是否存在未经授权的 ROS 2 接口变化。

## 审查范围

检查改动是否影响：

- Topic、Service 或 Action 名称。
- 消息、服务或 action 类型。
- `.msg`、`.srv`、`.action` 字段名、类型、常量、单位或语义。
- QoS、队列深度、durability、reliability 或高频控制 QoS 辅助函数。
- ROS 参数名称、默认值、取值范围或语义。
- Launch 参数、namespace、remap、启动节点或配置文件选择。
- `package.xml`、`CMakeLists.txt`、`setup.py` 或等效文件中的依赖。
- 包 README 中描述的外部接口约定。

## 必查步骤

1. 涉及接口时，先阅读相关包 README 和 `src/robot_interfaces/README.md`。
2. 检查改动过的源码、launch、YAML 和接口文件。
3. 通过 diff 或逻辑对比判断接口前后差异。
4. 对每个接口差异分类：
   - 无变化。
   - 兼容的内部变化。
   - 需要用户明确授权的公共接口变化。
   - 破坏性变化。
5. 如发现未经授权的公共接口变化，应暂停并先报告。

## 高风险接口

重点关注：

- 控制链路 topic：desired、accepted、hardware commands、hardware states、driver states。
- 状态和意图 topic。
- driver 维护服务。
- RL 模型管理服务和状态消息。
- `robot_bringup` 下的 namespace 行为。
- 高频控制 topic 使用的 QoS。

## 输出格式

报告：

- 已检查文件。
- 发现的接口变化。
- 兼容性判断。
- 是否需要用户授权或迁移工作。
- 建议验证命令。

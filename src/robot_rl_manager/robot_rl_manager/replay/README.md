# Replay 工具

这个包提供的是 RL 模型离线 replay 的入口，用来验证模型在不同后端上的输出是否一致。
当前 replay 只接受序列样本 `inputs_seq.npz`，并且要求同目录下必须存在配套的 `manifest.yaml`。
replay 不再猜测模型输入契约，也不再兼容旧的单帧 `inputs.npz` 作为正式输入格式。

## 当前职责

- 把 sample 输入归一化成 `.npz`
- 从文件加载并运行基线 ONNX 模型或 RKNN 模型
- 对比保存好的输出，并生成 JSON 报告
- 保持编排层足够薄，尽量只做文件流转

## 默认目录约定

为了让 x86 和 RK3588 两边使用同一份样本数据，`replay` 采用统一的仓库内路径：

- 输入样本：`config/data/samples/<model>/inputs_seq.npz`
- 样本清单：`config/data/samples/<model>/manifest.yaml`
- 基线输出：`config/data/samples/<model>/baseline_outputs.npz`
- RKNN 输出：`config/data/samples/<model>/rknn_outputs.npz`
- 对比报告：`config/data/samples/<model>/report.json`

也就是说，`compare` 不会重新跑模型，它只会读取前面已经落盘的
`baseline_outputs.npz` 和 `rknn_outputs.npz`。

## 命令行入口

入口命令是 `robot_rl_manager_rknn_replay`。

示例：

```bash
python3 -m robot_rl_manager.replay --model t4_heart --compare
```

## 手动工作流

`t4_heart` 的模型输入是 `obs`，所以 replay 的 sample 文件也必须包含 `obs` 键。
默认样本位置是 `config/data/samples/<model>/inputs_seq.npz`，配套清单是
`config/data/samples/<model>/manifest.yaml`。日常使用时通常只需要传 `--model`，
`--sample` 只在你要临时测试别的样本时才需要显式指定。
如果你已经在仿真运行时开启了样本采集，那么 `rl_manager` 会把模型输入序列和
manifest 一起写到这个目录。

如果你已经有了带 `obs` 的输入样本和对应 manifest，可以直接跑：

```bash
python3 -m robot_rl_manager.replay --model t4_heart --run-baseline
python3 -m robot_rl_manager.replay --model t4_heart --run-rknn
python3 -m robot_rl_manager.replay --model t4_heart --compare
```

上面三步执行后，默认会生成：

- `config/data/samples/t4_heart/baseline_outputs.npz`
- `config/data/samples/t4_heart/rknn_outputs.npz`
- `config/data/samples/t4_heart/report.json`

其中 `compare` 直接从这两个输出文件读数据，然后写报告。

`t4_walk` 一般需要先抓一个 sample，再把同一个文件拿去跑两个后端：

```bash
python3 -m robot_rl_manager.replay --model t4_walk --capture
python3 -m robot_rl_manager.replay --model t4_walk --run-baseline
python3 -m robot_rl_manager.replay --model t4_walk --run-rknn
python3 -m robot_rl_manager.replay --model t4_walk --compare
```

`--capture` 只会归一化并重写 sample 输入，不会生成新的机器人轨迹，也不会补写 manifest。
它适合把已经存在的序列样本整理成标准 `.npz`，然后给后续 replay 使用。
如果你显式传了 `--sample`，就会使用你指定的样本路径覆盖默认路径。
`--run-baseline` 使用 ONNX 后端，`--run-rknn` 使用配置好的 RKNN 后端。
`--compare` 会加载已经保存的输出，并在未显式覆盖路径时，把 `report.json` 写到 sample 同级目录。

对 `t4_heart` 这类标准参考样本，默认会直接使用 `config/data/samples/t4_heart/inputs_seq.npz`。

## 仿真样本采集

如果你在 `robot_rl_manager` 里打开了 `enable_model_input_dump`：

- 仿真节点在调用 `build_model_input()` 后，会把当前模型输入序列写入 `config/data/samples/<model>/inputs_seq.npz`
- 同时会写出 `config/data/samples/<model>/manifest.yaml`
- 默认每个模型只保留最新一段序列，切换模型、达到时间/帧数限制或节点退出时会收口覆盖
- 这个路径由 `launch/zl_robot.launch.py` 传入，和 `replay` 读取的是同一套目录

也就是说，仿真采样和离线 replay 共享同一个仓库内样本目录，不需要额外维护两份数据。

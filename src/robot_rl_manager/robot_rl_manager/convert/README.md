# RKNN 转换工具

这个目录保存的是离线转换工具，用来把 `t4_` 系列 ONNX 策略模型转换成 RK3588 上可用的 RKNN 模型。

## 功能说明

- 读取 `config/humanoid/rl_models.yaml` 中已有的运行时模型注册表
- 按前缀规则筛选允许转换的模型
- 按 `manifest.yaml` 中的配置对特定模型应用 ONNX 图补丁
- 使用 `rknn-toolkit2` 将补丁后的 ONNX 模型转换成 `.rknn`
- 可选地做一轮 ONNX Runtime 数值验证

## 文件说明

- `cli.py`：命令行入口
- `converter.py`：转换主流程
- `manifest.yaml`：按模型划分的转换清单
- `patches.py`：ONNX 图补丁注册表
- `verify.py`：输出对比辅助工具
- `utils.py`：通用路径和 YAML 辅助函数

## 当前工作流

这套统一流程目前是手动执行、配置驱动的：

1. 在 `manifest.yaml` 中新增或更新模型条目
2. 在 x86 开发机上执行转换
3. 把生成的 `.rknn` 文件拷贝到 RK3588 目标板
4. 在运行时通过 `rknnlite.api.RKNNLite` 加载并推理

转换脚本默认会把 `.rknn` 导出到源 ONNX 同级目录，例如：

- `model/t4_heart.onnx` -> `model/t4_heart.rknn`

如果手动传了 `--output`，则会改写成你指定的路径。

## 当前清单策略

- 允许前缀：`t4_`
- 排除前缀：`t4_kd_`
- 默认目标平台：`rk3588`
- 默认开启验证

## 现有模型补丁

目前共用的图补丁只有一个：

- `random_normal_like_to_zero`

这个补丁会把 `RandomNormalLike` 节点替换成确定性的零常量。

## 转换示例

列出所有允许转换的 ONNX 模型：

```bash
PYTHONPATH=/home/zoomlion/work/99_robot/zl_robot/src/robot_rl_manager \
python3 -m robot_rl_manager.convert --list
```

转换单个模型：

```bash
PYTHONPATH=/home/zoomlion/work/99_robot/zl_robot/src/robot_rl_manager \
python3 -m robot_rl_manager.convert --model t4_heart
```

即使清单里关闭了验证，也强制执行验证：

```bash
PYTHONPATH=/home/zoomlion/work/99_robot/zl_robot/src/robot_rl_manager \
python3 -m robot_rl_manager.convert --model t4_heart --verify
```

如果 RKNN 已经可以正常构建和导出，但数值比较不稳定，可以跳过验证：

```bash
PYTHONPATH=/home/zoomlion/work/99_robot/zl_robot/src/robot_rl_manager \
python3 -m robot_rl_manager.convert --model t4_heart --no-verify
```

手动指定 RKNN 输出位置：

```bash
PYTHONPATH=/home/zoomlion/work/99_robot/zl_robot/src/robot_rl_manager \
python3 -m robot_rl_manager.convert --model t4_heart --output /tmp/t4_heart.rknn
```

## 终端手动操作步骤

如果你想在终端里手动完成一次转换，建议按下面的顺序执行。

### 1. 进入仓库根目录

```bash
cd /home/zoomlion/work/99_robot/zl_robot
```

### 2. 加载 RKNN 虚拟环境

如果你本机已经准备好了 `rknn-toolkit2` 的虚拟环境，可以先激活它。常见路径是：

```bash
source /home/zoomlion/work/99_robot/zl_robot/.venv-rknn/bin/activate
```

如果你实际使用的是其他虚拟环境路径，就把上面的路径替换成你自己的环境位置。

### 3. 设置 Python 模块搜索路径

`robot_rl_manager` 当前是源码目录直接运行的方式，所以要先把包路径加到 `PYTHONPATH`：

```bash
export PYTHONPATH=/home/zoomlion/work/99_robot/zl_robot/src/robot_rl_manager
```

如果你希望临时执行，也可以把它写在单条命令前面。

### 4. 查看可转换模型

```bash
python3 -m robot_rl_manager.convert --list
```

这个命令会列出当前清单里允许转换的模型，例如 `t4_heart`、`t4_walk` 等。

### 5. 转换单个模型

```bash
python3 -m robot_rl_manager.convert --model t4_heart
```

如果你想显式指定输入的 ONNX 清单、模型注册表或输出路径，也可以继续加参数：

```bash
python3 -m robot_rl_manager.convert \
  --model t4_heart \
  --manifest /home/zoomlion/work/99_robot/zl_robot/src/robot_rl_manager/robot_rl_manager/convert/manifest.yaml \
  --registry /home/zoomlion/work/99_robot/zl_robot/config/humanoid/rl_models.yaml \
  --output /tmp/t4_heart.rknn
```

### 6. 调试时常用的两个开关

- `--verify`：强制执行数值验证
- `--no-verify`：跳过数值验证，只做导出

例如：

```bash
python3 -m robot_rl_manager.convert --model t4_heart --no-verify
```

### 7. 把生成的 RKNN 拷到目标板

转换完成后，生成的 `.rknn` 文件需要放到 RK3588 目标板上，再由运行时加载。
当前运行时侧使用的是 `rknnlite.api.RKNNLite`。

### 8. 理解验证结果

默认情况下，转换会尝试做一轮 ONNX Runtime 对比验证。
验证的作用是帮助你发现明显的算子差异或精度偏差，不是导出门禁。

当前代码的实际行为是：

- `--verify`：强制开启验证
- `--no-verify`：完全跳过验证
- 验证失败时，默认只打印 warning，不会阻断 `.rknn` 导出

也就是说，即使你看到 `verification failed` 的提示，只要导出流程已经走完，`.rknn` 文件通常还是可用的。
如果你只是想批量导出模型，可以先用 `--no-verify` 跑通流程；如果你想排查数值偏差，再单独打开验证。

### 9. 清理临时文件

转换时，RKNN Toolkit2 会在当前目录生成一些调试检查文件，例如：

- `check*.onnx`
- `check*.json`
- `check*.txt`
- `check*.rknn`

脚本结束时会自动清理这些文件，正常情况下不会保留。
如果你希望保留某次转换的补丁版 ONNX 方便排查，可以额外传 `--keep-patched-onnx`。

## `t4_heart` 说明

- 输入 shape：`1 x 144`
- 不需要额外图补丁
- 转换器可以自动推断输入 shape，但清单里显式写出来更利于复现
- 这个模型在 RKNN float16 路径下的输出和 ONNX Runtime 之间可能会有可见偏差；如果只想导出，可以用 `--no-verify`

## 验证说明

验证会用确定性的合成输入，对比 RKNN 输出和 ONNX Runtime 输出。
它只是开发期检查，不是正式的数值认证。

如果你只想把模型批量导出来，验证结果可以当成辅助诊断信息；如果后续需要更严格的门禁控制，可以再在命令行层增加“验证失败即退出”的开关。

如果模型里用了随机算子或 RKNN 不支持的算子，就在 `patches.py` 里加补丁，并在 `manifest.yaml` 中引用。

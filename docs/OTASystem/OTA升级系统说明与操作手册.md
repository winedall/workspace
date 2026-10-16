# OTA 升级系统说明与操作手册

本文说明当前仓库中 `robot_ota` 已实现的 OTA 升级系统。内容以
`src/robot_ota`、`src/robot_bringup/launch/zl_robot_ota.launch.py` 和
`src/robot_bringup/config/ota_params.yaml` 的现行代码为准。

## 1. 系统定位

`robot_ota` 用于管理 ZL Robot 软件版本发布、设备登记、Agent 部署、升级任务下发、
升级进度回传、回滚和审计记录。系统由两部分组成：

- OTA Server：运行在服务端或调试主机上，通过 ROS 2 节点启动 FastAPI Web/API 服务。
- OTA Agent：运行在机器人设备上，周期性向 Server 心跳、拉取升级任务，并在本机执行升级。

升级采用 Agent 主动拉取模式。Server 创建部署任务后不会直接控制机器人进程，Agent 每 2 秒
向 Server 查询是否有待执行任务。设备在线状态也由 Agent 心跳计算，当前判定窗口为最近 5 秒。

## 2. 运行入口

### 2.1 Server 入口

OTA Server 的 ROS 2 launch 入口为：

```bash
ros2 launch robot_bringup zl_robot_ota.launch.py
```

该 launch 启动 `robot_ota` 包中的 `robot_ota_node`，节点内部启动 uvicorn，并加载：

```text
src/robot_bringup/config/ota_params.yaml
```

当前默认参数为：

```yaml
robot_ota_node:
  ros__parameters:
    port: 8000
    host: "0.0.0.0"
    secret_key: "CHANGE_ME_IN_PRODUCTION"
    admin_username: "admin"
    admin_password_hash: ""
```

其中 `host` 和 `port` 是当前代码实际使用的监听参数。`robot_ota_node` 还会自动推导工作区下的
`data` 目录作为默认数据目录，并通过环境变量 `DATA_ROOT` 传给 FastAPI 服务。

### 2.2 Web 访问

Server 启动后，通过浏览器访问：

```text
http://<server_ip>:8000
```

Web 页面主要包括：

- 设备管理：添加设备、下载或 SSH 部署 Agent、查看设备详情和版本历史。
- 版本管理：上传发布包和签名文件，查看上传进度，删除版本。
- 部署管理：创建升级任务，查看任务状态和进度，对最新成功部署执行回滚。
- 审计日志：查看关键操作和升级终态记录。

当前认证逻辑由 `server/auth.py` 实现，默认管理员账号密码为 `admin/admin`，登录成功后签发内存
Bearer Token。服务重启后已签发 Token 会失效。生产环境使用前应修改默认认证方案或接入外部认证。

## 3. 数据与目录结构

### 3.1 Server 端

默认数据目录位于工作区：

```text
data/
├── ota.db                 # SQLite 数据库
├── releases/              # 已上传发布包和签名文件
├── deploy_packages/       # 按设备生成的 Agent 部署包
└── upload_tasks/          # 分阶段上传任务临时目录
```

SQLite 中主要表包括：

| 表 | 作用 |
|---|---|
| `devices` | 设备身份、Agent 地址、当前版本、心跳和机器人状态 |
| `releases` | 发布版本、归档路径、签名路径、SHA256、发布说明 |
| `deployments` | 每台设备的一次升级任务，批量部署通过 `batch_id` 关联 |
| `agent_deployments` | 通过 SSH 安装 Agent 的任务记录和日志 |
| `audit_logs` | 创建设备、上传版本、部署、升级终态和回滚等审计记录 |
| `release_uploads` | Web 上传版本包时的进度记录 |

### 3.2 Agent 端

Agent 默认安装目录为：

```text
~/.zl_robot
```

典型目录结构为：

```text
~/.zl_robot/
├── ota_agent/                 # Agent 代码、虚拟环境、公钥和服务模板
├── releases/                  # 本机保留的版本目录
├── current -> releases/x.y.z  # 当前运行版本软链接
└── shared/
    ├── device/device.yaml     # 设备 ID、Token、Server 地址和安装路径
    ├── downloads/             # 发布包下载缓存
    ├── logs/                  # Agent 日志和 ROS 日志
    └── ota-state/             # 升级状态持久化文件
```

Agent 安装脚本会注册并启用两个 systemd 服务：

- `ota_agent.service`：OTA Agent 自身。
- `zl_robot.service`：机器人主程序服务，由 OTA 升级过程停止、安装服务文件并重新启动。

## 4. 设备接入流程

### 4.1 添加设备

在 Web 的“设备管理”页面点击“添加设备”，填写设备 ID 和 Server 地址。Server 会：

1. 为设备生成随机 Token。
2. 在数据库中保存 Token 的 SHA256 哈希。
3. 生成该设备专用的 Agent 部署包：

```text
data/deploy_packages/<device_id>/ota_agent.tar.gz
```

部署包内包含 Agent 代码、安装脚本、systemd 服务模板、签名公钥和设备专用 `device.yaml`。

设备 Token 明文只在设备创建和部署包生成时使用。Agent 通过 `device.yaml` 中的 Token 访问
Server 的 Agent API。

### 4.2 部署 Agent

设备管理页面提供两种 Agent 部署方式：

- 下载部署包后手动拷贝到设备执行 `install.sh`。
- 点击“部署”，由 Server 通过 SSH 上传部署包并远程执行安装脚本。

SSH 部署流程为：

```text
连接目标设备
  -> 清理并创建 /tmp/ota_agent
  -> 上传 ota_agent.tar.gz
  -> 解压部署包
  -> 执行 bash /tmp/ota_agent/install.sh --agent-root <安装路径>
  -> 重启 ota_agent.service
```

页面中“SSH / Sudo 密码”会同时用于 SSH 登录和远程 `sudo -S` 执行安装命令。目标设备需要具备：

- 可 SSH 登录。
- 已安装 Python 3 和 `venv` 支持。
- 当前用户可通过 sudo 写入 `/etc/systemd/system/` 并执行 `systemctl`。
- 能访问 OTA Server 的 HTTP 地址。

部署完成后，设备列表会等待 Agent 心跳。Agent 最近 5 秒内上报心跳则显示“在线”。

## 5. 发布包要求

### 5.1 文件命名

Web 上传页面按文件名解析产品、版本和平台。发布包文件名应符合：

```text
{product}-{version}-{platform}.tar.gz
```

示例：

```text
zl_robot-1.2.3-linux-amd64.tar.gz
```

页面也接受 `.tar.zst` 后缀用于命名解析，但当前下载接口的 media type 仍按 gzip 返回，Agent
使用 Python `tarfile` 自动识别归档格式。建议现阶段优先使用 `.tar.gz`。

签名文件名必须为：

```text
{archive_filename}.sig
```

示例：

```text
zl_robot-1.2.3-linux-amd64.tar.gz.sig
```

### 5.2 包内结构

发布包解压后必须包含 `manifest.json`。当前代码支持两种结构：

```text
manifest.json
scripts/
install.sh
scripts/run.sh
...
```

或：

```text
<single_root_dir>/
├── manifest.json
├── scripts/
│   ├── run.sh
│   └── install.sh
└── ...
```

`manifest.json` 至少应包含：

```json
{
  "version": "1.2.3",
  "files": {
    "scripts/run.sh": "<sha256>"
  }
}
```

`files` 中列出的每个文件都会在 Agent 端逐个计算 SHA256 并比对。目标版本要被认为是有效版本，
目录内还必须存在：

```text
scripts/run.sh
```

### 5.3 可选脚本和服务文件

如果发布包中存在：

```text
scripts/install.sh
```

Agent 会在切换 `current` 软链接之后、启动新版本之前执行该脚本。该脚本适合处理新版本需要的
udev、setcap、ldconfig 等系统配置。脚本超时时间为 120 秒，返回非零会触发失败处理。

如果发布包中存在：

```text
scripts/service/zl_robot.service
```

Agent 会用它安装 `/etc/systemd/system/zl_robot.service`。否则使用 Agent 自带的
`zl_robot.service` 模板。服务文件中的以下占位符会被替换：

| 占位符 | 替换值 |
|---|---|
| `__AGENT_ROOT__` | Agent 安装根目录 |
| `__USER__` | Agent 根目录属主对应的用户名 |

## 6. 版本上传流程

在“版本管理”页面点击“上传版本”，选择发布包和 `.sig` 文件。上传逻辑为：

1. 浏览器提交版本号、产品名、平台、发布说明、发布包和签名文件。
2. Server 将文件写入临时目录。
3. 后台移动文件到 `data/releases/`。
4. 计算发布包 SHA256。
5. 写入 `releases` 表。
6. 写入审计日志。

版本唯一性由 `(product, version, platform)` 约束保证。重复上传同一产品、版本和平台会失败。

上传进度页面状态包括：

| 状态 | 含义 |
|---|---|
| `created` | 已创建上传任务 |
| `uploading` | 浏览器或服务端正在接收文件 |
| `processing` | 文件已接收，正在移动、计算哈希和入库 |
| `succeeded` | 上传完成，版本可用于部署 |
| `failed` | 上传失败，错误信息显示在页面 |

## 7. 升级部署流程

### 7.1 创建部署

在“部署管理”页面点击“新建部署”，选择发布版本和目标设备。Server 会为每台设备创建一条
`deployments` 记录，同一批操作共享一个 `batch_id`。

部署创建后初始状态为：

```text
pending
```

Agent 下次轮询 `/api/v1/agent/tasks/next` 时会领取最早的待执行任务，Server 将任务状态更新为：

```text
CREATED
```

### 7.2 Agent 执行升级

Agent 领取任务后，在独立线程中执行升级。正常升级路径为：

```text
CREATED
  -> TRANSFERRING
  -> VERIFYING
  -> STAGED
  -> SAFETY_CHECKING
  -> STOPPING
  -> SWITCHING
  -> STARTING
  -> HEALTH_CHECKING
  -> SUCCEEDED
```

各阶段含义如下：

| 状态 | 进度 | 含义 |
|---|---:|---|
| `CREATED` | 0% | Agent 已领取任务 |
| `TRANSFERRING` | 10% | 下载发布包和签名文件 |
| `VERIFYING` | 25% | 校验签名、解包并按 manifest 校验文件哈希 |
| `STAGED` | 40% | 新版本已准备到本机版本目录 |
| `SAFETY_CHECKING` | 50% | 等待机器人进入可升级状态 |
| `STOPPING` | 60% | 停止 `zl_robot.service` |
| `SWITCHING` | 70% | 原子切换 `current` 软链接到目标版本 |
| `STARTING` | 80% | 安装服务文件并启动 `zl_robot.service` |
| `HEALTH_CHECKING` | 90% | 检查新服务是否稳定运行 |
| `SUCCEEDED` | 100% | 升级成功，设备当前版本更新为目标版本 |

### 7.3 安全等待

非首次安装时，Agent 在 `SAFETY_CHECKING` 阶段会尝试读取：

```text
state
```

如果 `zl_robot.service` 没有运行，Agent 会跳过安全等待，允许升级修复已崩溃或已停止的服务。

如果服务正在运行，Agent 会在 30 秒内读取机器人状态。当前逻辑只阻塞运动状态，状态值为
`3` 到 `9` 时继续等待；其他状态会放行升级。超过 30 秒仍未进入可升级状态，则任务失败。

### 7.4 健康检查

当前代码中的健康检查以 systemd 服务状态为准：

```bash
systemctl is-active zl_robot.service
```

Agent 按 `healthcheck_interval` 间隔检查，连续达到 `stable_window` 次成功即认为健康检查通过。
默认配置为间隔 2 秒、连续 3 次成功、总超时 60 秒。

注意：代码注释中有 ROS 状态检查描述，但当前 `check_robot_healthy()` 的实际实现只检查
`zl_robot.service` 是否 active。现场判断升级质量时，可结合 ROS topic、业务日志和系统日志进一步确认。

## 8. 校验、切换与回滚机制

### 8.1 完整性和安全校验

Agent 在启动新版本前会执行多层校验：

1. 使用 Ed25519 公钥验证发布包签名。
2. 解包前拒绝绝对路径、`..` 路径穿越、越界符号链接、字符设备、块设备和 FIFO。
3. 加载 `manifest.json`。
4. 对 manifest 中列出的文件逐个计算 SHA256 并比对。

任一校验失败，任务进入 `FAILED`，不会停止当前正在运行的机器人服务，也不会切换 `current`。

### 8.2 原子切换

版本切换通过 `current` 软链接完成：

```text
~/.zl_robot/current -> ~/.zl_robot/releases/<version>
```

切换时先创建临时软链接 `current.tmp`，再执行 `rename` 覆盖 `current`。在 POSIX 文件系统上该
操作是原子的，可避免升级过程中出现半切换状态。

### 8.3 自动回滚

如果已经进入切换后的阶段，并在启动或健康检查过程中失败，Agent 会自动回滚：

```text
STARTING / HEALTH_CHECKING
  -> ROLLING_BACK
  -> ROLLED_BACK 或 RECOVERY_REQUIRED
```

自动回滚会：

1. 检查旧版本目录是否仍然有效。
2. 将 `current` 软链接切回旧版本。
3. 重新安装旧版本服务文件。
4. 启动 `zl_robot.service`。
5. 再次执行健康检查。

如果旧版本不存在、旧版本缺少 `manifest.json` 或 `scripts/run.sh`、旧版本启动失败，任务会进入：

```text
RECOVERY_REQUIRED
```

该状态表示新旧版本均无法自动恢复，需要人工登录设备处理。

### 8.4 手动回滚

Web 的“部署管理”页面只允许对某设备最新一条 `SUCCEEDED` 部署显示“回滚”按钮。点击回滚后，
Server 会查找该部署的 `from_version` 对应发布包，并创建一个新的部署任务，将设备切回上一版本。

手动回滚本质上也是一次新的 OTA 部署，因此仍由 Agent 拉取并执行。

## 9. 状态排查

### 9.1 Server 侧检查

检查 OTA Server 是否启动：

```bash
curl http://127.0.0.1:8000/health
```

返回：

```json
{"status":"ok"}
```

检查 ROS 节点：

```bash
ros2 node list
```

应能看到：

```text
/robot_ota_node
```

### 9.2 Agent 侧检查

检查 Agent 服务：

```bash
systemctl status ota_agent.service
```

检查机器人服务：

```bash
systemctl status zl_robot.service
```

查看 Agent 日志：

```bash
journalctl -u ota_agent.service -f
```

或查看 Agent 文件日志：

```text
~/.zl_robot/shared/logs/ota_agent.log
```

查看当前版本软链接：

```bash
readlink -f ~/.zl_robot/current
```

查看设备身份配置：

```bash
cat ~/.zl_robot/shared/device/device.yaml
```

### 9.3 常见问题

| 现象 | 可能原因 | 处理建议 |
|---|---|---|
| 设备一直离线 | Agent 未安装、服务未启动、Server 地址不可达、Token 不匹配 | 检查 `ota_agent.service`、`device.yaml` 和网络连通性 |
| Agent 部署失败 | SSH 凭据错误、sudo 权限不足、目标机缺少 Python venv | 查看页面部署日志和目标机 systemd 权限 |
| 上传版本失败 | 版本重复、文件名不符合格式、签名文件名不匹配 | 更换版本号或修正文件名 |
| 升级停在 `pending` | Agent 未在线或未成功拉取任务 | 检查 Agent 心跳和 Server URL |
| 升级 `VERIFYING` 失败 | 签名无效、manifest 缺失、文件哈希不匹配、归档结构异常 | 重新生成发布包、manifest 和签名 |
| 升级 `SAFETY_CHECKING` 失败 | 机器人长时间处于运动状态或状态读取异常 | 让机器人进入非运动状态后重新部署 |
| 升级后 `RECOVERY_REQUIRED` | 新版本失败且旧版本也无法自动恢复 | 人工登录设备，检查 `current`、版本目录和 `zl_robot.service` |

## 10. 关键 API

Web 页面使用以下主要 API。除下载接口和 Agent 专用接口外，管理接口需要管理员 Bearer Token。

| 方法 | 路径 | 作用 |
|---|---|---|
| `GET` | `/health` | Server 存活检查 |
| `GET` | `/api/v1/devices/` | 查询设备列表 |
| `POST` | `/api/v1/devices/` | 添加设备并生成部署包 |
| `GET` | `/api/v1/devices/{device_id}/deploy-package` | 下载设备专用 Agent 部署包 |
| `POST` | `/api/v1/devices/{device_id}/agent-deployments` | 通过 SSH 部署 Agent |
| `GET` | `/api/v1/releases/` | 查询版本列表 |
| `POST` | `/api/v1/releases/upload` | 上传发布包和签名文件 |
| `GET` | `/api/v1/releases/{release_id}/download` | Agent 下载发布包 |
| `GET` | `/api/v1/releases/{release_id}/download.sig` | Agent 下载签名文件 |
| `POST` | `/api/v1/deployments/` | 创建升级部署 |
| `POST` | `/api/v1/deployments/{deployment_id}/rollback` | 创建回滚部署 |
| `POST` | `/api/v1/agent/heartbeat` | Agent 上报心跳 |
| `GET` | `/api/v1/agent/tasks/next` | Agent 拉取下一个任务 |
| `POST` | `/api/v1/agent/tasks/{task_id}/status` | Agent 回报任务状态 |

Agent 本地还提供简易 HTTP API，默认监听 `0.0.0.0:8001`：

| 方法 | 路径 | 作用 |
|---|---|---|
| `GET` | `/v1/status` | 查询 Agent 当前任务状态 |
| `POST` | `/v1/tasks/install` | 本地触发安装任务 |
| `POST` | `/v1/tasks/rollback` | 本地触发回滚任务 |
| `POST` | `/v1/tasks/{id}/cancel` | 取消切换前的任务 |

## 11. 使用注意事项

1. 生产环境不要继续使用默认 `admin/admin`。
2. 签名私钥只能保存在打包机或受控环境，设备侧和 Server 部署包中只应包含公钥。
3. 发布包中的 `manifest.json` 应覆盖关键脚本、配置、二进制和模型文件。
4. 新版本的 `scripts/install.sh` 应保持幂等，避免重复执行造成系统配置损坏。
5. `zl_robot.service` 的服务文件应与发布包中的目录结构匹配，尤其是 `ExecStart`、环境变量和用户权限。
6. 升级前应确认机器人不处于运动状态，避免在 `SAFETY_CHECKING` 阶段长时间等待或失败。
7. 删除版本会同时删除关联部署记录和磁盘文件，删除设备会删除该设备相关部署记录和 Agent 部署记录。
8. `RECOVERY_REQUIRED` 需要人工介入，不应继续批量下发新任务。
9. 当前健康检查只确认 systemd 服务 active，正式验收时应结合 ROS topic、业务状态和运动安全检查。


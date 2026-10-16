# robot_ota

机器人固件 OTA（Over-The-Air）更新系统。包含 **服务端**（FastAPI Web 管理平台）和 **客户端 Agent**（机器人端守护进程）两部分，支持固件发布、批量部署、安全校验、自动回滚。

## 架构概览

```
┌──────────────────────────────────────────────────────┐
│  OTA Server (robot_ota_node)                         │
│  ┌─────────────┐  ┌──────────┐  ┌─────────────────┐ │
│  │ FastAPI      │  │ SQLite   │  │ Jinja2 Web UI   │ │
│  │ REST API     │  │ (WAL)    │  │ (dashboard/…)   │ │
│  └──────┬───────┘  └──────────┘  └─────────────────┘ │
│         │                                              │
└─────────┼──────────────────────────────────────────────┘
          │ HTTP (pull: heartbeat + task poll, every 2s)
┌─────────┼──────────────────────────────────────────────┐
│  OTA Agent (systemd: ota_agent.service)                │
│  ┌──────┴───────┐  ┌──────────────┐  ┌─────────────┐ │
│  │ Sync Loop    │  │ UpgradeEngine│  │ HTTP Server  │ │
│  │ (heartbeat + │  │ (15-state    │  │ (port 8001)  │ │
│  │  task poll)  │  │  state mach) │  │              │ │
│  └──────────────┘  └──────────────┘  └─────────────┘ │
│                                                       │
│  robot_app (systemd: zl_robot.service)  ← 被升级目标   │
└───────────────────────────────────────────────────────┘
```

## 快速开始

### 启动服务端

```bash
# 作为 ROS 2 节点启动
ros2 launch robot_bringup zl_robot_ota.launch.py

# 或直接运行
ros2 run robot_ota robot_ota_node --ros-args -p data_root:=/var/lib/zl_robot/ota
```

服务端启动后，访问 `http://<server_ip>:8000/dashboard` 进入管理界面。默认账号 `admin`/`admin`。

### 部署 Agent 到机器人

1. 在 Web UI 的 Devices 页面点击 "Add Device"，输入设备 ID
2. 下载生成的部署包（`.tar.gz`），或通过 SSH 直接部署
3. 在机器人上运行：
   ```bash
   tar xzf robot-01_ota_agent.tar.gz
   cd robot-01_ota_agent
   sudo bash install.sh
   ```
4. Agent 自动注册为 systemd 服务并开始心跳

### 发布固件

1. 准备符合规范的发布包（`.tar.gz` + Ed25519 签名 `.sig`）
2. 在 Web UI 的 Releases 页面点击 "Upload Release"
3. 选择要部署的设备和版本，创建 Deployment
4. Agent 自动拉取任务并执行升级

## 升级流程（15 状态状态机）

```
CREATED → TRANSFERRING → VERIFYING → STAGED → SAFETY_CHECKING
                                                    ↓
                                               STOPPING
                                                    ↓
                                               SWITCHING
                                                    ↓
                                               STARTING
                                                    ↓
                                           HEALTH_CHECKING
                                                    ↓
                                               SUCCEEDED
```

### 各阶段说明

| 阶段 | 进度 | 操作 |
|------|------|------|
| `CREATED` | 0% | 任务创建，等待 Agent 拉取 |
| `TRANSFERRING` | 10% | 下载 `.tar.gz` 和 `.sig` |
| `VERIFYING` | 25% | Ed25519 签名校验 + 安全解压 + SHA256 文件完整性校验 |
| `STAGED` | 40% | 文件就位，设置权限 |
| `SAFETY_CHECKING` | 50% | 等待机器人进入安全状态（非运动状态），超时 30s |
| `STOPPING` | 60% | `systemctl stop zl_robot.service` |
| `SWITCHING` | 70% | 原子切换 `current` 符号链接 + 安装新 systemd unit + daemon-reload |
| `STARTING` | 80% | `systemctl start zl_robot.service` |
| `HEALTH_CHECKING` | 90% | 持续健康检查（默认 2s 间隔，需连续 3 次成功，总超时 60s） |
| `SUCCEEDED` | 100% | 清理旧版本（保留最新 3 个），清除持久化状态 |

### 错误处理

- **SWITCHING 之前失败** → `FAILED`（可重试）或 `CANCELLED`（手动取消）
- **SWITCHING 之后失败** → 自动回滚到 `from_version` → `ROLLED_BACK` 或 `RECOVERY_REQUIRED`

### 取消规则

- 在 `STOPPING` 之前允许取消
- `STOPPING` 之后不允许取消（机器人已停止，必须完成切换）

## REST API

### 管理端 API（需 Bearer Token）

| 方法 | 路径 | 说明 |
|------|------|------|
| `GET/POST` | `/api/v1/devices` | 设备列表/注册 |
| `GET/DELETE` | `/api/v1/devices/{id}` | 设备详情/删除 |
| `GET` | `/api/v1/devices/{id}/deploy-package` | 下载 Agent 部署包 |
| `POST` | `/api/v1/devices/{id}/agent-deployments` | SSH 部署 Agent |
| `GET/POST` | `/api/v1/releases` | 版本列表/上传 |
| `GET/DELETE` | `/api/v1/releases/{id}` | 版本详情/删除 |
| `GET` | `/api/v1/releases/{id}/download` | 下载固件（Agent 使用） |
| `GET/POST` | `/api/v1/deployments` | 部署列表/批量创建 |
| `POST` | `/api/v1/deployments/{id}/rollback` | 回滚部署 |
| `GET` | `/api/v1/audit` | 审计日志 |

### Agent API（需 Device Token）

| 方法 | 路径 | 说明 |
|------|------|------|
| `POST` | `/api/v1/agent/heartbeat` | Agent 心跳（含版本、机器人状态） |
| `GET` | `/api/v1/agent/tasks/next` | 拉取待执行任务 |
| `POST` | `/api/v1/agent/tasks/{id}/status` | 上报任务状态和进度 |

### Agent 本地 API（端口 8001）

| 方法 | 路径 | 说明 |
|------|------|------|
| `GET` | `/v1/status` | Agent 状态查询 |
| `POST` | `/v1/tasks/install` | 推送升级任务 |
| `POST` | `/v1/tasks/rollback` | 推送回滚任务 |
| `POST` | `/v1/tasks/{id}/cancel` | 取消任务 |

## 数据库（SQLite，6 张表）

| 表 | 用途 |
|----|------|
| `devices` | 设备注册信息（ID、token_hash、当前版本、心跳时间） |
| `releases` | 固件版本（版本号、归档路径、SHA256、签名、manifest） |
| `deployments` | 部署记录（设备、版本、状态机阶段、进度、错误信息） |
| `agent_deployments` | Agent 部署记录（SSH 目标、状态、日志） |
| `audit_logs` | 操作审计日志 |
| `release_uploads` | 上传任务状态 |

## 安全机制

1. **固件签名**：Ed25519 签名校验（PyNaCl），Agent 在 `VERIFYING` 阶段验证
2. **文件完整性**：SHA256 逐文件校验，与 manifest.json 对比
3. **安全解压**：阻止路径穿越（`..`）、绝对路径、符号链接逃逸、设备节点
4. **设备认证**：每个设备有独立 Token，服务器存储 SHA256(token_hash)
5. **原子操作**：符号链接切换和状态持久化均使用临时文件+rename（POSIX 原子操作）
6. **SAFETY_CHECKING**：仅在机器人非运动状态时允许升级

## 断电容错

- `OtaStateStore` 在每次状态转换后以原子写入方式持久化到 `~/.zl_robot/shared/ota-state/ota_state.json`
- Agent 启动时检查持久化文件：若为终态则清除，若为非终态则记录告警
- `SWITCHING` 使用原子符号链接切换，不会出现"半切换"状态

## 目录结构

### 服务端
```
robot_ota/server/
  app.py              FastAPI 应用入口
  config.py           环境变量配置（pydantic-settings）
  database.py         SQLite 数据库（WAL + 线程本地连接）
  models.py           数据类实体
  schemas.py          Pydantic 请求/响应模型
  auth.py             认证（Bearer Token + Cookie + Device Token）
  audit_log.py        审计日志
  agent_deploy.py     SSH Agent 部署器（paramiko）
  routers/
    devices.py         设备 CRUD + 部署包生成
    releases.py        版本上传/下载/删除
    deployments.py     部署管理 + 回滚
    agent.py           Agent 心跳 + 任务拉取/上报
    audit.py           审计日志查询
    web.py             Jinja2 页面路由
```

### Agent
```
robot_ota/agent/
  main.py             入口（同步循环 + 断电容错恢复）
  config.py           AgentConfig 数据类
  client.py           服务端 HTTP 客户端
  state_machine.py    15 状态升级状态机
  upgrade_engine.py   升级编排引擎
  server.py           本地 HTTP API
  healthcheck.py      机器人健康检查
  persistence.py      OtaStateStore（断电容错）
  package.py          归档校验/解压/manifest/清理
  systemd_ctl.py      systemctl 封装
  install.sh          Agent 安装脚本
  ota_agent.service   systemd unit 模板
  zl_robot.service    systemd unit 模板
```

## 版本清理策略

Agent 保留当前版本 + 上一个版本，最多保留 3 个版本目录。`SUCCEEDED` 后自动清理超出保留数的旧版本。

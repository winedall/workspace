# zl_robot 仓库开发、测试与发布操作指导

本文用于手动模拟 `zl_robot` 仓库从日常开发、测试验证到版本发布的完整流程。流程与《机器人运动控制程序版本发布规范-V0.1》中的 DEV、RC、RELEASE、HOTFIX 状态保持一致。

## 1. 基本约定

### 1.1 分支约定

```text
main                       正式发布分支，只保存 RELEASE / HOTFIX 版本
develop                    日常集成分支，对应 DEV 内测状态
feature/<功能名>            单功能开发分支
release/vX.Y.Z             RC 验证和正式发布准备分支
hotfix/vX.Y.Z-hotfix.N     现场紧急修复分支
```

禁止直接在 `main` 上做日常开发。

### 1.2 版本号约定

```text
DEV      v1.2.6-dev.1
RC       v1.2.6-rc.1
RELEASE  v1.2.6
HOTFIX   v1.2.6-hotfix.1
```

发布包命名：

```text
zl_robot-1.2.6-linux-amd64.tar.gz
zl_robot-1.2.6-linux-amd64.tar.gz.sig
```

### 1.3 发布一致性要求

每个 RC、RELEASE、HOTFIX 版本必须能对应到：

```text
Git tag
Git commit SHA
VERSION.txt 发布记录
release 发布包
发布说明 / 测试记录
```

## 2. 日常开发流程 DEV

### 2.1 从 develop 拉功能分支

```bash
git checkout develop
git pull origin develop
git checkout -b feature/example-change
```

### 2.2 开发和本地提交

修改代码、配置或文档后，查看变更：

```bash
git status
git diff
```

提交：

```bash
git add <changed-files>
git commit -m "feat: describe example change"
```

### 2.3 本地测试

只改某个包时，优先跑相关包：

```bash
colcon build --packages-select robot_state
colcon test --packages-select robot_state
colcon test-result --verbose --test-result-base build/robot_state
```

涉及接口、launch、配置、控制链路时，建议跑全量：

```bash
colcon build
colcon test
colcon test-result --verbose
```

### 2.4 合并回 develop

```bash
git checkout develop
git pull origin develop
git merge --no-ff feature/example-change
git push origin develop
```

功能分支合并后可以删除：

```bash
git branch -d feature/example-change
```

## 3. RC 提测流程

RC 用于跨团队真机联合验证。禁止跳过 RC 直接发布 RELEASE，HOTFIX 除外。

### 3.1 从 develop 创建 release 分支

示例版本：`v1.2.6`。

```bash
git checkout develop
git pull origin develop
git checkout -b release/v1.2.6
```

### 3.2 更新 VERSION.txt

在 `VERSION.txt` 末尾增加一段发布记录，建议包含：

```text
date=2026-08-03
version=zl_robot-v1.2.6-rc.1
status=RC
git_tag=v1.2.6-rc.1
git_commit=<待打 tag 前确认>
robot_type=humanoid,dog
change_summary=...
known_issues=...
```

然后提交：

```bash
git add VERSION.txt
git commit -m "chore: prepare v1.2.6 rc1"
```

### 3.3 构建和测试

```bash
colcon build
colcon test
colcon test-result --verbose
```

### 3.4 打 RC tag

先确认当前 commit：

```bash
git rev-parse HEAD
```

打 tag：

```bash
git tag -a v1.2.6-rc.1 -m "zl_robot v1.2.6 RC1"
```

### 3.5 生成 RC 发布包

RC 包需要显式传入带状态的版本号：

```bash
./scripts/release.sh 1.2.6-rc.1
```

生成结果应类似：

```text
releases/zl_robot-1.2.6-rc.1-linux-amd64.tar.gz
releases/zl_robot-1.2.6-rc.1-linux-amd64.tar.gz.sig
```

### 3.6 RC 验证问题处理

如果 RC1 发现问题，在 `release/v1.2.6` 上修复，重复测试后发布 RC2：

```bash
git add <changed-files>
git commit -m "fix: address rc1 validation issue"
git tag -a v1.2.6-rc.2 -m "zl_robot v1.2.6 RC2"
./scripts/release.sh 1.2.6-rc.2
```

不要移动或覆盖已有 tag。

## 4. 正式 RELEASE 发布流程

RC 真机验证通过后，才能进入正式发布。

### 4.1 更新正式发布记录

在 `release/v1.2.6` 上更新 `VERSION.txt`，记录正式版本：

```text
date=2026-08-03
version=zl_robot-v1.2.6
status=RELEASE
git_tag=v1.2.6
git_commit=<待打 tag 前确认>
robot_type=humanoid,dog
change_summary=...
known_issues=...
package=zl_robot-1.2.6-linux-amd64.tar.gz
```

提交：

```bash
git add VERSION.txt
git commit -m "chore: release v1.2.6"
```

### 4.2 发布前验证

```bash
colcon build
colcon test
colcon test-result --verbose
```

生成正式包：

```bash
./scripts/release.sh 1.2.6
```

### 4.3 合并到 main 并打正式 tag

```bash
git checkout main
git pull origin main
git merge --no-ff release/v1.2.6
git tag -a v1.2.6 -m "zl_robot v1.2.6 RELEASE"
```

推送：

```bash
git push origin main
git push origin v1.2.6
```

### 4.4 回合 develop

正式发布后，必须把 `main` 回合到 `develop`，避免后续开发丢失发布修正：

```bash
git checkout develop
git pull origin develop
git merge --no-ff main
git push origin develop
```

## 5. HOTFIX 紧急补丁流程

HOTFIX 只用于现场紧急故障修复，不能混入普通功能开发。

### 5.1 从正式 tag 创建 hotfix 分支

假设当前线上正式版本是 `v1.2.6`：

```bash
git checkout v1.2.6
git checkout -b hotfix/v1.2.6-hotfix.1
```

### 5.2 修复、测试和发布包

修复后提交：

```bash
git add <changed-files>
git commit -m "fix: address field hotfix issue"
```

验证：

```bash
colcon build
colcon test
colcon test-result --verbose
```

生成 HOTFIX 包：

```bash
./scripts/release.sh 1.2.6-hotfix.1
```

### 5.3 合并和打 tag

```bash
git checkout main
git pull origin main
git merge --no-ff hotfix/v1.2.6-hotfix.1
git tag -a v1.2.6-hotfix.1 -m "zl_robot v1.2.6 HOTFIX1"
git push origin main
git push origin v1.2.6-hotfix.1
```

再同步到 `develop`：

```bash
git checkout develop
git pull origin develop
git merge --no-ff main
git push origin develop
```

## 6. 发布包检查

生成发布包后，建议检查文件：

```bash
ls -lh releases/
```

检查包名：

```text
zl_robot-<version>-linux-amd64.tar.gz
zl_robot-<version>-linux-amd64.tar.gz.sig
```

查看包内结构：

```bash
tar -tzf releases/zl_robot-1.2.6-linux-amd64.tar.gz | head -50
```

包内至少应包含：

```text
manifest.json
VERSION.txt
install/
config/
scripts/run.sh
scripts/install.sh
```

## 7. 现场或样机验证建议

发布前建议至少完成：

```bash
./scripts/run.sh
ros2 topic echo /robot/version
ros2 topic list
```

根据版本内容选择验证项：

```text
启动流程
遥控器连接与超时保护
idle / prepare / walk / stop / fault 流程
关节状态和告警
robot_recorder 采样文件
OTA 升级
OTA 回滚
humanoid / dog 对应机型配置
```

## 8. 禁止事项

```text
禁止无 tag 生成正式发布包
禁止覆盖或移动已经发布的 tag
禁止同一版本号对应多个不同发布包
禁止只改现场代码不回流 Git
禁止只替换 install/ 或只替换 config/
禁止 HOTFIX 混入普通功能开发
禁止 RELEASE 未经 RC 验证直接发布，现场紧急 HOTFIX 除外
```

## 9. 手动模拟建议

如果只是本地模拟完整流程，可以使用测试版本号，例如：

```text
v9.9.0-dev.1
v9.9.0-rc.1
v9.9.0
v9.9.0-hotfix.1
```

模拟时可以只在本地打 tag，不执行 `git push`。确认流程无误后，再清理本地模拟 tag：

```bash
git tag -d v9.9.0-rc.1
git tag -d v9.9.0
git tag -d v9.9.0-hotfix.1
```

如果已经推送到远端，不要随意删除远端 tag，先确认该 tag 没有被任何发布包或测试记录引用。

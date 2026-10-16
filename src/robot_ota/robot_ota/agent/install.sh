#!/usr/bin/env bash
# agent/install.sh
# 将 OTA Agent 部署到本地设备。
#
# 使用方式：
#   bash install.sh
#   bash install.sh --device-id xxx --token yyy --server-url http://...
#   bash install.sh --agent-root /home/robot/.local/zl_robot

set -euo pipefail
shopt -s nullglob

DEVICE_ID=""
DEVICE_TOKEN=""
SERVER_URL=""
AGENT_ROOT=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --device-id)
            DEVICE_ID="$2"
            shift 2
            ;;
        --token)
            DEVICE_TOKEN="$2"
            shift 2
            ;;
        --server-url)
            SERVER_URL="$2"
            shift 2
            ;;
        --agent-root)
            AGENT_ROOT="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: bash install.sh [--agent-root DIR] [--device-id ID] [--token TOKEN] [--server-url URL]"
            exit 1
            ;;
    esac
done

# 解析 agent_root 默认值：检测 sudo 场景下的真实用户 home
if [ -z "$AGENT_ROOT" ]; then
    if [ -n "${SUDO_USER:-}" ]; then
        AGENT_ROOT="$(getent passwd "$SUDO_USER" | cut -d: -f6)/.zl_robot"
    else
        AGENT_ROOT="${HOME}/.zl_robot"
    fi
fi

# 解析真实用户名（用于 systemd service User= 设置）
if [ -n "${SUDO_USER:-}" ]; then
    _REAL_USER="$SUDO_USER"
else
    _REAL_USER="$USER"
fi

_SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OTA_DIR="$(cd "$_SCRIPT_DIR/.." && pwd)"
PUBLIC_KEY_SRC="$OTA_DIR/scripts/signing/public_key.key"
if [ ! -f "$PUBLIC_KEY_SRC" ]; then
    PUBLIC_KEY_SRC="$_SCRIPT_DIR/scripts/signing/public_key.key"
fi

AGENT_INSTALL_DIR="$AGENT_ROOT/ota_agent"
AGENT_RELEASES_DIR="$AGENT_ROOT/releases"
AGENT_SHARED_DIR="$AGENT_ROOT/shared"
AGENT_OTA_STATE_DIR="$AGENT_SHARED_DIR/ota-state"
AGENT_DOWNLOADS_DIR="$AGENT_SHARED_DIR/downloads"
AGENT_LOGS_DIR="$AGENT_SHARED_DIR/logs"
AGENT_DEVICE_DIR="$AGENT_SHARED_DIR/device"

rewrite_device_agent_root() {
    local device_file="$1"
    if grep -q '^agent_root:' "$device_file"; then
        sed -i "s|^agent_root:.*|agent_root: \"$AGENT_ROOT\"|" "$device_file"
    else
        printf '\nagent_root: "%s"\n' "$AGENT_ROOT" >> "$device_file"
    fi
}
# ── 本地安装模式 ────────────────────────────────────

# ── 创建目录结构 ──────────────────────────────────────
# shared/ota_state:  升级任务持久化状态（掉电恢复用）
# shared/downloads:  发布包下载缓存
# shared/logs:       Agent 运行日志
# shared/device:     设备身份配置
echo "Creating directory structure under $AGENT_ROOT ..."
mkdir -p "$AGENT_RELEASES_DIR"
mkdir -p "$AGENT_OTA_STATE_DIR"
mkdir -p "$AGENT_DOWNLOADS_DIR"
mkdir -p "$AGENT_LOGS_DIR"
mkdir -p "$AGENT_DEVICE_DIR"
mkdir -p "$AGENT_INSTALL_DIR"

# ── 拷贝 Agent 代码 ──────────────────────────────────
# 拷贝全部 Python 文件后，删除安装过程自身相关文件（避免污染目标目录）
echo "Copying agent Python files..."
cp -r "$_SCRIPT_DIR/"* "$AGENT_INSTALL_DIR/"
rm -f "$AGENT_INSTALL_DIR/ota_agent.service"
rm -f "$AGENT_INSTALL_DIR/install.sh"
rm -f "$AGENT_INSTALL_DIR/device.yaml"

# ── 拷贝公钥 ──────────────────────────────────────────
# Agent 用公钥验证发布包签名，私钥保留在打包机上
echo "Copying public key..."
mkdir -p "$AGENT_INSTALL_DIR/scripts/signing"
cp "$PUBLIC_KEY_SRC" "$AGENT_INSTALL_DIR/scripts/signing/"

# ── Python 虚拟环境 ──────────────────────────────────
# 隔离的 venv，不依赖系统全局 Python 包
echo "Creating Python virtual environment and installing dependencies..."
WHEELS_DIR="$_SCRIPT_DIR/wheels"
if [ ! -d "$WHEELS_DIR" ] || ! compgen -G "$WHEELS_DIR/*.whl" > /dev/null; then
    echo "Missing offline Python wheels in $WHEELS_DIR"
    exit 1
fi
python3 -m venv "$AGENT_INSTALL_DIR/venv"
"$AGENT_INSTALL_DIR/venv/bin/pip" install --no-index --find-links "$WHEELS_DIR" pynacl pyyaml

# ── 离线安装 sherpa-onnx TTS 运行时依赖（系统 Python）─────────────────
# sherpa_onnx 以离线 wheel 随部署包内置（wheels/），需安装到系统 Python，
# 供机器人 TTS 节点（robot_audio）import 使用，无需联网和手动安装。
# TTS 为非关键链路：安装失败仅告警，不影响 Agent 与机器人主流程。
echo "Installing sherpa-onnx TTS dependencies (system python)..."
SHERPA_WHEELS=( "$WHEELS_DIR"/sherpa_onnx-*.whl "$WHEELS_DIR"/sherpa_onnx_core-*.whl )
if compgen -G "$WHEELS_DIR/sherpa_onnx-*.whl" >/dev/null && compgen -G "$WHEELS_DIR/sherpa_onnx_core-*.whl" >/dev/null; then
  if sudo python3 -m pip install --no-index "${SHERPA_WHEELS[@]}"; then
    if python3 -c "import sherpa_onnx" 2>/dev/null; then
      echo "  [OK] sherpa-onnx installed and importable"
    else
      echo "  [警告] sherpa-onnx installed but import failed, check system python"
    fi
  else
    echo "  [警告] sherpa-onnx install failed, TTS unavailable (robot continues)"
  fi
else
  echo "  [警告] No bundled sherpa-onnx wheels found, skipping"
fi

# ── 安装 deb 依赖包 ──────────────────────────────────
echo "Installing deb dependency packages..."
DEPENDS_DIR="$_SCRIPT_DIR/depends"
if [ -d "$DEPENDS_DIR" ] && compgen -G "$DEPENDS_DIR/*.deb" > /dev/null; then
    sudo dpkg -i "$DEPENDS_DIR"/*.deb
    echo "  [OK] deb packages installed from $DEPENDS_DIR"
else
    echo "  [警告] No deb packages found in $DEPENDS_DIR, skipping"
fi

# ── 注册 systemd 服务 ───────────────────────────────
# 使 Agent 和机器人程序随系统自启、崩溃自动重启
# 替换服务文件中的路径和用户占位符
echo "Installing systemd services (agent_root=$AGENT_ROOT)..."
sed "s|__AGENT_ROOT__|$AGENT_ROOT|g; s|__USER__|$_REAL_USER|g" "$_SCRIPT_DIR/ota_agent.service" | sudo tee /etc/systemd/system/ota_agent.service > /dev/null
sed "s|__AGENT_ROOT__|$AGENT_ROOT|g; s|__USER__|$_REAL_USER|g" "$_SCRIPT_DIR/zl_robot.service" | sudo tee /etc/systemd/system/zl_robot.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable ota_agent.service
sudo systemctl enable zl_robot.service

# ── 设备配置 ────────────────────────────────────────
if [ -n "$DEVICE_ID" ] && [ -n "$DEVICE_TOKEN" ] && [ -n "$SERVER_URL" ]; then
    # 参数提供：直接生成配置，无需手动编辑
    cat > "$AGENT_DEVICE_DIR/device.yaml" <<EOF
# 设备身份配置（由 OTA Server 自动生成）
device_id: "$DEVICE_ID"
device_token: "$DEVICE_TOKEN"
server_url: "$SERVER_URL"
agent_root: "$AGENT_ROOT"
EOF
    echo "Device configuration written to $AGENT_DEVICE_DIR/device.yaml"
elif [ -f "$_SCRIPT_DIR/device.yaml" ]; then
    # 部署包自带设备身份配置：重装时也要覆盖旧 token / server_url
    cp "$_SCRIPT_DIR/device.yaml" "$AGENT_DEVICE_DIR/device.yaml"
    echo "Packaged device configuration written to $AGENT_DEVICE_DIR/device.yaml"
elif [ ! -f "$AGENT_DEVICE_DIR/device.yaml" ]; then
    # 无参数且文件不存在：拷贝模板，提示手动编辑
    echo "Copying device.yaml template..."
    cp "$_SCRIPT_DIR/device.yaml" "$AGENT_DEVICE_DIR/device.yaml"
    echo ">>> IMPORTANT: Edit $AGENT_DEVICE_DIR/device.yaml with the correct device_id, device_token, and server_url"
fi

rewrite_device_agent_root "$AGENT_DEVICE_DIR/device.yaml"

sudo systemctl restart ota_agent.service

# ── 修复文件属主 ──────────────────────────────────────
# 如果脚本以 root 身份运行（sudo bash install.sh 或 Deploy 按钮触发的 SSH 部署），
# 将 $AGENT_ROOT 下所有文件改回真实用户属主，确保用户进程可读写
if [ -n "${SUDO_USER:-}" ]; then
    chown -R "$SUDO_USER:$SUDO_USER" "$AGENT_ROOT"
fi

echo "Installation complete!"
echo ""
echo "Next steps:"
echo "  1. Configure $AGENT_DEVICE_DIR/device.yaml"
echo "  2. Check status: systemctl status ota_agent"

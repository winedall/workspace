#!/bin/bash
# scripts/release.sh
# 用法：./release.sh [版本号]
#   版本号可选，不传时自动从 VERSION.txt 读取。
#   可通过环境变量覆盖路径：
#     WORKSPACE=/path OTA_DIR=/path ./release.sh
# 功能：编译产物打包为带签名的 OTA 发布包（tar.gz + Ed25519 签名）。

set -euo pipefail

# ── 路径初始化 ──────────────────────────────────────────
# _SCRIPT_DIR: 脚本自身所在目录，用于推导 OTA_DIR 默认值
# WORKSPACE:   zl_robot 工作区根目录（内含 install/, config/, scripts/）
# OTA_DIR:     zl_robot_ota 根目录（内含 scripts/signing/ 等）
_SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "$_SCRIPT_DIR/.." && pwd)}"
OUTPUT_DIR="${WORKSPACE}/releases"
ARCH="amd64"

# ── 版本号获取 ──────────────────────────────────────────
# 优先使用命令行参数，未传入时从 VERSION.txt 首个版本块提取。
# VERSION.txt 格式示例: version=zl_robot-v1.1.6
# 提取规则: 去掉 version= 前缀和 zl_robot-v 前缀，取 X.Y.Z 部分。
if [ $# -ge 1 ]; then
    VERSION="$1"
else
    VERSION_FILE="$WORKSPACE/VERSION.txt"
    if [ ! -f "$VERSION_FILE" ]; then
        echo "ERROR: VERSION.txt not found at $VERSION_FILE, provide version as argument"
        exit 1
    fi
    VERSION=$(grep '^version=' "$VERSION_FILE" \
      | head -1 \
      | sed 's/^version=//' \
      | sed 's/^.*-v//' \
      | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' || true)
    if [ -z "$VERSION" ]; then
        echo "ERROR: Failed to extract version from $VERSION_FILE"
        exit 1
    fi
    echo "Auto-detected version from VERSION.txt: $VERSION"
fi

# ── 临时目录与时间戳 ────────────────────────────────────
# TMP_DIR 在工作完成后会被 trap 清理
TMP_DIR=$(mktemp -d /tmp/zl_release_XXXXX)
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

trap "rm -rf $TMP_DIR" EXIT

# ── 依赖检查 ──────────────────────────────────────────
command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 required"; exit 1; }
command -v tar >/dev/null 2>&1 || { echo "ERROR: tar required"; exit 1; }
python3 -c "from nacl.bindings import crypto_sign" 2>/dev/null \
  || { echo "ERROR: pynacl (nacl) not installed, run: pip install pynacl"; exit 1; }

# gzip 由 tar 内置支持，无需额外安装

# ── 版本号格式校验 ────────────────────────────────────
if ! echo "$VERSION" | grep -qE '^[0-9]+\.[0-9]+\.[0-9]+(-[a-zA-Z0-9.]+)?$'; then
    echo "ERROR: Version must be in format X.Y.Z (e.g. 1.2.0)"
    exit 1
fi

# ── 工作区完整性检查 ──────────────────────────────────
if [ ! -d "$WORKSPACE/install" ] || [ ! -d "$WORKSPACE/config" ]; then
    echo "ERROR: Workspace must have install/ and config/ directories. Run colcon build first."
    exit 1
fi

# ══════════════════════════════════════════════════════════
#  第一步：构建发布包目录结构
# ══════════════════════════════════════════════════════════

PKG_DIR="$TMP_DIR/zl_robot-${VERSION}-linux-${ARCH}"
mkdir -p "$PKG_DIR"/{install,config,scripts}

# ── 拷贝 install/ ──────────────────────────────────────
# colcon build 产物，不含 build/ 和 log/。
# 分两步：先拷贝普通文件，再拷贝有效符号链接（跳过断链）。
find "$WORKSPACE/install" -type f -print0 | while IFS= read -r -d '' f; do
    rel="${f#$WORKSPACE/install/}"
    mkdir -p "$PKG_DIR/install/$(dirname "$rel")"
    cp "$f" "$PKG_DIR/install/$rel"
done

# 拷贝有效符号链接（指向存在的目标），跳过断裂链接
find "$WORKSPACE/install" -type l -print0 | while IFS= read -r -d '' f; do
    if [ -e "$f" ]; then
        rel="${f#$WORKSPACE/install/}"
        mkdir -p "$PKG_DIR/install/$(dirname "$rel")"
        cp -L "$f" "$PKG_DIR/install/$rel"
    fi
done

# ── 拷贝 config/ ──────────────────────────────────────
# 版本配套的运行配置，随版本一起升级和回滚
find "$WORKSPACE/config" -type f -print0 | while IFS= read -r -d '' f; do
    rel="${f#$WORKSPACE/config/}"
    mkdir -p "$PKG_DIR/config/$(dirname "$rel")"
    cp "$f" "$PKG_DIR/config/$rel"
done

# ── 拷贝 scripts/ ─────────────────────────────────────
# 排除 build.sh 和 clear.sh（仅开发环境使用），
# 保留 service/、udev/、depends/ 等子目录。
rsync -a \
  --exclude='build.sh' \
  --exclude='clear.sh' \
  --exclude='release.sh' \
  --exclude='signing/' \
  "$WORKSPACE/scripts/" "$PKG_DIR/scripts/"

# ── 拷贝 VERSION.txt ─────────────────────────────────────
# 版本信息文件，运行时由 robot_monitor 节点读取并发布到 version。
cp "$WORKSPACE/VERSION.txt" "$PKG_DIR/"

# 健康检查已由 Agent 内置实现（check_robot_healthy），不再生成独立脚本。

# ══════════════════════════════════════════════════════════
#  第二步：生成 manifest.json（文件清单）
# ══════════════════════════════════════════════════════════
# 记录每个文件的 SHA256 哈希，Agent 解包后逐文件校验完整性。
# 通过环境变量传参给 Python，避免 shell 注入风险。
export PY_PKG_DIR="$PKG_DIR"
export PY_VERSION="$VERSION"
export PY_ARCH="$ARCH"
export PY_TIMESTAMP="$TIMESTAMP"
python3 -c "
import json, os, hashlib

pkg_dir = os.environ['PY_PKG_DIR']
manifest = {
    'schema_version': 1,
    'product': 'zl_robot',
    'version': os.environ['PY_VERSION'],
    'architecture': os.environ['PY_ARCH'],
    'os': 'ubuntu-22.04',
    'ros_distribution': 'humble',
    'created_at': os.environ['PY_TIMESTAMP'],
    'min_agent_version': '1.0.0',
    'entrypoint': 'scripts/run.sh',
    'files': {},
}
for root, dirs, files in os.walk(pkg_dir):
    for fname in files:
        fpath = os.path.join(root, fname)
        if fname == 'manifest.json':
            continue
        rel = os.path.relpath(fpath, pkg_dir)
        h = hashlib.sha256()
        with open(fpath, 'rb') as f:
            for chunk in iter(lambda: f.read(65536), b''):
                h.update(chunk)
        manifest['files'][rel] = h.hexdigest()
with open(os.path.join(pkg_dir, 'manifest.json'), 'w') as f:
    json.dump(manifest, f, indent=2)
"

# ══════════════════════════════════════════════════════════
#  第三步：签名函数
# ══════════════════════════════════════════════════════════
# 使用 Ed25519 私钥对归档文件签名，生成 .sig 文件。
# Agent 持公钥，验证发布包来源可信且未被篡改。
_sign() {
    local archive="$1"
    local sig="${archive}.sig"
    export PY_KEY_PATH="${WORKSPACE}/scripts/signing/private_key.key"
    export PY_ARCHIVE_PATH="$archive"
    export PY_SIG_PATH="$sig"
    python3 -c "
import os
from nacl.bindings import crypto_sign

with open(os.environ['PY_KEY_PATH'], 'rb') as f:
    sk = f.read()
with open(os.environ['PY_ARCHIVE_PATH'], 'rb') as f:
    archive_data = f.read()
sig = crypto_sign(archive_data, sk)[:64]
with open(os.environ['PY_SIG_PATH'], 'wb') as f:
    f.write(sig)
"
}

# ══════════════════════════════════════════════════════════
#  第四步：打包并签名
# ══════════════════════════════════════════════════════════
# 先打包 → 签名 → 往 manifest 写入归档哈希 → 重新打包 → 重新签名。
# 这样 manifest.json 中的 archive_hash 与最终的包一致。

mkdir -p "$OUTPUT_DIR"
ARCHIVE_NAME="zl_robot-${VERSION}-linux-${ARCH}.tar.gz"
ARCHIVE_PATH="$OUTPUT_DIR/$ARCHIVE_NAME"
SIGNATURE_PATH="${ARCHIVE_PATH}.sig"

cd "$TMP_DIR/zl_robot-${VERSION}-linux-${ARCH}"
tar -czf "$ARCHIVE_PATH" .
_sign "$ARCHIVE_PATH"

# 计算归档文件的 SHA256，写入 manifest
ARCHIVE_HASH=$(sha256sum "$ARCHIVE_PATH" | cut -d' ' -f1)
export PY_ARCHIVE_HASH="$ARCHIVE_HASH"
export PY_MANIFEST_PATH="$TMP_DIR/zl_robot-${VERSION}-linux-${ARCH}/manifest.json"
python3 -c "
import json, os
manifest_path = os.environ['PY_MANIFEST_PATH']
with open(manifest_path, 'r') as f:
    m = json.load(f)
m['archive_hash'] = os.environ['PY_ARCHIVE_HASH']
with open(manifest_path, 'w') as f:
    json.dump(m, f, indent=2)
"

# manifest 内容变了，重新打包和签名
cd "$TMP_DIR/zl_robot-${VERSION}-linux-${ARCH}"
tar -czf "$ARCHIVE_PATH" .
_sign "$ARCHIVE_PATH"

echo "==================================="
echo "Release package created:"
echo "  $ARCHIVE_PATH"
echo "  $SIGNATURE_PATH"
echo "  Archive SHA256: $ARCHIVE_HASH"
echo "==================================="

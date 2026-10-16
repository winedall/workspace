#!/bin/bash
#
# zl_robot 系统环境配置脚本（x86_64 / aarch64 通用版）
# 完成系统级的一键配置（udev、用户组、动态库路径、EtherCAT 权限）。
# 此脚本仅在首次部署或硬件/驱动更新时执行一次。
# 服务文件安装由 OTA agent 负责（install.sh + upgrade_engine.py）。
#
# 用法：
#   1. 修改下方 PLATFORM_TYPE
#   2. 执行 ./install.sh
#
# 支持平台：
#   PLATFORM_TYPE="x86_64"   # x86 PC
#   PLATFORM_TYPE="aarch64"  # RK3588 / ARM64
#
# 注意：本脚本不含卸载逻辑，如需卸载请手动处理：
#   sudo rm /etc/udev/rules.d/99-robot-usb.rules
#   sudo rm /etc/ld.so.conf.d/ros2-workspace.conf

set -e

# ── 用户可配置项 ────────────────────────────────────────────────
# 手动指定运行平台：x86_64 或 aarch64
PLATFORM_TYPE="${ZL_ROBOT_RUNTIME_PLATFORM:-$(uname -m)}"

# ROS 发行版
ROS_DISTRO="humble"
ROS_ROOT="/opt/ros/${ROS_DISTRO}"

# sudo 密码；也可以通过环境变量覆盖：
#   export ZL_ROBOT_SUDO_PASSWORD=xxxxxx
PASSWD="${ZL_ROBOT_SUDO_PASSWORD:-admin}"
# PASSWD="${ZL_ROBOT_SUDO_PASSWORD:-123456}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_PATH="${SCRIPT_DIR}/.."

# ── 平台配置 ────────────────────────────────────────────────────
# 所有平台差异集中放在这里，后续新增平台时只需要扩展 case。
case "${PLATFORM_TYPE}" in
  x86_64)
    PLATFORM_DISPLAY_NAME="x86_64"
    ROS_MULTIARCH_LIB_DIR="${ROS_ROOT}/lib/x86_64-linux-gnu"
    ;;

  aarch64)
    PLATFORM_DISPLAY_NAME="aarch64 (RK3588/ARM64)"
    ROS_MULTIARCH_LIB_DIR="${ROS_ROOT}/lib/aarch64-linux-gnu"
    ;;

  *)
    echo "[FATAL] 不支持的平台类型: ${PLATFORM_TYPE}"
    echo "        PLATFORM_TYPE 只能设置为: x86_64 或 aarch64"
    exit 1
    ;;
esac

# 检查手动配置的平台与当前机器架构是否一致。
# 这里只警告，不自动修改，确保实际使用的平台由 PLATFORM_TYPE 明确控制。
HOST_ARCH="$(uname -m)"
if [ "${HOST_ARCH}" != "${PLATFORM_TYPE}" ]; then
  echo "[WARN] PLATFORM_TYPE=${PLATFORM_TYPE}，但当前系统 uname -m=${HOST_ARCH}"
  echo "[WARN] 请确认平台配置是否正确。"
fi

# 缓存 sudo 权限
# 优先使用配置密码，保持与原脚本行为一致。
echo "${PASSWD}" | sudo -S -v


echo "=========================================="
echo "  zl_robot 系统安装/配置"
echo "=========================================="
echo "  平台类型 : ${PLATFORM_DISPLAY_NAME}"
echo "  系统架构 : ${HOST_ARCH}"
echo "  ROS      : ${ROS_ROOT}"
echo "=========================================="

# ── 第1部分：系统环境配置（仅部署时执行一次） ──────────────────────
echo ""
echo "=== 1/5  系统环境配置 ==="

# 1.1 udev rules
echo "  -> 安装 udev rules ..."
sudo cp "${SCRIPT_DIR}/udev/99-robot-usb.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  [OK] udev rules 已安装"

# 1.2 用户组
echo "  -> 添加用户到 dialout 组 ..."
REAL_LOGIN_USER="${REAL_USER:-$(whoami)}"
sudo usermod -aG dialout "${REAL_LOGIN_USER}"
echo "  [OK] 已添加 ${REAL_LOGIN_USER} 到 dialout 组"
echo "       提示：请重新登录以使组权限生效"

# 1.3 动态库路径
# robot_driver_node 会设置 file capability，因此不能只依赖 LD_LIBRARY_PATH；
# ROS 及工作空间动态库目录需要写入 ld.so 配置并执行 ldconfig。
echo "  -> 配置动态库搜索路径 ..."

LD_CONF_FILE="/etc/ld.so.conf.d/ros2-workspace.conf"
TMP_LD_CONF="$(mktemp)"
trap 'rm -f "${TMP_LD_CONF}"' EXIT

# 工作空间中的 lib 目录
if [ -d "${PROJECT_PATH}/install" ]; then
  find "${PROJECT_PATH}/install" -type d -name lib | sort -u > "${TMP_LD_CONF}"
else
  echo "  [WARN] 未找到 ${PROJECT_PATH}/install，工作空间动态库路径暂不写入"
  : > "${TMP_LD_CONF}"
fi

# ROS 公共库目录
printf '%s\n' "${ROS_ROOT}/lib" >> "${TMP_LD_CONF}"

# 当前平台对应的 ROS multiarch 库目录
if [ -d "${ROS_MULTIARCH_LIB_DIR}" ]; then
  printf '%s\n' "${ROS_MULTIARCH_LIB_DIR}" >> "${TMP_LD_CONF}"
else
  echo "  [WARN] 当前平台 ROS multiarch 目录不存在: ${ROS_MULTIARCH_LIB_DIR}"
fi

sort -u "${TMP_LD_CONF}" | sudo tee "${LD_CONF_FILE}" >/dev/null
sudo ldconfig

echo "  [OK] 动态库路径已配置"
echo "       配置文件: ${LD_CONF_FILE}"
echo "       ROS平台库: ${ROS_MULTIARCH_LIB_DIR}"

# 关键库检查：CycloneDDS RMW 和 DDS 实现。
if ! ldconfig -p | grep -q 'librmw_cyclonedds_cpp.so'; then
  echo "  [WARN] ldconfig 缓存中未发现 librmw_cyclonedds_cpp.so"
fi

if ! ldconfig -p | grep -q 'libddsc.so.0'; then
  echo "  [WARN] ldconfig 缓存中未发现 libddsc.so.0"
fi

# ── 第2部分：EtherCAT 权限 ───────────────────────────────────────────
# setcap 设置在文件的扩展属性中，重启后依然保留。
# 但如果 robot_driver_node 二进制被更新/替换，需重新执行本步骤。
echo ""
echo "=== 2/5  EtherCAT 权限设置 ==="

DRIVER_BIN="${PROJECT_PATH}/install/robot_driver/lib/robot_driver/robot_driver_node"
if [ -f "${DRIVER_BIN}" ]; then
  sudo setcap 'cap_net_raw,cap_net_admin,cap_sys_nice+eip' "${DRIVER_BIN}"
  echo "  [OK] EtherCAT 权限已设置: ${DRIVER_BIN}"
  getcap "${DRIVER_BIN}" || true
else
  echo "  [WARN] 未找到 ${DRIVER_BIN}，请编译后重新运行 install.sh"
fi

# ── 第3部分：复制 CycloneDDS 配置文件 ────────────────────────────────
echo ""
echo "=== 3/5  复制 CycloneDDS 配置文件 ==="

mkdir -p "${HOME}/.config"
for config_file in \
  cyclonedds_bridge_whitelist.xml \
  cyclonedds_localhost.xml; do
  if [ -f "${PROJECT_PATH}/config/${config_file}" ]; then
    cp "${PROJECT_PATH}/config/${config_file}" "${HOME}/.config/"
    echo "  [OK] 已复制 ${config_file}"
  else
    echo "  [WARN] 未找到 ${PROJECT_PATH}/config/${config_file}"
  fi
done

# ── 第4部分：替换 .bashrc ───────────────────────────────────────────
echo ""
echo "=== 4/5  替换 .bashrc ==="

BASHRC_SRC="${SCRIPT_DIR}/env/.bashrc"
BASHRC_DST="${HOME}/.bashrc"
if [ -f "${BASHRC_SRC}" ]; then
  if [ -f "${BASHRC_DST}" ]; then
    BASHRC_BACKUP="${BASHRC_DST}.bak.$(date +%Y%m%d_%H%M%S)"
    cp "${BASHRC_DST}" "${BASHRC_BACKUP}"
    echo "  [OK] 已备份原 .bashrc -> ${BASHRC_BACKUP}"
  fi
  cp "${BASHRC_SRC}" "${BASHRC_DST}"
  echo "  [OK] 已替换 .bashrc"
else
  echo "  [WARN] 未找到 ${BASHRC_SRC}，跳过"
fi

# ── 第5部分：完成提示 ───────────────────────────────────────────────
echo ""
echo "=== 5/5  配置检查 ==="

echo "  PLATFORM_TYPE=${PLATFORM_TYPE}"
echo "  HOST_ARCH=${HOST_ARCH}"
echo "  ROS_MULTIARCH_LIB_DIR=${ROS_MULTIARCH_LIB_DIR}"

if [ -f "${DRIVER_BIN}" ]; then
  echo "  robot_driver_node capability:"
  getcap "${DRIVER_BIN}" || true
fi

echo ""
echo "=========================================="
echo "  安装完成"
echo "=========================================="
echo ""
echo "注意："
echo "  1. dialout 组权限需要重新登录后生效"
echo "  2. .bashrc 已更新，新终端会自动生效；当前终端请执行: source ~/.bashrc"
echo "  3. 如果 robot_driver_node 二进制文件被更新，需重新执行: ./install.sh"
echo "  4. 切换平台时只修改 PLATFORM_TYPE，然后重新执行 install.sh"

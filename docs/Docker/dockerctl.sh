#!/usr/bin/env bash
set -euo pipefail

# 镜像与容器默认配置。
# 可通过环境变量覆盖，例如：
#   DOCKERCTL_IMAGE_NAME=zl_robot_test ./dockerctl.sh run
IMAGE_NAME="${DOCKERCTL_IMAGE_NAME:-zl_robot}"
DEFAULT_TAG="${DOCKERCTL_DEFAULT_TAG:-base}"
CONTAINER_NAME="${DOCKERCTL_CONTAINER_NAME:-zl_robot}"
DOCKER_BIN="${DOCKERCTL_DOCKER_BIN:-docker}"
DOCKER_SUDO="${DOCKERCTL_SUDO-sudo}"
DRY_RUN="${DOCKERCTL_DRY_RUN:-0}"
NETWORK_MODE="${DOCKERCTL_NETWORK_MODE:-host}"
SOFTWARE_GL="${DOCKERCTL_SOFTWARE_GL:-0}"

# 打印帮助信息。
usage() {
  cat <<EOF
Usage:
  $0 run                  Create and start zl_robot:base as a systemd container
  $0 start                Start the existing stopped container
  $0 shell                Enter the running container
  $0 stop                 Stop the container
  $0 rm                   Remove the stopped container
  $0 ps                   List zl_robot containers
  $0 ip                   Show the container bridge IP
  $0 images               List all local Docker images
  $0 logs                 Follow container logs
  $0 journal              Show systemd journal inside the container
  $0 commit <version>     Commit the container as zl_robot:<version>
  $0 tag <version>        Tag zl_robot:<version> as zl_robot:base
  $0 help                 Show this help

Defaults:
  container: ${CONTAINER_NAME}
  image:     ${IMAGE_NAME}:${DEFAULT_TAG}
  network:   ${NETWORK_MODE}
  init:      /sbin/init
  ssh:       managed by systemd ssh.service, no host port mapping

The run command uses host networking by default so MuJoCo/GLFW can connect to
the host X server. Override with DOCKERCTL_NETWORK_MODE=bridge when you need a
separate container IP for OTA bridge-network tests.
EOF
}

# 统一错误退出。
die() {
  echo "[ERROR] $*" >&2
  exit 1
}

# 执行命令；测试时设置 DOCKERCTL_DRY_RUN=1 只打印命令，不真正操作 Docker。
run_or_print() {
  if [[ "${DRY_RUN}" == "1" ]]; then
    printf '%s' "$1"
    shift
    for arg in "$@"; do
      printf ' %s' "$arg"
    done
    printf '\n'
  else
    "$@"
  fi
}

# commit/tag 必须显式传版本号，例如 v1.0。
require_version() {
  local version="${1:-}"
  [[ -n "${version}" ]] || die "version is required, for example: $0 commit v1.0"
  [[ "${version}" != *":"* ]] || die "pass only the tag version, for example v1.0, not ${IMAGE_NAME}:v1.0"
  printf '%s' "${version}"
}

# 后台启动 base 镜像，容器 PID 1 为 systemd。
# 默认使用 host 网络并挂载 Xauthority，方便 MuJoCo/GLFW 直接显示到宿主机桌面。
# sshd、ota_agent.service、zl_robot.service 均交给容器内 systemd 管理。
run_container() {
  local image="${IMAGE_NAME}:${DEFAULT_TAG}"
  local host_xauthority="${DOCKERCTL_XAUTHORITY:-${XAUTHORITY:-${HOME}/.Xauthority}}"
  local docker_args=(
    run -d
    --name "${CONTAINER_NAME}"
    --network "${NETWORK_MODE}"
    --privileged
    --cgroupns=host
    --tmpfs /run
    --tmpfs /run/lock
    -v /sys/fs/cgroup:/sys/fs/cgroup:rw
    -v /dev/input:/dev/input:rw
    -v /dev/bus/usb:/dev/bus/usb:rw
    -v /run/udev:/run/udev:ro
  )

  if [[ "${NETWORK_MODE}" == "host" ]]; then
    docker_args+=(
      -e "DISPLAY=${DISPLAY:-}"
      -e XAUTHORITY=/root/.Xauthority
      -e QT_X11_NO_MITSHM=1
      -e "LIBGL_ALWAYS_SOFTWARE=${SOFTWARE_GL}"
      -v "${host_xauthority}:/root/.Xauthority:ro"
    )
  fi

  docker_args+=(
    "${image}"
    /sbin/init
  )

  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" "${docker_args[@]}"
}

# 进入正在运行的容器。
shell_container() {
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" exec -it "${CONTAINER_NAME}" bash
}

# 启动已经存在但处于停止状态的容器。
start_container() {
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" start "${CONTAINER_NAME}"
}

# 停止容器。
stop_container() {
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" stop "${CONTAINER_NAME}"
}

# 删除已停止的容器。
remove_container() {
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" rm "${CONTAINER_NAME}"
}

# 查看当前容器状态。
list_containers() {
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" ps -a --filter "name=${CONTAINER_NAME}"
}

# 查看容器在 bridge 网络中的 IP。host 网络模式下容器没有独立 IP。
show_container_ip() {
  if [[ "${NETWORK_MODE}" == "host" ]]; then
    echo "[INFO] ${CONTAINER_NAME} uses host network; it has no separate bridge IP." >&2
  fi
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${CONTAINER_NAME}"
}

# 查看当前全部镜像。
list_images() {
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" image list
}

# 跟随容器日志。
logs_container() {
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" logs -f "${CONTAINER_NAME}"
}

# 查看容器内 systemd journal，排查 ssh/ota/zl_robot 服务。
journal_container() {
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" exec -it "${CONTAINER_NAME}" journalctl -xe
}

# 把当前容器提交为指定版本镜像，例如 zl_robot:v1.0。
commit_container() {
  local version
  version="$(require_version "${1:-}")"
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" commit "${CONTAINER_NAME}" "${IMAGE_NAME}:${version}"
}

# 把指定版本标记为 base。
tag_image() {
  local version
  version="$(require_version "${1:-}")"
  run_or_print \
    ${DOCKER_SUDO:+${DOCKER_SUDO}} "${DOCKER_BIN}" tag "${IMAGE_NAME}:${version}" "${IMAGE_NAME}:${DEFAULT_TAG}"
}

# 命令分发入口。
main() {
  local command="${1:-help}"
  shift || true

  case "${command}" in
    run) run_container "$@" ;;
    start) start_container "$@" ;;
    shell) shell_container "$@" ;;
    stop) stop_container "$@" ;;
    rm|remove) remove_container "$@" ;;
    ps|list) list_containers "$@" ;;
    ip) show_container_ip "$@" ;;
    images) list_images "$@" ;;
    logs) logs_container "$@" ;;
    journal) journal_container "$@" ;;
    commit) commit_container "$@" ;;
    tag) tag_image "$@" ;;
    help|-h|--help) usage ;;
    *) usage >&2; die "unknown command: ${command}" ;;
  esac
}

main "$@"

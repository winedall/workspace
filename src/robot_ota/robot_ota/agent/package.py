import os
import json
import shutil
import hashlib
import tarfile
from typing import Optional, Dict


def verify_archive(archive_path: str, sig_path: str, public_key_path: str) -> bool:
    """使用 Ed25519 公钥验证发布包的签名。

    三步验证：
      1. 三者文件均存在
      2. 使用 PyNaCl crypto_sign_open 验证签名
      3. 签名有效返回 True
    """
    if not all(os.path.exists(p) for p in [archive_path, sig_path, public_key_path]):
        return False
    try:
        from nacl.bindings import crypto_sign_open
        with open(archive_path, "rb") as f:
            archive_data = f.read()
        with open(sig_path, "rb") as f:
            sig = f.read()
        with open(public_key_path, "rb") as f:
            pk = f.read()
        crypto_sign_open(sig + archive_data, pk)
        return True
    except Exception:
        return False


def verify_files(pkg_dir: str, manifest: dict) -> bool:
    """根据 manifest 中记录的 SHA256 逐文件校验完整性。"""
    files = manifest.get("files", {})
    for rel_path, expected_hash in files.items():
        full_path = os.path.join(pkg_dir, rel_path)
        if not os.path.exists(full_path):
            return False
        actual_hash = hashlib.sha256()
        with open(full_path, "rb") as f:
            while True:
                chunk = f.read(65536)
                if not chunk:
                    break
                actual_hash.update(chunk)
        if actual_hash.hexdigest() != expected_hash:
            return False
    return True


def safe_extract(archive_path: str, output_dir: str) -> bool:
    """安全解压 tar 归档，防止路径穿越攻击。

    安全检查项：
      - 文件路径不为绝对路径
      - 不包含 .. 跳转
      - 符号链接目标不越界
      - 禁止字符设备和块设备
    """
    try:
        with tarfile.open(archive_path, "r:*") as tar:
            for member in tar.getmembers():
                name = member.name
                if os.path.isabs(name) or ".." in name:
                    return False
                if member.issym() or member.islnk():
                    link_target = member.linkname
                    if os.path.isabs(link_target) or ".." in link_target:
                        return False
                if member.ischr() or member.isblk() or member.isfifo():
                    return False
            tar.extractall(path=output_dir, filter="data")
        return True
    except Exception:
        return False


def load_manifest(pkg_dir: str) -> Optional[dict]:
    """从解包目录加载 manifest.json。

    支持两种解包结构：
      - 扁平：manifest.json 直接在 pkg_dir 根目录
      - 嵌套：manifest.json 在 pkg_dir 下的唯一子目录中
    """
    manifest_path = os.path.join(pkg_dir, "manifest.json")
    if os.path.exists(manifest_path):
        with open(manifest_path, "r") as f:
            return json.load(f)
    for entry in os.listdir(pkg_dir):
        sub = os.path.join(pkg_dir, entry)
        if os.path.isdir(sub):
            candidate = os.path.join(sub, "manifest.json")
            if os.path.exists(candidate):
                with open(candidate, "r") as f:
                    return json.load(f)
    return None


def create_release_dir(releases_root: str, version: str) -> str:
    """在 releases 目录下为指定版本创建子目录。"""
    target = os.path.join(releases_root, version)
    os.makedirs(target, exist_ok=True)
    return target


def set_tree_owner(root: str, uid: int, gid: int):
    """Set a release tree owner without following symlinks."""
    os.chown(root, uid, gid, follow_symlinks=False)
    for current_root, directories, files in os.walk(root, followlinks=False):
        for name in directories:
            os.chown(os.path.join(current_root, name), uid, gid, follow_symlinks=False)
        for name in files:
            os.chown(os.path.join(current_root, name), uid, gid, follow_symlinks=False)


def switch_symlink(link_path: str, target_path: str):
    """原子切换软链接（先创建 .tmp 再 rename，掉电安全）。

    rename 在 POSIX 系统上是原子操作，确保不会出现中间状态。
    """
    tmp_link = link_path + ".tmp"
    os.symlink(target_path, tmp_link)
    os.rename(tmp_link, link_path)


def cleanup_old_versions(releases_root: str, current_version: str,
                         previous_version: Optional[str] = None,
                         max_retain: int = 3):
    """清理旧版本目录，保留当前版本和前一个版本，

    超出 max_retain 阈值的旧版本按时间排序后删除。
    """
    versions = []
    for entry in os.listdir(releases_root):
        entry_path = os.path.join(releases_root, entry)
        if os.path.isdir(entry_path):
            versions.append(entry)
    versions.sort(reverse=True)
    protected = {current_version}
    if previous_version:
        protected.add(previous_version)
    for v in versions[max_retain:]:
        if v not in protected:
            shutil.rmtree(os.path.join(releases_root, v), ignore_errors=True)

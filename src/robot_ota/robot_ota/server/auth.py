import hashlib
import secrets
from fastapi import Depends, HTTPException, Request, status
from fastapi.responses import RedirectResponse
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

security = HTTPBearer()

# ── 默认管理员凭据 ────────────────────────────────────
# 生产环境应修改密码，或接入外部认证系统
_USERNAME = "admin"
_PASSWORD_HASH = hashlib.sha256(b"admin").hexdigest()
_TOKENS: set = set()


def verify_password(password: str) -> bool:
    """验证管理员密码（SHA256 比对）。"""
    return hashlib.sha256(password.encode()).hexdigest() == _PASSWORD_HASH


def create_token() -> str:
    """登录成功后签发会话 Token（32 字节随机 hex 串）。"""
    token = secrets.token_hex(32)
    _TOKENS.add(token)
    return token


def verify_token(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """FastAPI 依赖注入：验证请求头中的 Bearer Token。"""
    token = credentials.credentials
    if token not in _TOKENS:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid token")
    return token


def verify_web_token(request: Request):
    """Web 页面依赖注入：优先读 Authorization 头，其次读 cookie。
    未认证时重定向到 /login。
    """
    token = request.headers.get("authorization", "").removeprefix("Bearer ").strip()
    if not token:
        token = request.cookies.get("token", "")
    if not token or token not in _TOKENS:
        raise HTTPException(status_code=status.HTTP_303_SEE_OTHER, headers={"Location": "/login"}, detail="Not authenticated")
    return token

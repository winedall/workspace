import os
import subprocess
from fastapi import APIRouter, Request, Form, Depends
from fastapi.responses import RedirectResponse, HTMLResponse
from fastapi.templating import Jinja2Templates
from ..auth import verify_password, create_token, verify_token, verify_web_token
from ..audit_log import write_audit_log
from ..config import Settings

router = APIRouter(tags=["web"])

TEMPLATES_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "templates")
_templates = None
if os.path.exists(TEMPLATES_DIR):
    _templates = Jinja2Templates(directory=TEMPLATES_DIR)


def _get_server_ips() -> list[str]:
    """获取本机非虚拟网卡的 IPv4 地址列表（过滤 docker/br-/veth/tun/tap/vnet）。"""
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show"],
            capture_output=True, text=True, timeout=5,
        )
    except Exception:
        return []
    # virtual_prefixes = ("docker", "br-", "veth", "tun", "tap", "vnet")
    virtual_prefixes = ("br-", "veth", "tun", "tap", "vnet")
    ips = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4:
            iface = parts[1]
            if not any(iface.startswith(p) for p in virtual_prefixes):
                addr = parts[3].split("/")[0]
                if not addr.startswith("127."):
                    ips.append(addr)
    return ips


def _render(request: Request, template_name: str, **kwargs):
    """渲染 Jinja2 模板，模板目录不存在时返回纯文本占位。"""
    if _templates:
        return _templates.TemplateResponse(request, template_name, {"request": request, **kwargs})
        # return _templates.TemplateResponse(template_name, {"request": request, **kwargs})
    return HTMLResponse(f"<html><body><h1>{template_name.replace('.html', '').title()}</h1></body></html>")


@router.get("/")
def index():
    """根路径重定向到 Dashboard。"""
    return RedirectResponse(url="/dashboard")


@router.get("/login")
def login_page(request: Request):
    """登录页面。"""
    return _render(request, "login.html")


@router.post("/login")
def login_action(username: str = Form(...), password: str = Form(...), request: Request = None):
    """处理登录表单，验证通过后签发 Token 写入 Cookie。"""
    ip = request.client.host if request and request.client else ""
    if username == "admin" and verify_password(password):
        token = create_token()
        write_audit_log("login", target="admin", detail="successful", ip_address=ip)
        resp = RedirectResponse(url="/dashboard", status_code=302)
        resp.set_cookie(key="token", value=token)
        return resp
    write_audit_log("login_failed", target=username, detail="invalid password", ip_address=ip)
    return HTMLResponse("Invalid credentials", status_code=401)


@router.get("/dashboard")
def dashboard(request: Request, _=Depends(verify_web_token)):
    """Dashboard 概览页面。"""
    return _render(request, "dashboard.html")


@router.get("/devices")
def devices_page(request: Request, _=Depends(verify_web_token)):
    """设备管理页面。"""
    return _render(request, "devices.html", default_agent_root=Settings().agent_root, server_ips=_get_server_ips())


@router.get("/releases")
def releases_page(request: Request, _=Depends(verify_web_token)):
    """版本管理页面。"""
    return _render(request, "releases.html")


@router.get("/deployments")
def deployments_page(request: Request, _=Depends(verify_web_token)):
    """部署管理页面。"""
    return _render(request, "deployments.html")

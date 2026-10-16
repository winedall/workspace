import os
import logging
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from .database import Database, get_db_path
from .routers import devices, releases, deployments, audit, web, agent

# ── 日志初始化 ──────────────────────────────────────────
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ota_server")

app = FastAPI(title="ZL Robot OTA Server", version="1.0.0")


@app.on_event("startup")
def startup():
    """应用启动时初始化数据库，挂载静态文件目录。"""
    db = Database(get_db_path())
    db.init_db()
    logger.info("Database initialized")
    static_dir = os.path.join(os.path.dirname(__file__), "static")
    if os.path.exists(static_dir):
        app.mount("/static", StaticFiles(directory=static_dir), name="static")


# ── 注册路由 ──────────────────────────────────────────
# 按顺序注册：Web 页面（需优先处理 /login）、设备、版本、部署、审计
app.include_router(web.router)
app.include_router(devices.router)
app.include_router(devices._agent_deploy_router)
app.include_router(releases.router)
app.include_router(deployments.router)
app.include_router(agent.router)
app.include_router(audit.router)


@app.get("/health")
def health():
    """服务存活检查，负载均衡器可用此接口探测。"""
    return {"status": "ok"}

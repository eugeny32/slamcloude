from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.routes import router
from app.config import get_settings


def create_app() -> FastAPI:
    app = FastAPI(
        title="slamcloude",
        description="Cloud processing platform for SHARE S20 (LiDAR + RTK + SLAM) scan data",
        version="0.1.0",
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=get_settings().cors_origins,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    app.include_router(router)
    return app


app = create_app()

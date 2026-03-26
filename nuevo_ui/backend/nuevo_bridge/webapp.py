"""
FastAPI app factory for the shared bridge runtime.
"""
from __future__ import annotations

import os
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .auth import decode_token
from .auth_router import router as auth_router
from .runtime import BridgeRuntime


def create_app(runtime: Optional[BridgeRuntime] = None) -> FastAPI:
    runtime = runtime or BridgeRuntime()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        await runtime.start()
        yield
        await runtime.stop()

    app = FastAPI(title="NUEVO Bridge", lifespan=lifespan)
    app.state.bridge_runtime = runtime
    app.include_router(auth_router)

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket, token: str = Query(None)):
        try:
            if not token:
                raise ValueError("No token provided")
            decode_token(token)
        except Exception:
            await websocket.close(code=4001)
            return

        await runtime.ws_manager.connect(websocket)
        for message in runtime.message_router.get_cached_ws_messages():
            await runtime.ws_manager.send_to(websocket, message)

        try:
            while True:
                data = await websocket.receive_json()
                runtime.handle_ws_command(data.get("cmd"), data.get("data", {}))
        except WebSocketDisconnect:
            runtime.ws_manager.disconnect(websocket)
        except Exception as e:
            print(f"[WS] Error in websocket handler: {e}")
            runtime.ws_manager.disconnect(websocket)

    @app.get("/health")
    async def health_check():
        return runtime.health_dict()

    static_dir = os.path.join(os.path.dirname(__file__), "..", "static")
    if os.path.exists(static_dir) and os.listdir(static_dir):
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")

        @app.get("/")
        async def serve_index():
            return FileResponse(os.path.join(static_dir, "index.html"))

    return app

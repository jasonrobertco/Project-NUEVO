from __future__ import annotations

import os
import sys
from pathlib import Path

import uvicorn


def _ensure_shared_bridge_import() -> None:
    try:
        import nuevo_bridge  # noqa: F401
        return
    except ImportError:
        pass

    candidates = []
    env_path = os.getenv("NUEVO_BRIDGE_SOURCE")
    if env_path:
        candidates.append(Path(env_path))

    repo_guess = Path(__file__).resolve().parents[4] / "nuevo_ui" / "backend"
    candidates.append(repo_guess)

    for candidate in candidates:
        if candidate.exists():
            sys.path.insert(0, str(candidate))
            try:
                import nuevo_bridge  # noqa: F401
                return
            except ImportError:
                continue

    raise ImportError("Could not import shared nuevo_bridge runtime. Set NUEVO_BRIDGE_SOURCE or PYTHONPATH.")


def main() -> None:
    _ensure_shared_bridge_import()

    from nuevo_bridge.runtime import BridgeRuntime
    from nuevo_bridge.webapp import create_app

    from .ros_controller import RosBridgeController

    runtime = BridgeRuntime(ros_controller_factory=lambda rt: RosBridgeController(rt))
    app = create_app(runtime)

    uvicorn.run(
        app,
        host=os.getenv("NUEVO_HOST", "0.0.0.0"),
        port=int(os.getenv("NUEVO_PORT", "8000")),
        reload=False,
        log_level="info",
    )

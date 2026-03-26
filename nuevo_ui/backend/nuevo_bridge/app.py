"""Compatibility module exposing the default plain-Python FastAPI app."""

from .runtime import BridgeRuntime
from .webapp import create_app


runtime = BridgeRuntime()
app = create_app(runtime)

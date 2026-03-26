"""
Entry point for running nuevo_bridge as a module.

Usage (plain Python / dev):
    python -m nuevo_bridge

The ROS-integrated entrypoint lives in ros2_ws/src/bridge.
"""
import uvicorn


def main():
    uvicorn.run(
        "nuevo_bridge.app:app",
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="info",
    )


if __name__ == "__main__":
    main()

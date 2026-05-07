from __future__ import annotations

import time

import cv2


class ManagedCamera:
    def __init__(
        self,
        device: str,
        width: int,
        height: int,
        fps: float,
        reconnect_delay_sec: float,
        log_interval_sec: float,
        logger,
    ) -> None:
        self.device = device
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.reconnect_delay_sec = max(0.1, float(reconnect_delay_sec))
        self.log_interval_sec = max(1.0, float(log_interval_sec))
        self.logger = logger
        self.capture: cv2.VideoCapture | None = None
        self.connected = False

    def ensure(self) -> bool:
        if self.capture is not None and self.capture.isOpened():
            return True

        self.release()
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            self.logger.warn(
                "Waiting for camera device %s"
                % self.device,
                throttle_duration_sec=self.log_interval_sec,
            )
            capture.release()
            time.sleep(self.reconnect_delay_sec)
            return False

        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        if self.height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        if self.fps > 0.0:
            capture.set(cv2.CAP_PROP_FPS, self.fps)

        self.capture = capture
        self.connected = True
        self.logger.info(
            "Connected vision camera %s at requested %dx%d @ %.1f fps"
            % (self.device, self.width, self.height, self.fps)
        )
        return True

    def read(self):
        if self.capture is None:
            return False, None
        return self.capture.read()

    def release(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        if self.connected:
            self.connected = False
            self.logger.warn("Vision camera disconnected; waiting for %s" % self.device)

    def handle_read_failure(self) -> None:
        self.logger.warn(
            "Failed to read a frame from %s; reconnecting"
            % self.device,
            throttle_duration_sec=self.log_interval_sec,
        )
        self.release()
        time.sleep(self.reconnect_delay_sec)

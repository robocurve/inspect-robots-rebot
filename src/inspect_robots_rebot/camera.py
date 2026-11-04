"""Read the latest overhead frame without blocking each control step on HTTP."""

import threading
import time

import numpy as np
import numpy.typing as npt


class LiveCamera:
    """Cache the latest overhead frame with a bounded read timeout."""

    def __init__(self, url: str) -> None:
        self.url = url
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.frame: npt.NDArray[np.uint8] | None = None
        self.at: float | None = None
        self.error: str | None = None
        self.thread = threading.Thread(target=self._read, name="rig4-overhead", daemon=True)
        self.thread.start()

    def _read(self) -> None:
        import cv2

        capture = None
        try:
            capture = cv2.VideoCapture(
                self.url,
                cv2.CAP_FFMPEG,
                [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000],
            )
            if not capture.isOpened():
                raise RuntimeError("Cannot open the overhead stream")
            while not self.stop.is_set():
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError("The overhead stream stopped")
                if frame.shape != (480, 640, 3):
                    raise RuntimeError("Overhead frame is not 640 by 480 RGB")
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                with self.condition:
                    self.frame, self.at = frame, time.monotonic()
                    self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.error = str(error)
                self.condition.notify_all()
        finally:
            if capture is not None:
                capture.release()

    def get(self) -> npt.NDArray[np.uint8]:
        """Return a fresh RGB frame or raise if the stream failed."""
        deadline = time.monotonic() + 6
        with self.condition:
            while self.frame is None and self.error is None and not self.stop.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("No overhead frame arrived")
                self.condition.wait(remaining)
            if self.error or self.stop.is_set():
                raise RuntimeError(self.error or "The overhead reader is closed")
            if self.at is None or time.monotonic() - self.at > 1.0:
                raise RuntimeError("The overhead frame is stale")
            if self.frame is None:
                raise RuntimeError("No overhead frame is available")
            result: npt.NDArray[np.uint8] = np.array(self.frame, dtype=np.uint8, copy=True)
            return result

    def close(self) -> None:
        """Stop and join the reader before returning."""
        self.stop.set()
        self.thread.join(timeout=6)
        if self.thread.is_alive():
            raise RuntimeError("Camera reader did not stop")

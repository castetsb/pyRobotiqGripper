"""Shared background Qt event loop for this package's optional PySide6 tools.

Qt only supports a single ``QApplication`` per process, driven by a single
thread. Every optional PySide6-based visualizer in this package (e.g.
:class:`~pyrobotiqgripper.visualizer.GripperVisualizer`,
:class:`~pyrobotiqgripper.joystick_visual_tool.JoystickVisualTool`) needs a Qt
event loop running somewhere other than the thread driving the gripper, but
each spinning up its own ``QApplication``/thread would conflict the moment
more than one is used in the same process. This module lazily starts exactly
one background thread/``QApplication``/event loop, shared by all of them.

Not specific to any particular visualizer.
"""

from __future__ import annotations

import queue
import threading
from typing import Callable, List, Optional

try:
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication
except ImportError as exc:
    raise ImportError(
        "qt_app_host requires the optional PySide6 package. "
        "Install it with `pip install PySide6`."
    ) from exc


class QtAppHost:
    """Owns the single background thread/``QApplication`` shared by visualizers.

    Not meant to be instantiated directly; use :func:`get_qt_app_host`.
    """

    def __init__(self) -> None:
        self._thread: Optional[threading.Thread] = None
        self._app: Optional[QApplication] = None
        self._pending: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._lock = threading.Lock()
        self._ready_event = threading.Event()
        self._stop_event = threading.Event()

    def run_on_gui_thread(self, factory: Callable[[], None]) -> None:
        """Run ``factory`` on the shared Qt thread; blocks until it has run.

        Starts the shared thread/``QApplication`` first if not already
        running. Re-raises any exception ``factory`` raised.

        Args:
            factory: A zero-argument callable, typically building a window
                and its own per-window ``QTimer``.
        """
        self._ensure_started()

        done = threading.Event()
        error: List[BaseException] = []

        def _invoke() -> None:
            try:
                factory()
            except BaseException as exc:  # noqa: BLE001 - re-raised on caller's thread
                error.append(exc)
            finally:
                done.set()

        self._pending.put(_invoke)
        done.wait(timeout=5.0)
        if error:
            raise error[0]

    def shutdown(self) -> None:
        """Stop the shared event loop and join its thread.

        Call this once, after every individual visualizer using this host
        has already been stopped. Does nothing if the host was never started.
        """
        with self._lock:
            thread = self._thread
            if thread is None:
                return
            self._stop_event.set()
        thread.join(timeout=5.0)
        with self._lock:
            self._thread = None
            self._app = None

    def _ensure_started(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            self._stop_event.clear()
            self._ready_event.clear()
            self._thread = threading.Thread(target=self._run, name="QtAppHost", daemon=True)
            self._thread.start()
        self._ready_event.wait(timeout=5.0)

    def _run(self) -> None:
        app = QApplication.instance() or QApplication([])
        self._app = app

        pump = QTimer()
        pump.timeout.connect(self._pump)
        pump.start(20)

        self._ready_event.set()
        app.exec_()

        pump.stop()

    def _pump(self) -> None:
        if self._stop_event.is_set():
            self._app.quit()
            return
        while True:
            try:
                job = self._pending.get_nowait()
            except queue.Empty:
                break
            job()


_host: Optional[QtAppHost] = None
_host_lock = threading.Lock()


def get_qt_app_host() -> QtAppHost:
    """Return the process-wide :class:`QtAppHost`, creating it on first use."""
    global _host
    with _host_lock:
        if _host is None:
            _host = QtAppHost()
        return _host

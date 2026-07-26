"""Generic two-axis mouse-as-joystick input device, plus an optional live
signal visualizer.

Exposes mouse position as two axes (X and Y) normalized to [-1, 1] or
[0, 1], mirroring the interface of pygame.joystick.Joystick.get_axis(), so it
can be used as a drop-in replacement wherever only get_axis(axis_index)
is needed.

:class:`MouseJoystickVisualizer` is a separate, optional, generic overlay bar
that shows a live marker position plus caller-defined colored "control
zones". It knows nothing about mice, joysticks, or grippers -- it just draws
whatever position/zones it's given -- but lives in this module because it's
meant to be paired with :class:`MouseJoystick` (e.g. to visualize the mouse's
live X position and, for a gripper caller, its control-signal zones).
"""

from __future__ import annotations

import threading
from typing import Dict, List, Optional, Tuple

from pynput import mouse

AXIS_X = 0
AXIS_Y = 1

try:
    from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
    from PySide6.QtGui import QColor, QCursor, QGuiApplication, QPainter
    from PySide6.QtWidgets import QWidget
    _PYSIDE6_AVAILABLE = True
except ImportError:
    _PYSIDE6_AVAILABLE = False


class MouseJoystick:
    """Reports mouse position as two axes, each with a dead zone
    centered on the screen, sized as a fraction of the corresponding
    screen dimension.

    Axis 0 (AXIS_X): horizontal position, -1 (left) to 1 (right).
    Axis 1 (AXIS_Y): vertical position, 1 (top) to -1 (bottom).
    """

    def __init__(self, deadzone: float = 0, output_range: str = "signed"):
        """
        Args:
            deadzone (float, optional): Fraction (0-1) of the screen
                dimension, centered on the middle of the screen, that
                reports 0 for that axis. Defaults to 0.10 (10%).
            output_range (str, optional): Range reported by :meth:`get_axis`:
                ``"signed"`` for [-1, 1] (default, matching
                ``pygame.joystick.Joystick.get_axis()``), or ``"unsigned"``
                for [0, 1].
        """
        if output_range not in ("signed", "unsigned"):
            raise ValueError(
                f"output_range must be 'signed' or 'unsigned', got {output_range!r}"
            )

        self.deadzone = deadzone
        self.output_range = output_range
        self.x: float = 0.0
        self.y: float = 0.0

        # Start listener in background
        self.listener = mouse.Listener(on_move=self._on_move)
        self.listener.start()

    def stop(self) -> None:
        """Stop the background listener and release the OS-level mouse hook.

        Call this before the process exits: a live low-level mouse hook
        (installed by the listener) has its callback run on this listener
        thread, competing for the GIL with anything else happening during
        shutdown; leaving it running until the process fully terminates can
        make the whole system's mouse input feel laggy in the meantime.
        """
        self.listener.stop()

    def _on_move(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def _normalize(self, value: float, screen_dimension: float) -> float:
        center = screen_dimension / 2
        raw = (value - center) / center
        normalized = max(-1.0, min(1.0, raw))

        if abs(normalized) <= self.deadzone:
            signed = 0.0
        else:
            sign = 1.0 if normalized > 0 else -1.0
            signed = sign * (abs(normalized) - self.deadzone) / (1.0 - self.deadzone)

        if self.output_range == "unsigned":
            return (signed + 1) / 2
        return signed

    def get_axis(self, axis):
        """Return the requested axis value, in [-1, 1] or [0, 1] depending
        on :attr:`output_range`.

        Args:
            axis (int): AXIS_X (0) for horizontal position, AXIS_Y (1)
                for vertical position.
        """
        # Lazy import to avoid depending on pyautogui if unused
        import pyautogui

        screen_width, screen_height = pyautogui.size()

        if axis == AXIS_X:
            return self._normalize(self.x, screen_width)
        if axis == AXIS_Y:
            return self._normalize(self.y, screen_height)
        raise ValueError(f"MouseJoystick only supports axis 0 (X) or 1 (Y), got {axis}")


class MouseJoystickVisualizer:
    """Thin, full-screen-width overlay bar showing a live marker position and
    optional colored control zones.

    Generic: this class knows nothing about mice, joysticks, or grippers. A
    caller feeds it a marker position (e.g. a live mouse X fraction, or any
    other value in [0, 1]) via :meth:`set_marker_position`, and, optionally,
    a set of colored zones it wants drawn behind the marker (e.g. deadzones,
    force-ramp ranges) via :meth:`add_control_zone` /
    :meth:`clear_control_zones`. Runs entirely on the shared background Qt
    thread (see :mod:`pyrobotiqgripper.qt_app_host`), started and stopped
    explicitly with :meth:`start` and :meth:`stop`, so calling its setters
    from a control loop costs at most a lock-protected assignment or list
    update -- nothing on that hot path ever touches Qt directly.

    The bar is always click-through (clicks/hover never reach it; they pass
    straight to whatever's underneath), so it never blocks normal use of the
    screen. To stay legible despite that, it also polls the OS-level cursor
    position on its own redraw timer and turns 60% transparent whenever the
    cursor happens to be over it, so whatever's underneath remains visible.

    Args:
        refresh_interval_ms (int, optional): Redraw period, in milliseconds.
            Defaults to ``33`` (~30 FPS).
        bar_height (int, optional): Height of the overlay bar, in pixels.
            Defaults to ``48``.
        anchor (str, optional): Which edge of the primary screen to pin the
            bar to: ``"top"`` or ``"bottom"``. Positioned against the
            screen's *available* area (i.e. excluding the taskbar), so
            ``"bottom"`` sits just above the taskbar rather than under it.
            Defaults to ``"bottom"``, out of the way of the menu bars most
            applications keep at the top of their window.

    Warning:
        Requires the optional ``PySide6`` package (``pip install PySide6``).

    Warning:
        See the same PySide6/background-thread shutdown warning documented on
        :class:`~pyrobotiqgripper.visualizer.GripperVisualizer`: end your
        script with ``os._exit(0)`` after calling :meth:`stop`, rather than
        returning normally.

    Examples:
        >>> from pyrobotiqgripper.mouse_joystick import MouseJoystickVisualizer
        >>> visualizer = MouseJoystickVisualizer()
        >>> visualizer.start()
        >>> visualizer.set_marker_position(0.5)
        >>> zone_id = visualizer.add_control_zone(0.0, 0.05, "cyan")
        >>> visualizer.clear_control_zones()
        >>> visualizer.stop()
    """

    def __init__(self,
                 refresh_interval_ms: int = 33,
                 bar_height: int = 48,
                 anchor: str = "bottom"):
        if not _PYSIDE6_AVAILABLE:
            raise ImportError(
                "MouseJoystickVisualizer requires the optional PySide6 package. "
                "Install it with `pip install PySide6`."
            )
        if anchor not in ("top", "bottom"):
            raise ValueError(f"anchor must be 'top' or 'bottom', got {anchor!r}")

        self._refresh_interval_ms = refresh_interval_ms
        self._bar_height = bar_height
        self._anchor = anchor

        self._lock = threading.Lock()
        self._marker_fraction = 0.5
        self._next_zone_id = 0
        self._zones_by_id: Dict[int, Tuple[float, float, object]] = {}

        self._window: Optional["_MouseJoystickVisualizerWindow"] = None
        self._poll_timer: Optional["QTimer"] = None

    # --- Public, thread-safe API --------------------------------------

    def set_marker_position(self, fraction: float) -> None:
        """Set the marker's horizontal position along the bar.

        Args:
            fraction (float): Position along the bar, clamped to [0, 1].
        """
        with self._lock:
            self._marker_fraction = max(0.0, min(1.0, float(fraction)))

    def add_control_zone(self, start: float, end: float, color: object) -> int:
        """Add a colored zone drawn behind the marker.

        Args:
            start (float): Start of the zone, in [0, 1].
            end (float): End of the zone, in [0, 1].
            color: Any Qt color spec accepted by ``QColor`` (e.g. the name
                ``"cyan"``, an ``(r, g, b)``/``(r, g, b, a)`` tuple, or a
                ``QColor`` instance).

        Returns:
            int: An opaque id, usable with :meth:`remove_control_zone`.
        """
        with self._lock:
            zone_id = self._next_zone_id
            self._next_zone_id += 1
            self._zones_by_id[zone_id] = (float(start), float(end), _to_qcolor(color))
        return zone_id

    def remove_control_zone(self, zone_id: int) -> None:
        """Remove a single zone previously added with :meth:`add_control_zone`.

        Does nothing if ``zone_id`` isn't currently defined.
        """
        with self._lock:
            self._zones_by_id.pop(zone_id, None)

    def clear_control_zones(self) -> None:
        """Remove every zone currently defined.

        Meant to be called whenever a caller's own state changes (e.g. a
        gripper's control mode), right before defining a fresh set of zones
        for the new state.
        """
        with self._lock:
            self._zones_by_id.clear()

    def _snapshot(self) -> Tuple[float, List[Tuple[float, float, object]]]:
        with self._lock:
            return self._marker_fraction, list(self._zones_by_id.values())

    # --- Lifecycle -------------------------------------------------------

    def start(self) -> None:
        """Start the overlay bar on the shared Qt background thread.

        Does nothing if the bar is already running.
        """
        if self._window is not None:
            return

        from .qt_app_host import get_qt_app_host

        def _build() -> None:
            screen = QGuiApplication.primaryScreen()
            full_geometry = screen.geometry()
            available_geometry = screen.availableGeometry()

            window = _MouseJoystickVisualizerWindow(self._bar_height)
            if self._anchor == "top":
                y = available_geometry.y()
            else:
                y = available_geometry.y() + available_geometry.height() - self._bar_height
            window.setGeometry(full_geometry.x(), y, full_geometry.width(), self._bar_height)
            window.show()

            poll_timer = QTimer()
            poll_timer.timeout.connect(self._poll)
            poll_timer.start(self._refresh_interval_ms)

            self._window = window
            self._poll_timer = poll_timer

        get_qt_app_host().run_on_gui_thread(_build)

    def stop(self) -> None:
        """Close the overlay bar and stop its poll timer.

        Does nothing if the bar is not running. Only this visualizer's own
        window/timer are torn down; the shared Qt event loop keeps running
        for any other visualizer using it (see
        :mod:`pyrobotiqgripper.qt_app_host`).
        """
        if self._window is None:
            return

        from .qt_app_host import get_qt_app_host

        def _teardown() -> None:
            if self._poll_timer is not None:
                self._poll_timer.stop()
            if self._window is not None:
                self._window.close()
            self._window = None
            self._poll_timer = None

        get_qt_app_host().run_on_gui_thread(_teardown)

    def is_running(self) -> bool:
        """Return whether the overlay bar is currently running.

        Returns:
            bool: True if the bar is currently open.
        """
        return self._window is not None

    def _poll(self) -> None:
        """Timer callback run on the shared Qt thread: repaint the bar, or
        tear down if the window was closed from the UI.

        Also polls the OS-level cursor position against the bar's on-screen
        rectangle to detect hover: the window itself is click-through and so
        never receives real mouse-move/enter/leave events.
        """
        window = self._window
        if window is None:
            return
        if not window.isVisible():
            if self._poll_timer is not None:
                self._poll_timer.stop()
            window.close()
            self._window = None
            self._poll_timer = None
            return
        marker_fraction, zones = self._snapshot()
        hovered = window.geometry().contains(QCursor.pos())
        window.refresh(marker_fraction, zones, hovered)


if _PYSIDE6_AVAILABLE:

    def _to_qcolor(color: object) -> "QColor":
        """Normalize a caller-supplied color spec into a ``QColor``.

        Accepts a ``QColor`` instance, a name/hex string (anything
        ``QColor(x)`` understands), or an ``(r, g, b)``/``(r, g, b, a)``
        tuple/list -- unlike ``QColor``'s own constructor, which doesn't
        accept a plain tuple/list.
        """
        if isinstance(color, (tuple, list)):
            return QColor(*color)
        return QColor(color)

    class _MouseJoystickVisualizerWindow(QWidget):
        """Borderless, always-on-top, click-through overlay bar.

        Not meant to be instantiated directly; use
        :class:`MouseJoystickVisualizer`.
        """

        _BACKGROUND_COLOR = QColor(224, 224, 224)
        _MARKER_COLOR = QColor(120, 120, 120)
        _NORMAL_OPACITY = 1.0
        _HOVER_OPACITY = 0.4  # 60% transparent, so whatever's underneath stays visible

        def __init__(self, bar_height: int):
            super().__init__()
            self.setWindowFlags(
                Qt.FramelessWindowHint
                | Qt.WindowStaysOnTopHint
                | Qt.Tool
                | Qt.WindowTransparentForInput
            )
            self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            self.setAttribute(Qt.WA_ShowWithoutActivating, True)
            self.setFixedHeight(bar_height)
            self.setWindowOpacity(self._NORMAL_OPACITY)

            self._marker_fraction = 0.5
            self._zones: List[Tuple[float, float, object]] = []
            self._hovered = False

        def refresh(self, marker_fraction: float, zones: List[Tuple[float, float, object]], hovered: bool) -> None:
            """Update the bar's state and repaint.

            Args:
                marker_fraction (float): Marker position, in [0, 1].
                zones (List[Tuple[float, float, object]]): Zones to draw,
                    each as ``(start, end, color)``.
                hovered (bool): Whether the OS cursor is currently over the
                    bar. Toggles its opacity so whatever's underneath stays
                    visible while the cursor is there.
            """
            self._marker_fraction = marker_fraction
            self._zones = zones
            if hovered != self._hovered:
                self._hovered = hovered
                self.setWindowOpacity(self._HOVER_OPACITY if hovered else self._NORMAL_OPACITY)
            self.update()

        def paintEvent(self, event) -> None:  # noqa: N802 - Qt override
            width = self.width()
            height = self.height()

            painter = QPainter(self)
            try:
                painter.fillRect(0, 0, width, height, self._BACKGROUND_COLOR)

                for start, end, color in self._zones:
                    x0 = max(0.0, min(1.0, start)) * width
                    x1 = max(0.0, min(1.0, end)) * width
                    painter.fillRect(QRectF(x0, 0, x1 - x0, height), color)

                painter.setRenderHint(QPainter.Antialiasing)
                painter.setBrush(self._MARKER_COLOR)
                painter.setPen(Qt.NoPen)
                radius = height * 0.35
                marker_x = self._marker_fraction * width
                painter.drawEllipse(QPointF(marker_x, height / 2), radius, radius)
            finally:
                painter.end()

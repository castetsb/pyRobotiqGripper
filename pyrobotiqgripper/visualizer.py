"""Live visualization window for a RobotiqGripper's state.

This module renders two scrolling line charts built from a
:class:`~pyrobotiqgripper.gripper.RobotiqGripper` history buffer:

* A "bounded" chart for registers whose raw value is naturally in ``[0, 255]``
  (actual position, commanded speed/force, ...).
* A "state" chart for small enumerated / flag registers (object detection,
  activation status, faults, ...).

The window runs on a shared background Qt thread (see
:mod:`pyrobotiqgripper.qt_app_host`), so it can be dropped into an existing
control loop (e.g. the joystick CLI) without interfering with it, the same
way :class:`~pyrobotiqgripper.bipper.Bipper` runs its own audio thread. It
never talks to the gripper itself: it only polls
:meth:`~pyrobotiqgripper.gripper.RobotiqGripper.commandHistory` (write-side
registers: ``rARD``, ``rATR``, ``rGTO``, ``rACT``, ``rPR``, ``rSP``, ``rFR``)
and :meth:`~pyrobotiqgripper.gripper.RobotiqGripper.statusHistoryNumpy`
(read-side registers: ``gOBJ``, ``gSTA``, ``gGTO``, ``gACT``, ``kFLT``,
``gFLT``, ``gPR``, ``gPO``, ``gCU``), so it can be attached to a gripper that
is being driven by any other thread. Commanded registers only change when a
new command is actually sent, so they are plotted held flat at their last
known value out to the current time rather than stopping wherever the last
command happened to be logged.

Requires the optional ``PySide6`` package (``pip install PySide6``).
"""

from __future__ import annotations

import gc
from typing import Dict, List, Optional, Sequence

import numpy as np

try:
    from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
    from PySide6.QtCore import QEvent, QPointF, Qt, QTimer
    from PySide6.QtGui import QColor, QPainter
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QGroupBox,
        QHBoxLayout,
        QMainWindow,
        QScrollArea,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise ImportError(
        "GripperVisualizer requires the optional PySide6 package. "
        "Install it with `pip install PySide6`."
    ) from exc

from .constants import (
    COMMAND_HISTORY_COLUMNS_NAME_2_ID,
    STATUS_HISTORY_COLUMNS_NAME_2_ID,
    TIME,
)
from .qt_app_host import get_qt_app_host

#: History column names whose raw register value is naturally in [0, 255].
BOUNDED_SIGNALS: List[str] = ["gPO", "rPR", "rSP", "rFR", "gPR", "gCU"]

#: History column names carrying small enumerated / flag register values.
STATE_SIGNALS: List[str] = [
    "gOBJ", "gSTA", "gGTO", "gACT", "kFLT", "gFLT", "rARD", "rATR", "rGTO", "rACT", "eOBJ",
]

#: Signals checked by default on the bounded chart when none are given: actual
#: position, commanded speed and commanded force.
DEFAULT_BOUNDED_SIGNALS: List[str] = ["gPO", "rPR", "rSP", "rFR"]

#: Signals checked by default on the state chart when none are given.
DEFAULT_STATE_SIGNALS: List[str] = ["gOBJ"]

#: Fixed timeline window, in seconds, shown on both charts. Matches
#: :data:`~pyrobotiqgripper.constants.MAX_HISTORY` (~5s at a typical 100Hz
#: control loop), the most history the gripper itself ever retains.
TIMELINE_DURATION_S: float = 5.0


class GripperVisualizer:
    """Background window plotting a gripper's live history.

    The window is created and driven on a shared background Qt thread (see
    :mod:`pyrobotiqgripper.qt_app_host`), started and stopped explicitly with
    :meth:`start` and :meth:`stop`. Closing the window from the UI also tears
    it down.

    Args:
        gripper: The RobotiqGripper instance to read history from. Only its
            ``commandHistory()`` and ``statusHistoryNumpy()`` methods are
            called; the gripper is never written to or read live from Modbus
            by this class.
        bounded_signals (Sequence[str], optional): History column names
            initially checked on the 0-255 chart. Must be a subset of
            :data:`BOUNDED_SIGNALS`. Defaults to
            :data:`DEFAULT_BOUNDED_SIGNALS` (actual position, commanded
            speed, commanded force).
        state_signals (Sequence[str], optional): History column names
            initially checked on the state chart. Must be a subset of
            :data:`STATE_SIGNALS`. Defaults to :data:`DEFAULT_STATE_SIGNALS`
            (object detection).
        refresh_interval_ms (int, optional): Redraw period, in milliseconds.
            Defaults to ``100``.
        window_title (str, optional): Title of the visualization window.
            Defaults to ``"Gripper Live State"``.

    Note:
        The timeline window shown on both charts is fixed at
        :data:`TIMELINE_DURATION_S` (5 seconds), matching the gripper's own
        history buffer (:data:`~pyrobotiqgripper.constants.MAX_HISTORY`
        samples at the typical 100 Hz control loop rate) and is not
        adjustable from the window.

    Note:
        Pressing Space while the window is active freezes the display (the
        charts stop updating, and the status bar shows "Frozen..."); pressing
        it again resumes. Useful to pause on an interesting moment without
        stopping the underlying control loop.

    Warning:
        Reading the gripper's history buffers concurrently with whichever
        thread is filling them (e.g. a joystick control loop) is not lock
        protected. For a live debugging display this is an acceptable
        trade-off: at worst a single redraw reads a partially updated row.

    Warning:
        PySide6 requires ``QApplication`` to live on a process' main thread.
        This class (via :mod:`pyrobotiqgripper.qt_app_host`) runs it on a
        shared background thread instead (so the thread driving the gripper
        stays free), which works correctly while running, but reliably
        segfaults during Python's normal interpreter shutdown once that
        shared host has been started, regardless of :meth:`stop` having
        already run cleanly. This is a Qt/PySide6 limitation, not something
        :meth:`stop` can work around. If your script calls :meth:`start`, end
        it with ``os._exit(0)`` (after flushing stdout/stderr and any other
        cleanup) instead of a normal return, to skip that crash-prone
        shutdown sequence entirely -- see how
        :func:`pyrobotiqgripper.joystick_cli.main` does it.

    Examples:
        >>> import pyrobotiqgripper as rq
        >>> from pyrobotiqgripper.visualizer import GripperVisualizer
        >>> gripper = rq.RobotiqGripper()
        >>> visualizer = GripperVisualizer(gripper)
        >>> visualizer.start()
        >>> # ... drive the gripper from the main thread ...
        >>> visualizer.stop()
    """

    def __init__(self,
                 gripper,
                 bounded_signals: Optional[Sequence[str]] = None,
                 state_signals: Optional[Sequence[str]] = None,
                 refresh_interval_ms: int = 100,
                 window_title: str = "Gripper Live State"):
        self._gripper = gripper
        self._bounded_signals = (
            list(bounded_signals) if bounded_signals is not None else list(DEFAULT_BOUNDED_SIGNALS)
        )
        self._state_signals = (
            list(state_signals) if state_signals is not None else list(DEFAULT_STATE_SIGNALS)
        )
        self._refresh_interval_ms = refresh_interval_ms
        self._window_title = window_title

        self._window: Optional["_GripperVisualizerWindow"] = None
        self._poll_timer: Optional[QTimer] = None

    def start(self) -> None:
        """Start the visualization window on the shared Qt background thread.

        Does nothing if the window is already running.
        """
        if self._window is not None:
            return

        def _build() -> None:
            window = _GripperVisualizerWindow(
                self._gripper,
                bounded_signals=self._bounded_signals,
                state_signals=self._state_signals,
                window_title=self._window_title,
            )
            window.showMaximized()

            poll_timer = QTimer()
            poll_timer.timeout.connect(self._poll)
            poll_timer.start(self._refresh_interval_ms)

            self._window = window
            self._poll_timer = poll_timer

        get_qt_app_host().run_on_gui_thread(_build)

    def stop(self) -> None:
        """Close the window and stop its poll timer.

        Does nothing if the window is not running. Only this visualizer's
        own window/timer are torn down; the shared Qt event loop keeps
        running for any other visualizer using it (see
        :mod:`pyrobotiqgripper.qt_app_host`).
        """
        if self._window is None:
            return

        def _teardown() -> None:
            if self._poll_timer is not None:
                self._poll_timer.stop()
            if self._window is not None:
                self._window.close()
            # Qt objects created on this thread must also be torn down on
            # this thread. Left to Python's garbage collector, they could
            # otherwise be finalized later from an unrelated thread (e.g. at
            # interpreter shutdown), which reliably segfaults PySide6's
            # native bindings.
            self._window = None
            self._poll_timer = None
            gc.collect()

        get_qt_app_host().run_on_gui_thread(_teardown)

    def is_running(self) -> bool:
        """Return whether the visualization window is currently running.

        Returns:
            bool: True if the window is currently open.
        """
        return self._window is not None

    def _poll(self) -> None:
        """Timer callback run on the shared Qt thread: refresh the plot, or
        tear down if the window was closed from the UI."""
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
        window.refresh()


class _GripperVisualizerWindow(QMainWindow):
    """Qt window holding the two charts and their signal controls.

    Not meant to be instantiated directly; use :class:`GripperVisualizer`.

    Args:
        gripper: The RobotiqGripper instance to read history from.
        bounded_signals (Sequence[str]): History column names initially
            checked on the bounded chart.
        state_signals (Sequence[str]): History column names initially
            checked on the state chart.
        window_title (str): Title of the window.
    """

    def __init__(self,
                 gripper,
                 bounded_signals: Sequence[str],
                 state_signals: Sequence[str],
                 window_title: str):
        super().__init__()
        self._gripper = gripper
        self._duration = TIMELINE_DURATION_S
        self._frozen = False

        self.setWindowTitle(window_title)
        self.resize(1000, 700)

        # Space toggles freezing the display. Installed on the QApplication
        # (rather than overriding keyPressEvent on this window) so it fires
        # regardless of which child widget currently has keyboard focus --
        # e.g. a checkbox in the side panel, which would otherwise consume
        # Space itself to toggle its own checked state.
        QApplication.instance().installEventFilter(self)

        bounded_chart, bounded_view, self._bounded_axis_x, self._bounded_axis_y, self._bounded_series = (
            self._build_chart("Position / Speed / Force (0-255)", y_range=(0, 255), y_tick_interval=25)
        )
        state_chart, state_view, self._state_axis_x, self._state_axis_y, self._state_series = (
            self._build_chart("Status flags", y_range=(-1, 16), y_tick_interval=1)
        )
        self._bounded_chart = bounded_chart
        self._state_chart = state_chart

        for i, name in enumerate(BOUNDED_SIGNALS):
            color = self._distinct_color(i, len(BOUNDED_SIGNALS))
            self._add_series(bounded_chart, self._bounded_axis_x, self._bounded_axis_y, self._bounded_series, name, color)
        for i, name in enumerate(STATE_SIGNALS):
            color = self._distinct_color(i, len(STATE_SIGNALS))
            self._add_series(state_chart, self._state_axis_x, self._state_axis_y, self._state_series, name, color)

        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(bounded_view)
        splitter.addWidget(state_view)

        side_panel = self._build_side_panel(bounded_signals, state_signals)

        central_layout = QHBoxLayout()
        central_layout.addWidget(splitter, 1)
        central_layout.addWidget(side_panel)
        central_widget = QWidget()
        central_widget.setLayout(central_layout)
        self.setCentralWidget(central_widget)

        # Apply the initial signal selection (independently of whether the
        # checkboxes above emitted a toggled signal while being built).
        for name in BOUNDED_SIGNALS:
            self._bounded_series[name].setVisible(name in bounded_signals)
        for name in STATE_SIGNALS:
            self._state_series[name].setVisible(name in state_signals)

        # Colors are assigned per currently-*visible* series (see
        # _recolor_visible), not once for the full signal list: spreading
        # hues across every signal that could ever be shown -- most of which
        # start hidden -- left adjacent-by-coincidence pairs like the first
        # and last entries (e.g. gOBJ/eOBJ) only one hue-step apart on the
        # wheel and hard to tell apart once both happened to be checked.
        self._recolor_visible(BOUNDED_SIGNALS, self._bounded_series)
        self._recolor_visible(STATE_SIGNALS, self._state_series)

    def eventFilter(self, watched, event) -> bool:  # noqa: N802 - Qt override
        """Toggle freezing the display when Space is pressed.

        Installed on the QApplication itself (see :meth:`__init__`), so it
        sees every key press in the process before any focused child widget
        does. Only reacts while this window is the active one, so it stays
        inert if some other top-level window (e.g. a
        :class:`~pyrobotiqgripper.joystick_visual_tool.JoystickVisualTool`
        overlay, though that one never takes focus) happens to share the
        same Qt event loop.
        """
        if (event.type() == QEvent.KeyPress
                and event.key() == Qt.Key_Space
                and self.isActiveWindow()):
            self._set_frozen(not self._frozen)
            return True
        return super().eventFilter(watched, event)

    def _set_frozen(self, frozen: bool) -> None:
        """Set whether :meth:`refresh` is currently a no-op, and reflect it in the status bar."""
        self._frozen = frozen
        if frozen:
            self.statusBar().showMessage("Frozen (press Space to resume)")
        else:
            self.statusBar().clearMessage()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        """Remove the app-wide event filter installed in :meth:`__init__`.

        Without this, the filter would keep referencing this window (via a
        now-invalid native handle) after it's closed, since it's registered
        on the shared, long-lived QApplication rather than on this window.
        """
        QApplication.instance().removeEventFilter(self)
        super().closeEvent(event)

    def _build_chart(self, title: str, y_range: Sequence[float], y_tick_interval: float):
        """Create an empty chart with a time X axis and a value Y axis.

        Args:
            title (str): Title displayed above the chart.
            y_range (Sequence[float]): Initial ``(min, max)`` Y axis range.
            y_tick_interval (float): Spacing between Y axis graduations, anchored at 0.

        Returns:
            tuple: ``(chart, view, axis_x, axis_y, series_by_name)`` where
            ``chart`` is the QChart, ``view`` is the QChartView wrapping it,
            ``axis_x``/``axis_y`` are its QValueAxis instances, and
            ``series_by_name`` is an initially empty dict meant to be filled
            by :meth:`_add_series`.
        """
        chart = QChart()
        chart.setTitle(title)
        chart.legend().setVisible(True)
        chart.legend().setAlignment(Qt.AlignBottom)

        axis_x = QValueAxis()
        axis_x.setTitleText("Time (s)")
        axis_x.setRange(-self._duration, 0)
        axis_x.setTickType(QValueAxis.TicksDynamic)
        axis_x.setTickAnchor(0)
        axis_x.setTickInterval(0.5)
        axis_x.setMinorTickCount(4)  # 0.5s major ticks split into 0.1s minor ticks
        chart.addAxis(axis_x, Qt.AlignBottom)

        axis_y = QValueAxis()
        axis_y.setRange(*y_range)
        axis_y.setTickType(QValueAxis.TicksDynamic)
        axis_y.setTickAnchor(0)
        axis_y.setTickInterval(y_tick_interval)
        chart.addAxis(axis_y, Qt.AlignLeft)

        view = QChartView(chart)
        view.setRenderHint(QPainter.Antialiasing)

        series_by_name: Dict[str, QLineSeries] = {}
        return chart, view, axis_x, axis_y, series_by_name

    def _add_series(self, chart, axis_x, axis_y, series_by_name: Dict[str, QLineSeries], name: str, color: "QColor") -> None:
        """Create and attach one named line series to a chart.

        Args:
            chart: The QChart to add the series to.
            axis_x: The QValueAxis to attach as the series' X axis.
            axis_y: The QValueAxis to attach as the series' Y axis.
            series_by_name (Dict[str, QLineSeries]): Registry the new series
                is added to, keyed by history column name.
            name (str): History column name (e.g. ``"gPO"``) this series
                plots.
            color (QColor): Color for this series' line. Assigned explicitly
                (see :meth:`_distinct_color`) rather than left to QChart's
                default theme, whose palette is only ~5 colors deep and so
                silently repeats -- and becomes visually indistinguishable --
                once a chart has more series than that (as both charts here
                do).
        """
        series = QLineSeries()
        series.setName(name)
        series.setColor(color)
        chart.addSeries(series)
        series.attachAxis(axis_x)
        series.attachAxis(axis_y)
        series_by_name[name] = series

    @staticmethod
    def _distinct_color(index: int, total: int) -> "QColor":
        """Return one of ``total`` colors spaced evenly around the hue wheel.

        Used instead of QChart's default theme palette so that charts with
        many series (this window's "state" chart has 11) never assign the
        same color to two different series.

        Args:
            index (int): Position of this color in the sequence, ``0``-based.
            total (int): Total number of colors needed; must be >= 1.
        """
        hue = index / max(total, 1)
        return QColor.fromHsvF(hue, 0.65, 0.85)

    def _recolor_visible(self, all_signals: Sequence[str], series_by_name: Dict[str, QLineSeries]) -> None:
        """Re-spread distinct colors across just the currently *visible* series.

        Called once at startup and again every time a checkbox toggles a
        series on/off. Only the signals someone actually has checked need to
        be told apart from each other, so this recomputes colors for exactly
        that subset each time (in ``all_signals`` order, for a stable,
        deterministic assignment) rather than fixing colors once across the
        full, mostly-hidden signal list -- which is what previously let two
        *visible* series end up hue-adjacent (and hard to tell apart) purely
        by where they happened to fall in the full list. Hidden series keep
        whatever color they last had; it's repainted the moment they're shown
        again.

        Args:
            all_signals (Sequence[str]): Every signal name selectable for
                this chart, in the fixed order colors are assigned in.
            series_by_name (Dict[str, QLineSeries]): Series registry for this
                chart.
        """
        visible_names = [name for name in all_signals if series_by_name[name].isVisible()]
        for i, name in enumerate(visible_names):
            series_by_name[name].setColor(self._distinct_color(i, len(visible_names)))

    def _build_side_panel(self,
                           bounded_signals: Sequence[str],
                           state_signals: Sequence[str]) -> QScrollArea:
        """Build the scrollable control panel (signal checkboxes).

        Args:
            bounded_signals (Sequence[str]): Signals initially checked for
                the bounded chart.
            state_signals (Sequence[str]): Signals initially checked for the
                state chart.

        Returns:
            QScrollArea: The scrollable panel widget, ready to be placed in
            the window layout.
        """
        bounded_box = self._build_checkbox_group(
            "Position / Speed / Force (0-255)", BOUNDED_SIGNALS, bounded_signals, self._bounded_series
        )
        state_box = self._build_checkbox_group(
            "Status flags", STATE_SIGNALS, state_signals, self._state_series
        )

        panel_layout = QVBoxLayout()
        panel_layout.addWidget(bounded_box)
        panel_layout.addWidget(state_box)
        panel_layout.addStretch(1)
        panel_widget = QWidget()
        panel_widget.setLayout(panel_layout)

        scroll_area = QScrollArea()
        scroll_area.setWidget(panel_widget)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFixedWidth(260)
        return scroll_area

    def _build_checkbox_group(self,
                               title: str,
                               all_signals: Sequence[str],
                               checked_signals: Sequence[str],
                               series_by_name: Dict[str, QLineSeries]) -> QGroupBox:
        """Build a group of checkboxes toggling a chart's series visibility.

        Args:
            title (str): Title of the group box.
            all_signals (Sequence[str]): Every signal name selectable in this
                group.
            checked_signals (Sequence[str]): Signal names initially checked.
            series_by_name (Dict[str, QLineSeries]): Series registry the
                checkboxes control.

        Returns:
            QGroupBox: The populated group box widget.
        """
        group = QGroupBox(title)
        layout = QVBoxLayout()
        for name in all_signals:
            checkbox = QCheckBox(name)
            checkbox.setChecked(name in checked_signals)
            checkbox.toggled.connect(
                lambda checked, series_name=name, registry=series_by_name, signals=all_signals:
                    self._on_signal_toggled(checked, series_name, registry, signals)
            )
            layout.addWidget(checkbox)
        group.setLayout(layout)
        return group

    def _on_signal_toggled(self,
                            checked: bool,
                            name: str,
                            series_by_name: Dict[str, QLineSeries],
                            all_signals: Sequence[str]) -> None:
        """Checkbox callback: show/hide one series, then recolor the visible set.

        Args:
            checked (bool): New checked state of the checkbox.
            name (str): Signal name the checkbox controls.
            series_by_name (Dict[str, QLineSeries]): Series registry for this chart.
            all_signals (Sequence[str]): Every signal name selectable for this chart.
        """
        series_by_name[name].setVisible(checked)
        self._recolor_visible(all_signals, series_by_name)

    def refresh(self) -> None:
        """Redraw both charts from the gripper's latest command and status history.

        Called periodically from :class:`GripperVisualizer`'s poll timer.
        Does nothing if the gripper has no valid history yet, or while the
        display is frozen (Space toggles this; see :meth:`eventFilter`).
        """
        if self._frozen:
            return

        command_history = self._sorted_valid(self._gripper.commandHistory())
        status_history = self._sorted_valid(self._gripper.statusHistoryNumpy())

        if command_history.shape[0] == 0 and status_history.shape[0] == 0:
            return

        last_times = []
        if command_history.shape[0]:
            last_times.append(command_history[-1, TIME])
        if status_history.shape[0]:
            last_times.append(status_history[-1, TIME])
        t_now = max(last_times)

        self._bounded_axis_x.setRange(-self._duration, 0)
        self._state_axis_x.setRange(-self._duration, 0)

        self._refresh_group(self._bounded_series, command_history, status_history, t_now)
        self._refresh_group(
            self._state_series, command_history, status_history, t_now, autoscale_axis=self._state_axis_y
        )

    @staticmethod
    def _sorted_valid(raw_history: np.ndarray) -> np.ndarray:
        """Drop unfilled ring-buffer rows and sort what remains by time.

        Args:
            raw_history (numpy.ndarray): A raw ``commandHistory()`` or
                ``statusHistoryNumpy()`` array, whose unfilled rows are
                entirely ``-1`` (the history buffer's "no data" sentinel).

        Returns:
            numpy.ndarray: Rows with a valid (``!= -1``) time, sorted
            ascending by time.
        """
        valid = raw_history[raw_history[:, TIME] != -1]
        return valid[valid[:, TIME].argsort()]

    def _refresh_group(self,
                        series_by_name: Dict[str, QLineSeries],
                        command_history: np.ndarray,
                        status_history: np.ndarray,
                        t_now: float,
                        autoscale_axis: Optional[QValueAxis] = None) -> None:
        """Update every visible series in a chart group.

        Args:
            series_by_name (Dict[str, QLineSeries]): Series registry for
                this chart.
            command_history (numpy.ndarray): Valid, time-sorted rows from
                ``commandHistory()``.
            status_history (numpy.ndarray): Valid, time-sorted rows from
                ``statusHistoryNumpy()``.
            t_now (float): Reference time (seconds) mapped to ``x=0``; the
                more recent of the command and status histories' last
                timestamps.
            autoscale_axis (QValueAxis, optional): If given, this axis' range
                is rescaled to fit the visible series' min/max value with a
                10% padding. Left untouched (fixed range) if omitted.
        """
        visible_min = None
        visible_max = None

        for name, series in series_by_name.items():
            if not series.isVisible():
                continue

            if name in COMMAND_HISTORY_COLUMNS_NAME_2_ID:
                column = COMMAND_HISTORY_COLUMNS_NAME_2_ID[name]
                points = self._build_command_points(command_history, column, t_now)
            else:
                column = STATUS_HISTORY_COLUMNS_NAME_2_ID[name]
                points = self._build_status_points(status_history, column, t_now)

            series.replace(points)

            if points:
                values = [p.y() for p in points]
                visible_min = min(values) if visible_min is None else min(visible_min, *values)
                visible_max = max(values) if visible_max is None else max(visible_max, *values)

        if autoscale_axis is not None and visible_min is not None:
            padding = max(1.0, (visible_max - visible_min) * 0.1)
            autoscale_axis.setRange(visible_min - padding, visible_max + padding)

    def _build_command_points(self,
                               command_history: np.ndarray,
                               column: int,
                               t_now: float) -> List["QPointF"]:
        """Build plot points for a commanded (write-side) register.

        A command register only changes when a new command is actually
        sent, so its last known value is held flat out to ``t_now`` (drawn
        at ``x=0``) instead of the line stopping wherever the last command
        happened to be logged. Similarly, if an earlier command already
        applies at the left edge of the window, a carried-over point is
        added there so the line doesn't start empty.

        Args:
            command_history (numpy.ndarray): Valid, time-sorted rows from
                ``commandHistory()``.
            column (int): Column index of the register to plot.
            t_now (float): Reference time (seconds) mapped to ``x=0``.

        Returns:
            List[QPointF]: Points to feed to ``QLineSeries.replace()``.
        """
        if command_history.shape[0] == 0:
            return []

        window_start = -self._duration
        times = command_history[:, TIME] - t_now
        values = command_history[:, column]

        points: List[QPointF] = []

        before_window = times < window_start
        if np.any(before_window):
            points.append(QPointF(window_start, values[before_window][-1]))

        in_window = times >= window_start
        for t, v in zip(times[in_window], values[in_window]):
            points.append(QPointF(max(t, window_start), v))

        if not points or points[-1].x() < 0.0:
            points.append(QPointF(0.0, values[-1]))

        return points

    def _build_status_points(self,
                              status_history: np.ndarray,
                              column: int,
                              t_now: float) -> List["QPointF"]:
        """Build plot points for a read-side status register.

        Unlike commanded registers, a status register reflects an actual
        sensor reading: only real samples are plotted, with no synthetic
        holding to ``t_now``.

        Args:
            status_history (numpy.ndarray): Valid, time-sorted rows from
                ``statusHistoryNumpy()``.
            column (int): Column index of the register to plot.
            t_now (float): Reference time (seconds) mapped to ``x=0``.

        Returns:
            List[QPointF]: Points to feed to ``QLineSeries.replace()``.
        """
        if status_history.shape[0] == 0:
            return []

        window_start = -self._duration
        times = status_history[:, TIME] - t_now
        values = status_history[:, column]

        mask = (times >= window_start) & (values != -1)
        if not np.any(mask):
            return []

        return [QPointF(t, v) for t, v in zip(times[mask], values[mask])]

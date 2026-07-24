"""Live visualization window for a RobotiqGripper's state.

This module renders two scrolling line charts built from a
:class:`~pyrobotiqgripper.gripper.RobotiqGripper` history buffer:

* A "bounded" chart for registers whose raw value is naturally in ``[0, 255]``
  (actual position, commanded speed/force, ...).
* A "state" chart for small enumerated / flag registers (object detection,
  activation status, faults, ...).

The window runs on its own background thread with its own Qt event loop, so
it can be dropped into an existing control loop (e.g. the joystick CLI)
without interfering with it, the same way
:class:`~pyrobotiqgripper.bipper.Bipper` runs its own audio thread. It never
talks to the gripper itself: it only polls
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
import threading
from typing import Dict, List, Optional, Sequence

import numpy as np

try:
    from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
    from PySide6.QtCore import QPointF, Qt, QTimer
    from PySide6.QtGui import QPainter
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

#: History column names whose raw register value is naturally in [0, 255].
BOUNDED_SIGNALS: List[str] = ["gPO", "rPR", "rSP", "rFR", "gPR", "gCU"]

#: History column names carrying small enumerated / flag register values.
STATE_SIGNALS: List[str] = [
    "gOBJ", "gSTA", "gGTO", "gACT", "kFLT", "gFLT", "rARD", "rATR", "rGTO", "rACT",
]

#: Signals checked by default on the bounded chart when none are given: actual
#: position, commanded speed and commanded force.
DEFAULT_BOUNDED_SIGNALS: List[str] = ["gPO", "rSP", "rFR"]

#: Signals checked by default on the state chart when none are given.
DEFAULT_STATE_SIGNALS: List[str] = ["gOBJ"]

#: Fixed timeline window, in seconds, shown on both charts. Matches
#: :data:`~pyrobotiqgripper.constants.MAX_HISTORY` (~5s at a typical 100Hz
#: control loop), the most history the gripper itself ever retains.
TIMELINE_DURATION_S: float = 5.0


class GripperVisualizer:
    """Background window plotting a gripper's live history.

    The window is created and driven entirely on a dedicated background
    thread, started and stopped explicitly with :meth:`start` and
    :meth:`stop`. Closing the window from the UI also stops the thread.

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

    Warning:
        Reading the gripper's history buffers concurrently with whichever
        thread is filling them (e.g. a joystick control loop) is not lock
        protected. For a live debugging display this is an acceptable
        trade-off: at worst a single redraw reads a partially updated row.

    Warning:
        An instance only supports a single :meth:`start`/:meth:`stop` cycle:
        Qt does not support re-creating a QApplication after a previous one
        has been destroyed in the same process. Calling :meth:`start` again
        after :meth:`stop` raises :class:`RuntimeError`; create a new
        ``GripperVisualizer`` instance instead.

    Warning:
        PySide6 requires ``QApplication`` to live on a process' main thread.
        This class runs it on a background thread instead (so the thread
        driving the gripper stays free), which works correctly while
        running, but reliably segfaults during Python's normal interpreter
        shutdown once a ``GripperVisualizer`` has been started, regardless
        of :meth:`stop` having already run cleanly. This is a Qt/PySide6
        limitation, not something :meth:`stop` can work around. If your
        script calls :meth:`start`, end it with ``os._exit(0)`` (after
        flushing stdout/stderr and any other cleanup) instead of a normal
        return, to skip that crash-prone shutdown sequence entirely -- see
        how :func:`pyrobotiqgripper.joystick_cli.main` does it.

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

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._stopped_once = False

    def start(self) -> None:
        """Start the visualization window on a background thread.

        Does nothing if the window is already running.

        Raises:
            RuntimeError: If this instance was already started and stopped
                once before. Qt does not support re-creating a QApplication
                after a previous one has been destroyed in the same process,
                so a given GripperVisualizer instance only supports a single
                start/stop cycle. Create a new instance for a second run.
        """
        if self._thread is not None:
            return
        if self._stopped_once:
            raise RuntimeError(
                "This GripperVisualizer instance was already stopped once and "
                "cannot be restarted (Qt does not support re-creating a "
                "QApplication in the same process). Create a new instance instead."
            )
        self._stop_event.clear()
        self._ready_event.clear()
        self._thread = threading.Thread(target=self._run, name="GripperVisualizer", daemon=True)
        self._thread.start()
        self._ready_event.wait(timeout=5.0)

    def stop(self) -> None:
        """Request the window to close and wait for its thread to exit.

        Does nothing if the window is not running.
        """
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=5.0)
        self._thread = None
        self._stopped_once = True

    def is_running(self) -> bool:
        """Return whether the visualization thread is currently running.

        Returns:
            bool: True if the background thread is alive.
        """
        return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        """Background thread entry point: build the window and run its Qt event loop."""
        app = QApplication.instance() or QApplication([])
        window = _GripperVisualizerWindow(
            self._gripper,
            bounded_signals=self._bounded_signals,
            state_signals=self._state_signals,
            window_title=self._window_title,
        )
        window.show()

        poll_timer = QTimer()
        poll_timer.timeout.connect(lambda: self._poll(app, window))
        poll_timer.start(self._refresh_interval_ms)

        self._ready_event.set()
        app.exec_()

        # Qt objects created on this thread must also be torn down on this
        # thread. Left to Python's garbage collector, they could otherwise
        # be finalized later from an unrelated thread (e.g. at interpreter
        # shutdown), which reliably segfaults PySide6's native bindings.
        poll_timer.stop()
        window.close()
        del poll_timer, window
        app.processEvents()
        del app
        gc.collect()

    def _poll(self, app: "QApplication", window: "_GripperVisualizerWindow") -> None:
        """Timer callback run on the Qt thread: refresh the plot or quit.

        Args:
            app: The QApplication running the window's event loop.
            window: The visualization window to refresh.
        """
        if self._stop_event.is_set() or not window.isVisible():
            app.quit()
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

        self.setWindowTitle(window_title)
        self.resize(1000, 700)

        bounded_chart, bounded_view, self._bounded_axis_x, self._bounded_axis_y, self._bounded_series = (
            self._build_chart("Position / Speed / Force (0-255)", y_range=(0, 255), y_tick_interval=25)
        )
        state_chart, state_view, self._state_axis_x, self._state_axis_y, self._state_series = (
            self._build_chart("Status flags", y_range=(0, 16), y_tick_interval=1)
        )
        self._bounded_chart = bounded_chart
        self._state_chart = state_chart

        for name in BOUNDED_SIGNALS:
            self._add_series(bounded_chart, self._bounded_axis_x, self._bounded_axis_y, self._bounded_series, name)
        for name in STATE_SIGNALS:
            self._add_series(state_chart, self._state_axis_x, self._state_axis_y, self._state_series, name)

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

    def _add_series(self, chart, axis_x, axis_y, series_by_name: Dict[str, QLineSeries], name: str) -> None:
        """Create and attach one named line series to a chart.

        Args:
            chart: The QChart to add the series to.
            axis_x: The QValueAxis to attach as the series' X axis.
            axis_y: The QValueAxis to attach as the series' Y axis.
            series_by_name (Dict[str, QLineSeries]): Registry the new series
                is added to, keyed by history column name.
            name (str): History column name (e.g. ``"gPO"``) this series
                plots.
        """
        series = QLineSeries()
        series.setName(name)
        chart.addSeries(series)
        series.attachAxis(axis_x)
        series.attachAxis(axis_y)
        series_by_name[name] = series

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
                lambda checked, series_name=name, registry=series_by_name: registry[series_name].setVisible(checked)
            )
            layout.addWidget(checkbox)
        group.setLayout(layout)
        return group

    def refresh(self) -> None:
        """Redraw both charts from the gripper's latest command and status history.

        Called periodically from :class:`GripperVisualizer`'s poll timer.
        Does nothing if the gripper has no valid history yet.
        """
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

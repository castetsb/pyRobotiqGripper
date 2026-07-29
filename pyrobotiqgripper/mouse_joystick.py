"""Generic two-axis mouse-as-joystick input device.

Exposes mouse position as two axes (X and Y) normalized to [-1, 1] or
[0, 1], mirroring the interface of pygame.joystick.Joystick.get_axis(), so it
can be used as a drop-in replacement wherever only get_axis(axis_index)
is needed -- including by :class:`~pyrobotiqgripper.joystick_visual_tool.JoystickVisualTool`.
"""

from __future__ import annotations

from pynput import mouse

#: Axis index for the horizontal mouse position, for use with
#: :meth:`~pyrobotiqgripper.mouse_joystick.MouseJoystick.get_axis`.
AXIS_X = 0

#: Axis index for the vertical mouse position, for use with
#: :meth:`~pyrobotiqgripper.mouse_joystick.MouseJoystick.get_axis`.
AXIS_Y = 1


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

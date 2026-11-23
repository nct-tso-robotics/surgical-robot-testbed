"""Decode Linux input events into immutable device snapshots."""

from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class GamepadState:
    left_joy_x: float = 0.0
    left_joy_y: float = 0.0
    right_joy_x: float = 0.0
    right_joy_y: float = 0.0
    l2_trigger: float = 0.0
    r2_trigger: float = 0.0
    l1: bool = False
    l2: bool = False
    r1: bool = False
    r2: bool = False
    l3: bool = False
    r3: bool = False
    x_btn: bool = False
    square_btn: bool = False
    circle_btn: bool = False
    triangle_btn: bool = False
    share_btn: bool = False
    start_btn: bool = False
    ps_btn: bool = False
    left_right_arrow: int = 0
    up_down_arrow: int = 0


@dataclass(frozen=True)
class PedalState:
    right_blue_pedal: bool = False
    left_blue_pedal: bool = False
    right_yellow_pedal: bool = False
    left_yellow_pedal: bool = False
    top_grey_pedal: bool = False
    bottom_grey_pedal: bool = False
    flap: bool = False


GAMEPAD_AXES = {
    "ABS_X": "left_joy_x",
    "ABS_Y": "left_joy_y",
    "ABS_RX": "right_joy_x",
    "ABS_RY": "right_joy_y",
}
GAMEPAD_TRIGGERS = {"ABS_Z": "l2_trigger", "ABS_RZ": "r2_trigger"}
GAMEPAD_BUTTONS = {
    "BTN_A": "x_btn",
    "BTN_B": "circle_btn",
    "BTN_X": "triangle_btn",
    "BTN_Y": "square_btn",
    "BTN_SOUTH": "x_btn",
    "BTN_NORTH": "triangle_btn",
    "BTN_EAST": "circle_btn",
    "BTN_WEST": "square_btn",
    "BTN_TL": "l1",
    "BTN_TL2": "l2",
    "BTN_TR": "r1",
    "BTN_TR2": "r2",
    "BTN_THUMBL": "l3",
    "BTN_THUMBR": "r3",
    "BTN_SELECT": "share_btn",
    "BTN_START": "start_btn",
    "BTN_MODE": "ps_btn",
}
GAMEPAD_HAT = {"ABS_HAT0X": "left_right_arrow", "ABS_HAT0Y": "up_down_arrow"}
PEDAL_KEYS = {
    "KEY_F6": "top_grey_pedal",
    "KEY_F7": "left_yellow_pedal",
    "KEY_F8": "right_yellow_pedal",
    "KEY_F9": "bottom_grey_pedal",
    "KEY_F10": "left_blue_pedal",
    "KEY_F11": "right_blue_pedal",
    "KEY_F12": "flap",
}


class GamepadDecoder:
    """Maintain a snapshot, normalizing axes from the device's reported range."""

    def __init__(self, deadzone: float = 0.09) -> None:
        if not 0 <= deadzone < 1:
            raise ValueError("deadzone must be in [0, 1)")
        self.deadzone = deadzone
        self.state = GamepadState()

    def update(
        self, code: str, value: int, minimum: int = 0, maximum: int = 255
    ) -> None:
        if code in GAMEPAD_AXES or code in GAMEPAD_TRIGGERS:
            if maximum <= minimum:
                raise ValueError("Axis range must increase")
            normalized = min(1.0, max(0.0, (value - minimum) / (maximum - minimum)))
            if code in GAMEPAD_AXES:
                normalized = normalized * 2 - 1
                if abs(normalized) < self.deadzone:
                    normalized = 0.0
                field = GAMEPAD_AXES[code]
            else:
                field = GAMEPAD_TRIGGERS[code]
            self.state = replace(self.state, **{field: normalized})
        elif code in GAMEPAD_BUTTONS:
            self.state = replace(self.state, **{GAMEPAD_BUTTONS[code]: bool(value)})
        elif code in GAMEPAD_HAT:
            self.state = replace(self.state, **{GAMEPAD_HAT[code]: int(value)})


class PedalDecoder:
    def __init__(self) -> None:
        self.state = PedalState()

    def update(self, code: str, value: int) -> None:
        if code in PEDAL_KEYS:
            self.state = replace(self.state, **{PEDAL_KEYS[code]: bool(value)})


@dataclass(frozen=True)
class FootSwitchState:
    right_switch: bool = False
    left_switch: bool = False


class FootSwitchDecoder:
    def __init__(self) -> None:
        self.state = FootSwitchState()

    def update(self, code: str, value: int) -> None:
        mapping = {"KEY_M": "right_switch", "KEY_N": "left_switch"}
        if code in mapping:
            self.state = replace(self.state, **{mapping[code]: bool(value)})

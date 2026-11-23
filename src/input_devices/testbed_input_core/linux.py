"""Nonblocking evdev acquisition; no middleware or background thread required."""

from __future__ import annotations

import select

from evdev import InputDevice, ecodes, list_devices

from testbed_input_core.devices import (
    GAMEPAD_BUTTONS,
    FootSwitchDecoder,
    FootSwitchState,
    GamepadDecoder,
    GamepadState,
    PedalDecoder,
    PedalState,
)


def open_device(path: str, name: str, index: int = 0) -> InputDevice:
    """Open a configured path or select the matching input device."""
    if path:
        return InputDevice(path)
    matches = 0
    for candidate in list_devices():
        device = InputDevice(candidate)
        capabilities = device.capabilities(absinfo=False)
        suitable = (
            name in device.name
            if name
            else ecodes.ABS_RX in capabilities.get(ecodes.EV_ABS, [])
        )
        if suitable and matches == index:
            return device
        matches += int(suitable)
        device.close()
    raise FileNotFoundError(f"Input device not found: name={name!r}, index={index}")


class GamepadReader:
    def __init__(self, path: str = "", name: str = "", index: int = 0) -> None:
        self.device = open_device(path=path, name=name, index=index)
        self.decoder = GamepadDecoder()
        self._axes = dict(self.device.capabilities(absinfo=True).get(ecodes.EV_ABS, []))

    def read(self) -> GamepadState:
        if select.select([self.device.fd], [], [], 0)[0]:
            for event in self.device.read():
                names = ecodes.bytype.get(event.type, {}).get(event.code, "")
                if isinstance(names, (list, tuple)):
                    names = next(
                        (name for name in names if name in GAMEPAD_BUTTONS), names[0]
                    )
                axis = (
                    self._axes.get(event.code) if event.type == ecodes.EV_ABS else None
                )
                self.decoder.update(
                    code=names,
                    value=event.value,
                    minimum=axis.min if axis else 0,
                    maximum=axis.max if axis else 255,
                )
        return self.decoder.state

    def close(self) -> None:
        self.device.close()


class PedalReader:
    def __init__(self, path: str = "", name: str = "Adafruit Feather") -> None:
        self.device = open_device(path=path, name=name)
        self.decoder = PedalDecoder()

    def read(self) -> PedalState:
        if select.select([self.device.fd], [], [], 0)[0]:
            for event in self.device.read():
                if event.type == ecodes.EV_KEY:
                    code = ecodes.KEY.get(event.code, "")
                    if isinstance(code, (list, tuple)):
                        code = code[0]
                    self.decoder.update(code=code, value=event.value)
        return self.decoder.state

    def close(self) -> None:
        self.device.close()


class FootSwitchReader:
    def __init__(self, path: str = "", name: str = "HID 1a86:e026 Keyboard") -> None:
        self.device = open_device(path=path, name=name)
        self.decoder = FootSwitchDecoder()

    def read(self) -> FootSwitchState:
        if select.select([self.device.fd], [], [], 0)[0]:
            for event in self.device.read():
                if event.type == ecodes.EV_KEY:
                    code = ecodes.KEY.get(event.code, "")
                    if isinstance(code, (list, tuple)):
                        code = code[0]
                    self.decoder.update(code=code, value=event.value)
        return self.decoder.state

    def close(self) -> None:
        self.device.close()

"""Discover CoFly over USB first, then transparent telemetry UARTs."""
from dataclasses import dataclass
import threading
import time

import serial
from serial.tools import list_ports

from protocol import ReplyFramer
from station_protocol import decode_fields


def flight_controller_port(port):
    description = " ".join(str(getattr(port, key, "") or "")
                           for key in ("description", "manufacturer", "product"))
    return getattr(port, "vid", None) == 0x16c0 or "teensy" in description.lower()


def telemetry_port(port):
    description = " ".join(str(getattr(port, key, "") or "")
                           for key in ("description", "manufacturer", "product")).lower()
    return (getattr(port, "vid", None) in (0x10c4, 0x0403, 0x1a86)
            or any(name in description for name in ("cp210", "ftdi", "ch340", "ch341", "3dr", "sik")))


@dataclass(frozen=True)
class DetectedLink:
    port: str
    link_type: str
    baudrate: int


class FlightControllerDetector(threading.Thread):
    def __init__(self, preferred_port="", usb_baud=921600, radio_baud=57600):
        super().__init__(daemon=True)
        self.preferred_port = preferred_port
        self.usb_baud = usb_baud
        self.radio_baud = radio_baud
        self.stopping = threading.Event()
        self.result = None

    def stop(self):
        self.stopping.set()

    def run(self):
        try:
            ports = list(list_ports.comports())
        except (OSError, serial.SerialException):
            return
        # Transport priority precedes the last selected port. Never choose a
        # generic USB-UART adapter based on its chip alone: require valid CAPS.
        candidates = []
        for port in ports:
            if flight_controller_port(port):
                candidates.append(DetectedLink(port.device, "USB", self.usb_baud))
            elif telemetry_port(port) or port.device == self.preferred_port:
                candidates.append(DetectedLink(port.device, "3DR X6", self.radio_baud))
        candidates.sort(key=lambda link: (link.link_type != "USB",
                                         link.port != self.preferred_port, link.port))
        for link in candidates:
            if self.stopping.is_set():
                return
            try:
                with serial.Serial(link.port, link.baudrate, timeout=0.05, write_timeout=0.3,
                                   rtscts=False, dsrdtr=False, xonxoff=False) as connection:
                    if self.stopping.is_set():
                        return
                    request_id = 4294967294
                    connection.write(f"GCS CAPS {request_id}\n".encode("ascii"))
                    framer = ReplyFramer()
                    deadline = time.monotonic() + (1.2 if link.link_type == "USB" else 3)
                    while not self.stopping.is_set() and time.monotonic() < deadline:
                        for reply_id, kind, fields in framer.feed(connection.read(1024)):
                            if reply_id == request_id and kind == "CAPS":
                                try:
                                    decode_fields(kind, fields)
                                except ValueError:
                                    continue
                                self.result = link
                                return
            except (OSError, serial.SerialException, ValueError):
                continue

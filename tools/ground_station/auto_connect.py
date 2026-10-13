"""Background USB discovery: confirm CoFly CAPS before offering a port."""
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


class FlightControllerDetector(threading.Thread):
    def __init__(self, preferred_port="", baudrate=921600):
        super().__init__(daemon=True)
        self.preferred_port = preferred_port
        self.baudrate = baudrate
        self.stopping = threading.Event()
        self.result = None

    def stop(self):
        self.stopping.set()

    def run(self):
        try:
            ports = [port for port in list_ports.comports() if flight_controller_port(port)]
        except (OSError, serial.SerialException):
            return
        ports.sort(key=lambda port: (port.device != self.preferred_port, port.device))
        for port in ports:
            if self.stopping.is_set():
                return
            try:
                with serial.Serial(port.device, self.baudrate, timeout=0.05, write_timeout=0.3) as connection:
                    if self.stopping.is_set():
                        return
                    request_id = 4294967294
                    connection.write(f"GCS CAPS {request_id}\n".encode("ascii"))
                    framer = ReplyFramer()
                    deadline = time.monotonic() + 1.2
                    while not self.stopping.is_set() and time.monotonic() < deadline:
                        for reply_id, kind, fields in framer.feed(connection.read(1024)):
                            if reply_id == request_id and kind == "CAPS":
                                decode_fields(kind, fields)
                                self.result = port.device
                                return
            except (OSError, serial.SerialException, ValueError):
                continue

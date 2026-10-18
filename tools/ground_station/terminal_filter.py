"""Hide successful automatic queries without changing protocol or raw recording."""
from collections import OrderedDict

from protocol import parse_reply


class AutomaticQueryFilter:
    def __init__(self):
        self.requests = OrderedDict()
        self.line = bytearray()
        self.passthrough = False

    def register(self, request_id, verb):
        self.requests[request_id] = verb
        # Retain a few timed-out IDs for late replies, with a fixed memory bound.
        while len(self.requests) > 256:
            self.requests.popitem(last=False)

    def feed(self, kind, payload):
        if kind == "tx":
            parts = payload.split()
            if len(parts) == 3 and parts[0] == b"GCS" and parts[2].isdigit():
                verb = self.requests.get(int(parts[2]))
                if verb and parts[1] == verb.encode("ascii"):
                    return b""
            return payload

        visible = bytearray()
        prefix = b"@COFLY\t"
        for byte in payload:
            if self.passthrough:
                visible.append(byte)
                if byte == 10:
                    self.passthrough = False
                continue
            self.line.append(byte)
            if byte == 10:
                reply = parse_reply(bytes(self.line).rstrip(b"\r\n"))
                if reply and self.requests.get(reply[0]) == reply[1]:
                    del self.requests[reply[0]]
                else:
                    visible.extend(self.line)
                self.line.clear()
            elif ((len(self.line) <= len(prefix) and not prefix.startswith(self.line))
                  or len(self.line) > 1024):
                # Ordinary debug bytes are displayed immediately, including
                # partial UTF-8 characters; only possible protocol lines wait.
                visible.extend(self.line)
                self.line.clear()
                self.passthrough = True
        return bytes(visible)

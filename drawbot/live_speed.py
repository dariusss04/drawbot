import socket
import struct
import threading

PORT = 30004
PROTOCOL_VERSION = 2
REQUEST_PROTOCOL_VERSION = ord("V")
SETUP_INPUTS = ord("I")
SETUP_OUTPUTS = ord("O")
TEXT_MESSAGE = ord("M")
START = ord("S")
PAUSE = ord("P")
DATA = ord("U")
INPUTS = b"speed_slider_mask,speed_slider_fraction"
EXPECTED_TYPES = "UINT32,DOUBLE"
OUTPUTS = b"timestamp"
OUTPUT_HZ = 1.0


class SpeedLink:
    """One RTDE connection that owns the robot's speed slider. Thread safe; one per app process."""

    def __init__(self):
        self.lock = threading.Lock()
        self.sock = None
        self.recipe = None
        self.ip = None
        self.fraction = None
        self.messages = []
        self.reader = None

    @property
    def connected(self):
        return self.sock is not None

    def _send(self, kind, payload=b""):
        self.sock.sendall(struct.pack(">HB", len(payload) + 3, kind) + payload)

    def _exact(self, n):
        data = b""
        while len(data) < n:
            chunk = self.sock.recv(n - len(data))
            if not chunk:
                raise ConnectionError("The robot closed the RTDE connection")
            data += chunk
        return data

    def _reply(self, kind):
        """The next message of this type. Text messages in between are kept, others skipped."""
        for _ in range(50):
            size, got = struct.unpack(">HB", self._exact(3))
            body = self._exact(size - 3)
            if got == kind:
                return body
            if got == TEXT_MESSAGE:
                text = "".join(chr(c) for c in body if 32 <= c < 127).strip()
                if text:
                    self.messages.append(text)
        raise ConnectionError(f"No RTDE answer of type {chr(kind)}")

    def _drain(self, sock):
        """Read and drop the clock messages, so the controller never blocks on a full socket."""
        try:
            sock.settimeout(None)
            while sock.recv(4096):
                pass
        except OSError:
            pass

    def connect(self, ip, port=PORT, timeout=3.0):
        with self.lock:
            self._close()
            self.messages = []
            self.sock = socket.create_connection((ip, port), timeout=timeout)
            try:
                self._send(REQUEST_PROTOCOL_VERSION, struct.pack(">H", PROTOCOL_VERSION))
                if not self._reply(REQUEST_PROTOCOL_VERSION)[:1] == b"\x01":
                    raise RuntimeError("The robot does not speak RTDE protocol version 2. Update PolyScope.")
                self._send(SETUP_INPUTS, INPUTS)
                body = self._reply(SETUP_INPUTS)
                recipe, types = body[0], body[1:].decode(errors="replace")
                if types != EXPECTED_TYPES:
                    hint = (" Another program already sets the speed slider, for example RoboDK's robot driver: "
                            "disconnect it.") if "IN_USE" in types else ""
                    raise RuntimeError(f"The robot refused the speed slider inputs ({types}).{hint}")
                self._send(SETUP_OUTPUTS, struct.pack(">d", OUTPUT_HZ) + OUTPUTS)
                body = self._reply(SETUP_OUTPUTS)
                if body[1:].decode(errors="replace") != "DOUBLE":
                    raise RuntimeError(f"The robot refused the clock output ({body[1:].decode(errors='replace')})")
                self._send(START)
                if not self._reply(START)[:1] == b"\x01":
                    said = f" The robot said: {'; '.join(self.messages[-3:])}" if self.messages else ""
                    raise RuntimeError("The robot refused to start RTDE." + said)
            except Exception:
                self._close()
                raise
            self.recipe, self.ip = recipe, ip
            self.reader = threading.Thread(target=self._drain, args=(self.sock,), daemon=True)
            self.reader.start()

    def set(self, fraction):
        """Set the speed slider, 0.0 to 1.0."""
        fraction = max(0.0, min(1.0, float(fraction)))
        with self.lock:
            if self.sock is None:
                raise RuntimeError("Not connected to the robot's speed slider")
            try:
                self._send(DATA, struct.pack(">BId", self.recipe, 1, fraction))
            except OSError:
                self._close()
                raise
            self.fraction = fraction
        return fraction

    def _close(self):
        if self.sock is not None:
            try:
                self._send(PAUSE)
            except OSError:
                pass
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()
        self.sock, self.recipe, self.ip, self.fraction, self.reader = None, None, None, None, None

    def close(self):
        with self.lock:
            self._close()


LINK = SpeedLink()

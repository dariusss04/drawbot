import os
import socket


def connect_existing():
    port = int(os.environ.get("ROBODK_API_PORT", "20500"))
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=2):
            pass
    except OSError as exc:
        raise RuntimeError(f"RoboDK is not available on localhost:{port}; open RoboDK and load the station") from exc
    from robodk.robolink import Robolink
    return Robolink(robodk_ip="127.0.0.1", port=port, args=["-API_NODELAY"], quit_on_close=False)

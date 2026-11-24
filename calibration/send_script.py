import socket
import sys
from pathlib import Path

ROBOT_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.100"
script = (Path(__file__).with_name("teach_whiteboard.script")).read_text(encoding="utf-8")

s = socket.create_connection((ROBOT_IP, 30002), timeout=5)
s.sendall(script.encode("utf-8") + b"\n")
s.close()
print("Script sent to", ROBOT_IP)

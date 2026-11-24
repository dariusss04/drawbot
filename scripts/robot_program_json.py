import os
from pathlib import Path
import sys

def _project_root():
    override = os.environ.get("DRAWBOT_HOME")
    if override is not None:
        candidate = Path(override).expanduser()
        if not candidate.is_absolute():
            raise RuntimeError("DRAWBOT_HOME must be an absolute DrawBot project path")
        candidates = [candidate]
    else:
        candidates = [Path(__file__).resolve().parents[1], Path(sys.prefix).parent]
    for candidate in candidates:
        if (candidate / "drawbot/robot/json_program.py").is_file() and (candidate / "config/drawing.json").is_file():
            return candidate.resolve()
    raise RuntimeError("Cannot locate DrawBot: use its .venv/bin/python or set absolute DRAWBOT_HOME")


PROJECT_ROOT = _project_root()
sys.path.insert(0, str(PROJECT_ROOT))

if __name__ == "__main__":
    from drawbot.robot.json_program import main
    main(PROJECT_ROOT)

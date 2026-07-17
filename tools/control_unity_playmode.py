#!/usr/bin/env python3
"""Control the open Unity Editor through SpiderPlayModeCapture.cs."""
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
COMMAND = ROOT / "Logs" / "spider_editor_command.txt"

def send(command: str) -> None:
    COMMAND.parent.mkdir(parents=True, exist_ok=True)
    COMMAND.write_text(command + "\n", encoding="utf-8")
    print(f"Sent {command} to Unity Editor")

if __name__ == "__main__":
    command = sys.argv[1].lower() if len(sys.argv) > 1 else "play5"
    if command in ("play5", "capture"):
        send("PLAY5")
    elif command == "stop":
        send("STOP")
    else:
        raise SystemExit("Usage: control_unity_playmode.py [play5|stop]")

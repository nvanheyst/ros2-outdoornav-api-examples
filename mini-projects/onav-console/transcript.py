"""Append-only JSONL session log under ~/.onav-console/sessions/."""
from __future__ import annotations

import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class Transcript:
    def __init__(self, root: Path | None = None) -> None:
        root = root or (Path.home() / ".onav-console" / "sessions")
        root.mkdir(parents=True, exist_ok=True)
        self.path = root / f"{datetime.now().strftime('%Y-%m-%d-%H%M%S')}.jsonl"
        self._fh = self.path.open("a")

    def write(self, kind: str, data: Any) -> None:
        try:
            self._fh.write(json.dumps({
                "ts": datetime.now().isoformat(timespec="seconds"),
                "kind": kind,
                "data": data,
            }, default=str) + "\n")
            self._fh.flush()
        except Exception as e:
            logger.warning("transcript write failed: %s", e)

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass

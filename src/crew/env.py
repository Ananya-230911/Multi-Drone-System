"""Settings for the AI crew, read from environment variables or a `.env` file in the project root."""

from __future__ import annotations

import importlib.util
import os
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = "llama-3.3-70b-versatile"
GROQ_BASE_URL = "https://api.groq.com/openai/v1"


def read_dotenv(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    try:
        text = path.read_text(encoding="utf-8-sig")
    except OSError:
        return out
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip().removeprefix("export ").strip()] = v.strip().strip("'\"")
    return out


@dataclass(frozen=True)
class Settings:
    api_key: str
    model: str
    base_url: str

    @property
    def crew_ready(self) -> bool:
        return bool(self.api_key) and importlib.util.find_spec("crewai") is not None


def load(root: Path = ROOT, environ: dict[str, str] | None = None) -> Settings:
    env = dict(read_dotenv(root / ".env"))
    env.update(os.environ if environ is None else environ)  # real environment variables win
    return Settings(env.get("GROQ_API_KEY", "").strip(), env.get("GROQ_MODEL", "").strip() or DEFAULT_MODEL,
                    env.get("GROQ_BASE_URL", "").strip() or GROQ_BASE_URL)


def status(settings: Settings) -> dict:
    """What the page shows next to the brief box. Never includes the key itself."""
    has_pkg = importlib.util.find_spec("crewai") is not None
    if settings.crew_ready:
        msg = f"AI crew ready (CrewAI + Groq, model {settings.model})"
    elif not has_pkg:
        msg = "CrewAI is not installed in this Python; using the built-in offline planner"
    else:
        msg = "No GROQ_API_KEY found in .env; using the built-in offline planner"
    return {"crew_ready": settings.crew_ready, "crewai_installed": has_pkg, "has_key": bool(settings.api_key),
            "model": settings.model, "message": msg}  # fmt: skip

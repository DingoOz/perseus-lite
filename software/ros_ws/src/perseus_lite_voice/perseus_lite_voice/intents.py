"""Voice → robot intent router.

Pure-function routing: takes a transcript and returns an Intent describing what
the orchestrator should do (drive the rover, toggle the explorer, ask vision,
or fall through to the LLM). Side effects live in voice_node.py so the router
stays trivially testable.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

# Order matters: more specific patterns come first.
_PATTERNS: list[tuple[str, re.Pattern[str]]] = [
    (
        "see",
        re.compile(
            r"\b(what (do|can) you see|describe what you see)\b|^:see\b", re.IGNORECASE
        ),
    ),
    ("stop_exploring", re.compile(r"\bstop exploring\b", re.IGNORECASE)),
    (
        "start_exploring",
        re.compile(
            r"\b(start exploring|begin exploring|go explore|explore)\b", re.IGNORECASE
        ),
    ),
    ("stop", re.compile(r"\b(stop|halt|freeze|hold)\b", re.IGNORECASE)),
    ("forward", re.compile(r"\b(go|move|drive|head)\s+forward(s)?\b", re.IGNORECASE)),
    (
        "backward",
        re.compile(r"\b(go|move|drive|head)\s+back(ward)?s?\b", re.IGNORECASE),
    ),
    ("turn_left", re.compile(r"\b(turn|rotate|spin)\s+left\b", re.IGNORECASE)),
    ("turn_right", re.compile(r"\b(turn|rotate|spin)\s+right\b", re.IGNORECASE)),
    (
        "reset_history",
        re.compile(
            r"\b(reset (the )?(history|conversation)|forget (everything|that))\b",
            re.IGNORECASE,
        ),
    ),
]


@dataclass
class Intent:
    name: str  # e.g. "stop", "forward", "chat"
    text: str  # original transcript (for the LLM fallthrough)
    speak: Optional[str] = None  # if set, say this acknowledgement before acting


def route(transcript: str) -> Intent:
    """Classify a transcript into a robot intent (or "chat" for LLM fallthrough)."""
    text = transcript.strip()
    if not text:
        return Intent(name="chat", text=text)

    for name, pat in _PATTERNS:
        if pat.search(text):
            return Intent(name=name, text=text, speak=_ack_for(name))

    return Intent(name="chat", text=text)


def _ack_for(name: str) -> Optional[str]:
    return {
        "stop": "Stopping.",
        "forward": "Moving forward.",
        "backward": "Backing up.",
        "turn_left": "Turning left.",
        "turn_right": "Turning right.",
        "start_exploring": "Starting to explore.",
        "stop_exploring": "Pausing exploration.",
        "reset_history": "History cleared.",
        "see": None,  # vision speaks the result itself
    }.get(name)

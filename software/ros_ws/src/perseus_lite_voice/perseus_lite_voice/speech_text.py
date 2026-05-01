"""Text helpers shared by Speaker and the LLM streaming pipeline.

Ported from ollama_voice/chat.py:46-62 and chat.py:732-743.
"""

from __future__ import annotations

import re

SENTENCE_END = re.compile(r"([.!?]+[\")\]]?\s+|\n{2,})")


def clean_for_speech(text: str) -> str:
    """Strip markdown noise that sounds awful spoken."""
    text = re.sub(r"```.*?```", " ", text, flags=re.DOTALL)
    text = re.sub(r"`([^`]*)`", r"\1", text)
    text = re.sub(r"\*\*(.*?)\*\*", r"\1", text)
    text = re.sub(r"(?<!\*)\*(?!\*)([^*\n]+)\*(?!\*)", r"\1", text)
    text = re.sub(r"^\s*[-*]\s+", "", text, flags=re.MULTILINE)
    text = re.sub(r"#{1,6}\s+", "", text)
    return text.strip()


def split_sentences(buffer: str) -> tuple[list[str], str]:
    """Split a buffer into completed sentences plus a leftover tail."""
    sentences: list[str] = []
    last_end = 0
    for m in SENTENCE_END.finditer(buffer):
        sentences.append(buffer[last_end : m.end()].strip())
        last_end = m.end()
    return [s for s in sentences if s], buffer[last_end:]


def parse_volume(s: str) -> float:
    """Accept '80', '80%', or '0.8' and return a Piper multiplier (0.0-2.0)."""
    s = s.strip().rstrip("%")
    v = float(s)
    if v > 2:
        v /= 100.0
    if v < 0:
        v = 0.0
    if v > 2.0:
        v = 2.0
    return v

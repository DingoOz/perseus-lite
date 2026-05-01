"""Ollama HTTP streaming client. Ported from ollama_voice/chat.py:705-719."""

from __future__ import annotations

import json
from typing import Iterator

import requests


def stream_chat(
    url: str, model: str, messages: list[dict], timeout_s: int = 600
) -> Iterator[str]:
    """Yield text chunks streamed from the Ollama chat endpoint."""
    payload = {"model": model, "messages": messages, "stream": True}
    with requests.post(url, json=payload, stream=True, timeout=timeout_s) as r:
        r.raise_for_status()
        for line in r.iter_lines(decode_unicode=True):
            if not line:
                continue
            data = json.loads(line)
            if "message" in data and "content" in data["message"]:
                chunk = data["message"]["content"]
                if chunk:
                    yield chunk
            if data.get("done"):
                return

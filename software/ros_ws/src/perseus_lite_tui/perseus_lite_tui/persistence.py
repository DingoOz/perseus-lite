# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Best-effort JSON persistence of option selections and live jobs (pure)."""

import json
import os
from pathlib import Path
from typing import Dict, Mapping, Optional

APP_ID = 'perseus_lite_tui'
FILENAME = 'selections.json'
JOBS_FILENAME = 'running_jobs.json'

# JSON scalars accepted as an option value (coerced to str); anything else in a
# hand-edited/corrupted selections file is dropped rather than crashing render.
_SCALARS = (str, bool, int, float)


def config_dir(env: Optional[Mapping[str, str]] = None) -> Path:
    """Return the XDG config directory for the TUI's state file."""
    env = os.environ if env is None else env
    base = env.get('XDG_CONFIG_HOME') or os.path.join(env.get('HOME', ''), '.config')
    return Path(base) / APP_ID


def state_path(env: Optional[Mapping[str, str]] = None) -> Path:
    """Return the full path to the selections state file."""
    return config_dir(env) / FILENAME


def jobs_path(env: Optional[Mapping[str, str]] = None) -> Path:
    """Return the path of the running-jobs (orphan detection) state file."""
    return config_dir(env) / JOBS_FILENAME


def _read_json(path: Path):
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def sanitize_selections(data) -> Dict[str, Dict[str, str]]:
    """Coerce loaded JSON into {profile: {option: str}}, dropping bad entries."""
    if not isinstance(data, dict):
        return {}
    clean: Dict[str, Dict[str, str]] = {}
    for slug, options in data.items():
        if not isinstance(slug, str) or not isinstance(options, dict):
            continue
        values = {
            str(key): str(value)
            for key, value in options.items()
            if isinstance(key, str) and isinstance(value, _SCALARS)
        }
        if values:
            clean[slug] = values
    return clean


def load_selections(path: Path) -> Dict[str, Dict[str, str]]:
    """Load saved selections; return an empty mapping on any failure."""
    return sanitize_selections(_read_json(path))


def save_selections(path: Path, data: Mapping[str, Dict[str, str]]) -> bool:
    """Atomically write selections; never raise, return success as a bool."""
    return _write_json(path, dict(data))


def boot_id() -> str:
    """Return the kernel boot id ('' if unavailable) to scope recorded pgids."""
    try:
        with open('/proc/sys/kernel/random/boot_id', 'r', encoding='ascii') as handle:
            return handle.read().strip()
    except OSError:
        return ''


def load_running_jobs(
    path: Path, current_boot: Optional[str] = None
) -> Dict[str, dict]:
    """
    Load jobs a previous TUI run recorded as live.

    Returns {key: {'pgid': int, 'label': str, 'resources': [str, ...]}}.
    Entries from a different boot (pgids are meaningless across a reboot) or
    with a malformed shape are dropped.
    """
    data = _read_json(path)
    if not isinstance(data, dict):
        return {}
    current_boot = boot_id() if current_boot is None else current_boot
    if data.get('boot_id', '') != current_boot:
        return {}
    jobs = data.get('jobs')
    if not isinstance(jobs, dict):
        return {}
    clean: Dict[str, dict] = {}
    for key, entry in jobs.items():
        if not isinstance(key, str) or not isinstance(entry, dict):
            continue
        pgid = entry.get('pgid')
        if isinstance(pgid, bool) or not isinstance(pgid, int) or pgid <= 1:
            continue
        resources = entry.get('resources', [])
        if not isinstance(resources, list):
            resources = []
        clean[key] = {
            'pgid': pgid,
            'label': str(entry.get('label', key)),
            'resources': [str(r) for r in resources if isinstance(r, str)],
        }
    return clean


def save_running_jobs(
    path: Path, jobs: Mapping[str, Mapping], current_boot: Optional[str] = None
) -> bool:
    """
    Record live job process groups; remove the file when there are none.

    `jobs` maps key -> {'pgid', 'label', 'resources'} (same shape as loaded).
    """
    if not jobs:
        try:
            Path(path).unlink()
        except FileNotFoundError:
            pass
        except OSError:
            return False
        return True
    payload = {
        'boot_id': boot_id() if current_boot is None else current_boot,
        'jobs': {
            key: {
                'pgid': int(entry['pgid']),
                'label': str(entry.get('label', key)),
                'resources': [str(r) for r in entry.get('resources', ())],
            }
            for key, entry in jobs.items()
        },
    }
    return _write_json(path, payload)


def _write_json(path: Path, payload) -> bool:
    path = Path(path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + '.tmp')
        with open(tmp, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
        os.replace(tmp, path)
        return True
    except OSError:
        return False

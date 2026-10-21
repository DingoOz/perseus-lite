# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Pure mission-progress logic for the orchestrator (no rclpy).

A mission is an ordered list of segments; the ``edit`` segments are the
challenges. ``MissionProgress`` owns the rules that keep the challenges in
order: a challenge only counts once every earlier challenge has passed, so
e.g. manually driving to the leak during the first drive segment cannot pass
Challenge 2 and skip Challenge 1.
"""

from dataclasses import dataclass


def find_segment_index(segments: list, target_id) -> int | None:
    """Return the index of the segment with the given id, or None."""
    for idx, seg in enumerate(segments):
        if isinstance(seg, dict) and seg.get('id') == target_id:
            return idx
    return None


def merge_unlocked_badges(existing, badge_ids: list) -> dict:
    """Merge badge ids into a badges document, preserving prior unlocks."""
    if not isinstance(existing, dict):
        existing = {}
    if not isinstance(existing.get('unlocked'), list):
        existing['unlocked'] = []
    for bid in badge_ids:
        if bid not in existing['unlocked']:
            existing['unlocked'].append(bid)
    return existing


def challenge_segment_indices(segments: list) -> list:
    """Return the indices of the ``edit`` (challenge) segments, in order."""
    return [
        idx
        for idx, seg in enumerate(segments)
        if isinstance(seg, dict) and seg.get('kind') == 'edit'
    ]


@dataclass(frozen=True)
class Decision:
    """Whether an incoming checker event was accepted, and why."""

    accepted: bool
    reason: str


class MissionProgress:
    """Ordered challenge bookkeeping for one mission run."""

    def __init__(self, segments: list):
        self.segments = [s for s in (segments or []) if isinstance(s, dict)]
        self.challenge_indices = challenge_segment_indices(self.segments)
        self.done = [False] * len(self.challenge_indices)
        self.params_fixed = [False] * len(self.challenge_indices)
        self.current_idx = 0

    @property
    def current_segment(self) -> dict | None:
        """Return the segment the viewer is currently in, if any."""
        if 0 <= self.current_idx < len(self.segments):
            return self.segments[self.current_idx]
        return None

    @property
    def complete(self) -> bool:
        """Return True once every challenge has passed."""
        return bool(self.done) and all(self.done)

    def challenge_segment(self, challenge: int) -> dict:
        """Return the edit segment for a 0-based challenge number."""
        return self.segments[self.challenge_indices[challenge]]

    def needs_params_fix(self, challenge: int) -> bool:
        """Return True when a challenge has a separate params-fixed check."""
        return bool(self.challenge_segment(challenge).get('params_checker_topic'))

    def active_challenge(self) -> int | None:
        """Return the first challenge not yet passed, or None when complete."""
        for challenge, done in enumerate(self.done):
            if not done:
                return challenge
        return None

    def _gate(self, challenge: int) -> Decision:
        if not 0 <= challenge < len(self.done):
            return Decision(False, f'mission has no challenge {challenge + 1}')
        if self.done[challenge]:
            return Decision(False, f'challenge {challenge + 1} already passed')
        active = self.active_challenge()
        if active != challenge:
            return Decision(
                False,
                f'challenge {challenge + 1} is out of order: '
                f'challenge {active + 1} has not passed yet',
            )
        return Decision(True, 'ok')

    def on_params_fixed(self, challenge: int) -> Decision:
        """
        Record that a challenge's config file now passes its params check.

        Accepted at most once per challenge, and only while that challenge
        is the active one (so the stack restart it triggers happens in
        mission order).
        """
        decision = self._gate(challenge)
        if not decision.accepted:
            return decision
        if self.params_fixed[challenge]:
            return Decision(False, f'challenge {challenge + 1} params already fixed')
        self.params_fixed[challenge] = True
        return Decision(True, 'ok')

    def on_challenge_passed(self, challenge: int) -> Decision:
        """
        Record a challenge's checker reporting success.

        Accepted only for the active challenge, and -- when the challenge
        has a separate params check (``params_checker_topic``) -- only after
        that check passed. On acceptance the viewer advances to the segment
        after that challenge's edit segment.
        """
        decision = self._gate(challenge)
        if not decision.accepted:
            return decision
        if self.needs_params_fix(challenge) and not self.params_fixed[challenge]:
            return Decision(
                False,
                f'challenge {challenge + 1} config has not been fixed yet',
            )
        self.done[challenge] = True
        self.current_idx = min(
            self.challenge_indices[challenge] + 1, max(len(self.segments) - 1, 0)
        )
        return Decision(True, 'ok')

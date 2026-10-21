# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for the pure mission logic (no ROS graph required)."""

from pathlib import Path

from perseus_lite_missions.checkers.config_validation import (
    check_nav2_params,
    check_slam_params,
    evaluate_slam_params,
    FAIL,
    INVALID,
    load_yaml_file,
    PASS,
)
from perseus_lite_missions.checkers.goal_reached_node import (
    latest_goal_ok,
    leak_from_mission,
    within_tolerance,
)
from perseus_lite_missions.mission_logic import (
    challenge_segment_indices,
    find_segment_index,
    merge_unlocked_badges,
    MissionProgress,
)
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
MISSION_DIR = PKG / 'missions' / 'm00-cold-open'
CONFIGS = MISSION_DIR / 'configs'

# Defaults used by mission_zero.launch.py / the checker nodes.
SLAM_THRESHOLDS = (10.0, 8.0)
NAV2_THRESHOLDS = (0.35, 0.2, 0.35)


def _slam_doc(resolution, max_laser_range):
    return {
        'slam_toolbox': {
            'ros__parameters': {
                'resolution': resolution,
                'max_laser_range': max_laser_range,
            }
        }
    }


def _load(name):
    doc, error = load_yaml_file(CONFIGS / name)
    assert error is None
    return doc


class TestEvaluateSlamParams:
    def test_reference_values_pass(self):
        assert evaluate_slam_params(_slam_doc(0.05, 12.0), 10.0, 8.0)

    def test_broken_starting_values_fail(self):
        assert not evaluate_slam_params(_slam_doc(0.30, 3.0), 10.0, 8.0)

    def test_threshold_boundary_passes(self):
        assert evaluate_slam_params(_slam_doc(0.1, 8.0), 10.0, 8.0)

    def test_coarse_resolution_fails(self):
        assert not evaluate_slam_params(_slam_doc(0.2, 12.0), 10.0, 8.0)

    def test_short_range_fails(self):
        assert not evaluate_slam_params(_slam_doc(0.05, 3.0), 10.0, 8.0)

    def test_zero_resolution_fails(self):
        assert not evaluate_slam_params(_slam_doc(0.0, 12.0), 10.0, 8.0)

    def test_missing_keys_fail(self):
        assert not evaluate_slam_params({}, 10.0, 8.0)
        assert not evaluate_slam_params(None, 10.0, 8.0)
        assert not evaluate_slam_params(
            {'slam_toolbox': {'ros__parameters': {'resolution': 0.05}}}, 10.0, 8.0
        )


class TestSlamConfigValidation:
    """Malformed student edits must be reported as invalid, never raise."""

    @pytest.mark.parametrize(
        'doc',
        [
            _slam_doc('0.05', 12.0),  # quoted number (used to raise TypeError)
            _slam_doc(0.05, '12'),
            _slam_doc(True, 12.0),
            _slam_doc(float('nan'), 12.0),
            _slam_doc(None, 12.0),
            ['slam_toolbox'],  # list document (used to raise AttributeError)
            {'slam_toolbox': None},  # emptied block
            {'slam_toolbox': {'ros__parameters': None}},
            {'slam_toolbox': {'ros__parameters': []}},
            {'slam_toolbox': 'oops'},
            'just a string',
            42,
            None,
        ],
    )
    def test_malformed_is_invalid(self, doc):
        result = check_slam_params(doc, *SLAM_THRESHOLDS)
        assert result.status == INVALID
        assert result.reason

    def test_negative_resolution_invalid(self):
        assert check_slam_params(_slam_doc(-0.05, 12.0), *SLAM_THRESHOLDS).status == (
            INVALID
        )

    def test_wrong_values_fail_with_reason(self):
        result = check_slam_params(_slam_doc(0.3, 12.0), *SLAM_THRESHOLDS)
        assert result.status == FAIL
        assert 'resolution' in result.reason

    def test_shipped_files(self):
        broken = check_slam_params(_load('slam_params.broken.yaml'), *SLAM_THRESHOLDS)
        ref = check_slam_params(_load('slam_params.reference.yaml'), *SLAM_THRESHOLDS)
        assert broken.status == FAIL
        assert ref.status == PASS


class TestLoadYamlFile:
    def test_missing_file(self, tmp_path):
        doc, error = load_yaml_file(tmp_path / 'nope.yaml')
        assert doc is None
        assert 'cannot read' in error

    def test_directory(self, tmp_path):
        doc, error = load_yaml_file(tmp_path)
        assert doc is None and error

    def test_syntax_error(self, tmp_path):
        path = tmp_path / 'bad.yaml'
        path.write_text('slam_toolbox: [unclosed\n')
        doc, error = load_yaml_file(path)
        assert doc is None
        assert 'YAML' in error

    def test_not_utf8(self, tmp_path):
        path = tmp_path / 'bin.yaml'
        path.write_bytes(b'\xff\xfe\x00bad')
        doc, error = load_yaml_file(path)
        assert doc is None and error


def _nav2_doc(local=None, global_=None):
    def costmap(overrides):
        params = {
            'robot_radius': 0.28,
            'plugins': ['obstacle_layer', 'inflation_layer'],
            'inflation_layer': {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'inflation_radius': 0.25,
            },
        }
        params.update(overrides or {})
        return params

    return {
        'local_costmap': {'local_costmap': {'ros__parameters': costmap(local)}},
        'global_costmap': {'global_costmap': {'ros__parameters': costmap(global_)}},
    }


class TestNav2ConfigValidation:
    def test_shipped_broken_fails(self):
        result = check_nav2_params(_load('nav2_params.broken.yaml'), *NAV2_THRESHOLDS)
        assert result.status == FAIL

    def test_shipped_reference_passes(self):
        result = check_nav2_params(
            _load('nav2_params.reference.yaml'), *NAV2_THRESHOLDS
        )
        assert result.status == PASS, result.reason

    def test_hinted_answer_passes(self):
        assert check_nav2_params(_nav2_doc(), *NAV2_THRESHOLDS).passed

    def test_one_costmap_still_broken_fails(self):
        doc = _nav2_doc(global_={'robot_radius': 0.85})
        assert check_nav2_params(doc, *NAV2_THRESHOLDS).status == FAIL

    def test_inflation_still_too_wide_fails(self):
        doc = _nav2_doc(
            local={
                'inflation_layer': {
                    'plugin': 'nav2_costmap_2d::InflationLayer',
                    'inflation_radius': 0.75,
                }
            }
        )
        assert check_nav2_params(doc, *NAV2_THRESHOLDS).status == FAIL

    def test_shrinking_robot_to_nothing_fails(self):
        doc = _nav2_doc(local={'robot_radius': 0.01}, global_={'robot_radius': 0.01})
        assert check_nav2_params(doc, *NAV2_THRESHOLDS).status == FAIL

    def test_polygon_footprint_overrides_radius(self):
        fp = '[[0.21, 0.22], [0.21, -0.22], [-0.42, -0.22], [-0.42, 0.22]]'
        doc = _nav2_doc(
            local={'footprint': fp, 'robot_radius': 0.85},
            global_={'footprint': fp, 'robot_radius': 0.85},
        )
        assert check_nav2_params(doc, *NAV2_THRESHOLDS).passed

    def test_empty_footprint_falls_back_to_radius(self):
        doc = _nav2_doc(local={'footprint': '[]', 'robot_radius': 0.85})
        assert check_nav2_params(doc, *NAV2_THRESHOLDS).status == FAIL

    @pytest.mark.parametrize(
        'local',
        [
            {'robot_radius': '0.28'},
            {'footprint': '[[0.2, 0.2], [0.2]]'},
            {'footprint': '[[0.2, 0.2], [0.2, -0.2'},
            {'footprint': 'not a list'},
            {'plugins': 'inflation_layer'},
            {'plugins': ['obstacle_layer']},
            {'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer'}},
            {
                'inflation_layer': {
                    'plugin': 'nav2_costmap_2d::InflationLayer',
                    'inflation_radius': -1.0,
                }
            },
        ],
    )
    def test_malformed_is_invalid(self, local):
        result = check_nav2_params(_nav2_doc(local=local), *NAV2_THRESHOLDS)
        assert result.status == INVALID, result

    def test_missing_size_is_invalid(self):
        doc = _nav2_doc()
        del doc['local_costmap']['local_costmap']['ros__parameters']['robot_radius']
        assert check_nav2_params(doc, *NAV2_THRESHOLDS).status == INVALID

    @pytest.mark.parametrize('doc', [None, [], 'x', {'local_costmap': []}])
    def test_wrong_shape_is_invalid(self, doc):
        assert check_nav2_params(doc, *NAV2_THRESHOLDS).status == INVALID


class TestShippedConfigsAgreeWithContract:
    """Frames/topics must match the lite robot (base_link, /scan_filtered)."""

    @pytest.mark.parametrize(
        'name',
        [
            'slam_params.broken.yaml',
            'slam_params.reference.yaml',
            'nav2_params.broken.yaml',
            'nav2_params.reference.yaml',
        ],
    )
    def test_no_base_footprint_and_filtered_scan(self, name):
        text = (CONFIGS / name).read_text()
        assert 'base_footprint' not in text
        doc = yaml.safe_load(text)
        topics = []

        def walk(node):
            if isinstance(node, dict):
                for key, value in node.items():
                    if key in ('topic', 'scan_topic'):
                        topics.append(value)
                    walk(value)

        walk(doc)
        assert topics
        assert all(t == '/scan_filtered' for t in topics), topics


class TestFindSegmentIndex:
    SEGMENTS = [{'id': 1, 'title': 'a'}, {'id': 5, 'title': 'b'}, {'id': 8}]

    def test_finds_segment(self):
        assert find_segment_index(self.SEGMENTS, 5) == 1
        assert find_segment_index(self.SEGMENTS, 8) == 2

    def test_missing_segment_returns_none(self):
        assert find_segment_index(self.SEGMENTS, 99) is None
        assert find_segment_index([], 1) is None


class TestMergeUnlockedBadges:
    def test_merge_into_empty(self):
        out = merge_unlocked_badges({}, ['a', 'b'])
        assert out['unlocked'] == ['a', 'b']

    def test_preserves_existing_and_dedupes(self):
        out = merge_unlocked_badges({'unlocked': ['a']}, ['a', 'b'])
        assert out['unlocked'] == ['a', 'b']

    def test_non_dict_input_is_reset(self):
        out = merge_unlocked_badges(['corrupt'], ['a'])
        assert out['unlocked'] == ['a']

    def test_non_list_unlocked_is_reset(self):
        out = merge_unlocked_badges({'unlocked': 'a'}, ['b'])
        assert out['unlocked'] == ['b']

    def test_preserves_unrelated_keys(self):
        out = merge_unlocked_badges({'other': 1, 'unlocked': []}, ['a'])
        assert out['other'] == 1


def _mission_segments():
    doc, error = load_yaml_file(MISSION_DIR / 'mission.yaml')
    assert error is None
    return doc['segments']


class TestMissionProgress:
    """Challenges count only in mission order (review finding: C2 skip)."""

    def test_shipped_mission_has_two_ordered_challenges(self):
        segments = _mission_segments()
        idx = challenge_segment_indices(segments)
        assert [segments[i]['id'] for i in idx] == [4, 7]
        assert segments[idx[0]]['restart_stack'] == 'slam'
        assert segments[idx[1]]['restart_stack'] == 'nav2'
        assert segments[idx[1]]['params_checker_topic']

    def test_challenge_2_before_challenge_1_is_rejected(self):
        # Driving to the leak during the segment-2 drive must not pass C2.
        progress = MissionProgress(_mission_segments())
        decision = progress.on_challenge_passed(1)
        assert not decision.accepted
        assert 'out of order' in decision.reason
        assert progress.done == [False, False]
        assert progress.current_idx == 0

    def test_params_fix_before_challenge_1_is_rejected(self):
        progress = MissionProgress(_mission_segments())
        assert not progress.on_params_fixed(1).accepted
        assert progress.params_fixed == [False, False]

    def test_challenge_2_needs_params_fix(self):
        progress = MissionProgress(_mission_segments())
        assert progress.on_challenge_passed(0).accepted
        decision = progress.on_challenge_passed(1)
        assert not decision.accepted
        assert 'not been fixed' in decision.reason

    def test_full_mission_in_order(self):
        segments = _mission_segments()
        progress = MissionProgress(segments)
        assert progress.on_challenge_passed(0).accepted
        assert progress.current_segment['id'] == 5
        assert progress.active_challenge() == 1
        assert progress.on_params_fixed(1).accepted
        assert not progress.on_params_fixed(1).accepted  # restart only once
        assert progress.on_challenge_passed(1).accepted
        assert progress.current_segment['id'] == 8
        assert progress.complete
        assert progress.active_challenge() is None

    def test_repeat_pass_is_rejected(self):
        progress = MissionProgress(_mission_segments())
        assert progress.on_challenge_passed(0).accepted
        assert not progress.on_challenge_passed(0).accepted

    def test_unknown_challenge_is_rejected(self):
        progress = MissionProgress(_mission_segments())
        assert not progress.on_challenge_passed(5).accepted
        assert not progress.on_challenge_passed(-1).accepted

    def test_empty_or_garbage_segments(self):
        progress = MissionProgress(None)
        assert progress.current_segment is None
        assert not progress.complete
        assert not progress.on_challenge_passed(0).accepted
        progress = MissionProgress(['junk', {'id': 1, 'kind': 'edit'}])
        assert progress.on_challenge_passed(0).accepted
        assert progress.complete


class TestGoalReachedHelpers:
    def test_latest_goal_must_be_running_or_succeeded(self):
        executing, succeeded, aborted, canceled = 2, 4, 6, 5
        assert not latest_goal_ok([])
        assert latest_goal_ok([(1, executing)])
        assert latest_goal_ok([(1, aborted), (2, succeeded)])
        assert not latest_goal_ok([(2, succeeded), (3, canceled)])
        assert not latest_goal_ok([(5, aborted), (1, executing)])

    def test_leak_from_mission(self):
        doc, _ = load_yaml_file(MISSION_DIR / 'mission.yaml')
        assert leak_from_mission(doc) == (9.8, 0.0)
        assert leak_from_mission(None) is None
        assert leak_from_mission({'poses': []}) is None
        assert leak_from_mission({'poses': {'leak': {'x': '1', 'y': 0}}}) is None
        assert leak_from_mission({'poses': {'leak': {'x': 1}}}) is None

    def test_within_tolerance(self):
        assert within_tolerance((9.4, 0.1), (9.8, 0.0), 0.5)
        assert not within_tolerance((9.0, 0.0), (9.8, 0.0), 0.5)

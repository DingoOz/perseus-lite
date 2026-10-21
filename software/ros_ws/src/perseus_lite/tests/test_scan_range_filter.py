import importlib.util
import math
import pathlib

import pytest

_SCRIPT = (
    pathlib.Path(__file__).resolve().parents[1] / "scripts" / "scan_range_filter.py"
)
_spec = importlib.util.spec_from_file_location("scan_range_filter", _SCRIPT)
scan_range_filter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scan_range_filter)


def test_readings_below_min_range_become_inf():
    out = scan_range_filter.filter_ranges([0.1, 0.49, 0.5, 2.0], 0.5)
    assert out[:2] == [math.inf, math.inf]
    assert out[2:] == [0.5, 2.0]


def test_nan_and_inf_become_inf():
    out = scan_range_filter.filter_ranges([math.nan, math.inf], 0.5)
    assert out == [math.inf, math.inf]


@pytest.mark.parametrize("min_range", [0.0, 0.3])
def test_valid_readings_pass_through(min_range):
    assert scan_range_filter.filter_ranges([0.3, 1.0], min_range) == [0.3, 1.0]

# Copyright (c) 2025 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Tests for small-terminal drawing and the lite configuration."""

import curses
from pathlib import Path
from types import SimpleNamespace

from teleop_diagnostics import teleop_tui
import yaml


class _TinyScreen:
    """A fake curses window that raises like curses does off-screen."""

    def __init__(self, rows=20, cols=60, keys=(-1, -1, ord('q'))):
        self.rows, self.cols = rows, cols
        self.keys = list(keys)
        self.refreshes = 0
        self.getch_calls = 0

    def getmaxyx(self):
        return self.rows, self.cols

    def addstr(self, y, x, text, attr=0):
        if y < 0 or y >= self.rows or x < 0 or x + len(text) > self.cols:
            raise curses.error('addwstr() returned ERR')

    def erase(self):
        pass

    def timeout(self, _ms):
        pass

    def refresh(self):
        self.refreshes += 1

    def getch(self):
        self.getch_calls += 1
        return self.keys.pop(0) if self.keys else ord('q')


def _fake_node():
    return SimpleNamespace(
        joy_data=teleop_tui.JoyData(axes=[0.0] * 8, buttons=[0] * 12),
        controller_config=teleop_tui.ControllerConfig(),
        joy_vel_data=teleop_tui.VelocityData(),
        cmd_vel_out_data=teleop_tui.VelocityData(linear_x=0.5),
        nav_vel_data=teleop_tui.VelocityData(),
        wheel_data=teleop_tui.WheelData(names=['fl_wheel'], velocities=[1.0]),
        wheel_separation=teleop_tui.DEFAULT_WHEEL_SEPARATION,
        wheel_radius=teleop_tui.DEFAULT_WHEEL_RADIUS,
        mux_priorities=dict(teleop_tui.DEFAULT_MUX_PRIORITIES),
        mux_sources=dict.fromkeys(teleop_tui.DEFAULT_MUX_PRIORITIES, 0.0),
        active_mux_source='none',
    )


def test_small_terminal_still_handles_quit(monkeypatch):
    monkeypatch.setattr(teleop_tui.curses, 'curs_set', lambda _v: None)
    tui = teleop_tui.TeleopTUI(_fake_node())
    screen = _TinyScreen()
    tui.run_curses(screen)  # must return on 'q' instead of spinning forever
    assert tui.running is False
    assert screen.getch_calls == 3
    assert screen.refreshes == 3


def test_draw_frame_never_raises_on_tiny_terminal():
    tui = teleop_tui.TeleopTUI(_fake_node())
    tui.stdscr = _TinyScreen(rows=5, cols=10)
    tui.draw_frame()
    tui.draw_box(3, 8, 10, 40, 'title')


def test_lite_wheel_geometry_defaults_and_yaml():
    assert teleop_tui.DEFAULT_WHEEL_RADIUS == 0.0725
    assert teleop_tui.DEFAULT_WHEEL_SEPARATION == 0.8286
    config = Path(__file__).resolve().parents[1] / 'config' / 'teleop_diagnostics.yaml'
    params = yaml.safe_load(config.read_text())['teleop_diagnostics']['ros__parameters']
    assert params['wheel']['radius'] == 0.0725
    assert params['wheel']['separation'] == 0.8286
    assert params['mux_priorities'] == teleop_tui.DEFAULT_MUX_PRIORITIES


def test_launch_file_loads_the_yaml():
    launch = Path(__file__).resolve().parents[1] / 'launch'
    text = (launch / 'teleop_diagnostics.launch.py').read_text()
    assert 'teleop_diagnostics.yaml' in text
    assert 'parameters=[config]' in text

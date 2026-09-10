#!/usr/bin/env python3
"""Shared GRASP_WAIT joint presets for automatic and manual arm tools.

Only shoulder/elbow/wrist are stored: the automatic entry motion deliberately
keeps the live ``base_joint`` position to avoid a sideways sweep.
"""

from __future__ import annotations


DEFAULT_GRASP_WAIT_PRESET = "legacy"

# Duplicate captures were averaged so one repeatable candidate represents each
# requested test pose.  ``legacy`` is a right-angle (직각) shoulder/elbow/wrist
# pose captured via capture_arm_pose.py.
GRASP_WAIT_PRESETS: dict[str, dict[str, float]] = {
    "legacy": {
        "shoulder_joint": -1.52914,
        "elbow_joint": 1.66749,
        "wrist_joint": 1.64125,
    },
    "grasp_wait1": {
        "shoulder_joint": 0.35133,
        "elbow_joint": 1.346955,
        "wrist_joint": 1.181585,
    },
    "grasp_wait2": {
        "shoulder_joint": 0.34296,
        "elbow_joint": 1.50238,
        "wrist_joint": 1.08402,
    },
    "grasp_wait3": {
        "shoulder_joint": 0.55816,
        "elbow_joint": 1.24477,
        "wrist_joint": 1.08350,
    },
    "grasp_wait4": {
        "shoulder_joint": 0.44244,
        "elbow_joint": 1.28718,
        "wrist_joint": 1.18246,
    },
    "grasp_wait5": {
        "shoulder_joint": 0.52735,
        "elbow_joint": 1.190575,
        "wrist_joint": 1.16187,
    },
}


def get_grasp_wait_preset(name: str) -> dict[str, float]:
    """Return a copy of a named preset or raise a diagnostic ValueError."""
    normalized = str(name).strip().lower()
    try:
        return dict(GRASP_WAIT_PRESETS[normalized])
    except KeyError as exc:
        choices = ", ".join(GRASP_WAIT_PRESETS)
        raise ValueError(
            f"unknown grasp_wait_preset {name!r}; choose one of: {choices}"
        ) from exc

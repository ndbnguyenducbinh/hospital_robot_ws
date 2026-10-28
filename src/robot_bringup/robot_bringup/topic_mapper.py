"""Canonical Topic Mapper for Hospital Reception Robot.

Resolves dynamic topic names from config/topic_map.yaml to prevent hardcoding.
"""

from typing import Any, Optional
import yaml


class TopicMapper:
    """Helper to dynamically resolve canonical ROS 2 topic paths from topic_map.yaml."""

    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            self._map = yaml.safe_load(f) or {}

    def resolve(self, key_path: str) -> Optional[str]:
        keys = key_path.split('.')
        val: Any = self._map
        for k in keys:
            if isinstance(val, dict) and k in val:
                val = val[k]
            else:
                raise KeyError(f"Topic key '{key_path}' not found in topic_map.yaml")
        return val


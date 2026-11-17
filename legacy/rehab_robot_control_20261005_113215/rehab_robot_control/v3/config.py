from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict
import os
import yaml


class ConfigError(RuntimeError):
    pass


def _load_yaml(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise ConfigError(f"missing config: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ConfigError(f"invalid YAML mapping: {path}")
    return data


def expand_path(value: str) -> str:
    return os.path.expandvars(os.path.expanduser(value))


@dataclass
class V3Config:
    root: Path
    task: Dict[str, Any]
    safety: Dict[str, Any]
    hardware: Dict[str, Any]

    @classmethod
    def load(cls, root: Path) -> "V3Config":
        root = Path(root).resolve()
        cfg = root / "config"
        obj = cls(
            root=root,
            task=_load_yaml(cfg / "rehab_v3_task.yaml"),
            safety=_load_yaml(cfg / "rehab_v3_safety.yaml"),
            hardware=_load_yaml(cfg / "rehab_v3_hardware.yaml"),
        )
        obj.validate()
        return obj

    def validate(self) -> None:
        order = self.task.get("route_order")
        points = self.task.get("teach_points")
        if not isinstance(order, list) or len(order) < 2:
            raise ConfigError("route_order must contain >= 2 points")
        if not isinstance(points, dict):
            raise ConfigError("teach_points missing")
        for name in order:
            p = points.get(name)
            if not isinstance(p, list) or len(p) != 3:
                raise ConfigError(f"invalid teach point {name}: {p}")
        for key in ("approach_target", "rehab_start", "rehab_end", "return_target"):
            if self.task.get(key) not in points:
                raise ConfigError(f"{key} references unknown point")
        fp = self.safety.get("fault_policy", {})
        if fp.get("auto_retry") or fp.get("auto_return") or fp.get("auto_release") or fp.get("auto_error_recovery"):
            raise ConfigError("V3 production safety policy forbids automatic recovery actions")

    @property
    def robot_ip(self) -> str:
        return str(self.hardware["fr3"]["robot_ip"])

    @property
    def moveit_server(self) -> str:
        return expand_path(str(self.hardware["moveit"]["plan_only_server"]))

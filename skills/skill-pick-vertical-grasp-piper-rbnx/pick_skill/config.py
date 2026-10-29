# SPDX-License-Identifier: MulanPSL-2.0
"""配置读取：把 manifest 的 config 块读成 Python 值，并统一处理坏值。

**唯一职责**：类型转换 + 范围钳位 + 一句统一的告警。键不存在就返回默认值；
值坏就告警后返回默认值——**配置永远不该让 init 失败**（技能是 stage-2，init 挂了
整包起不来），坏值退化成默认值、由日志说明。

谁消费谁定义键名与默认值：各模块用这些读取器实现自己的 `from_cfg`
（`CameraConfig.from_cfg` / `DetectConfig.from_cfg` / `GripperConfig.from_cfg` /
`HeightConfig.from_cfg` / `PlaceConfig.from_cfg`），本模块不认识任何具体配置键。
"""
from __future__ import annotations

import logging
from typing import Any, Optional, Sequence

log = logging.getLogger("pick_skill.config")


def _warn(key: str, value: Any, why: str) -> None:
    log.warning("ignoring invalid %s: %r (%s)", key, value, why)


def num(cfg: dict, key: str, default: float,
        lo: Optional[float] = None, hi: Optional[float] = None) -> float:
    """取一个数；坏值告警后回落 default，可选按 lo/hi 钳位。"""
    if key not in cfg:
        return default
    try:
        value = float(cfg[key])
    except (TypeError, ValueError):
        _warn(key, cfg[key], "not a number")
        return default
    if lo is not None and value < lo:
        value = lo
    if hi is not None and value > hi:
        value = hi
    return value


def text(cfg: dict, key: str, default: str = "") -> str:
    """取一个字符串；空串与纯空白都算缺失。"""
    value = str(cfg.get(key) or "").strip()
    return value or default


def flag(cfg: dict, key: str, default: bool) -> bool:
    """取一个布尔。

    YAML 的 true/false 直接就是 bool；字符串按真值词解析（"false" 是假——直接
    bool("false") 会得到 True，这种写法是坑，这里拦住）。
    """
    if key not in cfg:
        return default
    value = cfg[key]
    if isinstance(value, bool):
        return value
    word = str(value).strip().lower()
    if word in ("1", "true", "yes", "on"):
        return True
    if word in ("0", "false", "no", "off"):
        return False
    _warn(key, value, "not a boolean")
    return default


def floats(cfg: dict, key: str, default: Optional[list[float]] = None) -> Optional[list[float]]:
    """取一个浮点数组（标定畸变系数之类）；坏值告警后回落 default。"""
    if key not in cfg:
        return default
    try:
        return [float(x) for x in cfg[key]]
    except (TypeError, ValueError):
        _warn(key, cfg[key], "not a list of numbers")
        return default


def matrix(cfg: dict, key: str, rows: int = 3, cols: int = 3) -> Optional[list[list[float]]]:
    """取一个 rows×cols 浮点矩阵（手眼矩阵 / 相机内参）；形状不对算坏值。"""
    if key not in cfg:
        return None
    try:
        parsed = [[float(x) for x in row] for row in cfg[key]]
    except (TypeError, ValueError):
        _warn(key, cfg[key], f"not a {rows}x{cols} matrix")
        return None
    if len(parsed) != rows or any(len(row) != cols for row in parsed):
        _warn(key, cfg[key], f"not a {rows}x{cols} matrix")
        return None
    return parsed


def xy(cfg: dict, key: str, default: Optional[list[float]] = None) -> Optional[list[float]]:
    """取一个 [x, y] 点；坏值告警后回落 default。"""
    if key not in cfg:
        return default
    try:
        pair = cfg[key]
        return [float(pair[0]), float(pair[1])]
    except (TypeError, ValueError, IndexError):
        _warn(key, cfg[key], "not an [x, y] pair")
        return default


def xy_map(cfg: dict, key: str) -> dict[str, list[float]]:
    """取 {类名: [x, y]} 映射；单个条目坏掉只丢那一条，其余保留。"""
    raw = cfg.get(key)
    if not isinstance(raw, dict):
        if key in cfg:
            _warn(key, raw, "not a mapping of name -> [x, y]")
        return {}
    out: dict[str, list[float]] = {}
    for name, pair in raw.items():
        try:
            out[str(name)] = [float(pair[0]), float(pair[1])]
        except (TypeError, ValueError, IndexError):
            _warn(f"{key}[{name!r}]", pair, "not an [x, y] pair")
    return out


def choice(cfg: dict, key: str, options: Sequence[str], default: str) -> str:
    """取一个枚举值（大小写不敏感）；不在选项里就告警并回落 default。"""
    if key not in cfg:
        return default
    word = str(cfg[key]).strip().lower()
    if word in options:
        return word
    _warn(key, cfg[key], "expected one of " + ", ".join(repr(o) for o in options))
    return default


def pick_key(cfg: dict, *keys: str) -> Optional[str]:
    """按顺序返回第一个出现在配置里的键名（新键优先、旧键兼容时用）。"""
    for key in keys:
        if key in cfg:
            return key
    return None


def warn_deprecated(old: str, new: str) -> None:
    """旧键名仍被接受时，统一打一条弃用告警。"""
    log.warning("config key %r is deprecated — rename it to %r", old, new)

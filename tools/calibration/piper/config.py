# Description: 读取仓库 config/calibration/piper.yaml（每个部署一份：它认的是自己那份配置）。
from pathlib import Path
from typing import Any

import yaml

CONFIG_PATH = Path(__file__).resolve().parents[3] / "config" / "calibration" / "piper.yaml"


def load_config(config_file: str | Path) -> dict:
    """加载 YAML 配置文件。"""
    if not Path(config_file).exists():
        raise FileNotFoundError(f"配置文件未找到: {config_file}")

    with open(config_file, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


CONFIG = load_config(CONFIG_PATH)


def get_config_value(key: str, default=None, raise_if_missing: bool = True) -> Any:
    """取一个配置项。

    Args:
        key: 配置键名。
        default: 键不存在且不报错时返回的值。
        raise_if_missing: 键不存在时是否抛 KeyError。
    """
    if raise_if_missing and key not in CONFIG:
        raise KeyError(f"配置项未找到: {key}")
    return CONFIG.get(key, default)

"""Safety utilities for data validity checks."""

from typing import Dict, Any
import time


def data_is_stale(sample: Dict[str, Any], max_age_s: float = 0.1) -> bool:
    """Return True if sample is stale based on flag or timestamp age.

    If the dict has a boolean 'stale' we honor it. Otherwise, if it has a
    timestamp 'ts', mark stale if older than 'max_age_s'. Defaults to stale if
    neither is present.
    """
    if "stale" in sample:
        return bool(sample["stale"])
    if "ts" in sample:
        return (time.time() - float(sample["ts"])) > max_age_s
    return True

"""Pure helpers for resolving YOLO class names (no ROS / torch)."""

from __future__ import annotations

from typing import Mapping, Union

PERSON_CLASS = "person"


def resolve_person_class_ids(
    names: Mapping[Union[int, str], str],
) -> list[int]:
    """Return sorted class ids whose name is exactly ``person``."""
    if not names:
        return []
    person_ids: list[int] = []
    for key, value in names.items():
        if value == PERSON_CLASS:
            person_ids.append(int(key))
    return sorted(person_ids)

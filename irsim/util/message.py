"""Helpers for applying :mod:`irsim.msg` messages to simulation objects.

These functions resolve which local objects an incoming message addresses.
They only read the objects they are given, so a caller can validate a whole
batch of updates before applying any of them.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import Any

from irsim.msg import ObjectState, WorldState
from irsim.util.util import find_object_by_identity


def resolve_message_targets(
    objects: Iterable[Any],
    msg: WorldState | ObjectState | Any,
    object_name: str | None = None,
    object_id: int | None = None,
    *,
    default_target: Callable[[], Any],
) -> list[tuple[Any, Any]]:
    """
    Pair each incoming odometry with the distinct local object it addresses.

    A ``WorldState`` addresses every object it contains, an ``ObjectState``
    uses its embedded name and id, and a standalone odometry message goes to
    ``default_target`` unless a name or id selects another object.

    Args:
        objects (Iterable): Local objects the message can address.
        msg: A :class:`~irsim.msg.WorldState`, an
            :class:`~irsim.msg.ObjectState`, or an odometry-shaped message.
        object_name (str): Explicit target name for a standalone odometry or
            object message.
        object_id (int): Explicit target id for the same messages. Names take
            precedence when both are given.
        default_target (Callable): Called only when a standalone odometry
            selects no object; returns the object that receives it.

    Returns:
        list: ``(object, odometry)`` pairs, one per addressed object.

    Raises:
        ValueError: If a target cannot be found, a name or id is combined with
            a ``WorldState``, or one object is addressed more than once.
    """

    objects = list(objects)

    if isinstance(msg, WorldState):
        if object_name is not None or object_id is not None:
            raise ValueError(
                "object_name and object_id cannot be used with WorldState."
            )
        updates = [
            (_require_object(objects, obj_msg.name, obj_msg.id), obj_msg.odom)
            for obj_msg in msg.objects
        ]
    elif isinstance(msg, ObjectState):
        if object_name is None and object_id is None:
            object_name, object_id = msg.name, msg.id
        updates = [(_require_object(objects, object_name, object_id), msg.odom)]
    elif object_name is None and object_id is None:
        updates = [(default_target(), msg)]
    else:
        updates = [(_require_object(objects, object_name, object_id), msg)]

    seen: set[int] = set()
    for target, _ in updates:
        if id(target) in seen:
            raise ValueError(
                f"Received more than one state for object '{target.name}'."
            )
        seen.add(id(target))

    return updates


def _require_object(
    objects: list[Any], object_name: str | None, object_id: int | None
) -> Any:
    """Find an incoming message target, or report the identity that failed."""
    target = find_object_by_identity(objects, object_name, object_id)
    if target is not None:
        return target

    identity = (
        f"name={object_name!r}, id={object_id}"
        if object_name or object_id is not None
        else "without a name or id"
    )
    raise ValueError(f"No simulation object matches received message {identity}.")

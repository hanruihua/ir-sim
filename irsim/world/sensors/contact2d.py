from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D

from irsim.config import palette_param

if TYPE_CHECKING:
    from irsim.lib.algorithm.contact import Contact, ContactReport
    from irsim.world.object_base import ObjectBase


class Contact2D:
    """
    Contact sensor of one object: what the contact step recorded about it
    (``collision_mode: contact`` only), as a contact sensor mounted on the
    object reports.

    Every object carries one as ``obj.contact``, written by the environment
    and the contact solver, so its readings are always available:
    ``obj.contact.in_contact``, ``partners``, ``reports``, ``force``,
    ``contact_time``, ``air_time``, ``started`` and ``ended``.
    Listing it under the object's ``sensors`` as ``type: 'contact2d'`` makes
    that one the object's sensor and draws it: a marker at each contact
    point and a line along the contact normal whose length is the contact
    force times ``force_scale``.

    Args:
        state (np.ndarray): Initial state of the object ``[x, y, theta]``,
            kept for the common sensor interface; the readings do not
            depend on it.
        obj_id (int): ID of the associated object.
        force_scale (float): Length of the drawn force line per newton, in
            meters. Default 0.05.
        marker_size (float): Size of the contact point markers. Default 6.
        linewidth (float): Width of the force lines. Default 1.5.
        alpha (float): Transparency of the drawing. Default 0.9.
        **kwargs: ``plot`` sub-dict (preferred) with the visualization options
            above and ``color`` (default: the palette ``marker`` color), plus
            the ``name`` / ``type`` key of the YAML entry.

    Attributes:
        sensor_type (str): ``"contact2d"``.
        parent (ObjectBase): The object the sensor is mounted on.
        in_contact (bool): Whether a contact was resolved against ``parent``
            in the last step.
        partners (list): The objects ``parent`` touched in the last step,
            each once.
        records (list[Contact]): The contact records of the last step, in
            the solver's order.
        force (np.ndarray): Net world-frame contact force ``(2, 1)`` on
            ``parent`` in the last step, in newtons.
        contact_time (float): Seconds ``parent`` has been touching something.
        air_time (float): Seconds ``parent`` has been free; one of the two
            timers runs while the other is held at zero.
        started (bool): Whether the last step began a contact.
        ended (bool): Whether the last step ended one.
    """

    def __init__(
        self,
        state: np.ndarray | None = None,
        obj_id: int = 0,
        force_scale: float = 0.05,
        marker_size: float = 6.0,
        linewidth: float = 1.5,
        alpha: float = 0.9,
        **kwargs,
    ) -> None:
        self.sensor_type = "contact2d"
        self.obj_id = obj_id
        self._state = state
        self.parent: ObjectBase | None = None

        # Visualization params may be given under a `plot:` sub-dict
        # (preferred) or as flat top-level keys.
        _plot = kwargs.get("plot") or {}
        self.force_scale = _plot.get("force_scale", force_scale)
        self.marker_size = _plot.get("marker_size", marker_size)
        self.linewidth = _plot.get("linewidth", linewidth)
        self.alpha = _plot.get("alpha", alpha)
        self.color = _plot.get("color", kwargs.get("color", palette_param.marker))

        self.in_contact = False
        self.partners: list[Any] = []
        self.records: list[Contact] = []
        self.force = np.zeros((2, 1))
        self.contact_time = 0.0
        self.air_time = 0.0
        self.started = False
        self.ended = False
        self._was_in_contact = False

        self._point_artist = None
        self._force_artist = None

    # -- readings -----------------------------------------------------------

    @property
    def reports(self) -> list[ContactReport]:
        """Last step's contacts seen from ``parent``'s side."""
        return [contact.report(self.parent) for contact in self.records]

    # -- writes by the environment and the contact solver ------------------

    def clear(self) -> None:
        """Forget last step's contacts: flag, partners, records and force."""
        self.in_contact = False
        self.partners = []
        self.records = []
        self.force = np.zeros((2, 1))

    def reset(self) -> None:
        """Forget everything, the timers included, as at the start of a run."""
        self.clear()
        self.contact_time = 0.0
        self.air_time = 0.0
        self.started = False
        self.ended = False
        self._was_in_contact = False

    def add(self, contact: Contact) -> None:
        """Record a contact the solver resolved against ``parent``."""
        self.in_contact = True
        other = contact.b if contact.a is self.parent else contact.a
        if other not in self.partners:
            self.partners.append(other)
        self.records.append(contact)

    def add_force(self, force_xy: np.ndarray) -> None:
        """Accumulate a world-frame force ``(2,)`` or ``(2, 1)`` on ``parent``."""
        self.force = self.force + np.asarray(force_xy, dtype=float).reshape(2, 1)

    def tick(self, step_time: float) -> None:
        """Advance ``contact_time`` or ``air_time`` by one step and raise
        ``started`` or ``ended`` for a step that switched between them."""
        self.started = self.in_contact and not self._was_in_contact
        self.ended = self._was_in_contact and not self.in_contact
        self._was_in_contact = self.in_contact
        if self.in_contact:
            self.contact_time += step_time
            self.air_time = 0.0
        else:
            self.air_time += step_time
            self.contact_time = 0.0

    # -- sensor interface ----------------------------------------------------

    def step(self, state: np.ndarray) -> None:
        """Keep the sensor state; the readings are written by the contact step."""
        self._state = state

    def plot(self, ax, state: np.ndarray | None = None, **kwargs) -> None:
        """
        Draw the contact points and force lines on a given axis.

        Nothing is drawn on a 3D axis, since contacts are planar.
        """
        if isinstance(ax, Axes3D) or self._point_artist is not None:
            return
        (self._point_artist,) = ax.plot(
            [],
            [],
            linestyle="",
            marker="o",
            markersize=self.marker_size,
            color=self.color,
            alpha=self.alpha,
            zorder=4,
        )
        self._force_artist = LineCollection(
            [],
            colors=self.color,
            linewidths=self.linewidth,
            alpha=self.alpha,
            zorder=4,
        )
        ax.add_collection(self._force_artist)
        self.step_plot()

    def step_plot(self) -> None:
        """Move the markers and force lines to the last step's contacts."""
        if self._point_artist is None:
            return
        reports = self.reports
        self._point_artist.set_data(
            [r.point[0] for r in reports], [r.point[1] for r in reports]
        )
        self._force_artist.set_segments(
            [
                [r.point, r.point + r.normal * (r.force * self.force_scale)]
                for r in reports
            ]
        )

    def plot_clear(self) -> None:
        """Remove the drawing from the axis."""
        for artist in (self._point_artist, self._force_artist):
            if artist is not None:
                artist.remove()
        self._point_artist = None
        self._force_artist = None

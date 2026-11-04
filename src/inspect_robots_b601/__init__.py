"""Inspect Robots adapter for the Seeed reBot Arm B601-RS, driven directly over SocketCAN.

Registers one Inspect Robots component via entry point:

* embodiment ``b601`` -- :class:`~inspect_robots_b601.embodiment.B601Embodiment`

so ``inspect-robots run --embodiment b601 --policy agent --instruction ...`` works once the
package is installed. Use :func:`~inspect_robots_b601.preflight.run_preflight` (or the
``inspect-robots-b601-preflight`` CLI) to verify the embodiment before any motion, and the
``inspect-robots-b601`` CLI for bus checks, stop recovery and hardware runs.
"""

from __future__ import annotations

from inspect_robots_b601.embodiment import B601Embodiment
from inspect_robots_b601.preflight import run_preflight

try:
    from importlib.metadata import PackageNotFoundError
    from importlib.metadata import version as _pkg_version

    __version__ = _pkg_version("inspect-robots-rebot")
except PackageNotFoundError:  # Only in a source tree that is not installed.
    __version__ = "0.0.0+unknown"

__all__ = ["B601Embodiment", "__version__", "run_preflight"]

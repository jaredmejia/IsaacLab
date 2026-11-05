# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Newton Steel solver integration."""

from __future__ import annotations

from typing import TYPE_CHECKING

import warp as wp
from newton import Contacts, Model
from newton.solvers import SolverBase, SolverSteel

from .newton_solver import NewtonSolver

if TYPE_CHECKING:
    from .steel_solver_cfg import SteelSolverCfg


class SteelSolver(NewtonSolver):
    """Build and drive Newton's reduced-coordinate Steel solver."""

    solver_class = SolverSteel
    deterministic = True

    @classmethod
    def build(
        cls, cfg: SteelSolverCfg, model: Model, mode: wp.DeterministicMode, env_template: str | None = None
    ) -> SolverBase:
        """Construct Steel, first zeroing the authored torsion radius when ``cfg.contact_torsion`` is off."""
        if not cfg.contact_torsion:
            model.steel.min_torsional_patch_radius.zero_()
        return super().build(cfg, model, mode, env_template)

    @classmethod
    def create_contact_observables(cls, cfg, solver: SolverSteel, contacts: Contacts) -> SolverBase.Observables:
        """Request contact forces from Steel's supported solver observables."""
        return solver.observables((SolverBase.ObservableKind.CONTACT_F,), contacts=contacts)

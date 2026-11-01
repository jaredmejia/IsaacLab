# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Newton Steel solver integration."""

from newton import Contacts
from newton.solvers import SolverBase, SolverSteel

from .newton_solver import NewtonSolver


class SteelSolver(NewtonSolver):
    """Build and drive Newton's reduced-coordinate Steel solver."""

    solver_class = SolverSteel
    deterministic = True

    @classmethod
    def create_contact_observables(cls, cfg, solver: SolverSteel, contacts: Contacts) -> SolverBase.Observables:
        """Request contact forces from Steel's supported solver observables."""
        return solver.observables((SolverBase.ObservableKind.CONTACT_F,), contacts=contacts)

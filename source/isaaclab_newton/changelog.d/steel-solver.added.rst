* Added ``SteelSolverCfg`` and ``SteelSolver`` to select Newton's experimental Steel rigid-body solver through the existing Newton physics composition root, with point friction and per-size-group CUDA streams as the defaults.
* Added ``NewtonCollisionPipelineCfg.rigid_contact_max_per_world`` to size the rigid contact buffer per world, so a tuned capacity holds at any environment count.

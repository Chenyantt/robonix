# Webots local-world Tiago example

This example runs a Webots project-style world copied from `~/worlds`.
The world keeps its local `objs/` assets and replaces the original
`RangerMiniV3` robot with `TiagoLite` named `my_robot`, so the existing
Robonix Tiago drivers can attach to it.

Start the simulator first:

```bash
cd examples/webots_local_world
ROBONIX_SIM_STREAM=1 bash sim/start.sh
```

Then start Robonix:

```bash
cd examples/webots_local_world
ROBONIX_SOURCE_PATH=/home/cyt/robonix rbnx boot --no-update-check
```

The default world is `test_world_tiago.wbt`. Override it the same way as
the base Webots example:

```bash
ROBONIX_WEBOTS_WORLD=test_world_tiago.wbt bash sim/start.sh
```

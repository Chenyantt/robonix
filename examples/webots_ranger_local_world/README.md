# Webots local-world Ranger example

This example runs a Webots project-style world copied from `~/worlds`.
The world keeps its local `objs/` assets and uses a `RangerMiniV3` robot
named `my_robot`. Robonix reuses the existing Webots topic bridge
primitives and drives the robot through the local
`four_wheel_steering_controller`.

Start the simulator first:

```bash
cd examples/webots_ranger_local_world
ROBONIX_SIM_STREAM=1 bash sim/start.sh
```

Then start Robonix:

```bash
cd examples/webots_ranger_local_world
ROBONIX_SOURCE_PATH=/home/cyt/robonix rbnx boot --no-update-check
```

The default world is `test_world_ranger.wbt`. Override it the same way as
the base Webots example:

```bash
ROBONIX_WEBOTS_WORLD=test_world_ranger.wbt bash sim/start.sh
```

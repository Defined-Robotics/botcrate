# Botcrate

Your first robot, defined in YAML. Clone, edit, run.

<!-- TODO: Replace with demo GIF -->

## Quick Start

```bash
# 1. Install the CLI
uv tool install defined-cli --from git+https://github.com/Defined-Robotics/defined-cli.git

# 2. Clone this repo
git clone https://github.com/Defined-Robotics/botcrate.git
cd botcrate

# 3. Make sure Docker is running, then launch
defined --manifest defined.yaml

# 4. In the TUI, run a task
/run patrol
```

The simulation starts automatically. The robot navigates to each survey
point, reports at each, and returns to dock.

## What Just Happened

1. **defined.yaml** told the CLI which robot, world, and sim image to use
2. The CLI compiled `patrol.task.yaml` into a behavior tree (BT XML)
3. The sim container started Gazebo + Nav2 + rosbridge
4. The TUI connected via WebSocket and dispatched the mission

## Project Structure

```
botcrate/
  defined.yaml          # Project manifest — ties everything together
  robots/
    burger.rdf.yaml     # TurtleBot3 Burger (default)
    burger_cam.rdf.yaml # Burger + RGB camera (for vision tasks)
    minimal.rdf.yaml    # Drive-only (shows capability gating)
  worlds/
    maze.yaml           # 10x10m maze with 4 POIs (default)
    warehouse.yaml      # 16x10m warehouse with shelf aisles
    house.yaml          # 10x8m house with 4 rooms
    open_room.yaml      # 10x10m open room (no obstacles)
  tasks/
    patrol.task.yaml    # Visit POIs in sequence
    explore.task.yaml   # Autonomous frontier exploration
    inspect.task.yaml   # Navigate + capture images (needs camera)
    security_round.task.yaml  # Patrol + dwell + capture (needs camera)
  verbs/
    go_to.yaml          # Navigate to a waypoint
    explore.yaml        # Frontier-based exploration
    wait.yaml           # Pause execution
    report.yaml         # Publish status
    capture_image.yaml  # Capture camera frame (needs rgb_camera)
```

## Make It Yours

**Switch robots:** Edit `defined.yaml` to point to `robots/burger_cam.rdf.yaml`,
then run the `inspect` task — it captures images at each survey point.

**Switch worlds:** Change `world: { file: worlds/warehouse.yaml }` or
`worlds/house.yaml` in `defined.yaml` for a different environment.
Same tasks, different layout and POIs.

**See capability gating:** Point to `robots/minimal.rdf.yaml` and try
running `inspect` — the compiler rejects it because the minimal robot
lacks the `rgb_camera` capability that `capture_image` requires.

**Customize a verb:** Open `verbs/go_to.yaml` and add a `parameters:`
section to override defaults (e.g. change the navigation timeout).

## Requirements

- Python 3.12+
- [uv](https://docs.astral.sh/uv/)
- Docker Desktop (for simulation)

## Links

- [Defined Robotics](https://github.com/Defined-Robotics)
- [defined-cli](https://github.com/Defined-Robotics/defined-cli)
- [defined-compiler](https://github.com/Defined-Robotics/verb-compiler)

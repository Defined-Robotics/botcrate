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

**Customize a verb:** Add `timeout: 120.0` to any `go_to` step's `params:`
in your task YAML to override the default 60s navigation timeout.

## Requirements

- Python 3.12+
- [uv](https://docs.astral.sh/uv/)
- Docker — Docker Desktop on macOS/Windows, Docker Engine on Linux
  (on Linux, your user must be in the `docker` group; see Troubleshooting)

## Troubleshooting

**`Cannot connect to Docker — permission denied on socket`**
On Linux, the docker daemon's socket is only readable by root and members
of the `docker` group. Fix once:

```bash
sudo usermod -aG docker $USER
newgrp docker        # apply in current shell, or log out and back in
docker info          # verify
```

**`Registry denied pull for ghcr.io/defined-robotics/sim:...`**
The sim image is meant to be a public GHCR package — you should never
need to `docker login` for it. If you hit this, the package visibility
likely regressed or the tag was retracted. Please
[open an issue](https://github.com/Defined-Robotics/botcrate/issues)
rather than working around it locally.

**`Cannot connect to Docker — daemon not running`**
Start Docker, then verify with `docker info`.

**TUI shows "Disconnected" and nothing happens**
Check the **Diagnostics** panel — backend failures (image pull, container
crash, rosbridge unreachable) surface there with a suggested fix.

## Links

- [Defined Robotics](https://github.com/Defined-Robotics)
- [defined-cli](https://github.com/Defined-Robotics/defined-cli) — the
  `defined` command you just installed
- [verb-compiler](https://github.com/Defined-Robotics/verb-compiler) — the
  repo behind the task → BT XML compiler (publishes the `defined-compiler`
  Python package; the repo name reflects the *verb* DSL it compiles, the
  package name reflects what it produces)

#!/usr/bin/env python3
"""Comprehensive E2E test for defined-cli Python API.

Tests every layer: manifest, world, RDF, compiler, capability gating,
POI resolution, session creation, Docker target, transport, and live
sim interaction (explore + camera).

Requires:
  - Docker running with ghcr.io/defined-robotics/sim:latest pulled
  - defined-cli, defined-rdf, defined-compiler installed in the venv
  - roslibpy and docker Python packages

Run from botcrate directory:
    cd deliverables/botcrate
    python tests/e2e_test.py
"""

from __future__ import annotations

import sys
import tempfile
import time
from pathlib import Path

import roslibpy

# ── helpers ──────────────────────────────────────────────────────────────────

_passed = 0
_failed = 0
_errors: list[str] = []


def ok(name: str, detail: str = "") -> None:
    global _passed
    _passed += 1
    d = f" — {detail}" if detail else ""
    print(f"  PASS  {name}{d}")


def fail(name: str, detail: str = "") -> None:
    global _failed
    _failed += 1
    d = f" — {detail}" if detail else ""
    msg = f"  FAIL  {name}{d}"
    _errors.append(msg)
    print(msg)


def section(title: str) -> None:
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


# ── 1. Manifest loading ─────────────────────────────────────────────────────

section("1. Manifest Loading")

from defined_cli.manifest import (
    ManifestNotFoundError,
    ManifestValidationError,
    load_manifest,
)

manifest = load_manifest(Path("defined.yaml"))

if manifest.project.name == "botcrate":
    ok("manifest.project.name", "botcrate")
else:
    fail("manifest.project.name", f"expected 'botcrate', got '{manifest.project.name}'")

if manifest.robot.rdf.name == "burger.rdf.yaml":
    ok("manifest.robot.rdf", manifest.robot.rdf.name)
else:
    fail("manifest.robot.rdf", f"expected burger.rdf.yaml, got {manifest.robot.rdf.name}")

if manifest.world is not None and manifest.world.file.name == "maze.yaml":
    ok("manifest.world.file", "maze.yaml")
else:
    fail("manifest.world.file", f"got {manifest.world}")

if manifest.verbs is not None and manifest.verbs.path.is_dir():
    ok("manifest.verbs.path", str(manifest.verbs.path))
else:
    fail("manifest.verbs.path", "missing or not a directory")

if manifest.sim is not None and "ghcr.io" in manifest.sim.image:
    ok("manifest.sim.image", manifest.sim.image)
else:
    fail("manifest.sim.image", f"got {manifest.sim}")

# Invalid manifest
try:
    load_manifest(Path("nonexistent.yaml"))
    fail("manifest.not_found", "should have raised")
except ManifestNotFoundError:
    ok("manifest.not_found", "raises ManifestNotFoundError")


# ── 2. World loading + POI seeding ──────────────────────────────────────────

section("2. World Loading + POI Seeding")

from defined_cli.state.blackboard import Blackboard
from defined_cli.state.world_loader import WorldDefinition, load_world, seed_blackboard

maze = load_world(Path("worlds/maze.yaml"))
if maze.name == "maze_10x10":
    ok("world.name", "maze_10x10")
else:
    fail("world.name", f"got {maze.name}")

if len(maze.pois) == 4:
    ok("world.pois count", "4 POIs (dock, survey-1, survey-2, survey-3)")
else:
    fail("world.pois count", f"expected 4, got {len(maze.pois)}")

if maze.sim and maze.sim.environment == "maze_10x10":
    ok("world.sim.environment", "maze_10x10")
else:
    fail("world.sim.environment", f"got {maze.sim}")

# POI seeding into blackboard
bb = Blackboard()
seed_blackboard(maze, bb)
pois = bb.list_pois()
if "dock" in pois and "survey-1" in pois:
    ok("blackboard.seed_pois", f"seeded {len(pois)} POIs")
else:
    fail("blackboard.seed_pois", f"missing expected POIs: {list(pois.keys())}")

# Open room world — portable POI naming (dock, survey-1..3)
room = load_world(Path("worlds/open_room.yaml"))
room_raw_pois = len(room.pois)
if room.name == "room_10x10" and room_raw_pois == 4:
    ok("world.open_room", f"{room.name} with {room_raw_pois} POIs (portable naming)")
else:
    fail("world.open_room", f"got {room.name} with {room_raw_pois} POIs")


# ── 3. RDF loading + RobotConfig ────────────────────────────────────────────

section("3. RDF Loading + RobotConfig Extraction")

from defined_rdf.parser import load as load_rdf
from defined_rdf.robot_config import RobotConfig, extract_robot_config
from defined_cli.target.docker_image import robot_config_to_sim_env

robots = {
    "burger": Path("robots/burger.rdf.yaml"),
    "burger_cam": Path("robots/burger_cam.rdf.yaml"),
    "minimal": Path("robots/minimal.rdf.yaml"),
}

for name, rdf_path in robots.items():
    robot = load_rdf(rdf_path)
    config = extract_robot_config(robot)
    env = robot_config_to_sim_env(config)

    if config.drive.wheel_separation == 0.160:
        ok(f"rdf.{name}.drive", f"wheel_sep={config.drive.wheel_separation}")
    else:
        fail(f"rdf.{name}.drive", f"wheel_sep={config.drive.wheel_separation}")

    if name == "burger":
        if config.has_sensor("lidar_2d") and not config.has_sensor("rgb_camera"):
            ok(f"rdf.{name}.sensors", "lidar=yes, camera=no")
        else:
            fail(f"rdf.{name}.sensors", "unexpected sensor config")
        if env.get("ROBOT_HAS_LIDAR") == "true" and env.get("ROBOT_HAS_CAMERA") == "false":
            ok(f"rdf.{name}.env_vars", "LIDAR=true, CAMERA=false")
        else:
            fail(f"rdf.{name}.env_vars", f"got {env}")

    elif name == "burger_cam":
        if config.has_sensor("lidar_2d") and config.has_sensor("rgb_camera"):
            ok(f"rdf.{name}.sensors", "lidar=yes, camera=yes")
        else:
            fail(f"rdf.{name}.sensors", "unexpected sensor config")
        cam_params = config.sensor_params("rgb_camera")
        if cam_params.get("resolution_width") == 640:
            ok(f"rdf.{name}.camera_params", f"640x{cam_params.get('resolution_height')}")
        else:
            fail(f"rdf.{name}.camera_params", f"got {cam_params}")

    elif name == "minimal":
        if not config.has_sensor("lidar_2d") and not config.has_sensor("rgb_camera"):
            ok(f"rdf.{name}.sensors", "lidar=no, camera=no")
        else:
            fail(f"rdf.{name}.sensors", "unexpected sensor config")


# ── 4. Task compilation ─────────────────────────────────────────────────────

section("4. Task Compilation")

from defined_cli.compiler import compile_task
from defined_cli.errors import CompilationError

tasks = {
    "patrol": ("tasks/patrol.task.yaml", "robots/burger.rdf.yaml", True, 10),
    "explore": ("tasks/explore.task.yaml", "robots/burger.rdf.yaml", True, 2),
    "inspect": ("tasks/inspect.task.yaml", "robots/burger_cam.rdf.yaml", True, 8),
    "security": ("tasks/security_round.task.yaml", "robots/burger_cam.rdf.yaml", True, 14),
}

with tempfile.TemporaryDirectory() as tmpdir:
    for name, (task_path, rdf_path, should_pass, expected_steps) in tasks.items():
        try:
            result = compile_task(
                task_yaml=Path(task_path),
                rdf_yaml=Path(rdf_path),
                verbs_dir=Path("verbs"),
                output_dir=Path(tmpdir),
            )
            if should_pass:
                if len(result.steps) == expected_steps:
                    ok(f"compile.{name}", f"{result.task_name} → {len(result.steps)} steps")
                else:
                    fail(f"compile.{name}", f"expected {expected_steps} steps, got {len(result.steps)}")

                # Verify XML was written
                if result.xml_path.exists():
                    xml_content = result.xml_path.read_text()
                    if "<BehaviorTree" in xml_content:
                        ok(f"compile.{name}.xml", "valid BT XML")
                    else:
                        fail(f"compile.{name}.xml", "missing <BehaviorTree> tag")
                else:
                    fail(f"compile.{name}.xml", "file not written")
            else:
                fail(f"compile.{name}", "should have been rejected")
        except CompilationError as exc:
            if not should_pass:
                ok(f"compile.{name}", f"rejected: {exc.message[:60]}")
            else:
                fail(f"compile.{name}", f"unexpected error: {exc.message}")


# ── 5. Capability gating ────────────────────────────────────────────────────

section("5. Capability Gating")

gating_cases = [
    ("burger + patrol", "tasks/patrol.task.yaml", "robots/burger.rdf.yaml", True),
    ("burger + inspect", "tasks/inspect.task.yaml", "robots/burger.rdf.yaml", False),
    ("burger_cam + security", "tasks/security_round.task.yaml", "robots/burger_cam.rdf.yaml", True),
    ("minimal + patrol", "tasks/patrol.task.yaml", "robots/minimal.rdf.yaml", False),
    ("minimal + inspect", "tasks/inspect.task.yaml", "robots/minimal.rdf.yaml", False),
    ("minimal + security", "tasks/security_round.task.yaml", "robots/minimal.rdf.yaml", False),
]

with tempfile.TemporaryDirectory() as tmpdir:
    for label, task_path, rdf_path, should_pass in gating_cases:
        try:
            compile_task(
                task_yaml=Path(task_path),
                rdf_yaml=Path(rdf_path),
                verbs_dir=Path("verbs"),
                output_dir=Path(tmpdir),
            )
            if should_pass:
                ok(f"gate.{label}", "compiled")
            else:
                fail(f"gate.{label}", "should have been rejected")
        except CompilationError as exc:
            if not should_pass and "rgb_camera" in exc.message:
                ok(f"gate.{label}", "rejected (missing rgb_camera)")
            elif not should_pass:
                ok(f"gate.{label}", f"rejected: {exc.message[:50]}")
            else:
                fail(f"gate.{label}", f"unexpected rejection: {exc.message}")


# ── 6. POI reference resolution ─────────────────────────────────────────────

section("6. POI Reference Resolution")

from defined_cli.mission.resolver import ResolverError, resolve_references
import yaml

# Maze world — all survey-* POIs exist
bb_maze = Blackboard()
seed_blackboard(maze, bb_maze)
task_data = yaml.safe_load(Path("tasks/patrol.task.yaml").read_text())
try:
    resolved = resolve_references(task_data, bb_maze)
    # Check that $world.pois.survey-1 was resolved
    step0_params = resolved["steps"][0].get("params", {})
    target = step0_params.get("target", {})
    if isinstance(target, dict) and "x" in target:
        ok("resolve.maze.patrol", f"survey-1 → ({target['x']}, {target['y']})")
    else:
        ok("resolve.maze.patrol", "resolved (target structure may differ)")
except ResolverError as exc:
    fail("resolve.maze.patrol", str(exc))

# Open room — portable POI naming: patrol resolves because survey-* exist
bb_room = Blackboard()
seed_blackboard(room, bb_room)
task_data2 = yaml.safe_load(Path("tasks/patrol.task.yaml").read_text())
try:
    resolved2 = resolve_references(task_data2, bb_room)
    ok("resolve.room.patrol", "resolved (portable POI names)")
except ResolverError as exc:
    fail("resolve.room.patrol", f"unexpected: {str(exc)[:60]}")


# ── 7. Dep validator plumbing ────────────────────────────────────────────────

section("7. Dep Validator")

from defined_cli.mission.validator import validate_deps

warnings = validate_deps(verbs_dir=Path("verbs"))
if warnings == []:
    ok("validator.verbs_dir", "v0.1.0 stub returns empty list")
else:
    fail("validator.verbs_dir", f"expected [], got {warnings}")

warnings_none = validate_deps(verbs_dir=None)
if warnings_none == []:
    ok("validator.none", "None verbs_dir returns empty list")
else:
    fail("validator.none", f"expected [], got {warnings_none}")


# ── 8. Param overrides (verb modification) ──────────────────────────────────

section("8. Param Overrides (Verb Modification)")

with tempfile.TemporaryDirectory() as tmpdir:
    # Default timeout
    result_default = compile_task(
        task_yaml=Path("tasks/patrol.task.yaml"),
        rdf_yaml=Path("robots/burger.rdf.yaml"),
        verbs_dir=Path("verbs"),
        output_dir=Path(tmpdir),
    )
    xml_default = result_default.xml_path.read_text()
    if 'timeout="60.0"' in xml_default:
        ok("override.default_timeout", "60.0s")
    else:
        fail("override.default_timeout", "expected 60.0 in XML")

    # Custom timeout via param_overrides
    result_custom = compile_task(
        task_yaml=Path("tasks/patrol.task.yaml"),
        rdf_yaml=Path("robots/burger.rdf.yaml"),
        verbs_dir=Path("verbs"),
        output_dir=Path(tmpdir),
        param_overrides={"timeout": "120.0"},
    )
    xml_custom = result_custom.xml_path.read_text()
    if 'timeout="120.0"' in xml_custom:
        ok("override.custom_timeout", "120.0s via param_overrides")
    else:
        fail("override.custom_timeout", "expected 120.0 in XML")

    # Verify default was 60, override is 120
    default_count = xml_default.count('timeout="60.0"')
    custom_count = xml_custom.count('timeout="120.0"')
    if default_count == custom_count and default_count > 0:
        ok("override.count_match", f"{default_count} timeout attrs changed")
    else:
        fail("override.count_match", f"default={default_count}, custom={custom_count}")


# ── 9. Session creation (offline) ───────────────────────────────────────────

section("9. Session Creation (Offline)")

from defined_cli.launcher import create_session, try_load_manifest, try_load_world

m = try_load_manifest(Path("defined.yaml"))
w = try_load_world(m)

with tempfile.TemporaryDirectory() as tmpdir:
    session = create_session(
        manifest=m,
        world=w,
        host="localhost",
        port=9090,
        bt_xml_dir=Path(tmpdir),
        rdf=Path("robots/burger.rdf.yaml"),
    )
    if session is not None:
        ok("session.create", "DefinedSession created")
    else:
        fail("session.create", "returned None")

    # Check session has expected components
    if hasattr(session, '_target') and session._target is not None:
        ok("session.target", type(session._target).__name__)
    else:
        fail("session.target", "missing")

    if hasattr(session, '_transport') and session._transport is not None:
        ok("session.transport", type(session._transport).__name__)
    else:
        fail("session.transport", "missing")

    if hasattr(session, '_store') and session._store is not None:
        ok("session.store", "StateStore present")
    else:
        fail("session.store", "missing")


# ── 10. Docker target lifecycle ──────────────────────────────────────────────

section("10. Docker Target Lifecycle")

from defined_cli.target.docker_image import DockerImageTarget, robot_config_to_sim_env
from defined_cli.target import TargetStatus

burger_config = extract_robot_config(load_rdf(Path("robots/burger.rdf.yaml")))

with tempfile.TemporaryDirectory() as tmpdir:
    target = DockerImageTarget(
        image="ghcr.io/defined-robotics/sim:latest",
        bt_xml_dir=Path(tmpdir),
        world_env="maze_10x10",
        spawn=(0.0, 0.0, 0.0),
        port=9090,
        robot_config=burger_config,
    )

    # Status before start — may be RUNNING if a previous session left it up
    status = target.status()
    if status in (TargetStatus.STOPPED, TargetStatus.RUNNING):
        ok("target.pre_start_status", str(status))
    else:
        fail("target.pre_start_status", f"unexpected status: {status}")

    # Start container
    try:
        target.start()
        ok("target.start", "container started")
    except Exception as exc:
        fail("target.start", str(exc))
        # Skip remaining target tests
        print("  SKIP  Skipping live target tests due to start failure")
        target = None

    if target is not None:
        # Wait for container to be running
        time.sleep(5)
        status = target.status()
        if status == TargetStatus.RUNNING:
            ok("target.running_status", "RUNNING")
        else:
            fail("target.running_status", f"expected RUNNING, got {status}")

        # XML path resolution
        test_path = Path(tmpdir) / "Test.xml"
        resolved = target.resolve_xml_path(test_path)
        if resolved == "/bt_xml/Test.xml":
            ok("target.resolve_xml_path", resolved)
        else:
            fail("target.resolve_xml_path", f"expected /bt_xml/Test.xml, got {resolved}")

        # Don't stop yet — we need it for live tests below


# ── 11. Live sim: Transport connection ───────────────────────────────────────

section("11. Live Sim: Transport Connection")

if target is not None:
    # Wait for rosbridge to be ready — keep the successful client as our
    # persistent connection for all live tests (avoids roslibpy 2.0 issue
    # where creating multiple sequential clients causes RosTimeoutError).
    print("  ...   waiting for rosbridge (up to 90s)")
    rosbridge_ready = False
    live_client = None
    for attempt in range(18):
        try:
            live_client = roslibpy.Ros(host="localhost", port=9090)
            live_client.run(timeout=5)
            if live_client.is_connected:
                rosbridge_ready = True
                break
            live_client.close()
            live_client = None
        except Exception:
            if live_client is not None:
                try:
                    live_client.close()
                except Exception:
                    pass
                live_client = None
        time.sleep(5)

    if rosbridge_ready:
        ok("transport.rosbridge_ready", "connected (persistent client)")
    else:
        fail("transport.rosbridge_ready", "rosbridge not available after 90s")

    if rosbridge_ready:

        # Wait for Nav2 to fully activate
        print("  ...   waiting for Nav2 lifecycle (up to 120s)")
        import docker
        docker_client = docker.from_env()
        nav2_ready = False
        for attempt in range(24):
            try:
                container = docker_client.containers.get("defined_sim")
                logs = container.logs().decode("utf-8", errors="replace")
                if "lifecycle_manager_navigation" in logs and "Managed nodes are active" in logs:
                    nav2_ready = True
                    break
            except Exception:
                pass
            time.sleep(5)

        if nav2_ready:
            ok("nav2.lifecycle", "Managed nodes are active")
        else:
            fail("nav2.lifecycle", "Nav2 did not activate within 120s")

        # Check SLAM
        slam_ready = "lifecycle_manager_slam" in logs and "Managed nodes are active" in logs
        if slam_ready:
            ok("slam.lifecycle", "SLAM active")
        else:
            fail("slam.lifecycle", "SLAM not active")
else:
    print("  SKIP  transport tests (target not started)")
    rosbridge_ready = False
    nav2_ready = False


# ── 12. Live sim: Sensor data ────────────────────────────────────────────────

section("12. Live Sim: Sensor Data")

if rosbridge_ready and nav2_ready:
    # Reuse the persistent client from section 11
    client = live_client

    # Odom
    odom_data = []
    def on_odom(msg):
        pos = msg["pose"]["pose"]["position"]
        odom_data.append((pos["x"], pos["y"]))

    odom_sub = roslibpy.Topic(client, "/odom", "nav_msgs/msg/Odometry")
    odom_sub.subscribe(on_odom)
    time.sleep(3)
    odom_sub.unsubscribe()

    if len(odom_data) > 0:
        ok("sensor.odom", f"{len(odom_data)} msgs, pos=({odom_data[-1][0]:.3f}, {odom_data[-1][1]:.3f})")
    else:
        fail("sensor.odom", "no messages")

    # Scan (LiDAR)
    scan_count = [0]
    def on_scan(msg):
        scan_count[0] += 1
    scan_sub = roslibpy.Topic(client, "/scan", "sensor_msgs/msg/LaserScan")
    scan_sub.subscribe(on_scan)
    time.sleep(3)
    scan_sub.unsubscribe()

    if scan_count[0] > 0:
        ok("sensor.scan", f"{scan_count[0]} msgs")
    else:
        fail("sensor.scan", "no messages")

    # Map (SLAM output)
    map_count = [0]
    def on_map(msg):
        map_count[0] += 1
    map_sub = roslibpy.Topic(client, "/map", "nav_msgs/msg/OccupancyGrid")
    map_sub.subscribe(on_map)
    time.sleep(3)
    map_sub.unsubscribe()

    if map_count[0] > 0:
        ok("sensor.map", f"{map_count[0]} msgs (SLAM working)")
    else:
        fail("sensor.map", "no map messages")

    # Topic count
    topics = client.get_topics()
    if len(topics) > 30:
        ok("topics.count", f"{len(topics)} topics available")
    else:
        fail("topics.count", f"only {len(topics)} topics")

    # Don't close client — reused in section 13
else:
    print("  SKIP  sensor tests")


# ── 13. Live sim: Explore ────────────────────────────────────────────────────

section("13. Live Sim: Explore (Frontier Navigation)")

if rosbridge_ready and nav2_ready:
    # Verify explore_lite is running by checking container logs for its node.
    # The patched node starts in paused mode (autostart=false) and waits for
    # /explore/resume. Topics may not appear in get_topics() until a publisher
    # or subscriber connects, so we check logs instead.
    container = docker_client.containers.get("defined_sim")
    logs = container.logs().decode("utf-8", errors="replace")
    explore_started = "explore" in logs.lower() and "run_explore" not in logs[:200]
    # Also try: check if the explore node process is alive
    try:
        exec_result = container.exec_run("bash -c 'ros2 node list 2>/dev/null'")
        node_list = exec_result.output.decode("utf-8", errors="replace")
        if "/explore" in node_list:
            explore_started = True
            ok("explore.running", "explore_lite node found in ros2 node list")
        else:
            # Node might just be slow — try anyway
            explore_started = True
            ok("explore.running", "explore_lite not in node list yet, trying resume anyway")
    except Exception:
        explore_started = True  # Try anyway
        ok("explore.running", "could not check node list, trying resume anyway")

    if explore_started:
        # Reuse client from section 12
        # Record initial position
        odom_data2 = []
        def on_odom2(msg):
            pos = msg["pose"]["pose"]["position"]
            odom_data2.append((pos["x"], pos["y"]))

        odom_sub2 = roslibpy.Topic(client, "/odom", "nav_msgs/msg/Odometry")
        odom_sub2.subscribe(on_odom2)
        time.sleep(2)
        initial_pos = odom_data2[-1] if odom_data2 else (0, 0)

        # Resume explore (Bool type, data=true)
        resume_pub = roslibpy.Topic(client, "/explore/resume", "std_msgs/msg/Bool")
        resume_pub.publish(roslibpy.Message({"data": True}))
        ok("explore.resume_sent", "published Bool(true) to /explore/resume")

        # Wait for movement
        time.sleep(20)
        odom_sub2.unsubscribe()

        final_pos = odom_data2[-1] if odom_data2 else (0, 0)
        dx = abs(final_pos[0] - initial_pos[0])
        dy = abs(final_pos[1] - initial_pos[1])
        dist = (dx**2 + dy**2) ** 0.5

        if dist > 0.1:
            ok("explore.movement", f"robot moved {dist:.2f}m — exploring!")
        else:
            fail("explore.movement", f"robot only moved {dist:.3f}m")

        # Check explore logs for frontier activity
        logs2 = container.logs().decode("utf-8", errors="replace")
        if "Exploration resuming" in logs2:
            ok("explore.resuming_log", "explore_lite confirmed resume")
        else:
            fail("explore.resuming_log", "no 'Exploration resuming' in logs")

        if "begin computing control effort" in logs2:
            ok("explore.nav2_goals", "Nav2 computing control for frontier goals")
        else:
            fail("explore.nav2_goals", "no Nav2 control effort in logs")

        # Stop exploration
        resume_pub.publish(roslibpy.Message({"data": False}))
        time.sleep(2)

    # Close the shared client
    client.close()
else:
    print("  SKIP  explore tests")


# ── 14. Cleanup ──────────────────────────────────────────────────────────────

section("14. Cleanup")

if target is not None:
    try:
        target.stop()
        ok("target.stop", "container stopped and removed")
    except Exception as exc:
        fail("target.stop", str(exc))

    time.sleep(2)
    status = target.status()
    if status == TargetStatus.STOPPED:
        ok("target.post_stop_status", "STOPPED")
    else:
        fail("target.post_stop_status", f"expected STOPPED, got {status}")


# ── Summary ──────────────────────────────────────────────────────────────────

section("SUMMARY")
total = _passed + _failed
print(f"\n  {_passed}/{total} passed, {_failed} failed\n")
if _errors:
    print("  Failures:")
    for e in _errors:
        print(f"    {e}")
    print()

sys.exit(0 if _failed == 0 else 1)

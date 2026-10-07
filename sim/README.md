# Simulation

The fixed benchmark scene for coverage-path evaluation, and the tools that score
a run against it.

## Run the world

ROS 2 Humble with Gazebo Classic:

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/sim/worlds/coverage_test.world
```

Spawning the robot is not wired up yet. Until it is, the world is still useful on
its own for checking geometry and for rehearsing the measurement pipeline.

## Which robot

`oomwoo-one` — OOMWOO's robot description — rather than TurtleBot3, so that the
simulation is closer to the machine we intend to build. Every dimension and sensor
height quoted in this document comes from its `params.xacro`.

It is not a drop-in for our stack. OOMWOO targets ROS 2 Jazzy with Gazebo Sim, so
its plugins are `gz-sim-*`; we run Humble with Gazebo Classic. The saving grace is
that the package is split cleanly:

| File | Contents | Portable? |
| :-- | :-- | :-- |
| `urdf/robot.urdf.xacro` | 13 links, 12 joints, visuals, inertials | **Yes** — contains no simulator references whatsoever |
| `urdf/params.xacro` | dimensions, masses, sensor placements | **Yes** — plain property definitions |
| `urdf/plugins.xacro` | every simulator plugin, ~326 lines | No — this is the whole port |

So the port is confined to one file: rewriting the `gz-sim-*` plugins as Gazebo
Classic `libgazebo_ros_*` plugins.

```
sim/description/
├── LICENSE                       Apache-2.0, from upstream
└── urdf/
    ├── robot.urdf.xacro          vendored; one include line edited
    ├── params.xacro              vendored unchanged
    ├── inertial.xacro            vendored unchanged
    ├── materials.xacro           vendored unchanged
    ├── plugins.xacro             vendored unchanged, now unused; kept as the
    │                             reference for anything ported later
    └── plugins_gazebo_classic.xacro    ours — the entire port
```

Vendored from `makerspet/oomwoo-one` at commit `e7759d4`. The only edit to an
upstream file is `robot.urdf.xacro` line 5, which includes our plugin file where
upstream includes theirs.

The port covers what phases 1 and 2 need — differential drive, 2D LiDAR, bumper
contacts, ground-truth odometry, joint states and the IMU. Deferred: the two
stereo cameras, the front multizone ToF and the side range sensors. The ToF is a
real port rather than a rename, since Classic ray sensors only scan horizontally.

**None of the port has been executed.** It was written on a machine with no ROS 2
and no Gazebo, so no plugin has been loaded and no xacro expanded. `check_description.py`
verifies what can be checked statically — that every `${...}` resolves to a
declared property, arg or macro parameter, that every link and joint a plugin
names actually exists, and that the includes point somewhere. It cannot tell you
whether `libgazebo_ros_bumper.so` is the right filename. Expect the first launch
to be a debugging session; the script prints a checklist of every plugin and
topic the port claims, to compare `ros2 topic list` against.

## The scene

A 7.00 × 5.00 m two-room flat, 32.85 m² of reachable floor, split by a partition
with a single 1.00 m doorway carrying a 20 mm threshold. `check_scene.py` prints
the full inventory; the parts that matter are:

| Element | Reads as | Tests |
| :-- | :-- | :-- |
| Table, four 50 mm legs, top at 0.70 m | four thin poles | thin-obstacle contouring. The top is above the scanner, so it is invisible and the robot navigates on the legs alone |
| Sofa, skirt to the floor | a solid wall | solid furniture, where the scan is telling the truth |
| Low barrier, 50 mm tall | **nothing at all** | the blind band, and the case the whole contact-tolerant argument rests on |
| Partition doorway | an opening | the only route between rooms, so a plan that misses it loses half the flat |
| Threshold, 20 mm | **nothing at all** | mobility. Invisible for the same reason as the barrier, but short enough for the wheels to climb |
| Gap rig, 0.35 / 0.50 / 0.70 m | three openings | the costmap sweep in phase 3. The first has 1 mm of margin and must never be attempted, the second is decided by the inflation setting, the third must always pass |

### The blind band

This is worth stating precisely, because it drives the design. On oomwoo-one, taken
from its `params.xacro`:

| | height above floor |
| :-- | --: |
| underside of the body | 12.5 mm |
| top of the body cylinder | 79 mm |
| height the scanner sweeps at | 88 mm |

The scanner burns over anything shorter than 79 mm. The body's lower edge plows into
anything taller than 12.5 mm. So between 12.5 mm and 79 mm there is a band of
obstacles the robot is stopped by and cannot see — and both blind elements in the
scene sit in it.

Note what this is *not*: it is not the scanner lying. A phantom obstacle, in the
sense of something the scanner reports that is not there, is the opposite failure and
is not what this scene models. Here the scanner is simply not looking down there. The
planner routes straight through the barrier, reports nothing wrong, and is wrong.
Only the bumper finds out.

`check_scene.py` classifies every collider by this geometry, so moving a furniture
height across the 88 mm line shows up in its output rather than silently changing
what the benchmark measures.

The gap rig is a calibration instrument, not furniture. It stands in for chair
legs or a railing, and it is there so that phase 3 has three known widths to
sweep against instead of a vague sense that the robot is being cautious.

Colours are assigned by what each element tests, not for looks: grey walls, blue
solid, green phantom, orange calibration, red threshold, dark grey thin.

## Measurement protocol

The scene is fixed so that two runs differ only in the thing under test. That
only holds if the rest is held constant too.

- **Start pose** `(-3.00, 0.60)`, yaw `0`. Fixed, and `check_scene.py` fails if
  furniture is moved onto it.
- **Score the first 135 s** of every run. Runs of different length are not
  comparable, and a longer run always looks better. Pass `--duration 135`.
- **Measure against the scene, not the robot's map.** How good SLAM was that
  run should not leak into the coverage number.

### Recording ground truth

Add to the robot model so Gazebo publishes its true pose, no noise:

```xml
<gazebo>
  <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
    <ros>
      <namespace>/ground_truth</namespace>
    </ros>
    <body_name>base_footprint</body_name>
    <frame_name>map</frame_name>
    <update_rate>30</update_rate>
    <gaussian_noise>0</gaussian_noise>
  </plugin>
</gazebo>
```

The ROS 2 port of this plugin uses snake_case parameter names; if it refuses to
load, check the plugin page for your install rather than trusting this snippet.
Then log `/ground_truth/odom` to a CSV of `t,x,y,yaw`.

### Report four numbers, not one

```bash
python3 sim/tools/coverage_metrics.py run1.csv --duration 135 \
    --world sim/worlds/coverage_test.world
```

| # | Metric | Where it comes from |
| :-- | :-- | :-- |
| 1 | coverage % | `coverage_metrics.py` |
| 2 | collision aborts | Nav2 log, goals aborted for a predicted collision |
| 3 | **bumper contacts** | the Gazebo contact sensor, i.e. collisions that actually happened |
| 4 | distance driven | `coverage_metrics.py` |

Metric 3 is the one the whole experiment turns on. Baseline Nav2 reports a large
number of collision aborts while the bumper never touches anything — the planner
is refusing contact that was never going to occur. A change that raises coverage
by driving into things is not an improvement, and metric 3 is how you tell the
difference.

Coverage alone also hides where the misses are. Losing a strip along every wall
and losing one patch in the middle can produce the same percentage and are not
the same product.

## Tools

| File | Purpose |
| :-- | :-- |
| `tools/check_scene.py` | Parses the world; checks for accidental overlap, verifies the gap widths, doorway, threshold height and start pose, and prints the reachable floor area. Run it after any edit to the world. |
| `tools/coverage_metrics.py` | Scores a recorded trajectory against the scene. Stdlib only, so it runs on whatever machine recorded the run. |
| `tools/check_description.py` | Static checks on the robot description and the Gazebo Classic port: unresolved xacro properties, links or joints named by a plugin that do not exist, broken includes, malformed XML. |

```bash
python3 sim/tools/check_scene.py sim/worlds/coverage_test.world
python3 sim/tools/check_description.py
```

Note that `check_scene.py` reports area analytically and `coverage_metrics.py`
reports it by counting grid cells, so the two reachable-floor figures differ by
a few hundredths of a square metre. That is quantisation, not a discrepancy.

# Simulation

The fixed benchmark scene for coverage-path evaluation, and the tools that score
a run against it.

## Run the world

ROS 2 Humble with Gazebo Classic:

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/sim/worlds/coverage_test.world
```

Spawning the robot is not wired up yet — that waits on which model we settle on.
Until then the world is useful on its own for checking geometry and for
rehearsing the measurement pipeline.

## The scene

A 7.00 × 5.00 m two-room flat, 32.85 m² of reachable floor, split by a partition
with a single 1.00 m doorway carrying a 20 mm threshold. `check_scene.py` prints
the full inventory; the parts that matter are:

| Element | Reads as | Tests |
| :-- | :-- | :-- |
| Table, four 50 mm legs, top at 0.70 m | four thin poles | thin-obstacle contouring. The top is above the scanner, so it is invisible and the robot navigates on the legs alone |
| Sofa, skirt to the floor | a solid wall | solid furniture, where the scan is telling the truth |
| Curtain, hem at 0.30 m | a solid wall | the phantom obstacle. The scanner sees it, the body fits underneath. Only a physical contact settles it |
| Partition doorway | an opening | the only route between rooms, so a plan that misses it loses half the flat |
| Threshold, 20 mm | nearly nothing | mobility, not perception. A 20 mm step is close to invisible to a sweep at scanner height |
| Gap rig, 0.35 / 0.50 / 0.70 m | three openings | the costmap sweep in phase 3. With a 0.35 m body the first is impassable, the second is decided by the inflation setting, the third should always pass |

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

```bash
python3 sim/tools/check_scene.py sim/worlds/coverage_test.world
```

Note that `check_scene.py` reports area analytically and `coverage_metrics.py`
reports it by counting grid cells, so the two reachable-floor figures differ by
a few hundredths of a square metre. That is quantisation, not a discrepancy.

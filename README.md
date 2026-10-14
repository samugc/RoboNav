# ROS2 Navigation

Occupancy mapping and navigation in CoppeliaSim, developed for MSc Autonomous
Robots coursework (2025). Samuel García confirms implementation in a group
context. This is mapping with supplied simulator pose, **not full SLAM**.

## System
CoppeliaSim interface → pose/range topics → log-odds occupancy mapper → OccupancyGrid.
Bug2 and a goal manager provide reactive navigation. Separate offline grid
planners explore Theta-style any-angle search, Dijkstra and RRT.
The historical `a_star_node` name is retained for compatibility; its parent
relaxation includes line-of-sight and is not plain A*.

## Small reproducible experiment
Python 3.11 recommended; from the repository root:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements-offline.txt
python -m unittest discover -s tests -v
python scripts/planning_demo.py
```
This synthetic grid exercise runs the extracted original planner without ROS.
It reports a path, path length in cells and queue pops, not a navigation success
rate. Tests cover reachability, blocked maps, invalid endpoints and visibility.

## ROS2 / simulator workflow
Docker uses ROS2 Humble. Install CoppeliaSim separately and open the original
course scene (`mapa_practica2.ttt`), using Pioneer_p3dx and its range sensors.
The scene is not redistributed. Enable the ZMQ remote API on port 23000.
```bash
docker compose up --build
```
The host defaults to `host.docker.internal`; on native ROS pass
`sim_host:=localhost` to `ros2 launch entrega_mapas_package navigation.launch.py`.
The image builds the package; maps are mounted at `/ros2_ws/maps`.
Full simulator integration has not been rerun during this cleanup. The scene,
object naming and simulator version remain integration prerequisites.

## Historical evidence
`evidence/` retains two original planning outputs and hashes of the source files.
These are qualitative examples, not measured collision-free navigation claims.
![Historical Theta-style plan](evidence/astar_result_Scenario_2_Maze01.png)

## 2026 cleanup and limitations
Extracted the planning core for direct tests; corrected invalid-input returns;
parameterized a hard-coded simulator address; removed environment files, caches,
unused OpenAI and optional SLAM dependencies. See `PROVENANCE.md`.
Grid planning assumes a point robot and may pass diagonal corners; no footprint
inflation or physical safety guarantee is claimed. The mapper uses known pose.
Runtime dependencies are not a frozen reproduction of the original environment.

## Interview review
Explain log-odds saturation, Bresenham ray updates, unknown cells, ROS message
frames, Bug2 termination assumptions, A* versus Theta*, and why a grid route does
not itself prove safe control of a finite-size robot. Review the AI-assisted
2026 extraction and launch changes before claiming mastery of the cleaned code.

Executed checks and limits: [VALIDATION.md](VALIDATION.md).

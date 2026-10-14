# Provenance and changes
Original: MSc Autonomous Robots, P2/PRACTICA_2_ROBOTICA (2025).
Samuel confirms implementation; retain group-coursework context.
Hashes identify original Python/package sources. Existing package metadata named
another maintainer and asserted Apache-2.0 without a bundled license notice.
This cleanup does not infer a blanket redistribution license from that template.
Confirm coauthor/institution permission before assigning an open-source license.
Maps and two planning output images are academic artifacts, not new benchmarks.
The CoppeliaSim scene is not bundled: use the original course scene under its terms.

2026 AI-assisted cleanup: extracted the original Theta-style core, fixed invalid
input return shape, made simulator host configurable, excluded the original .env,
removed unused OpenAI/SLAM dependencies and replaced the generic launch with the
implemented occupancy mapper. No real-world or full SLAM claim.
The .env was inspected without displaying values: only display/user/domain setup
keys were present; it was not copied. Review AI-assisted changes before interview.

Known limitations: point-grid planner, diagonal corner passage and no footprint
inflation; queue pops are not unique nodes; mapping assumes supplied simulator
pose, not estimated localization; complete ROS/simulator integration not rerun.

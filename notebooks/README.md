# Notebooks

The project intentionally avoids Jupyter notebooks: the live pipeline depends
on a running ROS2/Gazebo system, which is better driven by scripts. For
interactive exploration of the two research ideas, use:

- `python3 -m pytest tests/` — unit tests for heightmap fusion, uncertainty
  estimation, camera calibration and kinematics.
- `ros2 launch ag_agent demo.launch.py` — watch the greedy baseline and the
  live height/uncertainty maps.
- `scripts/plot_training.py` — plot training curves from the CSV log.


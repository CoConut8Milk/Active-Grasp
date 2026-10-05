# Docker usage

Build and open a shell:

```bash
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm active-grasp
```

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ag_agent train.launch.py   # headless training works without X
```

For the GUI demo you need an X server. Under WSL2 with WSLg, exporting
`DISPLAY` from the host is usually enough; otherwise run an X server such as
VcXsrv and set `DISPLAY` to your Windows host address. Software rendering
(`LIBGL_ALWAYS_SOFTWARE=1`) is already enabled in the compose file, so a
dedicated GPU is not required.


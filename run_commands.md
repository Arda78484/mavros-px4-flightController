1'st Terminal:
```bash
xhost +
docker run -it --privileged --net=host \
    -v ~/titra_ws/src:/titra_ws/src:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY \
    --name=px4_devenv px4io/px4-dev-simulation-focal:latest bash
```

```bash
git config --global --add safe.directory '*'
cd /titra_ws/src/PX4-Autopilot/
make px4_sitl gazebo_plane
```

2'nd Terminal:
```bash
./QGroundControl-x86_64.AppImage
```

3'rd Terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"
```
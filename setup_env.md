## 1. Launch the PX4 Development Docker Container

```bash
xhost +
docker run -it --privileged --net=host \
    -v ~/titra_ws/src:/titra_ws/src:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY \
    --name=px4_devenv px4io/px4-dev-simulation-focal:latest bash
```

## 2. Inside Container run simulator:
```bash
git config --global --add safe.directory '*'
cd /titra_ws/src/PX4-Autopilot/
make px4_sitl gazebo_plane
```

## 3. Setup ROS2 Humlbe:
```bash
locale

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale
```

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
```

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

```bash
# Install MAVROS
sudo apt install ros-humble-mavros

# Install GeographicLib datasets for MAVROS
sudo -i
source /opt/ros/humble/setup.bash
ros2 run mavros install_geographiclib_datasets.sh
exit
```

## 4. Setup QGC
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
Download the App
```bash
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage  (or double click)
```